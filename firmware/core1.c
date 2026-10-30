#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/structs/m33.h>
#include <float.h>
#include <pico/multicore.h>
#include <math.h>
#include <string.h>

#include "core1.h"
#include "pio_spi.h"
#include "stimjim_context.h"
#include "adc.h"
#include "dac.h"

// Constants

#define SAMPLES 100

static const uint64_t nldac_mask = (1ull << NLDAC_A) | (1ull << NLDAC_B);
static const uint64_t nldac_ch_mask[2] = { 1ull << NLDAC_A, 1ull << NLDAC_B };
static const uint64_t oe0_mask[2] = { 1ull << OE0_A, 1ull << OE0_B };
static const uint64_t oe1_mask[2] = { 1ull << OE1_A, 1ull << OE1_B };
static const uint64_t gpio_mask[2] = {
    (1ull << LED_A) | (1ull << CHANNEL_IO_A) | (1ull << SYNC_A),
    (1ull << LED_B) | (1ull << CHANNEL_IO_B) | (1ull << SYNC_B),};

// clk_sys isn't defined as constant but is initialized only once at runtime
// using clock_get_hz(). Although this variable could easily be scoped to the
// pulsetrain_loop() function, clock_get_hz() exists in flash and thus shouldn't
// be included in our time-critical hot path.
static uint32_t cycles_per_us;

// The duration of an ADC read measured at startup, plus ADC_LEAD_MARGIN_US
static uint32_t adc_lead_cyc;

static uint32_t dropped_summaries;
static uint32_t train_number;
static uint32_t handled_request_seq;
static uint32_t trigger_notice[2];

// Pico SDK function inlined to avoid jitter related to XIP cache-miss
__always_inline static inline bool inline_queue_try_remove(queue_t *q, void *data) {
    uint32_t save = spin_lock_blocking(q->core.spin_lock);

    if (queue_get_level_unsafe(q) != 0) {
        assert(q->rptr <= q->element_count);
        void *ptr = (uint8_t *)q->data + q->rptr * q->element_size;
        memcpy(data, ptr, q->element_size);
        uint16_t next = q->rptr + 1;
        if (next > q->element_count)
            next = 0;
        q->rptr = next;
        lock_internal_spin_unlock_with_notify(&q->core, save);
        return true;
    }

    spin_unlock(q->core.spin_lock, save);
    return false;
}

// Pico SDK function inlined to avoid jitter related to XIP cache-miss
__always_inline static inline bool inline_queue_try_add(queue_t *q, const void *data) {
    uint32_t save = spin_lock_blocking(q->core.spin_lock);

    if (queue_get_level_unsafe(q) != q->element_count) {
        assert(q->wptr <= q->element_count);
        void *ptr = (uint8_t *)q->data + q->wptr * q->element_size;
        memcpy(ptr, data, q->element_size);
        uint16_t next = q->wptr + 1;
        if (next > q->element_count)
            next = 0;
        q->wptr = next;
        lock_internal_spin_unlock_with_notify(&q->core, save);
        return true;
    }

    spin_unlock(q->core.spin_lock, save);
    return false;
}

static bool __not_in_flash_func(measure_offsets)(offsets_t *offsets, const offsets_tx_t offsets_calibration) {
    offsets[0] = offsets_calibration.prev[0];
    offsets[1] = offsets_calibration.prev[1];
    bool all_read = true;

    if (offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_ADC) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(ch, OUTPUT_MODE_GND);
            adc_write_blocking(ch, ADC_BASE_CONFIG);
            float acc = 0.0f;
            uint8_t n = 0;
            for (uint8_t i = 0; i < SAMPLES; i++) {
                int16_t val;
                if (adc_read_blocking(ch, &val)) { acc += (float)val; n++; }
            }
            if (n) offsets[ch].adc = acc / (float)n;
            all_read &= n == SAMPLES;
        }
    }

    if (!(offsets_calibration.offset_tx_type & (OFFSETS_TX_CALIBRATE_CURRENT | OFFSETS_TX_CALIBRATE_VOLTAGE)))
        return all_read;

    const int8_t sweep_range = 50;
    dacs_write_blocking(0, 0);
    dacs_latch(nldac_mask);
    gpio_clr_mask64(nldac_mask);

    for (uint8_t line = 0; line < 2; line++) {
        if (line && !(offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_CURRENT))
            continue;
        if (!line && !(offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_VOLTAGE))
            continue;
        const uint16_t adc_cfg = ADC_BASE_CONFIG | (line ? ADC_CURRENT_LINE : 0);
        for (uint8_t ch = 0; ch < 2; ch++) {
            adc_write_blocking(ch, adc_cfg);
            set_output_mode(ch, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
            int8_t min_i = line ? offsets[ch].current : offsets[ch].voltage;
            float min = FLT_MAX;
            for (int8_t i = -sweep_range; i <= sweep_range; i++) {
                dac_write_blocking(ch, (uint16_t)(int16_t)i);
                sleep_us(10);
                float acc = 0.0f;
                uint8_t n = 0;
                for (uint8_t j = 0; j < SAMPLES; j++) {
                    int16_t val;
                    if (adc_read_blocking(ch, &val)) { acc += (float)val; n++; }
                }
                if (n != SAMPLES) { all_read = false; continue; }
                acc -= offsets[ch].adc * (float)SAMPLES;
                if (fabsf(acc) < min) { min = fabsf(acc); min_i = i; }
            }
            if (line) offsets[ch].current = min_i;
            else offsets[ch].voltage = min_i;
        }
    }

    gpio_set_mask64(nldac_mask);

    // Leave the DACs latched at the current offset (zero current), as the
    // Teensy did after boot; the sweep left them at the last swept code.
    dacs_write_blocking((uint16_t)(int16_t)offsets[0].current, (uint16_t)(int16_t)offsets[1].current);
    dacs_latch(nldac_mask);

    set_output_mode(0, OUTPUT_MODE_GND);
    set_output_mode(1, OUTPUT_MODE_GND);
    return all_read;
}

// masks to clear/set gpio atomically during a stimulus
typedef struct {
    uint64_t start_clr, start_set; // stimulus start: LEDs/sync on, mux as for stage 0
    uint64_t mode_clr, mode_set;   // mux to the output mode (pulse start)
    uint64_t gnd_set;              // mux to ground (inter-pulse gap)
    uint64_t end_clr, end_set;     // stimulus end: LEDs/sync off, mux to ground
    uint64_t nldac;
} stim_gpio_masks_t;

__always_inline static inline stim_gpio_masks_t compute_stim_gpio_masks(const pulsetrain_t *pt) {
    uint64_t led_sync = 0, mode_clr = 0, mode_set = 0, gnd_set = 0, nldac = 0;
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (!(pt->output_mode[ch] & OUTPUT_MODE_ACTIVE_MASK)) continue;
        led_sync |= gpio_mask[ch];
        mode_set |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? oe0_mask[ch] : 0)
                 |  (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? oe1_mask[ch] : 0);
        mode_clr |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? 0 : oe0_mask[ch])
                 |  (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? 0 : oe1_mask[ch]);
        gnd_set  |= oe0_mask[ch] | oe1_mask[ch];
        nldac    |= nldac_ch_mask[ch];
    }
    // A train with no stages only turns the LEDs and sync on for its duration,
    // leaving the outputs as they were
    if (pt->n_stages == 1)
        mode_clr = mode_set = gnd_set = nldac = 0;
    return (stim_gpio_masks_t){
        .start_clr = mode_clr,
        .start_set = led_sync | mode_set,
        .mode_clr  = mode_clr,
        .mode_set  = mode_set,
        .gnd_set   = gnd_set,
        .end_clr   = led_sync,
        .end_set   = gnd_set,
        .nldac     = nldac,
    };
}

__always_inline static inline void init_stim_telemetry(const pulsetrain_t *pt, core1_stim_telemetry_t *st) {
    st->n_stages = pt->n_stages;
    st->n_pulses = 0;
    st->output_mode[0] = pt->output_mode[0];
    st->output_mode[1] = pt->output_mode[1];
    st->cancelled = false;
}

// Rising edges latched on the armed trigger inputs, which share a register
static_assert(CHANNEL_IO_A >> 3 == CHANNEL_IO_B >> 3, "trigger inputs must share an interrupt register");
#define TRIGGER_EDGE_BITS (RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1) | RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1))

static volatile uint32_t latest_trigger;

static void __not_in_flash_func(trigger_isr)(void) {
    const uint32_t edges = io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_A >> 3] & TRIGGER_EDGE_BITS;
    uint32_t ch;
    if (edges & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1)) ch = 0;
    else if (edges & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1)) ch = 1;
    else return;
    io_bank0_hw->intr[CHANNEL_IO_A >> 3] = edges;
    latest_trigger = ((latest_trigger + 2) & ~1u) | ch;
}

// True if X cancelled the stimulus, or a newer T/U request or a trigger preempts it
__always_inline static inline bool stop_requested(const uint32_t seq, const uint32_t trigger) {
    return (sio_hw->doorbell_in_set & DOORBELL_STOP) || stimulus_request_seq != seq || latest_trigger != trigger;
}

__always_inline static inline bool pulsetrain_loop(const pulsetrain_t *pt, const stim_gpio_masks_t *masks,
                                                   core1_stim_telemetry_t *sr, uint32_t stage_start_cyc,
                                                   const uint32_t seq, const uint32_t trigger) {

    const uint8_t gap = pt->n_stages - 1;  // the last stage is the inter-pulse gap
    const uint8_t n_stages = pt->stage_duration[gap] ? pt->n_stages : gap;

    for (uint32_t pulse = 0; pulse < pt->n_pulses; pulse++) {
        sr->n_pulses = pulse + 1;  // pulses started, including one cut short
        for (uint8_t stage = 0; stage < n_stages; stage++) {

            // Preload DAC (blocking)
            uint8_t next_stage = (stage + 1 < n_stages) ? stage + 1 : 0;
            dacs_write_blocking(
                (uint16_t)pt->stage_amplitude[0][next_stage],
                (uint16_t)pt->stage_amplitude[1][next_stage]);

            // Calculate duration of stage in clock cycles. This can exceed the
            // range of the 32-bit cycle counter for long inter-pulse gaps.
            uint64_t stage_dur_cyc = (uint64_t)pt->stage_duration[stage] * cycles_per_us;

            uint64_t mux_clr = 0, mux_set = 0;
            if (next_stage == gap)  mux_set = masks->gnd_set;
            else if (stage == gap) { mux_clr = masks->mode_clr; mux_set = masks->mode_set; }

            // Wait out stages longer than INT32_MAX cycles in chunks the 32-bit
            // cycle counter can measure
            while (stage_dur_cyc > INT32_MAX) {
                while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < INT32_MAX / 2)
                    if (stop_requested(seq, trigger)) { sr->stop_stage = stage; return true; }
                stage_start_cyc += INT32_MAX / 2;
                stage_dur_cyc   -= INT32_MAX / 2;
            }

            if (stage != gap) {
                // Check for a stop request (X, T-1, a newer T/U or a trigger) while waiting to read the ADCs
                // just ahead of the next latch
                while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) + adc_lead_cyc < (uint32_t)stage_dur_cyc)
                    if (stop_requested(seq, trigger)) { sr->stop_stage = stage; return true; }

                // Read ADCs (blocking) and accumulate into the telemetry
                int16_t vals[2];
                if (adcs_read_get_value_blocking(vals)) {
                    sr->measured_amplitudes[0][stage] += vals[0];
                    sr->measured_amplitudes[1][stage] += vals[1];
                    sr->delivered_stages[stage]++;
                }
            }

            // Check for a stop request (X, T-1, a newer T/U or a trigger) while waiting for stage to end, then
            // spin out its last microsecond without them so the stage ends exactly
            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) + cycles_per_us < (uint32_t)stage_dur_cyc)
                if (stop_requested(seq, trigger)) { sr->stop_stage = stage; return true; }
            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < (uint32_t)stage_dur_cyc);

            // Don't latch on the last stage of the last pulse
            if (stage < n_stages - 1 || pulse < pt->n_pulses - 1) {
                gpio_clr_mask64(mux_clr);
                gpio_set_mask64(mux_set);
                dacs_latch(masks->nldac);
            }

            // Accumulate clock cycles of next stage transition
            stage_start_cyc += (uint32_t)stage_dur_cyc;
        }
    }
    return false;
}

// ADC line selection (voltage vs current) for a channel, derived from output mode.
__always_inline static inline uint16_t pt_adc_cfg(const pulsetrain_t *pt, uint8_t ch) {
    return ADC_BASE_CONFIG | ((pt->output_mode[ch] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0);
}

__always_inline static inline void run_pulsetrain(const pulsetrain_t *pt, const int8_t trigger_ch,
                                                  const uint32_t seq, const uint32_t trigger) {

    sio_hw->doorbell_out_set = DOORBELL_TRAIN_RUNNING;

    // This clears cancel requests for triggered pulse trains. Core0 clears
    // cancel requests for T/U pulse trains.
    if (trigger_ch >= 0)
        sio_hw->doorbell_in_clr = DOORBELL_STOP;

    // Start preloading DAC
    pio_spi_select_dac(0);
    pio_spi_select_dac(1);
    dac_write(0, (uint16_t)pt->stage_amplitude[0][0]);
    dac_write(1, (uint16_t)pt->stage_amplitude[1][0]);

    // Compute GPIO masks for analog multiplexer, sync output and LEDs
    stim_gpio_masks_t masks = compute_stim_gpio_masks(pt);

    // Wait for DAC preload to complete and then deselect DACs
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_dac(0);
    pio_spi_deselect_dac(1);

    // Record cycle counter as reference for first stage timing
    uint32_t stage_start_cyc = m33_hw->dwt_cyccnt;

    // Set LEDs, analog multiplexer, and sync output pins
    gpio_clr_mask64(masks.start_clr);
    gpio_set_mask64(masks.start_set);

    dacs_latch(masks.nldac);

    // Start the ADC line config (current vs voltage); it transfers on the PIO in
    // the background while we initialize the telemetry below.
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_write(0, pt_adc_cfg(pt, 0));
    adc_write(1, pt_adc_cfg(pt, 1));

    // Initialize stimulus telemetry (overlaps the ADC config transfer)
    static core1_stim_telemetry_t st;
    init_stim_telemetry(pt, &st);

    // Tell core0 which trigger input started this stimulus, if any
    st.notice = ++train_number << 8;
    st.triggered = trigger_ch >= 0;
    if (st.triggered) {
        st.notice |= trigger_notice[trigger_ch];
        if (multicore_fifo_wready())
            sio_hw->fifo_wr = st.notice;
    }

    // Wait for the ADC config to complete, then deselect
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);

    // All following stages in pulsetrain follow state machine in this loop
    st.cancelled = pulsetrain_loop(pt, &masks, &st, stage_start_cyc, seq, trigger);

    // Clear LEDs, analog multiplexer, and sync output
    gpio_clr_mask64(masks.end_clr);
    gpio_set_mask64(masks.end_set);

    // If the stimulus was cancelled or preempted, set the DACs to output nominal 0
    if (st.cancelled || !pt->stage_duration[pt->n_stages - 1]) {
        dacs_write_blocking(
            (uint16_t)pt->stage_amplitude[0][pt->n_stages - 1],
            (uint16_t)pt->stage_amplitude[1][pt->n_stages - 1]);
        dacs_latch(masks.nldac);
    }

    // Send telemetry about the delivered pulsetrain to core0
    st.dropped_summaries = dropped_summaries;
    if (inline_queue_try_add(&q_stimulus_telemetry, &st))
        dropped_summaries = 0;
    else
        dropped_summaries++;
    memset(&st, 0, sizeof(st));
    sio_hw->doorbell_out_clr = DOORBELL_TRAIN_RUNNING;
}

// Apply queued commands up to the first current stimulus and return it (setting
// *seq to its request number), or NULL. Stimuli a newer T/U supersedes are skipped.
// Inlined to reduce trigger latency by a few clock cycles.
__always_inline static inline const pulsetrain_t *handle_cmd_queue(pulsetrain_t pt_trigger[2], uint32_t *seq) {
    static pulsetrain_t stimulus;   // single-threaded; consumed before the next call
    core1_cmd_t cmd;

    while (inline_queue_try_remove(&q_core1_cmd, &cmd)) {
        switch (cmd.type) {
            case CORE1_CMD_STIMULUS:
                handled_request_seq = cmd.stimulus.seq;
                if (cmd.stimulus.seq != stimulus_request_seq || (sio_hw->doorbell_in_set & DOORBELL_STOP)) break;
                stimulus = cmd.stimulus.pt;
                *seq = cmd.stimulus.seq;
                return &stimulus;
            case CORE1_CMD_TRIGGER_CONFIG:
                pt_trigger[cmd.trigger_config.ch] = cmd.trigger_config.pt;
                trigger_notice[cmd.trigger_config.ch] = (uint32_t)(cmd.trigger_config.idx & 0x7F) << 1 | cmd.trigger_config.ch;
                break;
            case CORE1_CMD_OFFSETS: {
                offsets_rx_t rx;
                rx.adc_read_failed = !measure_offsets(rx.offsets, cmd.offsets);
                queue_add_blocking(&q_offsets_rx, &rx);
                break;
            }
            case CORE1_CMD_MANUAL: {
                manual_cmd_t *m = &cmd.manual;
                if (m->type == MANUAL_CMD_ADC_READ) {
                    const uint16_t adc_cfg = ADC_BASE_CONFIG | (m->line ? ADC_CURRENT_LINE : 0);
                    adc_write_blocking(m->ch, adc_cfg);
                    adc_result_t result;
                    result.ok = adc_read_blocking(m->ch, &result.value);
                    queue_add_blocking(&q_adc_result, &result);
                } 
                else {
                    if (m->type == MANUAL_CMD_DAC_SET) {
                        dac_write_blocking(m->ch, (uint16_t)m->dac_code);
                        dacs_latch(nldac_ch_mask[m->ch]);
                    } 
                    else {
                        set_output_mode(m->ch, m->output_mode);
                    }
                }
                break;
            }
            default: break;
        }
    }
    return NULL;
}

void __time_critical_func(main_core1)(void) {
    pio_spi_init(0);
    pio_spi_init(1);

    // Enable core1's DWT cycle counter for sub-µs stage timing
    m33_hw->demcr    |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;

    // Cache cycles/µs once so the timing loop never has to call
    // clock_get_hz(clk_sys) which exists in flash to avoid potential jitter due
    // to XIP cache-misses
    cycles_per_us = clock_get_hz(clk_sys) / 1000000u;

    dacs_init();
    adcs_write_blocking(ADC_BASE_CONFIG, ADC_BASE_CONFIG);
    sleep_ms(1); // Wait for ADCs to wake up, 500us minimum

    uint32_t adc_read_cyc = 0;
    for (uint8_t i = 0; i < 8; i++) {
        int16_t vals[2];
        const uint32_t start_cyc = m33_hw->dwt_cyccnt;
        const bool ok = adcs_read_get_value_blocking(vals);
        const uint32_t cyc = m33_hw->dwt_cyccnt - start_cyc;
        if (ok && cyc > adc_read_cyc) adc_read_cyc = cyc;
    }
    adc_lead_cyc = adc_read_cyc + ADC_LEAD_MARGIN_US * cycles_per_us;

    irq_set_exclusive_handler(IO_IRQ_BANK0, trigger_isr);
    irq_set_enabled(IO_IRQ_BANK0, true);

    multicore_fifo_push_blocking(CORE_HANDSHAKE_MESSAGE);

    // Pulse train armed on each trigger input, set via the R/S commands.
    pulsetrain_t pt_trigger[2] = {0};
    uint32_t handled_trigger = latest_trigger;

    while (true) {
        uint32_t seq;
        const pulsetrain_t *pt = handle_cmd_queue(pt_trigger, &seq);
        const uint32_t trigger = latest_trigger;
        int8_t trigger_ch = -1;
        // Check triggers unless a T/U is queued or about to be
        if (!pt && trigger != handled_trigger && stimulus_request_seq == handled_request_seq) {
            seq = handled_request_seq;
            trigger_ch = (int8_t)(trigger & 1);
            pt = &pt_trigger[trigger_ch];
        }
        handled_trigger = trigger;
        if (pt)
            run_pulsetrain(pt, trigger_ch, seq, trigger);
    }
}

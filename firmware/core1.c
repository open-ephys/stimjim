#include <hardware/clocks.h>
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

static const uint64_t nldac_mask = (1LL << NLDAC_A) | (1LL << NLDAC_B);
static const uint64_t oe0_mask[2] = { 1LL << OE0_A, 1LL << OE0_B };
static const uint64_t oe1_mask[2] = { 1LL << OE1_A, 1LL << OE1_B };
static const uint64_t gpio_mask[2] = {
    (1LL << LED_A) | (1LL << CHANNEL_IO_A),
    (1LL << LED_B) | (1LL << CHANNEL_IO_B),};
static const uint8_t channel_io_pins[2] = { CHANNEL_IO_A, CHANNEL_IO_B };

// clk_sys isn't defined as constant but is initialized only once at runtime
// using clock_get_hz(). Although this variable could easily be scoped to the
// pulsetrain_loop() function, clock_get_hz() exists in flash and thus shouldn't
// be included in our time-critical hot path.
static uint32_t cycles_per_us;

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

static void measure_offsets(offsets_t *offsets, const offsets_tx_t offsets_calibration) {
    if (offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_ADC) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(ch, OUTPUT_MODE_GND);
            adc_write_blocking(ch, ADC_BASE_CONFIG);
            float acc = 0.0f;
            for (uint8_t i = 0; i < SAMPLES; i++)
                acc += (float)adc_read_blocking(ch);
            offsets[ch].adc = acc / (float)SAMPLES;
        }
    }
    else {
        offsets[0].adc = offsets_calibration.adc[0];
        offsets[1].adc = offsets_calibration.adc[1];
    }

    const int8_t sweep_range = 50;
    dacs_write_blocking(0, 0);
    dacs_latch();
    gpio_clr_mask64(nldac_mask);

    for (uint8_t line = 0; line < 2; line++) {
        if (line && !(offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_CURRENT)) {
            offsets[0].current = 0;
            offsets[1].current = 0;
            continue;
        }
        if (!line && !(offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_VOLTAGE)) {
            offsets[0].voltage = 0;
            offsets[1].voltage = 0;
            continue;
        }
        const uint16_t adc_cfg = ADC_BASE_CONFIG | (line ? ADC_CURRENT_LINE : 0);
        for (uint8_t ch = 0; ch < 2; ch++) {
            adc_write_blocking(ch, adc_cfg);
            set_output_mode(ch, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
            int8_t min_i = -sweep_range;
            float min = FLT_MAX;
            for (int8_t i = -sweep_range; i <= sweep_range; i++) {
                dac_write_blocking(ch, (uint16_t)(int16_t)i);
                sleep_us(10);
                float acc = 0.0f;
                for (uint8_t j = 0; j < SAMPLES; j++)
                    acc += (float)adc_read_blocking(ch);
                acc -= offsets[ch].adc * (float)SAMPLES;
                if (fabsf(acc) < min) { min = fabsf(acc); min_i = i; }
            }
            if (line) offsets[ch].current = min_i;
            else offsets[ch].voltage = min_i;
        }
    }

    gpio_set_mask64(nldac_mask);
    set_output_mode(0, OUTPUT_MODE_GND);
    set_output_mode(1, OUTPUT_MODE_GND);
}

// masks to clear/set gpio atomically before/after stimulus
typedef struct {
    uint64_t start_clr, start_set; // gpio masks for stimulus start
    uint64_t end_clr, end_set;     // gpio masks for stimulus end
} stim_gpio_masks_t;

__always_inline static inline stim_gpio_masks_t compute_stim_gpio_masks(const pulsetrain_t *pt) {
    uint64_t start_clr = 0, start_set = 0, end_clr = 0, end_set = 0, oe_set = 0;
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (!(pt->output_mode[ch] & OUTPUT_MODE_ACTIVE_MASK)) continue;
        start_set |= gpio_mask[ch];
        oe_set    |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? oe0_mask[ch] : 0)
                  |  (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? oe1_mask[ch] : 0);
        start_clr |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? 0 : oe0_mask[ch])
                  |  (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? 0 : oe1_mask[ch]);
        end_clr   |= gpio_mask[ch];
        end_set   |= oe0_mask[ch] | oe1_mask[ch];
    }
    return (stim_gpio_masks_t){
        .start_clr = start_clr,
        .start_set = start_set | oe_set,
        .end_clr   = end_clr,
        .end_set   = end_set,
    };
}

// Scalars only. Zeroing the arrays compiles to memset() (a flash call) in the
// stage-0 window; pulsetrain_loop assigns on the first pulse instead.
__always_inline static inline void init_stim_telemetry(const pulsetrain_t *pt, core1_stim_telemetry_t *st) {
    st->n_stages       = pt->n_stages;
    st->output_mode[0] = pt->output_mode[0];
    st->output_mode[1] = pt->output_mode[1];
    st->cancelled      = false;
}

__always_inline static inline bool pulsetrain_loop(const pulsetrain_t *pt, core1_stim_telemetry_t *sr, uint32_t stage_start_cyc) {

    const uint32_t dac_settle_cyc = DAC_SETTLE_US * cycles_per_us;

    for (uint32_t pulse = 0; pulse < pt->n_pulses; pulse++) {
        for (uint8_t stage = 0; stage < pt->n_stages; stage++) {

            // Preload DAC (blocking)
            uint8_t next_stage = (stage + 1 < pt->n_stages) ? stage + 1 : 0;
            dacs_write_blocking(
                (uint16_t)pt->stage_amplitude[0][next_stage],
                (uint16_t)pt->stage_amplitude[1][next_stage]);

            // Calculate duration of stage in clock cycles
            uint32_t stage_dur_cyc = pt->stage_duration[stage] * cycles_per_us;

            // Check for cancel ('X') commands while waiting for DAC to settle
            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < dac_settle_cyc)
                if (sio_hw->doorbell_in_set & 1u) return true;  // cancel

            // Read ADCs (blocking). Assign on the first pulse, accumulate after,
            // so the telemetry arrays need no pre-zeroing memset.
            int16_t vals[2];
            adcs_read_get_value_blocking(vals);
            if (pulse == 0) {
                sr->measured_amplitudes[0][stage] = vals[0];
                sr->measured_amplitudes[1][stage] = vals[1];
                sr->delivered_stages[stage]       = 1;
            } else {
                sr->measured_amplitudes[0][stage] += vals[0];
                sr->measured_amplitudes[1][stage] += vals[1];
                sr->delivered_stages[stage]++;
            }

            // Check for cancel ('X') commands while waiting for stage to end
            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < stage_dur_cyc)
                if (sio_hw->doorbell_in_set & 1u) return true;  // cancel

            // Don't latch on the last stage of the last pulse
            if (stage < pt->n_stages - 1 || pulse < pt->n_pulses - 1)
                dacs_latch();

            // Accumulate clock cycles of next stage transition
            stage_start_cyc += stage_dur_cyc;
        }
    }
    return false;
}

// ADC line selection (voltage vs current) for a channel, derived from output mode.
__always_inline static inline uint16_t pt_adc_cfg(const pulsetrain_t *pt, uint8_t ch) {
    return ADC_BASE_CONFIG | ((pt->output_mode[ch] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0);
}

__always_inline static inline void run_pulsetrain(const pulsetrain_t *pt) {

    // Return if pulse train isn't initialized
    if (pt->n_pulses == 0 || pt->n_stages == 0) return;

    // Clear cancel doorbell
    sio_hw->doorbell_in_clr = 1u;

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

    // Set LEDs, analog multiplexer, and sync output output pins
    gpio_clr_mask64(masks.start_clr);
    gpio_set_mask64(masks.start_set);

    dacs_latch();

    // Record cycle counter as reference for first stage timing
    uint32_t stage_start_cyc = m33_hw->dwt_cyccnt;

    // Start the ADC line config (current vs voltage); it transfers on the PIO in
    // the background while we initialize the telemetry below.
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_write(0, pt_adc_cfg(pt, 0));
    adc_write(1, pt_adc_cfg(pt, 1));

    // Initialize stimulus telemetry (overlaps the ADC config transfer)
    core1_stim_telemetry_t st;
    init_stim_telemetry(pt, &st);

    // Wait for the ADC config to complete, then deselect
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);

    // All following stages in pulsetrain follow state machine in this loop
    st.cancelled = pulsetrain_loop(pt, &st, stage_start_cyc);

    // Clear LEDs, analog multiplexer, and sync output
    gpio_clr_mask64(masks.end_clr);
    gpio_set_mask64(masks.end_set);

    // If a cancel ('X') command was received, set the DACs to output nominal 0
    if (st.cancelled) {
        dacs_write_blocking(
            (uint16_t)pt->stage_amplitude[0][pt->n_stages - 1],
            (uint16_t)pt->stage_amplitude[1][pt->n_stages - 1]);
        dacs_latch();
    }

    // Send telemetry about the delivered pulsetrain to core0
    inline_queue_try_add(&q_stimulus_telemetry, &st);
}

// Discard any rising edges on the trigger inputs so that triggers which
// occurred while handling commands or stimulus delivery are ignored.
__always_inline static inline void clear_trigger_edges(void) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        uint8_t pin = channel_io_pins[ch];
        io_bank0_hw->intr[pin >> 3] = RISING_EDGE_INTERRUPT_BIT(pin, 1);
    }
}

// S/R (TRIGGER_CONFIG) and B/C/D (OFFSETS) are processed.
// Stimulus requests (U/T) and manual commands (A/M/V/E) received during a stimulus are discarded.
__always_inline static inline void handle_cmd_queue_during_stimulus(pulsetrain_t pt_trigger[2]) {
    core1_cmd_t cmd;
    while (inline_queue_try_remove(&q_core1_cmd, &cmd)) {
        switch (cmd.type) {
            case CORE1_CMD_TRIGGER_CONFIG:
                pt_trigger[cmd.trigger_config.ch] = cmd.trigger_config.pt;
                break;
            case CORE1_CMD_OFFSETS: {
                offsets_t offsets[2];
                measure_offsets(offsets, cmd.offsets);
                queue_add_blocking(&q_offsets_rx, offsets);
                break;
            }
            default: break;  // discard CORE1_CMD_STIMULUS and CORE1_CMD_MANUAL
        }
    }
}

__always_inline static inline void handle_cmd_queue(pulsetrain_t pt_trigger[2]) {
    core1_cmd_t cmd;
    core1_cmd_t latest_stimulus;
    bool has_stimulus = false;

    if (!inline_queue_try_remove(&q_core1_cmd, &cmd))
        return;

    do {
        if (cmd.type == CORE1_CMD_STIMULUS) {
            latest_stimulus = cmd;
            has_stimulus = true;
        } else {
            switch (cmd.type) {
                case CORE1_CMD_TRIGGER_CONFIG:
                    pt_trigger[cmd.trigger_config.ch] = cmd.trigger_config.pt;
                    break;
                case CORE1_CMD_OFFSETS: {
                    offsets_t offsets[2];
                    measure_offsets(offsets, cmd.offsets);
                    queue_add_blocking(&q_offsets_rx, offsets);
                    break;
                }
                case CORE1_CMD_MANUAL: {
                    manual_cmd_t *m = &cmd.manual;
                    if (m->type == MANUAL_CMD_DAC_SET) {
                        dac_write_blocking(m->ch, (uint16_t)m->dac_code);
                        dacs_latch();
                    } else if (m->type == MANUAL_CMD_SET_OUTPUT_MODE) {
                        set_output_mode(m->ch, m->output_mode);
                    } else {
                        const uint16_t adc_cfg = ADC_BASE_CONFIG | (m->line ? ADC_CURRENT_LINE : 0);
                        adc_write_blocking(m->ch, adc_cfg);
                        int16_t val = adc_read_blocking(m->ch);
                        queue_add_blocking(&q_adc_result, &val);
                    }
                    break;
                }
                default: break;
            }
        }
    } while (inline_queue_try_remove(&q_core1_cmd, &cmd));

    if (has_stimulus) {
        run_pulsetrain(&latest_stimulus.stimulus);
        handle_cmd_queue_during_stimulus(pt_trigger);
    }

    // Ignore any trigger that occurred while handling the user command
    clear_trigger_edges();
}

__always_inline static inline void handle_triggers(pulsetrain_t pt_trigger[2]) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        uint8_t pin = channel_io_pins[ch];
        uint32_t bit = RISING_EDGE_INTERRUPT_BIT(pin, 1);
        if (io_bank0_hw->proc1_irq_ctrl.ints[pin >> 3] & bit) {
            run_pulsetrain(&pt_trigger[ch]);
            handle_cmd_queue_during_stimulus(pt_trigger);
            // Ignore any trigger that occurred during the triggered stimulus
            clear_trigger_edges();
            break;
        }
    }
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

    multicore_fifo_push_blocking(CORE_HANDSHAKE_MESSAGE);

    // Pulse train armed on each trigger input, set via the R/S commands.
    pulsetrain_t pt_trigger[2] = {0};

    while (true) {
        handle_cmd_queue(pt_trigger);
        handle_triggers(pt_trigger);
    }
}

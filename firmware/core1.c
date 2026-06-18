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

#define SAMPLES 100

static const uint64_t gpio_mask[2] = {
    (1LL << LED_A) | (1LL << CHANNEL_IO_A),
    (1LL << LED_B) | (1LL << CHANNEL_IO_B),
};
static const uint64_t nldac_mask = (1LL << NLDAC_A) | (1LL << NLDAC_B);
static const uint64_t oe0_mask[2] = { 1LL << OE0_A, 1LL << OE0_B };
static const uint64_t oe1_mask[2] = { 1LL << OE1_A, 1LL << OE1_B };
static const uint8_t channel_io_pins[2] = { CHANNEL_IO_A, CHANNEL_IO_B };

// Inlined to keep queue ops in SRAM; avoids XIP jitter on trigger path
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

// Globals accessible from both main_core1 and isr_trigger
static pio_spi_t pio_spi[2] = {
    {   .pio = pio0, .base = 16, .sm_active = false,
        .miso = SPI_MISO_A, .mosi = SPI_MOSI_A, .sck = SPI_SCK_A,
        .cs_dac = CSA_A, .cs_adc = CSB_A },
    {   .pio = pio1, .base = 0,  .sm_active = false,
        .miso = SPI_MISO_B, .mosi = SPI_MOSI_B, .sck = SPI_SCK_B,
        .cs_dac = CSA_B, .cs_adc = CSB_B },
};
static pulsetrain_t pt_trigger[2];

// -1 idle; 0/1 ISR-triggered pending on ch; 2 software stimulus running
static volatile int8_t trigger_pending = -1;

// dwt_cyccnt ticks per µs; set once at startup
static uint32_t cycles_per_us;

// Output of stimulus_prelatch; written by isr_trigger, read by main loop
typedef struct { uint32_t stage_start_cyc; uint64_t end_clr, end_set; } prelatch_t;
static prelatch_t isr_prelatch;

// Skips stale STIMULUS items after a stimulus ends; cleared by next non-stimulus cmd
static bool drain_stimuli = false;

static void measure_offsets(offsets_t *offsets, const offsets_tx_t offsets_calibration) {
    if (offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_ADC) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(ch, OUTPUT_MODE_GND);
            adc_write_blocking(&pio_spi[ch], ADC_BASE_CONFIG);
            float acc = 0.0f;
            for (uint8_t i = 0; i < SAMPLES; i++)
                acc += (float)adc_read_blocking(&pio_spi[ch]);
            offsets[ch].adc = acc / (float)SAMPLES;
        }
    }
    else {
        offsets[0].adc = offsets_calibration.adc[0];
        offsets[1].adc = offsets_calibration.adc[1];
    }

    const int8_t sweep_range = 50;
    dacs_write_blocking(pio_spi, 0, 0);
    dacs_latch(nldac_mask);
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
            adc_write_blocking(&pio_spi[ch], adc_cfg);
            set_output_mode(ch, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
            int8_t min_i = -sweep_range;
            float min = FLT_MAX;
            for (int8_t i = -sweep_range; i <= sweep_range; i++) {
                dac_write_blocking(&pio_spi[ch], (uint16_t)(int16_t)i);
                sleep_us(10);
                float acc = 0.0f;
                for (uint8_t j = 0; j < SAMPLES; j++)
                    acc += (float)adc_read_blocking(&pio_spi[ch]);
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

// Loads DAC, asserts OE/LEDs/CHANNEL_IO, latches; returns prelatch_t anchored
// to the latch moment so stage timing starts from the CHANNEL_IO rise edge
__always_inline static prelatch_t stimulus_prelatch(const pulsetrain_t *pt) {
    pio_spi_select_dac(&pio_spi[0]);
    pio_spi_select_dac(&pio_spi[1]);
    dac_write(&pio_spi[0], (uint16_t)pt->stage_amplitude[0][0]);
    dac_write(&pio_spi[1], (uint16_t)pt->stage_amplitude[1][0]);

    prelatch_t r = { .end_clr = 0, .end_set = 0 };
    uint64_t oe_clr = 0, oe_set = 0, led_io_mask = 0;
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (!(pt->output_mode[ch] & OUTPUT_MODE_ACTIVE)) continue;
        led_io_mask |= gpio_mask[ch];
        oe_set |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? oe0_mask[ch] : 0)
                | (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? oe1_mask[ch] : 0);
        oe_clr |= (pt->output_mode[ch] & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND) ? 0 : oe0_mask[ch])
                | (pt->output_mode[ch] & (OUTPUT_MODE_FLOAT   | OUTPUT_MODE_GND) ? 0 : oe1_mask[ch]);
        r.end_clr |= gpio_mask[ch];
        r.end_set |= oe0_mask[ch] | oe1_mask[ch];
    }

    while (!pio_spi_is_done(&pio_spi[0]) || !pio_spi_is_done(&pio_spi[1]));
    pio_spi_deselect_dac(&pio_spi[0]);
    pio_spi_deselect_dac(&pio_spi[1]);

    // One write per bank: OE + LEDs + CHANNEL_IO change simultaneously; NLDAC unchanged
    uint64_t assert_mask = led_io_mask | oe_set;
    uint32_t pre_lo = (sio_hw->gpio_out   | (uint32_t)(assert_mask))       & ~(uint32_t)(oe_clr);
    uint32_t pre_hi = (sio_hw->gpio_hi_out | (uint32_t)(assert_mask >> 32)) & ~(uint32_t)(oe_clr >> 32);
    sio_hw->gpio_hi_out = pre_hi;
    sio_hw->gpio_out    = pre_lo;

    dacs_latch(nldac_mask);
    r.stage_start_cyc = m33_hw->dwt_cyccnt;
    return r;
}

// ISR: on trigger edge, runs prelatch inline, signals main loop (~1-2 µs)
__isr static void __time_critical_func(isr_trigger)(void) {
    int8_t ch = -1;
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t pin = channel_io_pins[i];
        if (io_bank0_hw->proc1_irq_ctrl.ints[pin >> 3] & RISING_EDGE_INTERRUPT_BIT(pin, 1)) {
            io_bank0_hw->intr[pin >> 3] = RISING_EDGE_INTERRUPT_BIT(pin, 1);
            if (ch < 0) ch = (int8_t)i;
        }
    }
    if (ch < 0 || trigger_pending >= 0) return;

    const pulsetrain_t *pt = &pt_trigger[ch];
    if (pt->n_pulses == 0 || pt->n_stages == 0) return;

    sio_hw->doorbell_in_clr = 1u;
    isr_prelatch = stimulus_prelatch(pt);
    trigger_pending = ch;  // last write: signals main loop
}

__always_inline static bool stimulus_loop(const pulsetrain_t *pt, stimulus_result_t *sr, pio_spi_t *p, uint32_t stage_start_cyc) {
    const uint32_t dac_settle_cyc = DAC_SETTLE_US * cycles_per_us;
    for (uint32_t pulse = 0; pulse < pt->n_pulses; pulse++) {
        for (uint8_t stage = 0; stage < pt->n_stages; stage++) {

            // Preload next stage SPI while current stage is still running
            uint8_t next_stage = (stage + 1 < pt->n_stages) ? stage + 1 : 0;
            dacs_write_blocking(p,
                (uint16_t)pt->stage_amplitude[0][next_stage],
                (uint16_t)pt->stage_amplitude[1][next_stage]);

            uint32_t stage_dur_cyc = pt->stage_duration[stage] * cycles_per_us;

            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < dac_settle_cyc)
                if (sio_hw->doorbell_in_set & 1u) return true;  // cancel

            int16_t vals[2];
            adcs_read_get_value_blocking(p, vals);
            for (uint8_t ch = 0; ch < 2; ch++)
                sr->measured_amplitudes[ch][stage] += vals[ch];
            sr->delivered_stages[stage]++;

            while ((uint32_t)(m33_hw->dwt_cyccnt - stage_start_cyc) < stage_dur_cyc)
                if (sio_hw->doorbell_in_set & 1u) return true;  // cancel

            if (stage < pt->n_stages - 1 || pulse < pt->n_pulses - 1)
                dacs_latch(nldac_mask);

            stage_start_cyc += stage_dur_cyc;
        }
    }
    return false;
}

// Post-latch body shared by triggered and software paths
__always_inline static void stimulus_postlatch(const pulsetrain_t *pt, prelatch_t p) {
    stimulus_result_t sr;
    sr.n_stages = pt->n_stages;
    sr.output_mode[0] = pt->output_mode[0];
    sr.output_mode[1] = pt->output_mode[1];
    sr.cancelled = false;
    for (uint8_t s = 0; s < pt->n_stages; s++)
        sr.measured_amplitudes[0][s] = sr.measured_amplitudes[1][s] = sr.delivered_stages[s] = 0;

    adcs_write_blocking(pio_spi,
        ADC_BASE_CONFIG | ((pt->output_mode[0] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0),
        ADC_BASE_CONFIG | ((pt->output_mode[1] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0));

    sr.cancelled = stimulus_loop(pt, &sr, pio_spi, p.stage_start_cyc);

    if (sr.cancelled)
        dacs_write_blocking(pio_spi,
            (uint16_t)pt->stage_amplitude[0][pt->n_stages - 1],
            (uint16_t)pt->stage_amplitude[1][pt->n_stages - 1]);

    gpio_clr_mask64(p.end_clr);
    gpio_set_mask64(p.end_set);

    inline_queue_try_add(&q_stimulus_result, &sr);
    drain_stimuli = true;
}

// CAS claims trigger_pending (-1→2); if ISR fires in the window, STREXB fails
// and we bail — main loop handles the triggered case via isr_prelatch
__always_inline static void run_stimulus(const pulsetrain_t *pt) {
    int8_t expected = -1;
    if (!__atomic_compare_exchange_n(&trigger_pending, &expected, (int8_t)2,
                                      false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
        return;
    if (pt->n_pulses == 0 || pt->n_stages == 0) {
        trigger_pending = -1;
        return;
    }
    sio_hw->doorbell_in_clr = 1u;
    stimulus_postlatch(pt, stimulus_prelatch(pt));
    trigger_pending = -1;
}

__always_inline static inline void process_cmd_queue(void) {
    core1_cmd_t cmd;
    while (inline_queue_try_remove(&q_core1_cmd, &cmd)) {
        if (cmd.type == CORE1_CMD_STIMULUS) {
            if (!drain_stimuli)
                run_stimulus(&cmd.stimulus);
            continue;
        }
        drain_stimuli = false;
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
                    dac_write_blocking(&pio_spi[m->ch], (uint16_t)m->dac_code);
                    dacs_latch(nldac_mask);
                } else if (m->type == MANUAL_CMD_SET_OUTPUT_MODE) {
                    set_output_mode(m->ch, m->output_mode);
                } else {
                    const uint16_t adc_cfg = ADC_BASE_CONFIG | (m->line ? ADC_CURRENT_LINE : 0);
                    adc_write_blocking(&pio_spi[m->ch], adc_cfg);
                    int16_t val = adc_read_blocking(&pio_spi[m->ch]);
                    queue_add_blocking(&q_adc_result, &val);
                }
                break;
            }
            default: break;
        }
    }
}

void __time_critical_func(main_core1)(void) {
    pio_spi_init(&pio_spi[0]);
    pio_spi_init(&pio_spi[1]);

    // Enable core1's DWT cycle counter for sub-µs stage timing
    m33_hw->demcr    |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
    cycles_per_us = clock_get_hz(clk_sys) / 1000000u;

    dacs_init(pio_spi);
    adcs_write_blocking(pio_spi, ADC_BASE_CONFIG, ADC_BASE_CONFIG);
    sleep_ms(1); // Wait for ADCs to wake up, 500us minimum

    irq_set_exclusive_handler(IO_IRQ_BANK0, isr_trigger);
    irq_set_enabled(IO_IRQ_BANK0, true);

    multicore_fifo_push_blocking(CORE_HANDSHAKE_MESSAGE);

    while (true) {
        int8_t ch = trigger_pending;
        if ((uint8_t)ch < 2u) {
            stimulus_postlatch(&pt_trigger[ch], isr_prelatch);
            trigger_pending = -1;
        }
        process_cmd_queue();
    }
}

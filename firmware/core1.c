#include <hardware/timer.h>
#include <float.h>
#include <pico/multicore.h>
#include <math.h>

#include "core1.h"
#include "pio_spi.h"
#include "stimjim_context.h"

static const stim_result_t sr_reset[2] = { 
    { .ch = 0, .delivered_stages = 0, .measured_amplitudes = 0, .output_mode = 0 },
    { .ch = 1, .delivered_stages = 0, .measured_amplitudes = 0, .output_mode = 0 }
};

// global variables
static volatile bool stage_transitioned[3] = { false };
static volatile bool stim_ending[3] = { false };
static volatile uint64_t transition_tick[3] = { 0 };

typedef enum {
    STIMULUS_STATE_IDLE,
    STIMULUS_STATE_ACTIVE,
    STIMULUS_STATE_END
} stimulus_state_t;

typedef struct {
    // state machine
    stimulus_state_t stimulus_state;
    bool pending_latch; 
    bool adc_read_initiated;
    bool adc_read_terminated;
    uint32_t stage_counter;
    uint32_t pulse_counter;
    // pulsetrain data
    pulsetrain_t pt_active;
    pulsetrain_t pt_trigger;
    // result accumulator sent back to core0
    stim_result_t sr;
    // hardware peripherals
    pio_spi_t *pio_spi;
    // pin constants
    const bool channel;
    const uint8_t channel_io_pin;
    const uint8_t led_pin;
    const uint8_t nldac_pin;
} stimulus_context_t;

static inline void adc_write_blocking(pio_spi_t *pio_spi, const uint16_t config) {
    pio_spi_select_adc(pio_spi);
    adc_write(pio_spi, config);
    pio_spi_wait_done(pio_spi);
    pio_spi_deselect_adc(pio_spi);
}

static int16_t adc_read_blocking(pio_spi_t *pio_spi) {
    int16_t val;
    pio_spi_select_adc(pio_spi);
    adc_read(pio_spi);
    pio_spi_wait_done(pio_spi);
    adc_get_value(pio_spi, &val);
    pio_spi_deselect_adc(pio_spi);
    return val;
}

static void dac_init(pio_spi_t *p) {
    const uint32_t dac_range = (0x08u << 16) | 4u;
    const uint32_t dac_power = (0x10u << 16) | 1u;

    pio_spi_select_dac(p);
    dac_write_config(p, dac_range);
    pio_spi_wait_done(p);
    pio_spi_deselect_dac(p);

    pio_spi_select_dac(p);
    dac_write_config(p, dac_power);
    pio_spi_wait_done(p);
    pio_spi_deselect_dac(p);
}

static void measure_offsets(stimulus_context_t *sc, offsets_t *offsets, const bool all_offsets) {
    if (all_offsets) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(sc[ch].channel, OUTPUT_MODE_GND);
            adc_write_blocking(sc[ch].pio_spi, 0x8010u);
            float acc = 0.0f;
            for (uint8_t i = 0; i < SAMPLES; i++)
                acc += (float)adc_read_blocking(sc[ch].pio_spi);
            offsets[ch].adc = acc / (float)SAMPLES;
        }
    }

    const int8_t sweep_range = 50;
    gpio_put(sc[0].nldac_pin, false);
    gpio_put(sc[1].nldac_pin, false);

    for (uint8_t line = 0; line < 2; line++) {
        const uint16_t adc_cfg = 0x8010u | ((uint16_t)line << 10);
        for (uint8_t ch = 0; ch < 2; ch++) {
            adc_write_blocking(sc[ch].pio_spi, adc_cfg);
            set_output_mode(sc[ch].channel, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
            int8_t min_i = -sweep_range;
            float  min   = FLT_MAX;
            for (int8_t i = -sweep_range; i <= sweep_range; i++) {
                pio_spi_select_dac(sc[ch].pio_spi);
                dac_write_output(sc[ch].pio_spi, i);
                pio_spi_wait_done(sc[ch].pio_spi);
                pio_spi_deselect_dac(sc[ch].pio_spi);
                for (uint32_t j = 0; j < 500; j++) { __asm volatile("nop\n\t"); }
                float acc = 0.0f;
                for (uint8_t j = 0; j < SAMPLES; j++)
                    acc += (float)adc_read_blocking(sc[ch].pio_spi);
                acc -= offsets[ch].adc * (float)SAMPLES;
                if (fabsf(acc) < min) { min = fabsf(acc); min_i = i; }
            }
            if (line) offsets[ch].current = min_i;
            else offsets[ch].voltage = min_i;
        }
    }

    gpio_put(sc[0].nldac_pin, true);
    gpio_put(sc[1].nldac_pin, true);
    set_output_mode(sc[0].channel, OUTPUT_MODE_GND);
    set_output_mode(sc[1].channel, OUTPUT_MODE_GND);
}

static inline void process_offsets_queue(stimulus_context_t *sc) {
    bool all_offsets;
    while (queue_try_remove(&q_tx_offsets, &all_offsets)) {
        offsets_t offsets[2];
        measure_offsets(sc, offsets, all_offsets);
        queue_add_blocking(&q_rx_offsets, offsets);
    }
}

static inline uint64_t inline_time_us_64(void) {
    timer_hw_t *timer = PICO_DEFAULT_TIMER_INSTANCE();
    uint32_t hi = timer->timerawh;
    uint32_t lo;
    do {
        lo = timer->timerawl;
        uint32_t next_hi = timer->timerawh;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (true);
    return ((uint64_t) hi << 32u) | lo;
}

static inline void dac_latch(uint64_t nldac_mask) {
    gpio_clr_mask64(nldac_mask);
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    gpio_set_mask64(nldac_mask);
}

static inline void isr_common(uint8_t index, uint64_t gpio_mask, uint64_t nldac_mask) {
    timer0_hw->intf &= ~(1u << index);
    timer0_hw->intr = 1u << index;
    transition_tick[index] = inline_time_us_64();
    if (stim_ending[index]) {
        if (index > 1) {
            set_output_mode(0, OUTPUT_MODE_GND);
            set_output_mode(1, OUTPUT_MODE_GND);
        }
        else {
            set_output_mode(index, OUTPUT_MODE_GND);
        }
        gpio_clr_mask64(gpio_mask);
    } 
    else {
        gpio_set_mask64(gpio_mask);
        dac_latch(nldac_mask);
    }
    stage_transitioned[index] = true;
}

static inline void __isr __time_critical_func(isr_ch0)(void) {
    isr_common(0, (1LL << LED_A) | (1LL << CHANNEL_IO_A), 1LL << NLDAC_A);
}

static inline void __isr __time_critical_func(isr_ch1)(void) {
    isr_common(1, (1LL << LED_B) | (1LL << CHANNEL_IO_B), 1LL << NLDAC_B);
}

static inline void __isr __time_critical_func(isr_sync)(void) {
    isr_common(2, (1LL << LED_A) | (1LL << CHANNEL_IO_A) | (1LL << LED_B) | (1LL << CHANNEL_IO_B), (1LL << NLDAC_A) | (1LL << NLDAC_B));
}

static inline void schedule_alarm(uint8_t index, uint64_t target_us) {    
    timer0_hw->intr = 1u << index;
    timer0_hw->alarm[index] = (uint32_t)target_us;
    if (inline_time_us_64() >= target_us)
        timer0_hw->intf = 1u << index;
}

static inline bool check_for_stim_cancel_signal(stimulus_context_t *sc, bool *sync) {
    if (!(sio_hw->doorbell_in_set & (1u << sc->channel))) return false;
    set_output_mode(sc->channel, OUTPUT_MODE_GND);
    gpio_put(sc->channel_io_pin, false);
    gpio_put(sc->led_pin, false);
    // alarm_pool_cancel_alarm(alarm_pool_core1, sc->alarm_id);
    pio_spi_deselect_adc(sc->pio_spi);
    pio_spi_deselect_dac(sc->pio_spi);
    pio_spi_select_dac(sc->pio_spi);
    dac_write_output(sc->pio_spi, sc->pt_active.stage_amplitude[sc->channel][sc->pt_active.n_stages - 1]);
    while (!pio_spi_is_done(sc->pio_spi));
    pio_spi_deselect_dac(sc->pio_spi);
    dac_latch(1LL << sc->nldac_pin);
    sc->stimulus_state = STIMULUS_STATE_END;
    *sync = false;
    return true;
}

static inline void clear_doorbell(bool ch) {
    sio_hw->doorbell_in_clr = 1 << ch;
}

static inline void init_stim_channel(stimulus_context_t *sc, uint8_t ch) {
    sc[ch].sr = (stim_result_t){
        .ch          = ch,
        .n_stages    = sc[ch].pt_active.n_stages,
        .output_mode = sc[ch].pt_active.output_mode[ch],
    };
    sc[ch].stage_counter       = 0;
    sc[ch].pulse_counter       = 0;
    sc[ch].adc_read_initiated  = true; 
    sc[ch].adc_read_terminated = true;
    sc[ch].stimulus_state      = STIMULUS_STATE_ACTIVE;
}

__always_inline static inline void initate_stim_sm(stimulus_context_t *sc, bool *sync, pulsetrain_t pt) {
    if (pt.output_mode[0] < OUTPUT_MODE_FLOAT && pt.output_mode[1] < OUTPUT_MODE_FLOAT) {
        if (sc[0].stimulus_state == STIMULUS_STATE_IDLE &&
            sc[1].stimulus_state == STIMULUS_STATE_IDLE) {
            sc[0].pt_active = sc[1].pt_active = pt;
            init_stim_channel(sc, 0);
            init_stim_channel(sc, 1);
 
            pio_spi_select_adc(sc[0].pio_spi);
            adc_write(sc[0].pio_spi, 0x8010u | ((uint16_t)pt.output_mode[0] << 10));
            pio_spi_select_adc(sc[1].pio_spi);
            adc_write(sc[1].pio_spi, 0x8010u | ((uint16_t)pt.output_mode[1] << 10));

            pio_spi_wait_done(sc[0].pio_spi);
            pio_spi_deselect_adc(sc[0].pio_spi);
            pio_spi_wait_done(sc[1].pio_spi);
            pio_spi_deselect_adc(sc[1].pio_spi);
 
            pio_spi_select_dac(sc[0].pio_spi);
            dac_write_output(sc[0].pio_spi, pt.stage_amplitude[0][0]);
            pio_spi_select_dac(sc[1].pio_spi);
            dac_write_output(sc[1].pio_spi, pt.stage_amplitude[1][0]);

            pio_spi_wait_done(sc[0].pio_spi);
            pio_spi_deselect_dac(sc[0].pio_spi);
            pio_spi_wait_done(sc[1].pio_spi);
            pio_spi_deselect_dac(sc[1].pio_spi);
 
            set_output_mode(0, pt.output_mode[0]);
            set_output_mode(1, pt.output_mode[1]);
 
            stage_transitioned[2] = false;
            stim_ending[2]        = false;
            *sync = true;
            schedule_alarm(2, inline_time_us_64() + 1);
        }
    }
    else if (pt.output_mode[0] < OUTPUT_MODE_FLOAT && sc[0].stimulus_state == STIMULUS_STATE_IDLE) {
        sc[0].pt_active = pt;
        init_stim_channel(sc, 0);
 
        adc_write_blocking(sc[0].pio_spi, 0x8010u | ((uint16_t)pt.output_mode[0] << 10));
 
        pio_spi_select_dac(sc[0].pio_spi);
        dac_write_output(sc[0].pio_spi, pt.stage_amplitude[0][0]);
        pio_spi_wait_done(sc[0].pio_spi);
        pio_spi_deselect_dac(sc[0].pio_spi);
 
        set_output_mode(0, pt.output_mode[0]);
        schedule_alarm(0, inline_time_us_64() + 1);
    }
    else if (pt.output_mode[1] < OUTPUT_MODE_FLOAT && sc[1].stimulus_state == STIMULUS_STATE_IDLE) {
        sc[1].pt_active = pt;
        init_stim_channel(sc, 1);
 
        adc_write_blocking(sc[1].pio_spi, 0x8010u | ((uint16_t)pt.output_mode[1] << 10));
 
        pio_spi_select_dac(sc[1].pio_spi);
        dac_write_output(sc[1].pio_spi, pt.stage_amplitude[1][0]);
        pio_spi_wait_done(sc[1].pio_spi);
        pio_spi_deselect_dac(sc[1].pio_spi);
 
        set_output_mode(1, pt.output_mode[1]);
        schedule_alarm(1, inline_time_us_64() + 1);
    }
    clear_doorbell(sc);
}

static inline bool process_trigger(stimulus_context_t *sc, bool *sync) {
    bool stim_start = false;
    if (stim_start = (bool)(io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_A >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1)))
        initate_stim_sm(sc, sync, sc[0].pt_trigger);
    else if (stim_start = (bool)(io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_B >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1))) 
        initate_stim_sm(sc, sync, sc[1].pt_trigger);
    return stim_start;
}

static inline bool process_stimulus_command_queue(stimulus_context_t *sc, bool *sync) {
    pulsetrain_t pt;
    if (queue_try_remove(&q_stimulus_cmd, &pt))
        initate_stim_sm(sc, sync, pt);
}

static inline void clear_trigger(uint8_t channel_io_pin) {
    io_bank0_hw->intr[channel_io_pin >> 3] = RISING_EDGE_INTERRUPT_BIT(channel_io_pin, 1);
}

static inline void flush_stimulus_command_queue(stimulus_context_t *sc) {
    pulsetrain_t dummy;
    while (queue_try_remove(&q_stimulus_cmd, &dummy));
}

static inline void process_manual_cmd_queue(stimulus_context_t *sc) {
    manual_cmd_t cmd;
    while (queue_try_remove(&q_manual_cmd, &cmd)) {
        if (cmd.type == MANUAL_CMD_DAC_SET) {
            pio_spi_select_dac(sc[cmd.ch].pio_spi);
            dac_write_output(sc[cmd.ch].pio_spi, cmd.dac_code);
            pio_spi_wait_done(sc[cmd.ch].pio_spi);
            pio_spi_deselect_dac(sc[cmd.ch].pio_spi);
            dac_latch(1LL << sc[cmd.ch].nldac_pin);
        } 
        else {
            const uint16_t adc_cfg = 0x8010u | ((uint16_t)cmd.line << 10);
            adc_write_blocking(sc[cmd.ch].pio_spi, adc_cfg);
            int16_t val = adc_read_blocking(sc[cmd.ch].pio_spi);
            queue_add_blocking(&q_adc_result, &val);
        }
    }
}

static inline void advance_stim(stimulus_context_t *sc) {
 
    if (!pio_spi_is_done(sc->pio_spi)) return;
 
    switch (sc->stimulus_state) {
 
        case STIMULUS_STATE_IDLE:
            break;
 
        case STIMULUS_STATE_ACTIVE:
            if (stage_transitioned[sc->channel]) {
                stage_transitioned[sc->channel] = false;
                if (stim_ending[sc->channel]) {
                    stim_ending[sc->channel] = false;
                    sc->stimulus_state = STIMULUS_STATE_END;
                    break;
                }
                uint8_t preload_stage = (sc->stage_counter + 1 < sc->pt_active.n_stages)
                                        ? sc->stage_counter + 1 : 0;
                pio_spi_select_dac(sc->pio_spi);
                dac_write_output(sc->pio_spi, sc->pt_active.stage_amplitude[sc->channel][preload_stage]);
                sc->adc_read_initiated  = false;
                sc->adc_read_terminated = false;
            }
 
            else if (!sc->adc_read_initiated && !sc->adc_read_terminated) {
                uint64_t elapsed = inline_time_us_64() - transition_tick[sc->channel];
                if (elapsed >= DAC_SETTLE_US) {
                    pio_spi_deselect_dac(sc->pio_spi);
                    pio_spi_select_adc(sc->pio_spi);
                    adc_read(sc->pio_spi);
                    sc->adc_read_initiated = true;
                }
            }
 
            else if (sc->adc_read_initiated && !sc->adc_read_terminated) {
                int16_t adc_value;
                adc_get_value(sc->pio_spi, &adc_value);
                pio_spi_deselect_adc(sc->pio_spi);
                sc->sr.measured_amplitudes[sc->stage_counter] += adc_value;
                sc->sr.delivered_stages[sc->stage_counter]++;
                uint64_t dur    = sc->pt_active.stage_duration[sc->stage_counter];
                uint64_t target = transition_tick[sc->channel] + dur;  // base on intended tick, not real
                uint64_t now    = inline_time_us_64();
                if (now >= target) target = now + DAC_SETTLE_US;
                schedule_alarm(sc->channel, target);
                if (++sc->stage_counter >= sc->pt_active.n_stages) {
                    sc->stage_counter = 0;
                    sc->pulse_counter++;
                }
                if (!(sc->stage_counter || sc->pulse_counter < sc->pt_active.n_pulses))
                    stim_ending[sc->channel] = true;
                sc->adc_read_terminated = true;
            }
            break;
 
        case STIMULUS_STATE_END:
            queue_try_add(&q_stim_result, &sc->sr);
            clear_trigger(sc->channel_io_pin);
            flush_stimulus_command_queue(sc);
            sc->stage_counter       = 0;
            sc->pulse_counter       = 0;
            sc->adc_read_initiated  = false;
            sc->adc_read_terminated = false;
            sc->sr = sr_reset[sc->channel];
            sc->stimulus_state = STIMULUS_STATE_IDLE;
            break;
    }
}

static inline void advance_stim_synced(stimulus_context_t *sc, bool *sync) {
 
    if (!(pio_spi_is_done(sc[0].pio_spi) && pio_spi_is_done(sc[1].pio_spi))) return;
 
    switch (sc[0].stimulus_state) {
 
        case STIMULUS_STATE_IDLE:
            break;
 
        case STIMULUS_STATE_ACTIVE:
 
            if (stage_transitioned[2]) {
                stage_transitioned[2] = false;
 
                if (stim_ending[2]) {
                    stim_ending[2] = false;
                    sc[0].stimulus_state = STIMULUS_STATE_END;
                    sc[1].stimulus_state = STIMULUS_STATE_END;
                    break;
                }
 
                for (int ch = 0; ch < 2; ch++) {
                    uint8_t preload_stage = (sc[ch].stage_counter + 1 < sc[ch].pt_active.n_stages)
                                           ? sc[ch].stage_counter + 1 : 0;
                    pio_spi_select_dac(sc[ch].pio_spi);
                    dac_write_output(sc[ch].pio_spi, sc[ch].pt_active.stage_amplitude[ch][preload_stage]);
                    sc[ch].adc_read_initiated  = false;
                    sc[ch].adc_read_terminated = false;
                }
            }
 
            else if (!sc[0].adc_read_initiated && !sc[0].adc_read_terminated) {
                uint64_t elapsed = inline_time_us_64() - transition_tick[2];
                if (elapsed >= DAC_SETTLE_US) {
                    for (int ch = 0; ch < 2; ch++) {
                        pio_spi_deselect_dac(sc[ch].pio_spi);
                        pio_spi_select_adc(sc[ch].pio_spi);
                        adc_read(sc[ch].pio_spi);
                        sc[ch].adc_read_initiated = true;
                    }
                }
            }
 
            else if (sc[0].adc_read_initiated && !sc[0].adc_read_terminated) {
                for (int ch = 0; ch < 2; ch++) {
                    int16_t adc_value;
                    adc_get_value(sc[ch].pio_spi, &adc_value);
                    pio_spi_deselect_adc(sc[ch].pio_spi);
                    sc[ch].sr.measured_amplitudes[sc[ch].stage_counter] += adc_value;
                    sc[ch].sr.delivered_stages[sc[ch].stage_counter]++;
                }
 
                uint64_t dur    = sc[0].pt_active.stage_duration[sc[0].stage_counter];
                uint64_t target = transition_tick[2] + dur;  // base on intended tick, not real
                uint64_t now    = inline_time_us_64();
                if (now >= target) target = now + DAC_SETTLE_US;
                schedule_alarm(2, target);
 
                sc[0].stage_counter++;
                sc[1].stage_counter++;
                if (sc[0].stage_counter >= sc[0].pt_active.n_stages) {
                    sc[0].stage_counter = 0;
                    sc[1].stage_counter = 0;
                    sc[0].pulse_counter++;
                    sc[1].pulse_counter++;
                }
 
                if (!(sc[0].stage_counter || sc[0].pulse_counter < sc[0].pt_active.n_pulses))
                    stim_ending[2] = true;
 
                for (int ch = 0; ch < 2; ch++)
                    sc[ch].adc_read_terminated = true;
            }
            break;
 
        case STIMULUS_STATE_END:
            for (int ch = 0; ch < 2; ch++) {
                queue_try_add(&q_stim_result, &sc[ch].sr);
                clear_trigger(sc[ch].channel_io_pin);
                sc[ch].stage_counter       = 0;
                sc[ch].pulse_counter       = 0;
                sc[ch].adc_read_initiated  = false;
                sc[ch].adc_read_terminated = false;
                sc[ch].sr                  = sr_reset[ch];
                sc[ch].stimulus_state      = STIMULUS_STATE_IDLE;
            }
            flush_stimulus_command_queue(sc);
            *sync = false;
            break;
    }
}

static inline void process_stimulus_trigger_queue(stimulus_context_t *sc) {
    while (queue_try_remove(&q_stimulus_trigger[0], &sc[0].pt_trigger));
    while (queue_try_remove(&q_stimulus_trigger[1], &sc[1].pt_trigger));
}

static inline void check_for_stim_start_signal(stimulus_context_t *sc, bool *sync) {
    if (process_trigger(sc, sync));
    else process_stimulus_command_queue(sc, sync);
}

void __time_critical_func(main_core1)(void) {

    pio_spi_t pio_spi[2] = {
        {   .pio = pio0, .base = 16, .sm_active = false,
            .miso = SPI_MISO_A, .mosi = SPI_MOSI_A, .sck = SPI_SCK_A,
            .cs_dac = CSA_A, .cs_adc = CSB_A
        },
        {   .pio = pio1, .base = 0,  .sm_active = false,
            .miso = SPI_MISO_B, .mosi = SPI_MOSI_B, .sck = SPI_SCK_B,
            .cs_dac = CSA_B, .cs_adc = CSB_B
        },
    };

    stimulus_context_t sc[2] = {
        {   .channel = 0, .pio_spi = &pio_spi[0], 
            .nldac_pin = NLDAC_A, .led_pin = LED_A,
            .channel_io_pin = CHANNEL_IO_A,
            .stimulus_state = STIMULUS_STATE_IDLE,
            .sr = { 0 }
        },
        {   .channel = 1, .pio_spi = &pio_spi[1], 
            .nldac_pin = NLDAC_B, .led_pin = LED_B,
            .channel_io_pin = CHANNEL_IO_B,
            .stimulus_state = STIMULUS_STATE_IDLE,
            .sr = { 0 }
        },
    };

    pio_spi_init(sc[0].pio_spi);
    pio_spi_init(sc[1].pio_spi);
    dac_init(sc[0].pio_spi);
    dac_init(sc[1].pio_spi);

    irq_set_exclusive_handler(TIMER0_IRQ_0, isr_ch0); 
    irq_set_exclusive_handler(TIMER0_IRQ_1, isr_ch1);
    irq_set_exclusive_handler(TIMER0_IRQ_2, isr_sync);
    hw_set_bits(&timer0_hw->inte, 0b111);
    irq_set_enabled(TIMER0_IRQ_0, true);
    irq_set_enabled(TIMER0_IRQ_1, true);
    irq_set_enabled(TIMER0_IRQ_2, true);

    bool sync = false;
    
    // Confirm to core0 that core1 is ready
    multicore_fifo_push_blocking(0xDEADBEEF);

    while (true) {

        if (sc[0].stimulus_state == STIMULUS_STATE_IDLE && sc[1].stimulus_state == STIMULUS_STATE_IDLE) {
            process_stimulus_trigger_queue(sc);
            process_offsets_queue(sc);
            process_manual_cmd_queue(sc);
            // reset timer here, timer interrupts are only 32 bit?
        }

        check_for_stim_start_signal(sc, &sync);

        if (sync)
            advance_stim_synced(sc, &sync);
        else {
            advance_stim(&sc[0]);
            advance_stim(&sc[1]);
        }

        if (sc[0].stimulus_state != STIMULUS_STATE_IDLE) 
            check_for_stim_cancel_signal(&sc[0], &sync);
        if (sc[1].stimulus_state != STIMULUS_STATE_IDLE)
            check_for_stim_cancel_signal(&sc[1], &sync);
    }
}
#include <hardware/timer.h>
#include <float.h>
#include <pico/multicore.h>
#include <math.h>
#include <string.h>

#include "core1.h"
#include "pio_spi.h"
#include "stimjim_context.h"

#define SAMPLES 100

typedef enum {
    STIMULUS_STATE_IDLE,
    STIMULUS_STATE_ACTIVE_TRANSITION,
    STIMULUS_STATE_ACTIVE_ADC_INITIATE,
    STIMULUS_STATE_ACTIVE_ADC_READ,
    STIMULUS_STATE_END
} stimulus_state_t;

typedef struct {
    // state machine
    stimulus_state_t stimulus_state;
    uint32_t stage_counter;
    uint32_t pulse_counter;
    uint32_t intended_tick;
    const pulsetrain_t *pt_active;
    pulsetrain_t pt_trigger; // pulse train set to deliver on trigger
    pulsetrain_t pt_cmd; // pulse train set to deliver on U/T command
    stim_result_t sr;
    // hardware peripherals
    pio_spi_t *pio_spi;
    // pin constants
    const bool channel;
    const uint8_t channel_io_pin;
    const uint8_t led_pin;
    const uint8_t nldac_pin;
} stimulus_context_t;


// global variables
static volatile bool stage_transitioned[2] = { false };
static volatile bool stim_ending[2] = { false };
static volatile int8_t first_channel_to_trigger = -1;

__always_inline static inline void adc_write_blocking(pio_spi_t *pio_spi, const uint16_t config) {
    pio_spi_select_adc(pio_spi);
    adc_write(pio_spi, config);
    pio_spi_wait_done(pio_spi);
    pio_spi_deselect_adc(pio_spi);
}

__always_inline static inline int16_t adc_read_blocking(pio_spi_t *pio_spi) {
    int16_t val;
    pio_spi_select_adc(pio_spi);
    adc_read(pio_spi);
    pio_spi_wait_done(pio_spi);
    adc_get_value(pio_spi, &val);
    pio_spi_deselect_adc(pio_spi);
    sleep_us(2);
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

static void measure_offsets(stimulus_context_t *sc, offsets_t *offsets, const offsets_tx_t offsets_calibration) {
    if (offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_ADC) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(sc[ch].channel, OUTPUT_MODE_GND);
            adc_write_blocking(sc[ch].pio_spi, 0x8010u);
            float acc = 0.0f;
            for (uint8_t i = 0; i < SAMPLES; i++)
                acc += (float)adc_read_blocking(sc[ch].pio_spi);
            offsets[ch].adc = acc / (float)SAMPLES;
        }
    }
    else {
        offsets[0].adc = offsets_calibration.adc[0];
        offsets[1].adc = offsets_calibration.adc[1];
    }

    const int8_t sweep_range = 50;
    gpio_put(sc[0].nldac_pin, false);
    gpio_put(sc[1].nldac_pin, false);

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
        const uint16_t adc_cfg = 0x8010u | ((uint16_t)line << 10);
        for (uint8_t ch = 0; ch < 2; ch++) {
            adc_write_blocking(sc[ch].pio_spi, adc_cfg);
            set_output_mode(sc[ch].channel, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
            int8_t min_i = -sweep_range;
            float min = FLT_MAX;
            for (int8_t i = -sweep_range; i <= sweep_range; i++) {
                pio_spi_select_dac(sc[ch].pio_spi);
                dac_write_output(sc[ch].pio_spi, i);
                pio_spi_wait_done(sc[ch].pio_spi);
                pio_spi_deselect_dac(sc[ch].pio_spi);
                sleep_us(10);
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

// inline version of pico SDK function to reduce pulse train jitter
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

// inline version of pico SDK function to reduce pulse train jitter
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

__always_inline static inline void process_offsets_queue(stimulus_context_t *sc) {
    offsets_tx_t offsets_calibration;
    while (inline_queue_try_remove(&q_offsets_tx, &offsets_calibration)) {
        offsets_t offsets[2];
        measure_offsets(sc, offsets, offsets_calibration);
        queue_add_blocking(&q_offsets_rx, offsets);
    }
}

__always_inline static inline void dac_latch(uint64_t nldac_mask) {
    gpio_clr_mask64(nldac_mask);
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    gpio_set_mask64(nldac_mask);
}

__isr static void __time_critical_func(isr_trigger)(void) {
    if (io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_A >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1)) {
        io_bank0_hw->intr[CHANNEL_IO_A >> 3] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1);
        if (first_channel_to_trigger == -1)
            first_channel_to_trigger = 0;
    }
    if (io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_B >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1)) {
        io_bank0_hw->intr[CHANNEL_IO_B >> 3] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1);
        if (first_channel_to_trigger == -1)
            first_channel_to_trigger = 1;
    }
}

__always_inline static inline void schedule_alarm(uint8_t index, int64_t target_us) {
    if (target_us < 0) return;
    timer0_hw->intr = 1u << index;
    timer0_hw->alarm[index] = MAX((uint32_t)target_us, timer0_hw->timerawl + 1);
}

__always_inline static inline void isr_common(uint8_t index, uint64_t gpio_mask, uint64_t nldac_mask) {
    timer0_hw->intf &= ~(1u << index);
    timer0_hw->intr = 1u << index;
    uint32_t t = timer0_hw->alarm[index];
    bool ending;
    if (index > 1) {
        stage_transitioned[0] = stage_transitioned[1] = true;
        ending = stim_ending[0] && stim_ending[1];
    }
    else {
        stage_transitioned[index] = true;
        ending = stim_ending[index];
    }
    if (ending) {
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
}

__isr static void __time_critical_func(isr_ch0)(void) {
    isr_common(0, (1LL << LED_A) | (1LL << CHANNEL_IO_A), 1LL << NLDAC_A);
}

__isr static void __time_critical_func(isr_ch1)(void) {
    isr_common(1, (1LL << LED_B) | (1LL << CHANNEL_IO_B), 1LL << NLDAC_B);
}

__isr static void __time_critical_func(isr_sync)(void) {
    isr_common(2, (1LL << LED_A) | (1LL << CHANNEL_IO_A) | (1LL << LED_B) | (1LL << CHANNEL_IO_B), (1LL << NLDAC_A) | (1LL << NLDAC_B));
}

__always_inline static inline bool check_for_stim_cancel_signal(stimulus_context_t *sc, bool *sync) {
    if (!(sio_hw->doorbell_in_set & (1u << sc->channel))) return false;
    sio_hw->doorbell_in_clr = 1u << sc->channel;
    set_output_mode(sc->channel, OUTPUT_MODE_GND);
    gpio_put(sc->channel_io_pin, false);
    gpio_put(sc->led_pin, false);
    pio_spi_deselect_adc(sc->pio_spi);
    pio_spi_deselect_dac(sc->pio_spi);
    pio_spi_select_dac(sc->pio_spi);
    dac_write_output(sc->pio_spi, sc->pt_active->stage_amplitude[sc->channel][sc->pt_active->n_stages - 1]);
    while (!pio_spi_is_done(sc->pio_spi));
    pio_spi_deselect_dac(sc->pio_spi);
    dac_latch(1LL << sc->nldac_pin);
    sc->stimulus_state = STIMULUS_STATE_END;
    *sync = false;
    return true;
}

__always_inline static inline void reset_stim_sm(stimulus_context_t *sc, uint8_t ch) {
    sc[ch].sr.ch          = ch;
    sc[ch].sr.n_stages    = sc[ch].pt_active->n_stages;
    sc[ch].sr.output_mode = sc[ch].pt_active->output_mode[ch];
    for (uint8_t i = 0; i < sc[ch].pt_active->n_stages; i++) {
        sc[ch].sr.measured_amplitudes[i] = 0;
        sc[ch].sr.delivered_stages[i]    = 0;
    }
    sc[ch].stage_counter  = 0;
    sc[ch].pulse_counter  = 0;
    sc[ch].stimulus_state = STIMULUS_STATE_ACTIVE_TRANSITION;
}

__always_inline static inline void flush_stimulus_cmd_queue(stimulus_context_t *sc) {
    pulsetrain_t dummy;
    while (inline_queue_try_remove(&q_stimulus_cmd, &dummy));
}

__always_inline static inline void process_manual_cmd_queue(stimulus_context_t *sc) {
    manual_cmd_t cmd;
    while (inline_queue_try_remove(&q_manual_cmd, &cmd)) {
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

__always_inline static inline int64_t advance_stim(stimulus_context_t *sc) {

    if (!pio_spi_is_done(sc->pio_spi)) return -1;

    switch (sc->stimulus_state) {

        case STIMULUS_STATE_IDLE:
            break;

        case STIMULUS_STATE_ACTIVE_TRANSITION:
            if (stage_transitioned[sc->channel]) {
                stage_transitioned[sc->channel] = false;
                if (stim_ending[sc->channel]) {
                    stim_ending[sc->channel] = false;
                    sc->stimulus_state = STIMULUS_STATE_END;
                    break;
                }
                uint8_t preload_stage = (sc->stage_counter + 1 < sc->pt_active->n_stages)
                                        ? sc->stage_counter + 1 : 0;
                pio_spi_deselect_adc(sc->pio_spi);
                pio_spi_select_dac(sc->pio_spi);
                dac_write_output(sc->pio_spi, sc->pt_active->stage_amplitude[sc->channel][preload_stage]);
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_ADC_INITIATE;
            }
            break;

        case STIMULUS_STATE_ACTIVE_ADC_INITIATE:
            if (timer0_hw->timerawl - sc->intended_tick >= DAC_SETTLE_US) {
                pio_spi_deselect_dac(sc->pio_spi);
                pio_spi_select_adc(sc->pio_spi);
                adc_read(sc->pio_spi);
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_ADC_READ;
            }
            break;

        case STIMULUS_STATE_ACTIVE_ADC_READ:
            {
                int16_t adc_value;
                adc_get_value(sc->pio_spi, &adc_value);
                pio_spi_deselect_adc(sc->pio_spi);
                sc->sr.measured_amplitudes[sc->stage_counter] += adc_value;
                sc->sr.delivered_stages[sc->stage_counter]++;
                sc->intended_tick += sc->pt_active->stage_duration[sc->stage_counter];
                if (++sc->stage_counter >= sc->pt_active->n_stages) {
                    sc->stage_counter = 0;
                    sc->pulse_counter++;
                }
                if (!(sc->stage_counter || sc->pulse_counter < sc->pt_active->n_pulses))
                    stim_ending[sc->channel] = true;
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_TRANSITION;
                return sc->intended_tick;
            }

        case STIMULUS_STATE_END:
            inline_queue_try_add(&q_stim_result, &sc->sr);
            first_channel_to_trigger = -1;
            flush_stimulus_cmd_queue(sc);
            sc->stimulus_state = STIMULUS_STATE_IDLE;
            break;
    }
    return -1;
}

__always_inline static inline void initiate_stim_ch(stimulus_context_t *sc, uint8_t ch, const pulsetrain_t *pt) {
    pio_spi_select_dac(sc[ch].pio_spi);
    dac_write_output(sc[ch].pio_spi, pt->stage_amplitude[ch][0]);
    sc[ch].pt_active = pt;
    stage_transitioned[ch] = false;
    stim_ending[ch] = false;
    pio_spi_wait_done(sc[ch].pio_spi);
    pio_spi_deselect_dac(sc[ch].pio_spi);

    set_output_mode(ch, pt->output_mode[ch]);
    timer0_hw->alarm[ch] = timer0_hw->timerawl + 1;
    sc[ch].intended_tick = timer0_hw->alarm[ch];

    pio_spi_select_adc(sc[ch].pio_spi);
    adc_write(sc[ch].pio_spi, 0x8010u | ((uint16_t)pt->output_mode[ch] << 10));
    reset_stim_sm(sc, ch);
}

__always_inline static inline void initiate_stim(stimulus_context_t *sc, bool *sync, int32_t *pending_target, const pulsetrain_t *pt) {
    *sync = false;
    if (pt->output_mode[0] < OUTPUT_MODE_FLOAT && pt->output_mode[1] < OUTPUT_MODE_FLOAT) {
        if (sc[0].stimulus_state == STIMULUS_STATE_IDLE && sc[1].stimulus_state == STIMULUS_STATE_IDLE) {

            pio_spi_select_dac(sc[0].pio_spi);
            pio_spi_select_dac(sc[1].pio_spi);
            dac_write_output(sc[0].pio_spi, pt->stage_amplitude[0][0]);
            dac_write_output(sc[1].pio_spi, pt->stage_amplitude[1][0]);

            sc[0].pt_active = sc[1].pt_active = pt;
            stage_transitioned[0] = stage_transitioned[1] = false;
            stim_ending[0] = stim_ending[1] = false;
            *sync = true;
            pending_target[0] = pending_target[1] = -1;

            pio_spi_wait_done(sc[0].pio_spi);
            pio_spi_wait_done(sc[1].pio_spi);
            pio_spi_deselect_dac(sc[0].pio_spi);
            pio_spi_deselect_dac(sc[1].pio_spi);

            set_output_mode(0, pt->output_mode[0]);
            set_output_mode(1, pt->output_mode[1]);
            timer0_hw->alarm[2] = timer0_hw->timerawl + 1;
            sc[0].intended_tick = sc[1].intended_tick = timer0_hw->alarm[2];

            pio_spi_select_adc(sc[0].pio_spi);
            pio_spi_select_adc(sc[1].pio_spi);
            adc_write(sc[0].pio_spi, 0x8010u | ((uint16_t)pt->output_mode[0] << 10));
            adc_write(sc[1].pio_spi, 0x8010u | ((uint16_t)pt->output_mode[1] << 10));
            reset_stim_sm(sc, 0);
            reset_stim_sm(sc, 1);
        }
    }
    else if (pt->output_mode[0] < OUTPUT_MODE_FLOAT && sc[0].stimulus_state == STIMULUS_STATE_IDLE)
        initiate_stim_ch(sc, 0, pt);
    else if (pt->output_mode[1] < OUTPUT_MODE_FLOAT && sc[1].stimulus_state == STIMULUS_STATE_IDLE)
        initiate_stim_ch(sc, 1, pt);
    sio_hw->doorbell_in_clr = (1u << 0) | (1u << 1);
}

__always_inline static inline bool process_trigger(stimulus_context_t *sc, bool *sync, int32_t *pending_target) {
    if (first_channel_to_trigger > -1) {
        initiate_stim(sc, sync, pending_target, &sc[first_channel_to_trigger].pt_trigger);
        return true;
    }
    return false;
}

__always_inline static inline void process_stimulus_cmd_queue(stimulus_context_t *sc, bool *sync, int32_t *pending_target) {
    pulsetrain_t pt;
    if (inline_queue_try_remove(&q_stimulus_cmd, &pt)) {
        sc[0].pt_cmd = pt;
        initiate_stim(sc, sync, pending_target, &sc[0].pt_cmd);
    }
}

__always_inline static inline void process_stimulus_trigger_queue(stimulus_context_t *sc) {
    while (inline_queue_try_remove(&q_stimulus_trigger[0], &sc[0].pt_trigger));
    while (inline_queue_try_remove(&q_stimulus_trigger[1], &sc[1].pt_trigger));
}

void __time_critical_func(main_core1)(void) {

    static pio_spi_t pio_spi[2] = {
        { .pio = pio0, .base = 16, .sm_active = false,
          .miso = SPI_MISO_A, .mosi = SPI_MOSI_A, .sck = SPI_SCK_A,
          .cs_dac = CSA_A, .cs_adc = CSB_A },
        { .pio = pio1, .base = 0,  .sm_active = false,
          .miso = SPI_MISO_B, .mosi = SPI_MOSI_B, .sck = SPI_SCK_B,
          .cs_dac = CSA_B, .cs_adc = CSB_B },
    };

    static stimulus_context_t sc[2] = {
        { .channel = 0, .pio_spi = &pio_spi[0],
          .nldac_pin = NLDAC_A, .led_pin = LED_A, .channel_io_pin = CHANNEL_IO_A,
          .stimulus_state = STIMULUS_STATE_IDLE },
        { .channel = 1, .pio_spi = &pio_spi[1],
          .nldac_pin = NLDAC_B, .led_pin = LED_B, .channel_io_pin = CHANNEL_IO_B,
          .stimulus_state = STIMULUS_STATE_IDLE },
    };
    
    pio_spi_init(sc[0].pio_spi);
    pio_spi_init(sc[1].pio_spi);
    dac_init(sc[0].pio_spi);
    dac_init(sc[1].pio_spi);

    adc_write_blocking(sc[0].pio_spi, 0x8010u);
    adc_write_blocking(sc[1].pio_spi, 0x8010u);
    sleep_ms(1); // wait for ADCs to wake up, 500us minimum

    irq_set_exclusive_handler(TIMER0_IRQ_0, isr_ch0);
    irq_set_exclusive_handler(TIMER0_IRQ_1, isr_ch1);
    irq_set_exclusive_handler(TIMER0_IRQ_2, isr_sync);
    irq_set_exclusive_handler(IO_IRQ_BANK0, isr_trigger);
    hw_set_bits(&timer0_hw->inte, 0b111);
    irq_set_enabled(TIMER0_IRQ_0, true);
    irq_set_enabled(TIMER0_IRQ_1, true);
    irq_set_enabled(TIMER0_IRQ_2, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    bool sync = false;
    int32_t pending_target[2] = { -1, -1 };

    // confirm to core0 that core1 is ready
    multicore_fifo_push_blocking(0xDEADBEEF);

    while (true) {

        if (sc[0].stimulus_state != STIMULUS_STATE_IDLE || sc[1].stimulus_state != STIMULUS_STATE_IDLE) {
            if (sync) {
                int32_t t[2] = { advance_stim(&sc[0]), advance_stim(&sc[1]) };
                if (t[0] >= 0) pending_target[0] = t[0];
                if (t[1] >= 0) pending_target[1] = t[1];
                if (pending_target[0] >= 0 && pending_target[1] >= 0) {
                    schedule_alarm(2, MIN(pending_target[0], pending_target[1]));
                    pending_target[0] = pending_target[1] = -1;
                }
            }
            else {
                schedule_alarm(0, advance_stim(&sc[0]));
                schedule_alarm(1, advance_stim(&sc[1]));
            }
        }

        if (sc[0].stimulus_state != STIMULUS_STATE_IDLE)
            check_for_stim_cancel_signal(&sc[0], &sync);
        if (sc[1].stimulus_state != STIMULUS_STATE_IDLE)
            check_for_stim_cancel_signal(&sc[1], &sync);

        if (!process_trigger(sc, &sync, pending_target)) {
            if ((sio_hw->doorbell_in_set & (1 << 2)) && sc[0].stimulus_state == STIMULUS_STATE_IDLE && sc[1].stimulus_state == STIMULUS_STATE_IDLE) {
                sio_hw->doorbell_in_clr = 1 << 2;
                process_stimulus_trigger_queue(sc);
                process_offsets_queue(sc);
                process_manual_cmd_queue(sc);
            }
            process_stimulus_cmd_queue(sc, &sync, pending_target);
        }
    }
}
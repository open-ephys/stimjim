#include <hardware/timer.h>
#include <float.h>
#include <pico/multicore.h>
#include <math.h>
#include <string.h>

#include "core1.h"
#include "pio_spi.h"
#include "stimjim_context.h"

#define SAMPLES 100
#define ADC_BASE_CONFIG 0x8010u
#define ADC_CURRENT_LINE 0x0400u

// DAC config frames: high byte = register addr, low 16 bits = value
#define DAC_REG_RANGE     0x08u
#define DAC_RANGE_PM10V   4u
#define DAC_REG_POWER     0x10u
#define DAC_POWER_ON      1u
#define DAC_CFG(reg, val) (((uint32_t)(reg) << 16) | (uint32_t)(val))

static const uint64_t gpio_mask[2] = {
    (1LL << LED_A) | (1LL << CHANNEL_IO_A),
    (1LL << LED_B) | (1LL << CHANNEL_IO_B),
};
static const uint64_t nldac_mask[2] = { 1LL << NLDAC_A, 1LL << NLDAC_B };

typedef enum {
    STIMULUS_STATE_IDLE,
    STIMULUS_STATE_ACTIVE_TRANSITIONED,
    STIMULUS_STATE_ACTIVE_DAC_SETTLING,
    STIMULUS_STATE_ACTIVE_ADC_READ_INITIATED,
    STIMULUS_STATE_END
} stimulus_state_t;

typedef struct {
    // state machine
    stimulus_state_t stimulus_state;
    uint32_t stage_counter;
    uint32_t pulse_counter;
    uint32_t next_alarm_us;
    pulsetrain_t pt_trigger; // pulse train set to deliver on trigger
    pulsetrain_t pt_active; // copy of pulse train being actively delivered
    stimulus_result_t sr; // struct passed to core0 for printing a report
    pio_spi_t *pio_spi; // struct for configuring pio spi
    // pin constants
    const uint8_t channel;
    const uint8_t channel_io_pin;
    const uint8_t led_pin;
    const uint8_t nldac_pin;
} stimulus_context_t;


// global variables
static volatile bool stage_transitioned[2] = { false };
static volatile bool stimulus_ending[2] = { false };
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
static volatile bool sync_active = false;

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

static void dac_write_config_blocking(pio_spi_t *p, uint32_t cfg) {
    pio_spi_select_dac(p);
    dac_write_config(p, cfg);
    pio_spi_wait_done(p);
    pio_spi_deselect_dac(p);
}

static void dac_init(pio_spi_t *p) {
    dac_write_config_blocking(p, DAC_CFG(DAC_REG_RANGE, DAC_RANGE_PM10V));
    dac_write_config_blocking(p, DAC_CFG(DAC_REG_POWER, DAC_POWER_ON));
}

__always_inline static inline void dac_latch(const uint64_t nldac_mask) {
    gpio_clr_mask64(nldac_mask);
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    gpio_set_mask64(nldac_mask);
}

static void measure_offsets(stimulus_context_t *sc, offsets_t *offsets, const offsets_tx_t offsets_calibration) {
    if (offsets_calibration.offset_tx_type & OFFSETS_TX_CALIBRATE_ADC) {
        for (uint8_t ch = 0; ch < 2; ch++) {
            set_output_mode(sc[ch].channel, OUTPUT_MODE_GND);
            adc_write_blocking(sc[ch].pio_spi, ADC_BASE_CONFIG);
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
    for (uint8_t ch = 0; ch < 2; ch++) {
        pio_spi_select_dac(sc[ch].pio_spi);
        dac_write_output(sc[ch].pio_spi, 0);
        pio_spi_wait_done(sc[ch].pio_spi);
        pio_spi_deselect_dac(sc[ch].pio_spi);
        dac_latch(nldac_mask[ch]);
    }
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
        const uint16_t adc_cfg = ADC_BASE_CONFIG | (line ? ADC_CURRENT_LINE : 0);
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

__always_inline static inline void schedule_latch(const uint8_t index, const uint32_t target_us) {
    timer0_hw->intr = 1u << index;
    timer0_hw->alarm[index] = MAX(target_us, timer0_hw->timerawl + 1);
}

__always_inline static inline void isr_stage_transition(const uint8_t ch) {
    timer0_hw->intf &= ~(1u << ch);
    timer0_hw->intr = 1u << ch;
    stage_transitioned[ch] = true;
    if (stimulus_ending[ch]) {
        set_output_mode(ch, OUTPUT_MODE_GND);
        gpio_clr_mask64(gpio_mask[ch]);
    }
    else {
        dac_latch(nldac_mask[ch]);
    }
}

__always_inline static inline void isr_stage_transition_sync(void) {
    timer0_hw->intf &= ~(1u << 2);
    timer0_hw->intr = 1u << 2;
    stage_transitioned[0] = stage_transitioned[1] = true;
    if (stimulus_ending[0] && stimulus_ending[1]) {
        set_output_mode(0, OUTPUT_MODE_GND);
        set_output_mode(1, OUTPUT_MODE_GND);
        gpio_clr_mask64(gpio_mask[0] | gpio_mask[1]);
    }
    else {
        dac_latch(nldac_mask[0] | nldac_mask[1]);
    }
}

__isr static void __time_critical_func(isr_ch0)(void) { isr_stage_transition(0); }
__isr static void __time_critical_func(isr_ch1)(void) { isr_stage_transition(1); }
__isr static void __time_critical_func(isr_sync)(void) { isr_stage_transition_sync(); }

__always_inline static inline void clear_cancel_stimulus_requests(void) {
    sio_hw->doorbell_in_clr = (1u << 0) | (1u << 1);
}

__always_inline static inline bool stimulus_cancel_requested(const stimulus_context_t *sc) {
    if (!(sio_hw->doorbell_in_set & (1u << sc->channel))) return false;
    sio_hw->doorbell_in_clr = 1u << sc->channel;
    return true;
}

__always_inline static inline void cancel_stimulus(stimulus_context_t *sc, const uint8_t ch) {
    timer0_hw->armed = (1u << ch) | (1u << 2);
    set_output_mode(ch, OUTPUT_MODE_GND);
    gpio_put(sc[ch].channel_io_pin, false);
    gpio_put(sc[ch].led_pin, false);
    pio_spi_deselect_dac(sc[ch].pio_spi);
    pio_spi_deselect_adc(sc[ch].pio_spi);
    pio_spi_select_dac(sc[ch].pio_spi);
    dac_write_output(sc[ch].pio_spi, sc[ch].pt_active.stage_amplitude[ch][sc[ch].pt_active.n_stages - 1]);
    while (!pio_spi_is_done(sc[ch].pio_spi));
    pio_spi_deselect_dac(sc[ch].pio_spi);
    dac_latch(nldac_mask[ch]);
    sc[ch].stimulus_state = STIMULUS_STATE_END;
    // If the other channel is still active (sync pair partially cancelled),
    // transition it from sync timing (alarm[2]) to independent timing.
    uint8_t other = ch ^ 1;
    bool other_active = sc[other].stimulus_state != STIMULUS_STATE_IDLE
                     && sc[other].stimulus_state != STIMULUS_STATE_END;
    if (other_active && !stage_transitioned[other])
        schedule_latch(other, sc[other].next_alarm_us);
}

__always_inline static inline bool try_cancel_stimulus(stimulus_context_t *sc) {
    bool cancelled = false;
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (sc[ch].stimulus_state != STIMULUS_STATE_IDLE && stimulus_cancel_requested(&sc[ch])) {
            cancel_stimulus(sc, ch);
            cancelled = true;
        }
    }
    return cancelled;
}

__always_inline static inline void reset_stimulus_sm(stimulus_context_t *sc) {
    sc->sr.ch = sc->channel;
    sc->sr.n_stages = sc->pt_active.n_stages;
    sc->sr.output_mode = sc->pt_active.output_mode[sc->channel];
    for (uint8_t i = 0; i < sc->pt_active.n_stages; i++) {
        sc->sr.measured_amplitudes[i] = 0;
        sc->sr.delivered_stages[i] = 0;
    }
    sc->stage_counter = 0;
    sc->pulse_counter = 0;
    sc->stimulus_state = STIMULUS_STATE_ACTIVE_TRANSITIONED;
}

__always_inline static inline void flush_stimulus_cmd_queue(void) {
    pulsetrain_t dummy;
    while (inline_queue_try_remove(&q_stimulus_cmd, &dummy));
}

__always_inline static inline void preload_trigger_dac(stimulus_context_t *sc) {
    uint8_t ch = sc->channel;
    if (!(sc->pt_trigger.output_mode[ch] & OUTPUT_MODE_ACTIVE) || sc->pt_trigger.n_pulses == 0)
        return;
    pio_spi_select_dac(sc->pio_spi);
    dac_write_output(sc->pio_spi, sc->pt_trigger.stage_amplitude[ch][0]);
    pio_spi_wait_done(sc->pio_spi);
}

__always_inline static inline void complete_stimulus(stimulus_context_t *sc) {
    inline_queue_try_add(&q_stimulus_result, &sc->sr);
    flush_stimulus_cmd_queue();
    sc->stimulus_state = STIMULUS_STATE_IDLE;
    preload_trigger_dac(sc);
}

__always_inline static inline void process_manual_cmd_queue(stimulus_context_t *sc) {
    manual_cmd_t cmd;
    while (inline_queue_try_remove(&q_manual_cmd, &cmd)) {
        if (cmd.type == MANUAL_CMD_DAC_SET) {
            pio_spi_select_dac(sc[cmd.ch].pio_spi);
            dac_write_output(sc[cmd.ch].pio_spi, cmd.dac_code);
            pio_spi_wait_done(sc[cmd.ch].pio_spi);
            pio_spi_deselect_dac(sc[cmd.ch].pio_spi);
            dac_latch(nldac_mask[cmd.ch]);
        }
        else {
            const uint16_t adc_cfg = ADC_BASE_CONFIG | (cmd.line ? ADC_CURRENT_LINE : 0);
            adc_write_blocking(sc[cmd.ch].pio_spi, adc_cfg);
            int16_t val = adc_read_blocking(sc[cmd.ch].pio_spi);
            queue_add_blocking(&q_adc_result, &val);
        }
    }
}

__always_inline static inline bool advance_stimulus(stimulus_context_t *sc) {

    if (!pio_spi_is_done(sc->pio_spi)) return false;

    switch (sc->stimulus_state) {

        case STIMULUS_STATE_IDLE:
            break;

        case STIMULUS_STATE_ACTIVE_TRANSITIONED:
            if (stage_transitioned[sc->channel]) {
                stage_transitioned[sc->channel] = false;
                if (stimulus_ending[sc->channel]) {
                    stimulus_ending[sc->channel] = false;
                    sc->stimulus_state = STIMULUS_STATE_END;
                    break;
                }
                // preload next stage's DAC value; wraps to 0 for next pulse's first stage
                uint8_t preload_stage = (sc->stage_counter + 1 < sc->pt_active.n_stages)
                                        ? sc->stage_counter + 1 : 0;
                pio_spi_deselect_adc(sc->pio_spi);
                pio_spi_select_dac(sc->pio_spi);
                dac_write_output(sc->pio_spi, sc->pt_active.stage_amplitude[sc->channel][preload_stage]);
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_DAC_SETTLING;
            }
            break;

        case STIMULUS_STATE_ACTIVE_DAC_SETTLING:
            if (timer0_hw->timerawl - sc->next_alarm_us >= DAC_SETTLE_US) {
                pio_spi_deselect_dac(sc->pio_spi);
                pio_spi_select_adc(sc->pio_spi);
                adc_read(sc->pio_spi);
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_ADC_READ_INITIATED;
            }
            break;

        case STIMULUS_STATE_ACTIVE_ADC_READ_INITIATED:
            {
                int16_t adc_value;
                adc_get_value(sc->pio_spi, &adc_value);
                pio_spi_deselect_adc(sc->pio_spi);
                sc->sr.measured_amplitudes[sc->stage_counter] += adc_value;
                sc->sr.delivered_stages[sc->stage_counter]++;
                sc->next_alarm_us += sc->pt_active.stage_duration[sc->stage_counter];
                if (++sc->stage_counter >= sc->pt_active.n_stages) {
                    sc->stage_counter = 0;
                    sc->pulse_counter++;
                }
                if (sc->stage_counter == 0 && sc->pulse_counter >= sc->pt_active.n_pulses)
                    stimulus_ending[sc->channel] = true;
                sc->stimulus_state = STIMULUS_STATE_ACTIVE_TRANSITIONED;
                return true;
            }

        case STIMULUS_STATE_END:
            complete_stimulus(sc);
            break;
    }
    return false;
}

__always_inline static inline void initiate_stimulus_single_ch(stimulus_context_t *sc, const pulsetrain_t *pt) {
    uint8_t ch = sc->channel;
    stimulus_ending[ch] = false;
    pio_spi_wait_done(sc->pio_spi);
    pio_spi_deselect_dac(sc->pio_spi);

    set_output_mode(ch, pt->output_mode[ch]);
    uint32_t tick = timer0_hw->timerawl;
    while (timer0_hw->timerawl == tick) { __asm__ volatile("nop"); }

    gpio_set_mask64(gpio_mask[ch]);
    dac_latch(nldac_mask[ch]);
    sc->next_alarm_us = timer0_hw->timerawl;
    stage_transitioned[ch] = true;

    sc->pt_active = *pt;
    pio_spi_select_adc(sc->pio_spi);
    adc_write(sc->pio_spi, ADC_BASE_CONFIG | ((pt->output_mode[ch] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0));
    reset_stimulus_sm(sc);
}

__always_inline static inline void initiate_stimulus_lockstep(stimulus_context_t *sc, const pulsetrain_t *pt) {
    stimulus_ending[0] = stimulus_ending[1] = false;

    pio_spi_wait_done(sc[0].pio_spi);
    pio_spi_wait_done(sc[1].pio_spi);
    pio_spi_deselect_dac(sc[0].pio_spi);
    pio_spi_deselect_dac(sc[1].pio_spi);

    set_output_mode(0, pt->output_mode[0]);
    set_output_mode(1, pt->output_mode[1]);
    uint32_t tick = timer0_hw->timerawl;
    while (timer0_hw->timerawl == tick) { __asm__ volatile("nop"); }

    gpio_set_mask64(gpio_mask[0] | gpio_mask[1]);
    dac_latch(nldac_mask[0] | nldac_mask[1]);
    sc[0].next_alarm_us = sc[1].next_alarm_us = timer0_hw->timerawl;
    stage_transitioned[0] = stage_transitioned[1] = true;

    sc[0].pt_active = sc[1].pt_active = *pt;
    pio_spi_select_adc(sc[0].pio_spi);
    pio_spi_select_adc(sc[1].pio_spi);
    adc_write(sc[0].pio_spi, ADC_BASE_CONFIG | ((pt->output_mode[0] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0));
    adc_write(sc[1].pio_spi, ADC_BASE_CONFIG | ((pt->output_mode[1] & OUTPUT_MODE_CURRENT) ? ADC_CURRENT_LINE : 0));
    reset_stimulus_sm(&sc[0]);
    reset_stimulus_sm(&sc[1]);
}

__always_inline static inline bool both_channels_idle(const stimulus_context_t *sc) {
    return sc[0].stimulus_state == STIMULUS_STATE_IDLE && sc[1].stimulus_state == STIMULUS_STATE_IDLE;
}

// returns -1 (no stim), 2 (lockstep), 0 or 1 (single channel)
__always_inline static inline int8_t stimulus_target(const stimulus_context_t *sc, const pulsetrain_t *pt) {
    if ((pt->output_mode[0] & OUTPUT_MODE_ACTIVE) && (pt->output_mode[1] & OUTPUT_MODE_ACTIVE))
        return both_channels_idle(sc) ? 2 : -1;
    if ((pt->output_mode[0] & OUTPUT_MODE_ACTIVE) && sc[0].stimulus_state == STIMULUS_STATE_IDLE)
        return 0;
    if ((pt->output_mode[1] & OUTPUT_MODE_ACTIVE) && sc[1].stimulus_state == STIMULUS_STATE_IDLE)
        return 1;
    return -1;
}

__always_inline static inline void initiate_stimulus(stimulus_context_t *sc, bool *sync, const pulsetrain_t *pt, const int8_t target) {
    clear_cancel_stimulus_requests();
    if (target == 2) {
        initiate_stimulus_lockstep(sc, pt);
        *sync = true;
    }
    else {
        initiate_stimulus_single_ch(&sc[target], pt);
        *sync = false;
    }
}

__isr static void __time_critical_func(isr_trigger)(void) {
    int8_t trig_ch;
    if (io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_A >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1)) {
        io_bank0_hw->intr[CHANNEL_IO_A >> 3] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, 1);
        trig_ch = 0;
    }
    else if (io_bank0_hw->proc1_irq_ctrl.ints[CHANNEL_IO_B >> 3] & RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1)) {
        io_bank0_hw->intr[CHANNEL_IO_B >> 3] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, 1);
        trig_ch = 1;
    }
    else return;

    const pulsetrain_t *pt = &sc[trig_ch].pt_trigger;
    int8_t target = stimulus_target(sc, pt);
    if (target < 0) return;  // silent reject: target channel(s) busy or pt empty

    bool sync_local;
    initiate_stimulus(sc, &sync_local, pt, target);
    sync_active = sync_local;
}

__always_inline static inline void preload_cmd_dac(stimulus_context_t *sc, const pulsetrain_t *pt, int8_t target) {
    if (target == 2) {
        pio_spi_select_dac(sc[0].pio_spi);
        pio_spi_select_dac(sc[1].pio_spi);
        dac_write_output(sc[0].pio_spi, pt->stage_amplitude[0][0]);
        dac_write_output(sc[1].pio_spi, pt->stage_amplitude[1][0]);
    }
    else {
        pio_spi_select_dac(sc[target].pio_spi);
        dac_write_output(sc[target].pio_spi, pt->stage_amplitude[target][0]);
    }
}

__always_inline static inline bool try_initiate_cmd(stimulus_context_t *sc) {
    static pulsetrain_t pt_cmd;
    if (!inline_queue_try_remove(&q_stimulus_cmd, &pt_cmd)) return false;

    uint32_t save = save_and_disable_interrupts();
    int8_t target = stimulus_target(sc, &pt_cmd);
    if (target < 0) {
        restore_interrupts(save);
        stimulus_result_t sr = { .rejected = true };
        inline_queue_try_add(&q_stimulus_result, &sr);
        return false;
    }
    preload_cmd_dac(sc, &pt_cmd, target);
    bool sync_local;
    initiate_stimulus(sc, &sync_local, &pt_cmd, target);
    sync_active = sync_local;
    restore_interrupts(save);
    return true;
}

__always_inline static inline void process_stimulus_trigger_queue(stimulus_context_t *sc) {
    bool updated[2] = { false, false };
    while (inline_queue_try_remove(&q_stimulus_trigger[0], &sc[0].pt_trigger)) updated[0] = true;
    while (inline_queue_try_remove(&q_stimulus_trigger[1], &sc[1].pt_trigger)) updated[1] = true;
    if (updated[0]) preload_trigger_dac(&sc[0]);
    if (updated[1]) preload_trigger_dac(&sc[1]);
}

void __time_critical_func(main_core1)(void) {
    
    pio_spi_init(sc[0].pio_spi);
    pio_spi_init(sc[1].pio_spi);
    dac_init(sc[0].pio_spi);
    dac_init(sc[1].pio_spi);

    adc_write_blocking(sc[0].pio_spi, ADC_BASE_CONFIG);
    adc_write_blocking(sc[1].pio_spi, ADC_BASE_CONFIG);
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

    bool sync_ready[2] = { false, false };

    // confirm to core0 that core1 is ready
    multicore_fifo_push_blocking(CORE_HANDSHAKE_MESSAGE);

    while (true) {

        try_initiate_cmd(sc);

        if (sync_active) {
            if (advance_stimulus(&sc[0])) sync_ready[0] = true;
            if (advance_stimulus(&sc[1])) sync_ready[1] = true;
            if (sync_ready[0] && sync_ready[1]) {
                schedule_latch(2, MIN(sc[0].next_alarm_us, sc[1].next_alarm_us));
                sync_ready[0] = sync_ready[1] = false;
            }
        }
        else {
            if (advance_stimulus(&sc[0])) schedule_latch(0, sc[0].next_alarm_us);
            if (advance_stimulus(&sc[1])) schedule_latch(1, sc[1].next_alarm_us);
        }

        if (try_cancel_stimulus(sc))
            sync_active = false;

        if (both_channels_idle(sc)) {
            process_offsets_queue(sc);
            process_manual_cmd_queue(sc);
            process_stimulus_trigger_queue(sc);
        }
    }
}
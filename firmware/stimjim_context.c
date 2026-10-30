#include <math.h>
#include <string.h>
#include "stimjim_context.h"
#include "core1.h"

void __not_in_flash_func(set_output_mode)(uint8_t ch, output_mode_t mode) {
    static const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
    static const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
    gpio_put(oe0_pin[ch], mode & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND));
    gpio_put(oe1_pin[ch], mode & (OUTPUT_MODE_FLOAT | OUTPUT_MODE_GND));
}

// convert mV or uA to dac_code
static inline int16_t dac_code(const offsets_t off, const bool current, const int32_t amplitude) {
    int32_t code = lroundf(amplitude / (current ? MICROAMPS_PER_DAC : MILLIVOLTS_PER_DAC)) + (current ? off.current : off.voltage);
    if (code < INT16_MIN) code = INT16_MIN;
    if (code > INT16_MAX) code = INT16_MAX;
    return (int16_t)code;
}

// for every stage in pulse train, convert mV or uA to dac code
pulsetrain_t stimjim_ctx_convert_pt(const stimjim_context_t *sc, const int8_t pt_idx) {

    pulsetrain_t pt = stimjim_ctx_get_pulsetrain(sc, pt_idx);

    pulsetrain_t pt_converted = {
        .output_mode = { pt.output_mode[0], pt.output_mode[1] },
        .n_pulses = pt.n_pulses,
    };

    uint32_t stage_duration_sum = 0;
    offsets_t offsets[2] = { stimjim_ctx_get_offsets(sc, 0), stimjim_ctx_get_offsets(sc, 1) };

    uint8_t n = 0;
    for (uint8_t i = 0; i < pt.n_stages; i++) {
        if (!pt.stage_duration[i]) continue;
        stage_duration_sum += pt.stage_duration[i];
        pt_converted.stage_duration[n] = pt.stage_duration[i];
        pt_converted.stage_amplitude[0][n] = dac_code(offsets[0], pt.output_mode[0] & OUTPUT_MODE_CURRENT, pt.stage_amplitude[0][i]);
        pt_converted.stage_amplitude[1][n] = dac_code(offsets[1], pt.output_mode[1] & OUTPUT_MODE_CURRENT, pt.stage_amplitude[1][i]);
        n++;
    }
    pt_converted.n_stages = n + 1;

    // inter-pulse gap: zero amplitude for the remainder of the period
    pt_converted.stage_duration[n]  = pt.period - stage_duration_sum;
    pt_converted.stage_amplitude[0][n] = dac_code(offsets[0], pt.output_mode[0] & OUTPUT_MODE_CURRENT, 0);
    pt_converted.stage_amplitude[1][n] = dac_code(offsets[1], pt.output_mode[1] & OUTPUT_MODE_CURRENT, 0);

    return pt_converted;
}

// init

void stimjim_ctx_init(stimjim_context_t *sc, queue_t *q_core1_cmd, queue_t *q_offsets_rx) {
    memset(sc, 0, sizeof(*sc));
    sc->q_core1_cmd = q_core1_cmd;
    sc->q_offsets_rx = q_offsets_rx;
    for (uint8_t ch = 0; ch < 2; ch++)
        stimjim_ctx_set_channel_io(sc, ch, -1, GPIO_OUT);
    offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC | OFFSETS_TX_CALIBRATE_CURRENT,
    };
    while (!stimjim_ctx_try_request_offsets(sc, &offsets_calibration));
    while (!stimjim_ctx_try_receive_offsets(sc, NULL));

    gpio_put(LED_A, false);
    gpio_put(LED_B, false);
}

// getters

channel_io_t stimjim_ctx_get_channel_io(const stimjim_context_t *sc, const uint8_t ch) {
    return sc->channel_io[ch];
}

pulsetrain_t stimjim_ctx_get_pulsetrain(const stimjim_context_t *sc, const int8_t pt_idx) {
    return pt_idx < 0 ? (pulsetrain_t){0} : sc->pulsetrains[pt_idx];
}

offsets_t stimjim_ctx_get_offsets(const stimjim_context_t *sc, const uint8_t ch) {
    return sc->offsets[ch];
}

// setters

void stimjim_ctx_set_channel_io(stimjim_context_t *sc, const uint8_t ch, const int8_t target, const bool dir) {
    const uint ch_pin = ch ? CHANNEL_IO_B : CHANNEL_IO_A;
    const uint32_t edge_bit = RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);
    const channel_io_t prev = sc->channel_io[ch];

    gpio_set_pulls(ch_pin, false, !dir);
    gpio_set_dir(ch_pin, dir);

    sc->channel_io[ch].dir = dir;
    sc->channel_io[ch].idx = target;
    if (dir || target < 0)
        hw_clear_bits(&io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3], edge_bit);
    else if (prev.dir || prev.idx < 0) {
        io_bank0_hw->intr[ch_pin >> 3] = edge_bit;
        hw_set_bits(&io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3], edge_bit);
    }
}

void stimjim_ctx_set_pulsetrain(stimjim_context_t *sc, const uint8_t pt_index, const pulsetrain_t *pt) {
    sc->pulsetrains[pt_index] = *pt;
}

bool stimjim_ctx_try_request_offsets(stimjim_context_t *sc, const offsets_tx_t *offsets_calibration) {
    core1_cmd_t cmd = { .type = CORE1_CMD_OFFSETS, .offsets = *offsets_calibration };
    return queue_try_add(sc->q_core1_cmd, &cmd);
}

bool stimjim_ctx_try_receive_offsets(stimjim_context_t *sc, bool *adc_read_failed) {
    offsets_rx_t rx;
    if (!queue_try_remove(sc->q_offsets_rx, &rx)) return false;
    sc->offsets[0] = rx.offsets[0];
    sc->offsets[1] = rx.offsets[1];
    if (adc_read_failed) *adc_read_failed = rx.adc_read_failed;
    return true;
}
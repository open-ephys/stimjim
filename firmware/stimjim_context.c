/*  
 *  Implements an opaque pointer in C. The benefits are:
 *  - Setters maintain parity between the context state and hardware state.
 *    The context's members cannot otherwise be accessed.
 *  - The context is locally scoped to main.(.. unless you call stimjim_ctx_init
 *    elsewhere as well.)
 */

#include <math.h>
#include "stimjim_context.h"
#include "core1.h"

struct stimjim_ctx {
    channel_io_t channel_io_pins[2];
    offsets_t  offsets[2];
    pulsetrain_t pulsetrains[MAX_PULSETRAINS];
    queue_t *q_core1_cmd, *q_offsets_rx;
};

// single static instance
static stimjim_context_t stimjim_ctx = { 0 };

void __not_in_flash_func(set_output_mode)(uint8_t ch, output_mode_t mode) {
    static const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
    static const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
    gpio_put(oe0_pin[ch], mode & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND));
    gpio_put(oe1_pin[ch], mode & (OUTPUT_MODE_FLOAT | OUTPUT_MODE_GND));
}

// convert mV or uA to dac_code
static inline int16_t dac_code(const offsets_t off, const bool line, const int32_t amplitude) {
    int32_t code = lroundf(amplitude / (line ? MICROAMPS_PER_DAC : MILLIVOLTS_PER_DAC)) + (line ? off.current : off.voltage);
    if (code < INT16_MIN) code = INT16_MIN;
    if (code > INT16_MAX) code = INT16_MAX;
    return (int16_t)code;
}

// for every stage in pulse train, convert mV or uA to dac code
pulsetrain_t stimjim_ctx_convert_pt(const stimjim_context_t *ctx, const int8_t pt_idx) {

    pulsetrain_t pt = stimjim_ctx_get_pulsetrain(ctx, pt_idx);

    pulsetrain_t pt_converted = {
        .output_mode = { pt.output_mode[0], pt.output_mode[1] },
        .n_pulses = pt.n_pulses,
    };

    uint32_t stage_duration_sum = 0;
    offsets_t offsets[2] = { stimjim_ctx_get_offsets(ctx, 0), stimjim_ctx_get_offsets(ctx, 1) };

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

stimjim_context_t *stimjim_ctx_init(queue_t *q_core1_cmd, queue_t *q_offsets_rx) {
    stimjim_ctx.q_core1_cmd = q_core1_cmd;
    stimjim_ctx.q_offsets_rx = q_offsets_rx;
    for (uint8_t ch = 0; ch < 2; ch++) 
        stimjim_ctx_set_channel_io(&stimjim_ctx, ch, -1, GPIO_OUT);
    offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC | OFFSETS_TX_CALIBRATE_CURRENT,
    };
    while (!stimjim_ctx_try_request_offsets(&stimjim_ctx, &offsets_calibration));
    while (!stimjim_ctx_try_receive_offsets(&stimjim_ctx));

    gpio_put(LED_A, false);
    gpio_put(LED_B, false);

    return &stimjim_ctx;
}

// getters

channel_io_t stimjim_ctx_get_channel_io(const stimjim_context_t *stimjim_ctx, const uint8_t ch) {
    return stimjim_ctx->channel_io_pins[ch];
}

pulsetrain_t stimjim_ctx_get_pulsetrain(const stimjim_context_t *stimjim_ctx, const int8_t pt_idx) {
    return pt_idx < 0 ? (pulsetrain_t){0} : stimjim_ctx->pulsetrains[pt_idx];
}

offsets_t stimjim_ctx_get_offsets(const stimjim_context_t *stimjim_ctx, const uint8_t ch) {
    return stimjim_ctx->offsets[ch];
}

// setters

void stimjim_ctx_set_channel_io(stimjim_context_t *stimjim_ctx, const uint8_t ch, const int8_t target, const bool dir) {
    const uint ch_pin = ch ? CHANNEL_IO_B : CHANNEL_IO_A;

    gpio_set_pulls(ch_pin, false, !dir);
    gpio_set_dir(ch_pin, dir);

    stimjim_ctx->channel_io_pins[ch].dir = dir;
    stimjim_ctx->channel_io_pins[ch].idx = target;
    if (dir || target < 0)
        io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3] &= ~RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);
    else {
        io_bank0_hw->intr[ch_pin >> 3] = RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);
        io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3] |= RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);
    }
}

void stimjim_ctx_set_pulsetrain(stimjim_context_t *stimjim_ctx, const uint8_t pt_index, const pulsetrain_t *pt) {
    stimjim_ctx->pulsetrains[pt_index] = *pt;
}

bool stimjim_ctx_try_request_offsets(stimjim_context_t *stimjim_ctx, const offsets_tx_t *offsets_calibration) {
    core1_cmd_t cmd = { .type = CORE1_CMD_OFFSETS, .offsets = *offsets_calibration };
    return queue_try_add(stimjim_ctx->q_core1_cmd, &cmd);
}

bool stimjim_ctx_try_receive_offsets(stimjim_context_t *stimjim_ctx) {
    return queue_try_remove(stimjim_ctx->q_offsets_rx, &stimjim_ctx->offsets);
}
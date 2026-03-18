/*  
 *  Implements an opaque pointer in C. The benefits are:
 *  - Setters maintain parity between the context state and hardware state.
 *    The context cannot otherwise be modified in main.
 *  - The context is locally scoped to main.(.. unless you initialize
 *    elsewhere as well.)
 */

#include "stimjim_context.h"

struct stimjim_ctx {
    channel_io_t channel_io_pins[2];
    offsets_t  offsets[2];
    pulsetrain_t pulsetrains[MAX_PULSETRAINS];
    queue_t *q_tx_offsets, *q_rx_offsets;
};

// single static instance — initialized and maintained in core0
static stimjim_context_t stimjim_ctx = { 0 };

// helpers

static inline int16_t dac_code(const offsets_t off, const bool line, const int amplitude) {
    return (int16_t)(amplitude / (line ? MICROAMPS_PER_DAC : MILLIVOLTS_PER_DAC)) + (line ? off.current : off.voltage);
}

pulsetrain_t convert_pt(const uint8_t pt_idx) {
    
    pulsetrain_t pt = stimjim_ctx_get_pulsetrain(&stimjim_ctx, pt_idx);
    
    pulsetrain_t pt_converted = {
        .output_mode = { pt.output_mode[0], pt.output_mode[1] },
        .n_stages = pt.n_stages + 1,
        .n_pulses = pt.n_pulses,
    };

    uint32_t stage_duration_sum = 0;
    offsets_t offsets[2] = { stimjim_ctx_get_offsets(&stimjim_ctx, 0), stimjim_ctx_get_offsets(&stimjim_ctx, 1) };
    
    for (uint8_t i = 0; i < pt.n_stages; i++) {
        stage_duration_sum += pt.stage_duration[i];
        pt_converted.stage_duration[i] = pt.stage_duration[i];
        pt_converted.stage_amplitude[0][i] = dac_code(offsets[0], pt.output_mode[0], pt.stage_amplitude[0][i]);
        pt_converted.stage_amplitude[1][i] = dac_code(offsets[1], pt.output_mode[1], pt.stage_amplitude[1][i]);
    }

    // Inter-pulse gap: zero amplitude for the remainder of the period
    pt_converted.stage_duration[pt.n_stages]  = pt.period - stage_duration_sum;
    pt_converted.stage_amplitude[0][pt.n_stages] = dac_code(offsets[0], pt.output_mode[0], 0);
    pt_converted.stage_amplitude[1][pt.n_stages] = dac_code(offsets[1], pt.output_mode[1], 0);

    return pt_converted;
}

// init

stimjim_context_t *stimjim_ctx_init(queue_t *q_tx_offsets, queue_t *q_rx_offsets) {
    stimjim_ctx.q_tx_offsets = q_tx_offsets;
    stimjim_ctx.q_rx_offsets = q_rx_offsets;
    for (uint8_t ch = 0; ch < 2; ch++) 
        stimjim_ctx_set_channel_io(&stimjim_ctx, ch, -1, GPIO_OUT);
    stimjim_ctx_set_offsets(&stimjim_ctx, true);

    gpio_put(LED_A, false);
    gpio_put(LED_B, false);

    return &stimjim_ctx;
}

// getters

const channel_io_t stimjim_ctx_get_channel_io(const stimjim_context_t *stimjim_ctx, const bool ch) {
    return stimjim_ctx->channel_io_pins[ch];
}

const pulsetrain_t stimjim_ctx_get_pulsetrain(const stimjim_context_t *stimjim_ctx, const uint8_t pt_index) {
    return stimjim_ctx->pulsetrains[pt_index];
}

const offsets_t stimjim_ctx_get_offsets(const stimjim_context_t *stimjim_ctx, const bool ch) {
    return stimjim_ctx->offsets[ch];
}

// setters

void stimjim_ctx_set_channel_io(stimjim_context_t *stimjim_ctx, bool ch, const int8_t target, const bool dir) {
    const uint ch_pin = ch ? CHANNEL_IO_B : CHANNEL_IO_A;

    gpio_set_pulls(ch_pin, false, !dir);
    gpio_set_dir(ch_pin, dir);

    stimjim_ctx->channel_io_pins[ch].dir = dir;
    stimjim_ctx->channel_io_pins[ch].idx = target;
    if (dir)
        io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3] &= ~RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);
    else
        io_bank0_hw->proc1_irq_ctrl.inte[ch_pin >> 3] |= RISING_EDGE_INTERRUPT_BIT(ch_pin, 1);

}

void stimjim_ctx_set_pulsetrain(stimjim_context_t *stimjim_ctx, const uint8_t pt_index, const pulsetrain_t *pt) {
    stimjim_ctx->pulsetrains[pt_index] = *pt;
}

void stimjim_ctx_set_offsets(stimjim_context_t *stimjim_ctx, const bool all_offsets) {
    queue_add_blocking(stimjim_ctx->q_tx_offsets, &all_offsets);
    queue_remove_blocking(stimjim_ctx->q_rx_offsets, &stimjim_ctx->offsets);
}
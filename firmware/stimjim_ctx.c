/*  
 *  Implements an opaque pointer in C. The benefits are:
 *  - "Setters" maintain parity between the context state and hardware state.
 *    Struct cannot be modified otherwise.
 *  - The context can be locally scoped to the a single function where to avoid
 *    conflict between cores.(.. unless you inititalize it in both cores)
 */

#include <hardware/gpio.h>
#include <pico/util/queue.h>
#include "adc.h"
#include "dac.h"
#include "stimjim_ctx.h"

#define RISING_EDGE_INTERRUPT_BIT(gpio_number, value) (value << (4 * ((gpio_number) % 8) + 3))

struct stimjim_ctx {
  channel_io_t channel_io[2];
  pulsetrain_t pulsetrains[MAX_PULSETRAINS];
  queue_t *channel_io_queue;
};

static inline int16_t dac_code(const offsets_t off, const bool line, const int amplitude) {
  return (int16_t)(amplitude / (line ? MICROAMPS_PER_DAC : MILLIVOLTS_PER_DAC))
         + (line ? off.current : off.voltage);
}

void __time_critical_func(set_output_mode)(const bool ch, const uint8_t output_mode) { 
  const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
  const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
  gpio_put(oe0_pin[ch], 1 & output_mode);
  gpio_put(oe1_pin[ch], (0b10 & output_mode) >> 1);
}

// "private" ctx
static stimjim_ctx_t ctx;

stimjim_ctx_t *stimjim_ctx_init(queue_t *channel_io_queue) {
  ctx.channel_io_queue = channel_io_queue;

  for (uint8_t i = 0; i < 2; i++) {
    set_output_mode(i, OUTPUT_MODE_FLOAT);
    stimjim_ctx_set_channel_io(&ctx, i, -1, GPIO_OUT);
  }
  
  for (int i = 0; i < MAX_PULSETRAINS; i++) {
    ctx.pulsetrains[i].period = 0;
    ctx.pulsetrains[i].n_pulses = 0;
    ctx.pulsetrains[i].n_stages = 0;
  }
  return &ctx;
}

const channel_io_t stimjim_ctx_get_channel_io(const stimjim_ctx_t *ctx, const bool ch) {
  return ctx->channel_io[ch]; 
}

const pulsetrain_t stimjim_ctx_get_pulsetrain_parameters(const stimjim_ctx_t *ctx, const uint8_t pulsetrain_index) {
  return ctx->pulsetrains[pulsetrain_index]; 
}

void stimjim_ctx_set_channel_io(stimjim_ctx_t *ctx, const bool ch, const int8_t target, const bool dir) {
  if (ch) {
    queue_add_blocking(ctx->channel_io_queue, &dir);
  }
  else {
    gpio_set_pulls(CHANNEL_IO_A, 0, !dir);
    gpio_set_dir(CHANNEL_IO_A, dir);
    io_bank0_hw->proc0_irq_ctrl.inte[4] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_A, !dir);
  }

  ctx->channel_io[ch].dir = dir;
  ctx->channel_io[ch].idx = target;
}

void stimjim_ctx_set_pulsetrain(stimjim_ctx_t *ctx, const uint8_t pt_index, const pulsetrain_t *pt) {
  ctx->pulsetrains[pt_index] = *pt;
}

const waveform_t pt_to_wf(bool ch, const pulsetrain_t pt, const offsets_t off) {

  if (pt.output_mode[ch] > 1) {
    waveform_t wf = { 0 };
    return wf;
  }

  const bool line = (bool)pt.output_mode[ch];

  waveform_t wf = {
    .n_pulses    = pt.n_pulses,
    .n_stages    = pt.n_stages + 1,
    .output_mode = line,
  };

  uint32_t stage_sum = 0;
  for (uint8_t i = 0; i < pt.n_stages; i++) {
    stage_sum += pt.stage_dur[i];
    wf.stage_duration[i] = pt.stage_dur[i];
    wf.stage_amplitude[i]= dac_code(off, line, pt.amplitude[ch][i]);
  }

  // Final inter-pulse gap stage: zero amplitude, remaining time
  wf.stage_duration[pt.n_stages]  = pt.period - stage_sum;
  wf.stage_amplitude[pt.n_stages] = dac_code(off, line, 0);

  return wf;
}

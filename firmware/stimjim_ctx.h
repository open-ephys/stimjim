#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAX_PULSETRAINS 100
#define MAX_STAGES 10

// opaque forward declaration
typedef struct stimjim_ctx stimjim_ctx_t;

typedef struct {
  float adc;
  int8_t voltage;
  int8_t current;
} offsets_t;

typedef enum {
  OUTPUT_MODE_VOLTAGE,
  OUTPUT_MODE_CURRENT,
  OUTPUT_MODE_FLOAT,
  OUTPUT_MODE_GND,
} output_mode_t;

typedef struct {
  output_mode_t output_mode[2];
  uint8_t n_stages;
  uint32_t period; // us
  uint32_t n_pulses;
  int amplitude[2][MAX_STAGES]; // mV or uA for each channel
  unsigned int stage_dur[MAX_STAGES]; // us
  uint64_t t_start; // us
} pulsetrain_t;

typedef struct {
  bool output_mode;
  uint8_t n_stages;
  int16_t stage_amplitude[MAX_STAGES+1]; // dac code
  uint32_t n_pulses; 
  uint32_t stage_duration[MAX_STAGES+1]; // us
} waveform_t;

typedef struct {
  bool dir; // GPIO_IN=0, GPIO_OUT=1
  uint8_t idx;
} channel_io_t;

// init
stimjim_ctx_t *stimjim_ctx_init(queue_t *channel_io_queue);

// getters
const channel_io_t stimjim_ctx_get_channel_io(const stimjim_ctx_t *ctx, const bool ch);
const pulsetrain_t stimjim_ctx_get_pulsetrain_parameters(const stimjim_ctx_t *ctx, const uint8_t pulsetrain_index);

// setters
void __time_critical_func(set_output_mode)(const bool ch, const uint8_t output_mode);
void stimjim_ctx_set_pulsetrain(stimjim_ctx_t *ctx, const uint8_t pulsetrain_index, const pulsetrain_t *pt);
void stimjim_ctx_set_channel_io(stimjim_ctx_t *ctx, const bool ch, const int8_t target, const bool out);

// convert pulse train struct to waveform struct
// - pulse train is the struct set directly by user.
// - waveform is the struct constructed from pulse train with inter-pulse stage
//   duration and pre-calculated dac values for a given channel. 
const waveform_t pt_to_wf(bool ch, const pulsetrain_t pt, const offsets_t off);

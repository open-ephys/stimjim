#pragma once

#include <hardware/gpio.h>
#include <pico/util/queue.h>

#define MAX_PULSETRAINS  100
#define MAX_STAGES 10

typedef struct stimjim_ctx stimjim_context_t;

typedef struct {
    float  adc;
    int8_t voltage;
    int8_t current;
} offsets_t;

typedef enum {
    OUTPUT_MODE_VOLTAGE = 1 << 0,
    OUTPUT_MODE_CURRENT = 1 << 1,
    OUTPUT_MODE_FLOAT   = 1 << 2,
    OUTPUT_MODE_GND     = 1 << 3,
} output_mode_t;

#define OUTPUT_MODE_ACTIVE (OUTPUT_MODE_VOLTAGE | OUTPUT_MODE_CURRENT)

typedef struct { 
    bool dir; // HIGH = trigger input, LOW = sync output
    int8_t idx; // pulse train that should be delivered when channel is triggered 
} channel_io_t;

typedef enum {
    OFFSETS_TX_CALIBRATE_ADC     = 1,
    OFFSETS_TX_CALIBRATE_VOLTAGE = 1 << 1,
    OFFSETS_TX_CALIBRATE_CURRENT = 1 << 2,
} offsets_tx_type_t;

typedef struct { 
    offsets_tx_type_t offset_tx_type;
    float adc[2]; // if only calculating voltage/current offsets, pass adc offsets
} offsets_tx_t;

typedef struct {
    output_mode_t output_mode[2];
    uint8_t n_stages;
    int32_t stage_amplitude[2][MAX_STAGES+1]; // mV or uA
    uint32_t period; // us
    uint32_t n_pulses;
    uint32_t stage_duration[MAX_STAGES+1]; // us
} pulsetrain_t;

__always_inline static inline void set_output_mode(uint8_t ch, output_mode_t mode) {
    static const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
    static const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
    gpio_put(oe0_pin[ch], mode & (OUTPUT_MODE_CURRENT | OUTPUT_MODE_GND));
    gpio_put(oe1_pin[ch], mode & (OUTPUT_MODE_FLOAT | OUTPUT_MODE_GND));
}

stimjim_context_t *stimjim_ctx_init(queue_t *q_offsets_tx, queue_t *q_offsets_rx);

const channel_io_t stimjim_ctx_get_channel_io(const stimjim_context_t *ctx, const uint8_t ch);
const pulsetrain_t stimjim_ctx_get_pulsetrain(const stimjim_context_t *ctx, const int8_t pt_index);
const offsets_t stimjim_ctx_get_offsets(const stimjim_context_t *ctx, const uint8_t ch);

void stimjim_ctx_set_channel_io(stimjim_context_t *ctx, const uint8_t ch, const int8_t target, const bool dir);
void stimjim_ctx_set_pulsetrain(stimjim_context_t *ctx, const uint8_t pt_index, const pulsetrain_t *pt);
void stimjim_ctx_set_offsets(stimjim_context_t *stimjim_ctx, const offsets_tx_t *offsets_calibration);

pulsetrain_t stimjim_ctx_convert_pt(const stimjim_context_t *ctx, const int8_t pt_idx);
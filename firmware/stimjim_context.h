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
    OUTPUT_MODE_VOLTAGE,
    OUTPUT_MODE_CURRENT,
    OUTPUT_MODE_FLOAT,
    OUTPUT_MODE_GND,
} output_mode_t;

typedef struct { 
    bool dir; // HIGH = trigger input, LOW = sync output
    uint8_t idx; // pulse train that should be delivered when channel is triggered 
} channel_io_t;

typedef enum {
    OFFSETS_TX_CALIBRATE_ADC = 1,
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

__always_inline static inline void set_output_mode(bool ch, uint8_t output_mode) {
    static const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
    static const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
    gpio_put(oe0_pin[ch], 1 & output_mode);
    gpio_put(oe1_pin[ch], (0b10 & output_mode) >> 1);
}

stimjim_context_t *stimjim_ctx_init(queue_t *q_offsets_tx, queue_t *q_offsets_rx);

const channel_io_t stimjim_ctx_get_channel_io(const stimjim_context_t *ctx, const bool ch);
const pulsetrain_t stimjim_ctx_get_pulsetrain(const stimjim_context_t *ctx, const uint8_t pt_index);
const offsets_t stimjim_ctx_get_offsets(const stimjim_context_t *ctx, const bool ch);

void stimjim_ctx_set_channel_io(stimjim_context_t *ctx, const bool ch, const int8_t target, const bool dir);
void stimjim_ctx_set_pulsetrain(stimjim_context_t *ctx, const uint8_t pt_index, const pulsetrain_t *pt);
void stimjim_ctx_set_offsets(stimjim_context_t *stimjim_ctx, const offsets_tx_t *offsets_calibration);

pulsetrain_t convert_pt(const uint8_t pt_idx);
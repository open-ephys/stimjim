#pragma once

#include "stimjim_context.h"

#define CORE_HANDSHAKE_MESSAGE 0xDEADBEEF

extern queue_t q_core1_cmd, q_offsets_rx, q_stimulus_telemetry, q_adc_result;
extern volatile uint32_t stimulus_request_seq;  // incremented by core0 for every T/U
#define DOORBELL_STOP          (1u << 0)
#define DOORBELL_TRAIN_RUNNING (1u << 1)

typedef struct {
    uint8_t n_stages;
    uint32_t n_pulses;
    int64_t measured_amplitudes[2][MAX_STAGES + 1];
    uint32_t delivered_stages[MAX_STAGES + 1];
    output_mode_t output_mode[2];
    bool cancelled;
    uint8_t stop_stage;
    uint32_t dropped_summaries, dropped_trigger_notices;
} core1_stim_telemetry_t;

typedef enum {
    MANUAL_CMD_DAC_SET,
    MANUAL_CMD_ADC_READ,
    MANUAL_CMD_SET_OUTPUT_MODE,
} manual_cmd_type_t;

typedef struct {
    uint8_t ch;
    bool line;
    manual_cmd_type_t type;
    int16_t dac_code;
    output_mode_t output_mode;
} manual_cmd_t;

typedef enum {
    CORE1_CMD_STIMULUS,
    CORE1_CMD_TRIGGER_CONFIG,
    CORE1_CMD_OFFSETS,
    CORE1_CMD_MANUAL,
} core1_cmd_type_t;

typedef struct {
    core1_cmd_type_t type;
    union {
        struct { uint32_t seq; pulsetrain_t pt; } stimulus;
        struct { uint8_t ch; pulsetrain_t pt; } trigger_config;
        offsets_tx_t offsets;
        manual_cmd_t manual;
    };
} core1_cmd_t;

void __time_critical_func(main_core1)(void);

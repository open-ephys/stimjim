#pragma once

#include "stimjim_context.h"

#define CORE_HANDSHAKE_MESSAGE 0xDEADBEEF

extern queue_t q_offsets_tx, q_offsets_rx, q_stim_result, q_manual_cmd, q_adc_result;
extern queue_t q_stimulus_cmd; 
extern queue_t q_stimulus_trigger[2]; 

typedef struct {
    uint8_t ch, n_stages;
    int32_t measured_amplitudes[MAX_STAGES + 1];
    uint32_t delivered_stages[MAX_STAGES + 1];
    output_mode_t output_mode;
} stim_result_t;

typedef enum {
    MANUAL_CMD_DAC_SET,
    MANUAL_CMD_ADC_READ,
} manual_cmd_type_t;

typedef struct {
    bool ch;
    bool line;
    manual_cmd_type_t type;
    int16_t dac_code;
} manual_cmd_t;

void __time_critical_func(main_core1)(void);
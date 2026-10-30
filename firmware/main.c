#define FIRMWARE_VERSION "v0.1.0"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>

#include "core1.h"
#include "stimjim_context.h"

#define BUF_LEN 1024
#define RUNS_AFTER_TRAIN "Info: Will run after the running train finishes."

queue_t q_core1_cmd, q_offsets_rx, q_stimulus_telemetry, q_adc_result;
volatile uint32_t stimulus_request_seq;
static queue_t q_deferred_commands;
static bool verbose;

static void keep_serving_while_waiting(stimjim_context_t *sc);

// Queue a command for core1. While the queue is full (core1 is busy with a
// stimulus), keep handling stop commands so they can still cancel it.
static void send_core1_cmd(stimjim_context_t *sc, const core1_cmd_t *cmd) {
    while (!queue_try_add(&q_core1_cmd, cmd))
        keep_serving_while_waiting(sc);
}

static bool is_delimiter(const char c) {
    return c == ',' || c == ';';
}

static const char *skip_ws(const char *p) {
    while (*p == ' ' || *p == '\t')
        p++;
    return p;
}

static void parse_skip(const char **p) {
    *p = skip_ws(*p);
    if (**p == ',')
        *p = skip_ws(*p + 1);
}

static bool try_parse_in_range(const char **p, long long *out, const char *name, const long long min, const long long max) {
    parse_skip(p);

    char *ep;
    long long val = strtoll(*p, &ep, 10);
    const char *next = skip_ws(ep);
    if (ep != *p && (*next == '\0' || is_delimiter(*next)) && val >= min && val <= max) {
        *out = val;
        *p = next;
        return true;
    }

    if (verbose) {
        const char *end = *p;
        while (*end && !is_delimiter(*end)) end++;
        printf("Error: Invalid %s: '%.*s' (must be %lld..%lld)\n", name, (int)(end - *p), *p, min, max);
    }
    return false;
}

static bool try_parse_i32_in_range(const char **p, int32_t *out, const char *name, const int32_t min, const int32_t max) {
    long long val;
    if (!try_parse_in_range(p, &val, name, min, max)) return false;
    *out = (int32_t)val;
    return true;
}

static bool try_parse_u32_in_range(const char **p, uint32_t *out, const char *name, const uint32_t min, const uint32_t max) {
    long long val;
    if (!try_parse_in_range(p, &val, name, min, max)) return false;
    *out = (uint32_t)val;
    return true;
}

static const char *teensy_second_arg(const char *args) {
    char *end;
    strtoll(args, &end, 10);
    return (end != args && *end == ',') ? end + 1 : "";
}

static void print_invalid_index(void) {
    if (!verbose) puts("Invalid PulseTrain index.");
}

static void print_out_of_range(const char *num) {
    if (verbose) return;
    num = skip_ws(num);
    const bool negative = *num == '-';
    if (*num == '-' || *num == '+') num++;
    while (num[0] == '0' && num[1] >= '0' && num[1] <= '9') num++;
    const int digits = (int)strspn(num, "0123456789");
    if (!digits || (digits == 1 && num[0] == '0')) puts("0 is out of range.");
    else printf("%s%.*s is out of range.\n", negative ? "-" : "", digits, num);
}

static bool check_for_extra_arguments(const char *p) {
    if (*p == '\0') return false;
    if (verbose) printf("Error: Unexpected '%s' after the arguments\n", p);
    return true;
}

static void print_offsets(const stimjim_context_t *sc) {
    const offsets_t off[2] = { stimjim_ctx_get_offsets(sc, 0), stimjim_ctx_get_offsets(sc, 1) };
    printf("ADC offsets (+-2.5V): %f, %f\n", 0.0, 0.0);
    printf("ADC offsets (+-10V): %f, %f\ncurrent offsets: %d, %d\nvoltage offsets: %d, %d\n\n",
           (double)off[0].adc, (double)off[1].adc, off[0].current, off[1].current,
           off[0].voltage, off[1].voltage);
}

static void print_channel_io(const stimjim_context_t *sc, const uint8_t ch) {
    const channel_io_t io = stimjim_ctx_get_channel_io(sc, ch);
    if (io.dir == GPIO_OUT)
        printf("Detaching interrupt to IN%d\nProgramming IN%d as output that indicates activity on output %d\n", ch, ch, ch);
    else
        printf("Attaching interrupt to IN%d to run PulseTrain[%d]\n", ch, io.idx);
}

static int read_output_mode(const uint8_t ch) {
    static const uint8_t oe0_pin[2] = { OE0_A, OE0_B };
    static const uint8_t oe1_pin[2] = { OE1_A, OE1_B };
    const bool oe0 = gpio_get_out_level(oe0_pin[ch]), oe1 = gpio_get_out_level(oe1_pin[ch]);
    return oe0 ? (oe1 ? 3 : 1) : (oe1 ? 2 : 0);
}

static void print_output_mode(const uint8_t ch) {
    printf("Set channel %d to mode %d\n", ch, read_output_mode(ch));
}

static void set_trigger_pulsetrain(stimjim_context_t *sc, const uint8_t ch, const int8_t idx) {
    core1_cmd_t cmd = {
        .type = CORE1_CMD_TRIGGER_CONFIG,
        .trigger_config = { .ch = ch, .idx = idx, .pt = stimjim_ctx_convert_pt(sc, idx) },
    };
    send_core1_cmd(sc, &cmd);
}

static void set_trigger_pulsetrains_w_new_offsets(stimjim_context_t *sc) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        channel_io_t io = stimjim_ctx_get_channel_io(sc, ch);
        set_trigger_pulsetrain(sc, ch, io.idx);
    }
}

static void print_pulsetrain(const stimjim_context_t *sc, const uint8_t n) {
    static const char *const mode_names[4] = {
        "Voltage output", "Current output", "No output (high-Z)", "No output (grounded)" };

    pulsetrain_t pt = stimjim_ctx_get_pulsetrain(sc, (int8_t)n);
    if (pt.n_pulses == 0)
        pt.output_mode[0] = pt.output_mode[1] = OUTPUT_MODE_GND;
    // output_mode is stored as a bit flag (1 << mode)
    const int mode[2] = { __builtin_ctz(pt.output_mode[0]), __builtin_ctz(pt.output_mode[1]) };
    const uint64_t duration = (uint64_t)pt.n_pulses * pt.period;
    puts("----------------------------------");
    printf("Parameters for PulseTrain[%d]\n  mode[ch0]: %d (%s)\n  mode[ch1]: %d (%s)\n",
           n, mode[0], mode_names[mode[0]], mode[1], mode_names[mode[1]]);
    printf("  period:    %" PRIu32 " usec (%0.3f sec, %0.3f Hz)\n  duration:  %llu usec (%0.3f sec)\n",
           pt.period, 0.000001 * pt.period, pt.period ? 1000000.0 / pt.period : 0.0,
           (unsigned long long)duration, 0.000001 * (double)duration);
    puts("\n  stage    duration     output0   output1");
    for (uint8_t i = 0; i < pt.n_stages; i++)
        printf("   %2d  %7" PRIu32 " usec %8" PRId32 "%s %8" PRId32 "%s\n", i, pt.stage_duration[i],
               pt.stage_amplitude[0][i], mode[0] == 0 ? "mV" : "uA",
               pt.stage_amplitude[1][i], mode[1] == 0 ? "mV" : "uA");
    puts("----------------------------------\n");
    if (pt.n_pulses == 0 && verbose)
        printf("Info: PulseTrain[%d] is not defined.\n", n);
}

static void print_high_voltage_warning(void) {
    if (verbose) printf("Warning: >%dV in voltage mode. The ADC can't accurately measure voltages beyond +/-%dV.\n",
           MAX_MEASURABLE_MILLIVOLTS / 1000, MAX_MEASURABLE_MILLIVOLTS / 1000);
}

static bool parse_pulsetrain(const char *p, pulsetrain_t *out) {
    int32_t mode0, mode1;
    uint32_t duration;
    pulsetrain_t pt = { 0 };

    if (!try_parse_i32_in_range(&p, &mode0, "mode0", 0, 3)
     || !try_parse_i32_in_range(&p, &mode1, "mode1", 0, 3)
     || !try_parse_u32_in_range(&p, &pt.period, "period_us",  1, UINT32_MAX)
     || !try_parse_u32_in_range(&p, &duration,  "total_dur_us", 1, UINT32_MAX))
        return false;

    pt.output_mode[0] = 1 << mode0;
    pt.output_mode[1] = 1 << mode1;

    if (*p != ';' && check_for_extra_arguments(p)) return false;
    int32_t amp_limit[2];
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (pt.output_mode[ch] & OUTPUT_MODE_VOLTAGE)      amp_limit[ch] = MAX_MILLIVOLTS;
        else if (pt.output_mode[ch] & OUTPUT_MODE_CURRENT) amp_limit[ch] = MAX_MICROAMPS;
        else                                               amp_limit[ch] = INT32_MAX;
    }

    while (*p == ';') {
        p = skip_ws(p + 1);
        if (!*p) break;
        if (pt.n_stages >= MAX_STAGES) {
            if (verbose) printf("Error: Invalid pulse train: more than %d stages.\n", MAX_STAGES);
            return false;
        }
        char field_name[32];
        snprintf(field_name, sizeof(field_name), "amp0 stage[%d]", pt.n_stages);
        if (!try_parse_i32_in_range(&p, &pt.stage_amplitude[0][pt.n_stages], field_name, -amp_limit[0], amp_limit[0])) return false;
        snprintf(field_name, sizeof(field_name), "amp1 stage[%d]", pt.n_stages);
        if (!try_parse_i32_in_range(&p, &pt.stage_amplitude[1][pt.n_stages], field_name, -amp_limit[1], amp_limit[1])) return false;
        snprintf(field_name, sizeof(field_name), "stage_dur_us stage[%d]", pt.n_stages);
        if (!try_parse_u32_in_range(&p, &pt.stage_duration[pt.n_stages], field_name, 0, UINT32_MAX)) return false;
        for (uint8_t ch = 0; ch < 2; ch++)
            if (!(pt.output_mode[ch] & OUTPUT_MODE_ACTIVE_MASK))
                pt.stage_amplitude[ch][pt.n_stages] = 0;
        pt.n_stages++;
        if (*p != ';' && check_for_extra_arguments(p)) return false;  // a 4th value in a stage
    }

    uint64_t stage_sum = 0;
    for (uint8_t i = 0; i < pt.n_stages; i++)
        stage_sum += pt.stage_duration[i];
    if (stage_sum > pt.period) {
        if (verbose) puts("Error: Invalid pulse train: summed stage durations exceed period.");
        return false;
    }

    pt.n_pulses = (uint32_t)(((uint64_t)duration + pt.period - 1) / pt.period);
    *out = pt;
    return true;
}

static void cmd_S(stimjim_context_t *sc, const char *args) {
    int32_t n;
    pulsetrain_t pt;

    const char *p = args;
    if (!try_parse_i32_in_range(&p, &n, "idx", 0, MAX_PULSETRAINS - 1)) {
        const long leading_idx = strtol(args, NULL, 10);
        if (leading_idx >= 0 && leading_idx < MAX_PULSETRAINS)
            print_pulsetrain(sc, (uint8_t)leading_idx);
        else if (!verbose)
            puts("Invalid PulseTrain index.");
        return;
    }
    if (*p == '\0' || !parse_pulsetrain(p, &pt)) { print_pulsetrain(sc, (uint8_t)n); return; }

    bool short_stage = false, has_stages = false, zero_duration_stage = false;
    for (uint8_t i = 0; i < pt.n_stages; i++) {
        short_stage |= pt.stage_duration[i] && pt.stage_duration[i] < 20u;
        has_stages  |= pt.stage_duration[i] > 0;
        zero_duration_stage |= pt.stage_duration[i] == 0;
    }

    bool high_voltage = false;
    for (uint8_t ch = 0; ch < 2; ch++)
        for (uint8_t i = 0; (pt.output_mode[ch] & OUTPUT_MODE_VOLTAGE) && i < pt.n_stages; i++)
            high_voltage |= labs(pt.stage_amplitude[ch][i]) > MAX_MEASURABLE_MILLIVOLTS;

    stimjim_ctx_set_pulsetrain(sc, (uint8_t)n, &pt);

    for (uint8_t ch = 0; ch < 2; ch++) {
        channel_io_t io = stimjim_ctx_get_channel_io(sc, ch);
        if (io.idx == n)
            set_trigger_pulsetrain(sc, ch, (int8_t)n);
    }

    print_pulsetrain(sc, (uint8_t)n);

    if (!has_stages)
        if (verbose) puts("Warning: No stages defined. This train only controls the LED and sync signal.");
    if (zero_duration_stage)
        if (verbose) puts("Warning: Stages with 0us duration are omitted from the pulse train.");
    if (short_stage)
        if (verbose) puts("Warning: <20us stage duration detected. Specified waveform timings are not guaranteed.");
    if (high_voltage)
        print_high_voltage_warning();
}

static bool stop_already_requested;

static void stop_stimulus(const char cmd_char) {
    if (!stop_already_requested)
        sio_hw->doorbell_out_set = DOORBELL_STOP;
    if (cmd_char == 'X')
        puts("Forcing train to stop");
    else
        printf("Forcing %c train to stop\n", cmd_char);
}

static void print_start_notice(const char cmd_char, const int idx) {
    stdio_putchar_raw('\n');
    printf("Started %c train with parameters of PulseTrain %d\n", cmd_char, idx);
}

static bool cmd_TU(stimjim_context_t *sc, const char cmd_char, const char *args) {
    long long idx = 0;
    const char *p = args;
    if (!try_parse_in_range(&p, &idx, "pulse train index", LLONG_MIN, MAX_PULSETRAINS - 1)
     || check_for_extra_arguments(p)) {
        print_invalid_index();
        return false;
    }
    if (idx < 0) { stop_stimulus(cmd_char); return true; }
    const uint32_t seq = stimulus_request_seq + 1;
    core1_cmd_t cmd = {
        .type = CORE1_CMD_STIMULUS,
        .stimulus = { .seq = seq, .pt = stimjim_ctx_convert_pt(sc, (int8_t)idx) },
    };
    // This clears cancel requests for T/U pulse trains. Core1 clears cancel
    // requests for triggered pulse trains.
    sio_hw->doorbell_out_clr = DOORBELL_STOP;
    // Bump the request number before queueing, so that a running stimulus stops
    // for this one, but core1 never sees this request as newer than itself
    stimulus_request_seq = seq;
    send_core1_cmd(sc, &cmd);
    print_start_notice(cmd_char, (int)idx);
    return true;
}

static void cmd_R(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, dir = 0;
    long long idx = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)) {
        print_channel_io(sc, 0);
        return;
    }
    const bool idx_ok = try_parse_in_range(&p, &idx, "pulse train index", LLONG_MIN, MAX_PULSETRAINS - 1);
    if (!idx_ok
     || (*p == ',' && !try_parse_i32_in_range(&p, &dir, "trigger/sync", 0, 1))
     || check_for_extra_arguments(p)) {
        if (!idx_ok && !verbose)
            print_invalid_index();
        else
            print_channel_io(sc, (uint8_t)ch);
        return;
    }

    const bool sync = dir || idx < 0;
    if (!sync)
        set_trigger_pulsetrain(sc, (uint8_t)ch, (int8_t)idx);
    stimjim_ctx_set_channel_io(sc, (uint8_t)ch, sync ? -1 : (int8_t)idx, sync ? GPIO_OUT : GPIO_IN);
    print_channel_io(sc, (uint8_t)ch);
}

static bool is_train_running(void) {
    return (sio_hw->doorbell_in_set & DOORBELL_TRAIN_RUNNING) || queue_get_level(&q_core1_cmd);
}

static void cmd_V(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, mv = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &mv, "mV", INT32_MIN, INT32_MAX)
     || check_for_extra_arguments(p)) {
        if (!verbose) {
            long long teensy_mv = strtoll(teensy_second_arg(args), NULL, 10);
            if (teensy_mv < INT32_MIN) teensy_mv = INT32_MIN;
            if (teensy_mv > INT32_MAX) teensy_mv = INT32_MAX;
            printf("%lld is out of range.\n",
                   llroundf((int32_t)teensy_mv / MILLIVOLTS_PER_DAC) + stimjim_ctx_get_offsets(sc, ch).voltage);
        }
        return;
    }

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    long long code = llroundf(mv / MILLIVOLTS_PER_DAC) + off.voltage;
    if (code < INT16_MIN || code > INT16_MAX) {
        printf(verbose ? "Error: DAC code %lld is out of range.\n" : "%lld is out of range.\n", code);
        return;
    }

    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_DAC_SET, .ch = (uint8_t)ch, .dac_code = (int16_t)code } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);
    printf("Set channel %" PRId32 " to amplitude %" PRId32 " mV (dac value %lld).\n", ch, mv, code);
    if (labs(mv) > MAX_MEASURABLE_MILLIVOLTS)
        print_high_voltage_warning();
}

static void cmd_A(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0;
    int32_t code = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &code, "dac_code", INT16_MIN, INT16_MAX)
     || check_for_extra_arguments(p)) {
        print_out_of_range(teensy_second_arg(args));
        return;
    }

    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_DAC_SET, .ch = (uint8_t)ch, .dac_code = code } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);
    printf("Set channel %" PRId32 " to amplitude %" PRId32 "\n", ch, code);
    if (read_output_mode((uint8_t)ch) == 0 && (abs(code) - INT8_MIN) * MILLIVOLTS_PER_DAC > MAX_MEASURABLE_MILLIVOLTS)
        print_high_voltage_warning();
}

static void cmd_E(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, line = 0;
    const char *p = args;
    if (try_parse_i32_in_range(&p, &ch, "channel", 0, 1) && try_parse_i32_in_range(&p, &line, "line", 0, 1))
        check_for_extra_arguments(p);

    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_ADC_READ, .ch = (uint8_t)ch, .line = (bool)line } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);

    adc_result_t result;
    while (!queue_try_remove(&q_adc_result, &result))
        keep_serving_while_waiting(sc);
    if (!result.ok) {
        if (verbose) puts("Error: ADC read failed.");
        return;
    }
    const int16_t val = result.value;

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    static const char  units[2][3] = { "mV", "uA" };
    static const float scale[2]    = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
    int32_t real = lroundf(((float)val - off.adc) * scale[line]);
    printf("Read value: %d (%" PRId32 "%s)\n", val, real, units[line]);
}

static void cmd_M(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, mode = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)) {
        print_output_mode(0);
        return;
    }
    if (!try_parse_i32_in_range(&p, &mode, "output mode", 0, 3) || check_for_extra_arguments(p)) {
        print_output_mode((uint8_t)ch);
        return;
    }
    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_SET_OUTPUT_MODE, .ch = (uint8_t)ch, .output_mode = 1 << mode } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);
    printf("Set channel %" PRId32 " to mode %" PRId32 "\n", ch, mode);
}

static void cmd_B(stimjim_context_t *sc, const char *args) {
    if (check_for_extra_arguments(args)) return;
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    if (verbose) puts("Info: Updating ADC offsets...");
    const offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC,
        .prev = { stimjim_ctx_get_offsets(sc, 0), stimjim_ctx_get_offsets(sc, 1) },
    };
    while (!stimjim_ctx_try_request_offsets(sc, &offsets_calibration))
        keep_serving_while_waiting(sc);
    bool adc_read_failed;
    while (!stimjim_ctx_try_receive_offsets(sc, &adc_read_failed))
        keep_serving_while_waiting(sc);

    set_trigger_pulsetrains_w_new_offsets(sc);
    if (verbose && adc_read_failed) puts("Error: ADC reads failed during calibration; offsets that couldn't be measured were kept.");
    if (verbose) puts("Info: Offsets updated");
}

static void cmd_C(stimjim_context_t *sc, const char *args) {
    if (check_for_extra_arguments(args)) return;
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    if (verbose) puts("Info: Updating ADC, current and voltage offsets...");
    const offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC | OFFSETS_TX_CALIBRATE_CURRENT | OFFSETS_TX_CALIBRATE_VOLTAGE,
        .prev = { stimjim_ctx_get_offsets(sc, 0), stimjim_ctx_get_offsets(sc, 1) },
    };
    while (!stimjim_ctx_try_request_offsets(sc, &offsets_calibration))
        keep_serving_while_waiting(sc);
    bool adc_read_failed;
    while (!stimjim_ctx_try_receive_offsets(sc, &adc_read_failed))
        keep_serving_while_waiting(sc);

    set_trigger_pulsetrains_w_new_offsets(sc);
    if (verbose && adc_read_failed) puts("Error: ADC reads failed during calibration; offsets that couldn't be measured were kept.");
    if (verbose) puts("Info: Offsets updated");
}

static void cmd_D(const stimjim_context_t *sc, const char *args) {
    check_for_extra_arguments(args);
    print_offsets(sc);
}

static void cmd_X(const char *args) {
    if (check_for_extra_arguments(args)) return;
    stop_stimulus('X');
}

static void cmd_W(const char *args) {
    if ((*args != '0' && *args != '1') || *skip_ws(args + 1) != '\0') {
        if (verbose) printf("Error: Invalid verbose mode: '%s' (must be 0..1)\n", args);
        return;
    }
    verbose = *args == '1';
    if (verbose) puts("Info: Verbose mode on");
}

static char *read_serial_line(void) {
    static char buf[BUF_LEN];
    static uint16_t nbuf = 0;
    static bool overflow = false; // discarding the rest of an overlong line

    int32_t c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return NULL;
    if (overflow) {
        if (c == '\n') { overflow = false; if (verbose) puts("Error: Line too long; ignored.\n"); }
        return NULL;
    }
    if (c != '\n' && nbuf >= sizeof(buf) - 1) { nbuf = 0; overflow = true; return NULL; }

    buf[nbuf++] = (char)c;
    if (buf[nbuf - 1] != '\n') return NULL;

    while (nbuf > 0 && (buf[nbuf - 1] == '\n' || buf[nbuf - 1] == '\r'))
        buf[--nbuf] = '\0';

    nbuf = 0;
    return buf;
}

static void handle_serial_line(stimjim_context_t *sc, char *line) {
    line = (char *)skip_ws(line);
    const char cmd = line[0];
    if (!cmd) return;
    const char *args = skip_ws(line + 1);

    switch (cmd) {
        case 'S': cmd_S(sc, args); return;
        case 'T': // intentional fall through
        case 'U': if (cmd_TU(sc, cmd, args)) return; break;
        case 'R': cmd_R(sc, args); return;
        case 'V': cmd_V(sc, args); return;
        case 'A': cmd_A(sc, args); return;
        case 'E': cmd_E(sc, args); return;
        case 'M': cmd_M(sc, args); return;
        case 'B': cmd_B(sc, args); break;
        case 'C': cmd_C(sc, args); break;
        case 'D': cmd_D(sc, args); return;
        case 'X': cmd_X(args); break;
        case 'W': cmd_W(args); break;
        default:  if (verbose) printf("Error: Unknown command: '%c'\n", cmd); break;
    }
    if (verbose) putchar('\n');
}

// X, T-1 or U-1
static bool is_stop_command(const char *line) {
    const char *p = skip_ws(line);
    const char cmd = *p;
    if (cmd != 'X' && cmd != 'T' && cmd != 'U') return false;
    p = skip_ws(p + 1);
    if (cmd == 'X') return *p == '\0';
    parse_skip(&p);
    char *ep;
    const long long idx = strtoll(p, &ep, 10);
    return ep != p && idx < 0 && *skip_ws(ep) == '\0';
}

static void cancel_deferred_stimuli(void) {
    static char line[BUF_LEN];
    for (uint n = queue_get_level(&q_deferred_commands); n > 0; n--) {
        queue_try_remove(&q_deferred_commands, line);
        const char cmd = *skip_ws(line);
        if (cmd != 'T' && cmd != 'U')
            queue_try_add(&q_deferred_commands, line);
    }
}

// While E/B/C wait on core1, which runs them only after a stimulus, a stop command
// takes effect right away so it can cancel the stimulus, but its reply is deferred
// with every other command. Input lines never contain '\n', so a deferred entry
// starting with one is a stop that has already been requested.
static void handle_user_input(stimjim_context_t *sc, bool waiting) {
    static char deferred_command[BUF_LEN];
    if (!waiting && queue_try_remove(&q_deferred_commands, deferred_command)) {
        stop_already_requested = deferred_command[0] == '\n';
        handle_serial_line(sc, deferred_command + stop_already_requested);
        stop_already_requested = false;
        return;
    }
    char *line = read_serial_line();
    if (!line) return;
    if (waiting) {
        if (is_stop_command(line)) {
            sio_hw->doorbell_out_set = DOORBELL_STOP;
            cancel_deferred_stimuli();
            static char reply[BUF_LEN];
            const char cmd = *skip_ws(line);
            strcpy(reply, cmd == 'X' ? "\nX" : cmd == 'T' ? "\nT-1" : "\nU-1");
            if (queue_try_add(&q_deferred_commands, reply))
                return;
            stop_already_requested = true;
        } else {
            if (!queue_try_add(&q_deferred_commands, line))
                if (verbose) puts("Error: Too many commands waiting; command ignored.\n");
            return;
        }
    }
    handle_serial_line(sc, line);
    stop_already_requested = false;
}

static void print_stimulus_telemetry(const stimjim_context_t *sc, const core1_stim_telemetry_t sr) {
    if (verbose && sr.dropped_summaries)
        printf("Warning: %" PRIu32 " train summaries were dropped (queue full).\n\n", sr.dropped_summaries);

    if (sr.n_pulses == 0) {
        if (verbose) puts("Warning: No pulses delivered (uninitialized pulse train).\n");
        return;
    }
    // Because of the inter-pulse stage, there is always at least one stage.
    // This checks if there are any user-defined stages.
    if (sr.n_stages == 1 && verbose) {
        printf("Info: No pulses delivered (pulse train has no stages).\n\n");
        return;
    }

    static const float scale[2] = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };

    if (!sr.cancelled)
        printf("Train complete. Delivered %" PRIu32 " pulses.", sr.n_pulses);
    else if (!verbose)
        return;
    else if (sr.stop_stage == sr.n_stages - 1)
        printf("Info: Train stopped after pulse %" PRIu32 ".", sr.n_pulses);
    else
        printf("Info: Train stopped during stage %u of pulse %" PRIu32 ".", sr.stop_stage, sr.n_pulses);
    stdio_putchar_raw('\n');
    printf("Current/Voltage by stage: \n           Ch0                Ch1 \n");
    // The last stage is the inter-pulse gap, which is grounded and not sampled
    for (uint8_t i = 0; i < sr.n_stages - 1; i++) {
        printf("Stage %d", i);
        for (uint8_t ch = 0; ch < 2; ch++) {
            const bool is_current = sr.output_mode[ch] & OUTPUT_MODE_CURRENT;
            long amplitude = 0;
            if ((sr.output_mode[ch] & OUTPUT_MODE_ACTIVE_MASK) && sr.delivered_stages[i]) {
                float adc_val = ((float)sr.measured_amplitudes[ch][i] / (float)sr.delivered_stages[i])
                              - stimjim_ctx_get_offsets(sc, ch).adc;
                amplitude = lroundf(adc_val * scale[is_current]);
            }
            printf("%6ld%s,          ", amplitude, (sr.output_mode[ch] & OUTPUT_MODE_VOLTAGE) ? "mV" : "uA");
        }
        putchar('\n');
    }
}

static uint32_t last_notice_train;

static void print_trigger_notice(const uint32_t notice) {
    print_start_notice(notice & 1 ? 'U' : 'T', (int)((notice >> 1) & 0x7Fu));
    last_notice_train = TRAIN_NUMBER(notice);
}

static bool train_precedes(const uint32_t a, const uint32_t b) {
    return ((a - b) & 0xFFFFFFu) >= 0x800000u;
}

static void handle_core1_messages(const stimjim_context_t *sc) {
    static bool have_notice;
    static uint32_t notice;
    if (!have_notice && multicore_fifo_rvalid()) {
        notice = multicore_fifo_pop_blocking();
        have_notice = true;
    }
    core1_stim_telemetry_t sr;
    const bool have_summary = queue_try_peek(&q_stimulus_telemetry, &sr);
    if (have_notice && !(have_summary && train_precedes(TRAIN_NUMBER(sr.notice), TRAIN_NUMBER(notice)))) {
        print_trigger_notice(notice);
        have_notice = false;
        return;
    }
    if (!have_summary) return;
    queue_try_remove(&q_stimulus_telemetry, &sr);
    if (sr.triggered && last_notice_train != TRAIN_NUMBER(sr.notice))
        print_trigger_notice(sr.notice);
    print_stimulus_telemetry(sc, sr);
}

static void keep_serving_while_waiting(stimjim_context_t *sc) {
    handle_user_input(sc, true);
    handle_core1_messages(sc);
}

int main(void) {
    if (!stdio_init_all())
        return EXIT_FAILURE;

    const uint64_t out_pins_mask =
        1ull << NLDAC_A | 1ull << OE0_A | 1ull << OE1_A | 1ull << LED_A | 1ull << CS_DAC_A | 1ull << CS_ADC_A |
        1ull << NLDAC_B | 1ull << OE0_B | 1ull << OE1_B | 1ull << LED_B | 1ull << CS_DAC_B | 1ull << CS_ADC_B;
    const uint64_t sync_pins_mask = 1ull << SYNC_A | 1ull << SYNC_B;

    gpio_set_dir_out_masked64(out_pins_mask | sync_pins_mask);
    gpio_set_mask64(out_pins_mask);
    gpio_set_function_masked64(out_pins_mask | sync_pins_mask, GPIO_FUNC_SIO);

    gpio_init(CHANNEL_IO_A);
    gpio_init(CHANNEL_IO_B);

    queue_init(&q_core1_cmd, sizeof(core1_cmd_t), 8);
    queue_init(&q_offsets_rx, sizeof(offsets_rx_t), 1);
    queue_init(&q_stimulus_telemetry, sizeof(core1_stim_telemetry_t), 128);
    queue_init(&q_adc_result, sizeof(adc_result_t), 5);
    queue_init(&q_deferred_commands, BUF_LEN, 8);

    multicore_doorbell_claim(DOORBELL_STOP_NUM, 0b11);
    multicore_doorbell_claim(DOORBELL_TRAIN_RUNNING_NUM, 0b11);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
    multicore_launch_core1(main_core1);
    if (multicore_fifo_pop_blocking() != CORE_HANDSHAKE_MESSAGE)
        return EXIT_FAILURE;

    static stimjim_context_t stimjim_ctx;
    stimjim_ctx_init(&stimjim_ctx, &q_core1_cmd, &q_offsets_rx);

    sleep_ms(3000);
    printf("Booting StimJim %s\n", FIRMWARE_VERSION);
    puts("Initializing triggers inputs...");
    print_offsets(&stimjim_ctx);
    puts("Ready to go!\n\n");

    while (true) {
        handle_user_input(&stimjim_ctx, false);
        handle_core1_messages(&stimjim_ctx);
    }
}
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

static void handle_user_input(stimjim_context_t *sc, bool waiting);

// Queue a command for core1. While the queue is full (core1 is busy with a
// stimulus), keep handling stop commands so they can still cancel it.
static void send_core1_cmd(stimjim_context_t *sc, const core1_cmd_t *cmd) {
    while (!queue_try_add(&q_core1_cmd, cmd))
        handle_user_input(sc, true);
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
    bool ok = (ep != *p) && (*next == '\0' || is_delimiter(*next)) && (val >= min) && (val <= max);
    if (ok) {
        *out = val;
        *p = next;
        return true;
    }

    if (verbose) {
        const char *end = *p;
        while (*end && !is_delimiter(*end)) end++;
        printf("Error: Invalid %s: '%.*s' (must be %lld..%lld)\n", name, (int)(end - *p), *p, min, max);
    } else if (ep != *p && (*next == '\0' || is_delimiter(*next))) {
        if (!strcmp(name, "pulse train index") && val >= MAX_PULSETRAINS)
            puts("Invalid PulseTrain index.");
        else if (!strcmp(name, "dac_code"))
            printf("%lld is out of range.\n", val);
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

static void set_trigger_pulsetrain(stimjim_context_t *sc, const uint8_t ch, const int8_t idx) {
    core1_cmd_t cmd = {
        .type = CORE1_CMD_TRIGGER_CONFIG,
        .trigger_config = { .ch = ch, .pt = stimjim_ctx_convert_pt(sc, idx) },
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

static void stop_stimulus(const char cmd_char) {
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
    int32_t idx = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &idx, "pulse train index", INT32_MIN, MAX_PULSETRAINS - 1)
     || check_for_extra_arguments(p))
        return false;
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
    print_start_notice(cmd_char, idx);
    return true;
}

static void cmd_R(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, idx = 0, dir = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &idx, "pulse train index", INT32_MIN, MAX_PULSETRAINS - 1))
        return;
    if ((*p == ',' && !try_parse_i32_in_range(&p, &dir, "trigger/sync", 0, 1)) || check_for_extra_arguments(p))
        return;

    const bool sync = dir || idx < 0;
    if (!sync)
        set_trigger_pulsetrain(sc, (uint8_t)ch, (int8_t)idx);
    stimjim_ctx_set_channel_io(sc, (uint8_t)ch, sync ? -1 : (int8_t)idx, sync ? GPIO_OUT : GPIO_IN);

    if (sync)
        printf("Detaching interrupt to IN%" PRId32 "\nProgramming IN%" PRId32 " as output that indicates activity on output %" PRId32 "\n",
               ch, ch, ch);
    else
        printf("Attaching interrupt to IN%" PRId32 " to run PulseTrain[%" PRId32 "]\n", ch, idx);
}

static bool is_train_running(void) {
    return (sio_hw->doorbell_in_set & DOORBELL_TRAIN_RUNNING) || queue_get_level(&q_core1_cmd);
}

static void cmd_V(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, mv = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &mv, "mV", INT32_MIN, INT32_MAX)
     || check_for_extra_arguments(p))
        return;

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
     || check_for_extra_arguments(p))
        return;

    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_DAC_SET, .ch = (uint8_t)ch, .dac_code = code } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);
    printf("Set channel %" PRId32 " to amplitude %" PRId32 "\n", ch, code);
    if ((abs(code) - INT8_MIN) * MILLIVOLTS_PER_DAC > MAX_MEASURABLE_MILLIVOLTS)
        print_high_voltage_warning();
}

static void cmd_E(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, line = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &line, "line", 0, 1)
     || check_for_extra_arguments(p))
        return;

    core1_cmd_t cmd = { .type = CORE1_CMD_MANUAL,
        .manual = { .type = MANUAL_CMD_ADC_READ, .ch = (uint8_t)ch, .line = (bool)line } };
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    send_core1_cmd(sc, &cmd);

    int16_t val;
    while (!queue_try_remove(&q_adc_result, &val))
        handle_user_input(sc, true);

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    static const char  units[2][3] = { "mV", "uA" };
    static const float scale[2]    = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
    int32_t real = lroundf(((float)val - off.adc) * scale[line]);
    printf("Read value: %d (%" PRId32 "%s)\n", val, real, units[line]);
}

static void cmd_M(stimjim_context_t *sc, const char *args) {
    int32_t ch = 0, mode = 0;
    const char *p = args;
    if (!try_parse_i32_in_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_in_range(&p, &mode, "output mode", 0, 3)
     || check_for_extra_arguments(p))
        return;
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
        handle_user_input(sc, true);
    while (!stimjim_ctx_try_receive_offsets(sc))
        handle_user_input(sc, true);

    set_trigger_pulsetrains_w_new_offsets(sc);
    if (verbose) puts("Info: Offsets updated");
}

static void cmd_C(stimjim_context_t *sc, const char *args) {
    if (check_for_extra_arguments(args)) return;
    if (verbose && is_train_running())
        puts(RUNS_AFTER_TRAIN);
    if (verbose) puts("Info: Updating ADC, current and voltage offsets...");
    const offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC | OFFSETS_TX_CALIBRATE_CURRENT | OFFSETS_TX_CALIBRATE_VOLTAGE,
    };
    while (!stimjim_ctx_try_request_offsets(sc, &offsets_calibration))
        handle_user_input(sc, true);
    while (!stimjim_ctx_try_receive_offsets(sc))
        handle_user_input(sc, true);

    set_trigger_pulsetrains_w_new_offsets(sc);
    if (verbose) puts("Info: Offsets updated");
}

static void cmd_D(const stimjim_context_t *sc, const char *args) {
    if (check_for_extra_arguments(args)) return;
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
    if (*p == 'X') return true;
    if (*p != 'T' && *p != 'U') return false;
    p = skip_ws(p + 1);
    return p[0] == '-' && p[1] == '1' && *skip_ws(p + 2) == '\0';
}

// While E/B/C wait on core1, which runs them only after a stimulus, handle stop
// commands right away so they can cancel the stimulus, and defer any other command.
static void handle_user_input(stimjim_context_t *sc, bool waiting) {
    static char deferred_command[BUF_LEN];
    if (!waiting && queue_try_remove(&q_deferred_commands, deferred_command)) {
        handle_serial_line(sc, deferred_command);
        return;
    }
    char *line = read_serial_line();
    if (!line) return;
    if (waiting && !is_stop_command(line)) {
        if (!queue_try_add(&q_deferred_commands, line))
            if (verbose) puts("Error: Too many commands waiting; command ignored.\n");
        return;
    }
    handle_serial_line(sc, line);
}

static void handle_stimulus_telemetry(const stimjim_context_t *sc) {
    core1_stim_telemetry_t sr;
    if (!queue_try_remove(&q_stimulus_telemetry, &sr)) return;

    if (verbose && sr.dropped_summaries)
        printf("Warning: %" PRIu32 " train summaries were dropped (queue full).\n\n", sr.dropped_summaries);
    if (verbose && sr.dropped_trigger_notices)
        printf("Warning: %" PRIu32 " stimulus start messages were dropped (queue full).\n\n", sr.dropped_trigger_notices);

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

static void handle_triggered_stimuli(const stimjim_context_t *sc) {
    if (!multicore_fifo_rvalid()) return;
    const uint32_t ch = multicore_fifo_pop_blocking();
    print_start_notice(ch ? 'U' : 'T', stimjim_ctx_get_channel_io(sc, ch).idx);
}

int main(void) {
    if (!stdio_init_all())
        return EXIT_FAILURE;

    const uint64_t out_pins_mask =
        1LL << NLDAC_A | 1LL << OE0_A | 1LL << OE1_A | 1LL << LED_A | 1LL << CSA_A | 1LL << CSB_A |
        1LL << NLDAC_B | 1LL << OE0_B | 1LL << OE1_B | 1LL << LED_B | 1LL << CSA_B | 1LL << CSB_B;
    const uint64_t sync_pins_mask = 1LL << SYNC_A | 1LL << SYNC_B;

    gpio_set_dir_out_masked64(out_pins_mask | sync_pins_mask);
    gpio_set_mask64(out_pins_mask);
    gpio_set_function_masked64(out_pins_mask | sync_pins_mask, GPIO_FUNC_SIO);

    gpio_init(CHANNEL_IO_A);
    gpio_init(CHANNEL_IO_B);

    queue_init(&q_core1_cmd, sizeof(core1_cmd_t), 8);
    queue_init(&q_offsets_rx, sizeof(offsets_t) * 2, 1);
    queue_init(&q_stimulus_telemetry, sizeof(core1_stim_telemetry_t), 128);
    queue_init(&q_adc_result, sizeof(int16_t), 5);
    queue_init(&q_deferred_commands, BUF_LEN, 8);

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
        handle_triggered_stimuli(&stimjim_ctx);
        handle_stimulus_telemetry(&stimjim_ctx);
    }
}
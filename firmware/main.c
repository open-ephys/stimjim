#define FIRMWARE_VERSION "v0.0.0"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <tusb.h>

#include "core1.h"
#include "stimjim_context.h"

#define BUF_LEN 1024

queue_t q_offsets_tx, q_offsets_rx, q_stimulus_result, q_manual_cmd, q_adc_result;
queue_t q_stimulus_cmd, q_stimulus_trigger[2];

// These parser functions help convert strings to numbers. They account for if a
// user tries to set a value that exceeds the range of a 32-bit type.

static bool is_delimiter(const char c) {
    return c == ',' || c == ';' || c == ' ';
}

static void parse_skip(const char **p) {
    if (**p == ',')
        (*p)++;
    while (**p == ' ')
        (*p)++;
}

static bool parse_i32_validate_range(const char **p, int32_t *out, const int32_t min, const int32_t max) {
    parse_skip(p);
    char *ep;
    long long val = strtoll(*p, &ep, 10);
    const char *end = ep;
    while (*end && !is_delimiter(*end)) end++;
    if (ep == *p || *ep != *end || val < min || val > max)
        return false;
    *out = (int32_t)val;
    *p = ep;
    return true;
}

static void print_parse_error(const char *p, const char *name, const int32_t min, const int32_t max) {
    const char *end = p;
    while (*end && !is_delimiter(*end)) end++;
    printf("Invalid %s: '%.*s' (must be %d..%d)\n\n", name, (int)(end - p), p, min, max);
}

static bool try_parse_i32_with_range(const char **p, int32_t *out, const char *name, const int32_t min, const int32_t max) {
    if (!parse_i32_validate_range(p, out, min, max)) {
        print_parse_error(*p, name, min, max);
        return false;
    }
    return true;
}

static void print_offsets(const stimjim_context_t *sc) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        offsets_t off = stimjim_ctx_get_offsets(sc, ch);
        printf("Channel %u offsets: adc=%.4f voltage=%d current=%d\n",
               ch, (double)off.adc, off.voltage, off.current);
    }
    putchar('\n');
}

static void refresh_trigger_pulsetrains(const stimjim_context_t *sc, const int32_t pt_idx) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        channel_io_t io = stimjim_ctx_get_channel_io(sc, ch);
        if (io.dir != GPIO_IN) continue;
        if (io.idx < 0) continue;
        if (pt_idx >= 0 && io.idx != pt_idx) continue;

        pulsetrain_t pt_raw = stimjim_ctx_get_pulsetrain(sc, (uint8_t)io.idx);
        if (!(pt_raw.output_mode[ch] & OUTPUT_MODE_ACTIVE) || pt_raw.n_pulses == 0) continue;

        pulsetrain_t pt_conv = stimjim_ctx_convert_pt(sc, (uint8_t)io.idx);
        queue_add_blocking(&q_stimulus_trigger[ch], &pt_conv);
    }
}

static void cmd_S(stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] =
        "S usage: S<idx>,<mode0>,<mode1>,<period_us>,<total_dur_us>;"
        " <amp0>,<amp1>,<stage_dur_us>; ...\n";

    int32_t n, mode0, mode1;
    int32_t duration;
    pulsetrain_t pt = { 0 };

    const char *p = args;
    if (!try_parse_i32_with_range(&p, &n, "idx", 0, MAX_PULSETRAINS - 1)
     || !try_parse_i32_with_range(&p, &mode0, "mode0", 0, 3)
     || !try_parse_i32_with_range(&p, &mode1, "mode1", 0, 3)
     || !try_parse_i32_with_range(&p, &pt.period, "period_us",  1, INT32_MAX)
     || !try_parse_i32_with_range(&p, &duration,  "total_dur_us", 1, INT32_MAX))
    { puts(cmd_usage); return; }

    pt.output_mode[0] = 1 << mode0;
    pt.output_mode[1] = 1 << mode1;

    if (*p != ';' && *p != '\0' && *p != '\r' && *p != '\n') { puts(cmd_usage); return; }
    int32_t amp_limit[2];
    for (uint8_t ch = 0; ch < 2; ch++) {
        if (pt.output_mode[ch] & OUTPUT_MODE_VOLTAGE)      amp_limit[ch] = 10000;
        else if (pt.output_mode[ch] & OUTPUT_MODE_CURRENT) amp_limit[ch] = 3333;
        else                                               amp_limit[ch] = 0;
    }

    while (*p == ';' || *p == ' ') {
        if (*p == ';') p++;
        while (*p == ' ') p++;
        if (!*p || pt.n_stages >= MAX_STAGES) break;
        char field_name[32];
        snprintf(field_name, sizeof(field_name), "amp0 stage[%d]", pt.n_stages);
        if (!try_parse_i32_with_range(&p, &pt.stage_amplitude[0][pt.n_stages], field_name, -amp_limit[0], amp_limit[0])) return;
        snprintf(field_name, sizeof(field_name), "amp1 stage[%d]", pt.n_stages);
        if (!try_parse_i32_with_range(&p, &pt.stage_amplitude[1][pt.n_stages], field_name, -amp_limit[1], amp_limit[1])) return;
        snprintf(field_name, sizeof(field_name), "stage_dur_us stage[%d]", pt.n_stages);
        if (!try_parse_i32_with_range(&p, &pt.stage_duration[pt.n_stages], field_name, 1, UINT16_MAX)) return;
        pt.n_stages++;
    }

    uint64_t stage_sum = 0;
    for (uint8_t i = 0; i < pt.n_stages; i++)
        stage_sum += pt.stage_duration[i];
    if (stage_sum > pt.period) {
        puts("Invalid pulse train: summed stage durations exceed period.\n");
        return;
    }

    pt.n_pulses = duration / pt.period;

    bool short_pulse = (pt.period - stage_sum) < 20u;
    for (uint8_t i = 0; !short_pulse && i < pt.n_stages; i++)
        short_pulse = (pt.stage_duration[i] < 20u);
    if (short_pulse)
        puts("Warning: <20us stage or inter-pulse gap detected. Desired pulse timings are not guaranteed.");

    stimjim_ctx_set_pulsetrain(sc, (uint8_t)n, &pt);
    refresh_trigger_pulsetrains(sc, n);

    printf("PulseTrain[%d]: mode[%d,%d], period=%u us, pulses=%u, %d stages\n",
           n, pt.output_mode[0], pt.output_mode[1], pt.period, pt.n_pulses, pt.n_stages);
    for (int32_t i = 0; i < pt.n_stages; i++)
        printf("  Stage %d: amp[%d,%d], dur=%u us\n",
               i, pt.stage_amplitude[0][i], pt.stage_amplitude[1][i], pt.stage_duration[i]);
    putchar('\n');
}

static void cmd_TU(const stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "T/U usage: T<idx> or U<idx>\n";

    int32_t idx = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &idx, "pulse train index", 0, MAX_PULSETRAINS - 1))
    { puts(cmd_usage); return; }
    pulsetrain_t pt = stimjim_ctx_convert_pt(sc, (uint8_t)idx);
    queue_add_blocking(&q_stimulus_cmd, &pt);
    sio_hw->doorbell_out_set = 1 << 2;
    printf("Started PulseTrain[%d].\n\n", idx);
}

static void cmd_R(stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "R usage: R<ch>,<pt_idx|-1>[,<0=trigger|1=sync>]\n";

    int32_t ch = 0, idx = 0, dir = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_with_range(&p, &idx, "pulse train index", -1, MAX_PULSETRAINS - 1))
    { puts(cmd_usage); return; }
    if (*p == ',' && !try_parse_i32_with_range(&p, &dir, "trigger/sync", 0, 1))
    { puts(cmd_usage); return; }

    stimjim_ctx_set_channel_io(sc, (uint8_t)ch, (int8_t)idx, (bool)dir);

    if (idx == -1) {
        printf("IN%d -> Disabled\n\n", ch);
    } 
    else if (dir) {
        printf("IN%d -> Sync signal\n\n", ch);
    } 
    else {
        pulsetrain_t pt = stimjim_ctx_get_pulsetrain(sc, (uint8_t)idx);
        if ((pt.output_mode[ch] & OUTPUT_MODE_ACTIVE) && pt.n_pulses > 0) {
            pulsetrain_t pt = stimjim_ctx_convert_pt(sc, (uint8_t)idx);
            queue_add_blocking(&q_stimulus_trigger[ch], &pt);
        }
        printf("IN%d -> PulseTrain[%d] trigger\n\n", ch, idx);
    }
}

static void cmd_V(const stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "V usage: V<ch>,<mV>\n";

    int32_t ch = 0, mv = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_with_range(&p, &mv, "mV", -10000, 10000))
    { puts(cmd_usage); return; }

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    int32_t code = (mv / MILLIVOLTS_PER_DAC) + off.voltage;
    if (code < INT16_MIN) code = INT16_MIN;
    if (code > INT16_MAX) code = INT16_MAX;

    manual_cmd_t cmd = { .type = MANUAL_CMD_DAC_SET, .ch = (uint8_t)ch, .dac_code = (int16_t)code };
    queue_add_blocking(&q_manual_cmd, &cmd);
    sio_hw->doorbell_out_set = 1 << 2;
    printf("Set channel %d to %d mV (DAC code %d).\n\n", ch, mv, code);
}

static void cmd_A(const char *args) {
    static const char cmd_usage[] = "A usage: A<ch>,<dac_code>\n";

    int32_t ch = 0;
    int32_t code = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_with_range(&p, &code, "dac_code", INT16_MIN, INT16_MAX))
    { puts(cmd_usage); return; }

    manual_cmd_t cmd = { .type = MANUAL_CMD_DAC_SET, .ch = (uint8_t)ch, .dac_code = code };
    queue_add_blocking(&q_manual_cmd, &cmd);
    sio_hw->doorbell_out_set = 1 << 2;
    printf("Set channel %d to DAC code %d.\n\n", ch, code);
}

static void cmd_E(const stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "E usage: E<ch>,<line>\n";

    int32_t ch = 0, line = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_with_range(&p, &line, "line", 0, 1))
    { puts(cmd_usage); return; }

    manual_cmd_t cmd = { .type = MANUAL_CMD_ADC_READ, .ch = (uint8_t)ch, .line = (bool)line };
    queue_add_blocking(&q_manual_cmd, &cmd);
    sio_hw->doorbell_out_set = 1 << 2;

    int16_t val;
    queue_remove_blocking(&q_adc_result, &val);

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    static const char  units[2][3] = { "mV", "uA" };
    static const float scale[2]    = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
    int32_t real = lroundf(((float)val - off.adc) * scale[line]);
    printf("Read value: %d (%d %s)\n\n", val, real, units[line]);
}

static void cmd_M(const stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] =
        "M usage: M<ch>,<mode>  (mode: 0=voltage 1=current 2=float 3=gnd)\n";

    int32_t ch = 0, mode = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1)
     || !try_parse_i32_with_range(&p, &mode, "output mode", 0, 3))
    { puts(cmd_usage); return; }
    set_output_mode((uint8_t)ch, 1 << mode);
    printf("Ch%d output mode set to %d\n\n", ch, mode);
}

static void cmd_B(stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "B usage: B\n";

    if (*args != '\0') { puts(cmd_usage); return; }
    printf("Updating ADC offsets...\n");
    const offsets_tx_t offsets_calibration = {
        .offset_tx_type = OFFSETS_TX_CALIBRATE_ADC | OFFSETS_TX_CALIBRATE_VOLTAGE | OFFSETS_TX_CALIBRATE_CURRENT,
    };
    stimjim_ctx_set_offsets(sc, &offsets_calibration);
    refresh_trigger_pulsetrains(sc, -1);
    printf("Offsets updated\n\n");
}

static void cmd_C(stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "C usage: C\n";

    if (*args != '\0') { puts(cmd_usage); return; }
    printf("Updating current and voltage offsets...\n");
    offsets_t offsets[2] = { stimjim_ctx_get_offsets(sc, 0), stimjim_ctx_get_offsets(sc, 1) };
    const offsets_tx_t offsets_calibration = { 
        .offset_tx_type = OFFSETS_TX_CALIBRATE_CURRENT | OFFSETS_TX_CALIBRATE_VOLTAGE,
        .adc = { offsets[0].adc, offsets[1].adc } 
    };
    stimjim_ctx_set_offsets(sc, &offsets_calibration);
    refresh_trigger_pulsetrains(sc, -1);
    printf("Offsets updated\n\n");
}

static void cmd_D(const stimjim_context_t *sc, const char *args) {
    static const char cmd_usage[] = "D usage: D\n";

    if (*args != '\0') { puts(cmd_usage); return; }
    print_offsets(sc);
}

static void cmd_X(const char *args) {
    static const char cmd_usage[] = "X usage: X[<ch>]\n";

    if (*args == '\0') {
        sio_hw->doorbell_out_set = (1u << 0) | (1u << 1);
        puts("Both channels cancelled\n");
        return;
    }
    int32_t ch = 0;
    const char *p = args;
    if (!try_parse_i32_with_range(&p, &ch, "channel", 0, 1))
    { puts(cmd_usage); return; }
    sio_hw->doorbell_out_set = (1u << ch);
    printf("Channel %d cancelled\n\n", ch);
}

static void process_user_cmds(stimjim_context_t *sc) {
    static char buf[BUF_LEN];
    static uint16_t nbuf = 0;

    int32_t c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    if (nbuf >= sizeof(buf) - 1) { nbuf = 0; return; }

    buf[nbuf++] = (char)c;
    if (buf[nbuf - 1] != '\n') return;

    while (nbuf > 0 && (buf[nbuf - 1] == '\n' || buf[nbuf - 1] == '\r'))
        buf[--nbuf] = '\0';

    const char cmd = buf[0];
    char *args = buf + 1;
    nbuf = 0;

    switch (cmd) {
        case 'S': cmd_S(sc, args); break;
        case 'T': // intentional fall through
        case 'U': cmd_TU(sc, args); break;
        case 'R': cmd_R(sc, args); break;
        case 'V': cmd_V(sc, args); break;
        case 'A': cmd_A(args); break;
        case 'E': cmd_E(sc, args); break;
        case 'M': cmd_M(sc, args); break;
        case 'B': cmd_B(sc, args); break;
        case 'C': cmd_C(sc, args); break;
        case 'D': cmd_D(sc, args); break;
        case 'X': cmd_X(args); break;
        default:  printf("Unknown command: '%c'\n\n", cmd); break;
    }
}

static void process_core1_reports(const stimjim_context_t *sc) {
    stimulus_result_t sr;
    if (!queue_try_remove(&q_stimulus_result, &sr)) return;

    if (sr.rejected) {
        printf("Stimulus rejected: channel(s) busy.\n\n");
        return;
    }

    static const char units[2][3] = { "mV", "uA" };
    static const float scale[2] = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };

    bool is_current = sr.output_mode & OUTPUT_MODE_CURRENT;
    printf("Channel %d pulse train terminated. Delivered:\n", sr.ch);
    float adc_offset = stimjim_ctx_get_offsets(sc, sr.ch).adc;
    for (uint8_t i = 0; i < sr.n_stages; i++) {
        if (sr.delivered_stages[i]) {
            float adc_val = ((float)sr.measured_amplitudes[i] / (float)sr.delivered_stages[i]) - adc_offset;
            sr.measured_amplitudes[i] = lroundf(adc_val * scale[is_current]);
        }
        if (i < sr.n_stages - 1)
            printf("  %dx Stage %d: %6ld %s\n", sr.delivered_stages[i], i, sr.measured_amplitudes[i], units[is_current]);
        else
            printf("  %dx Inter-pulse gap: %6ld %s\n\n", sr.delivered_stages[i], sr.measured_amplitudes[i], units[is_current]);
    }
}

int main(void) {
    if (!stdio_init_all())
        return EXIT_FAILURE;

    const uint64_t out_pins_mask =
        1LL << NLDAC_A | 1LL << OE0_A | 1LL << OE1_A | 1LL << LED_A | 1LL << CSA_A | 1LL << CSB_A |
        1LL << NLDAC_B | 1LL << OE0_B | 1LL << OE1_B | 1LL << LED_B | 1LL << CSA_B | 1LL << CSB_B;

    gpio_set_dir_out_masked64(out_pins_mask);
    gpio_set_mask64(out_pins_mask);
    gpio_set_function_masked64(out_pins_mask, GPIO_FUNC_SIO);

    gpio_init(CHANNEL_IO_A);
    gpio_init(CHANNEL_IO_B);

    queue_init(&q_offsets_tx, sizeof(offsets_tx_t), 1);
    queue_init(&q_offsets_rx, sizeof(offsets_t) * 2, 1);
    queue_init(&q_stimulus_cmd, sizeof(pulsetrain_t), 2);
    queue_init(&q_stimulus_trigger[0], sizeof(pulsetrain_t), 5);
    queue_init(&q_stimulus_trigger[1], sizeof(pulsetrain_t), 5);
    queue_init(&q_stimulus_result, sizeof(stimulus_result_t), 32);
    queue_init(&q_manual_cmd, sizeof(manual_cmd_t), 5); 
    queue_init(&q_adc_result, sizeof(int16_t), 5);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
    multicore_launch_core1(main_core1);
    if (multicore_fifo_pop_blocking() != 0xDEADBEEF)
        return EXIT_FAILURE;

    stimjim_context_t *stimjim_ctx = stimjim_ctx_init(&q_offsets_tx, &q_offsets_rx);

    while (!tud_cdc_connected()) sleep_ms(100);

    printf("StimJim %s\n\n", FIRMWARE_VERSION);
    print_offsets(stimjim_ctx);

    while (true) {
        process_user_cmds(stimjim_ctx);
        process_core1_reports(stimjim_ctx);
    }
}
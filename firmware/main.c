#define FIRMWARE_VERSION "v0.0.0"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <tusb.h>

#include "core1.h"
#include "stimjim_context.h"

#define BUF_LEN 1024

queue_t q_tx_offsets, q_rx_offsets, q_stim_result, q_manual_cmd, q_adc_result;
queue_t q_stimulus_cmd, q_stimulus_trigger[2];

// These parser functions help convert strings to numbers. They account for if a
// user tries to set a value that exceeds the range of a 32-bit type.

typedef struct {
    const char *p;
    bool ok;
} parser_t;

static parser_t parser_init(const char *s) {
    return (parser_t){ .p = s, .ok = true };
}

static void parse_skip(parser_t *ps) {
    while (*ps->p == ',' || *ps->p == ';' || *ps->p == ' ')
        ps->p++;
}

static void parse_i32(parser_t *ps, int32_t *out, const char *name, int32_t min, int32_t max) {
    if (!ps->ok) return;
    parse_skip(ps);
    errno = 0;
    char *ep;
    long val = strtol(ps->p, &ep, 10);
    if (ep == ps->p || errno == ERANGE || val < min || val > max) {
        printf("Invalid %s: %ld (must be %d..%d)\n\n", name, val, min, max);
        ps->ok = false;
        return;
    }
    *out = (int32_t)val;
    ps->p = ep;
}

static void parse_u32(parser_t *ps, uint32_t *out, const char *name, uint32_t min, uint32_t max) {
    if (!ps->ok) return;
    parse_skip(ps);
    if (*ps->p == '-') {
        printf("Invalid %s: must not be negative\n\n", name);
        ps->ok = false;
        return;
    }
    errno = 0;
    char *ep;
    unsigned long val = strtoul(ps->p, &ep, 10);
    if (ep == ps->p || errno == ERANGE || val > max) {
        printf("Invalid %s: %lu (must be %u..%u)\n\n", name, val, min, max);
        ps->ok = false;
        return;
    }
    *out = (uint32_t)val;
    ps->p = ep;
}

static void print_offsets(stimjim_context_t *sc) {
    for (uint8_t ch = 0; ch < 2; ch++) {
        offsets_t off = stimjim_ctx_get_offsets(sc, ch);
        printf("Channel %u offsets: adc=%.4f voltage=%d current=%d\n",
               ch, (double)off.adc, off.voltage, off.current);
    }
    putchar('\n');
}

static void cmd_S(stimjim_context_t *sc, char *args) {
    static const char usage[] =
        "S usage: S<idx>,<mode0>,<mode1>,<period_us>,<total_dur_us>;"
        " <amp0>,<amp1>,<stage_dur_us>; ...\n";

    int32_t n, mode0, mode1;
    uint32_t duration;
    pulsetrain_t pt = { 0 };

    parser_t ps = parser_init(args);
    parse_i32(&ps, &n, "pulse train index", 0, MAX_PULSETRAINS - 1);
    parse_i32(&ps, &mode0, "output mode", 0, 3);
    parse_i32(&ps, &mode1, "output mode", 0, 3);
    parse_u32(&ps, &pt.period, "period",  1, UINT32_MAX);
    parse_u32(&ps, &duration,  "duration", 1, UINT32_MAX);
    if (!ps.ok) { puts(usage); return; }

    pt.output_mode[0] = (uint8_t)mode0;
    pt.output_mode[1] = (uint8_t)mode1;

    parse_skip(&ps);
    while (ps.ok && *ps.p && pt.n_stages < MAX_STAGES) {
        parse_i32(&ps, &pt.stage_amplitude[0][pt.n_stages], "amplitude[0]", -150000, 150000);
        parse_i32(&ps, &pt.stage_amplitude[1][pt.n_stages], "amplitude[1]", -150000, 150000);
        parse_u32(&ps, &pt.stage_duration[pt.n_stages], "stage_dur", 1, UINT16_MAX);
        if (!ps.ok) return;
        pt.n_stages++;
        parse_skip(&ps);
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

    printf("PulseTrain[%d]: mode[%d,%d], period=%u us, pulses=%u, %d stages\n",
           n, pt.output_mode[0], pt.output_mode[1], pt.period, pt.n_pulses, pt.n_stages);
    for (int32_t i = 0; i < pt.n_stages; i++)
        printf("  Stage %d: amp[%d,%d], dur=%u us\n",
               i, pt.stage_amplitude[0][i], pt.stage_amplitude[1][i], pt.stage_duration[i]);
    putchar('\n');
}

static void cmd_TU(stimjim_context_t *sc, char *args) {
    int32_t idx = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &idx, "pulse train index", 0, MAX_PULSETRAINS - 1);
    if (!ps.ok) return;
    pulsetrain_t pt = convert_pt((uint8_t)idx);
    queue_add_blocking(&q_stimulus_cmd, &pt);
    printf("Started PulseTrain[%d].\n\n", idx);
}

static void cmd_R(stimjim_context_t *sc, char *args) {
    int32_t ch = 0, idx = 0, out = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch,  "channel",           0,  1);
    parse_i32(&ps, &idx, "pulse train index", -1, MAX_PULSETRAINS - 1);
    if (*ps.p == ',')
        parse_i32(&ps, &out, "trigger/sync", 0, 1);
    if (!ps.ok) { puts("R usage: R<ch>,<pt_idx|-1>[,<0=trigger|1=sync>]\n"); return; }

    stimjim_ctx_set_channel_io(sc, (bool)ch, (int8_t)idx, (bool)out);

    if (idx == -1) {
        printf("IN%d -> Disabled\n\n", ch);
    } 
    else if (out) {
        printf("IN%d -> Sync signal\n\n", ch);
    } 
    else {
        pulsetrain_t pt = stimjim_ctx_get_pulsetrain(sc, (uint8_t)idx);
        if (pt.output_mode[ch] < OUTPUT_MODE_FLOAT && pt.n_pulses > 0) {
            pulsetrain_t pt = convert_pt((uint8_t)idx);
            queue_add_blocking(&q_stimulus_trigger[ch], &pt);
        }
        printf("IN%d -> PulseTrain[%d] trigger\n\n", ch, idx);
    }
}

static void cmd_V(stimjim_context_t *sc, char *args) {
    int32_t ch = 0, mv = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch, "channel", 0, 1);
    parse_i32(&ps, &mv, "mV", INT32_MIN, INT32_MAX);
    if (!ps.ok) { puts("V usage: V<ch>,<mV>\n"); return; }

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    int32_t code = (mv / MILLIVOLTS_PER_DAC) + off.voltage;
    if (code < INT16_MIN || code > INT16_MAX) {
        printf("Invalid mV: computed DAC code %d out of int16 range [%d, %d]\n\n",
               code, INT16_MIN, INT16_MAX);
        return;
    }

    manual_cmd_t cmd = { .type = MANUAL_CMD_DAC_SET, .ch = (bool)ch, .dac_code = (int16_t)code };
    queue_add_blocking(&q_manual_cmd, &cmd);
    printf("Set channel %d to %d mV (DAC code %d).\n\n", ch, mv, code);
}

static void cmd_A(char *args) {
    int32_t ch = 0;
    int32_t code = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch, "channel",  0, 1);
    parse_i32(&ps, &code, "dac_code", INT16_MIN, INT16_MAX);
    if (!ps.ok) { puts("A usage: A<ch>,<dac_code>\n"); return; }

    manual_cmd_t cmd = { .type = MANUAL_CMD_DAC_SET, .ch = (bool)ch, .dac_code = code };
    queue_add_blocking(&q_manual_cmd, &cmd);
    printf("Set channel %d to DAC code %d.\n\n", ch, code);
}

static void cmd_E(stimjim_context_t *sc, char *args) {
    int32_t ch = 0, line = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch,   "channel", 0, 1);
    parse_i32(&ps, &line, "line",    0, 1);
    if (!ps.ok) { puts("E usage: E<ch>,<line>\n"); return; }

    manual_cmd_t cmd = { .type = MANUAL_CMD_ADC_READ, .ch = (bool)ch, .line = (bool)line };
    queue_add_blocking(&q_manual_cmd, &cmd);

    int16_t val;
    queue_remove_blocking(&q_adc_result, &val);

    offsets_t off = stimjim_ctx_get_offsets(sc, ch);
    static const char  units[2][3] = { "mV", "uA" };
    static const float scale[2]    = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
    int32_t real = lroundf(((float)val - off.adc) * scale[line]);
    printf("Read value: %d (%d %s)\n\n", val, real, units[line]);
}

static void cmd_M(stimjim_context_t *sc, char *args) {
    int32_t ch = 0, mode = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch,   "channel",     0, 1);
    parse_i32(&ps, &mode, "output mode", 0, 3);
    if (!ps.ok) { puts("M usage: M<ch>,<mode>  (mode: 0=voltage 1=current 2=float 3=gnd)\n"); return; }
    set_output_mode((bool)ch, (uint8_t)mode);
    printf("Ch%d output mode set to %d\n\n", ch, mode);
}

static void cmd_C(stimjim_context_t *sc, char *args) {
    if (*args != '\0') { puts("C usage: C\n"); return; }
    printf("Updating offsets...\n");
    stimjim_ctx_set_offsets(sc, &(bool){true});
    printf("Offsets updated\n\n");
}

static void cmd_D(stimjim_context_t *sc, char *args) {
    if (*args != '\0') { puts("D usage: D\n"); return; }
    print_offsets(sc);
}

static void cmd_X(char *args) {
    if (*args == '\0') {
        sio_hw->doorbell_out_set = (1u << 0) | (1u << 1);
        puts("Both channels cancelled\n");
        return;
    }
    int32_t ch = 0;
    parser_t ps = parser_init(args);
    parse_i32(&ps, &ch, "channel", 0, 1);
    if (!ps.ok) return;
    sio_hw->doorbell_out_set = (1u << ch);
    printf("Channel %d cancelled\n\n", ch);
}

static void process_cmds(stimjim_context_t *sc) {
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
        case 'C': cmd_C(sc, args); break;
        case 'D': cmd_D(sc, args); break;
        case 'X': cmd_X(args); break;
        default:  printf("Unknown command: '%c'\n\n", cmd); break;
    }
}

static void process_completed_pulsetrains(stimjim_context_t *sc) {
    stim_result_t sr;
    if (!queue_try_remove(&q_stim_result, &sr)) return;

    static const char units[2][3] = { "mV", "uA" };
    static const float scale[2] = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };

    printf("Channel %d pulse train terminated. Delivered:\n", sr.ch);
    float adc_offset = stimjim_ctx_get_offsets(sc, sr.ch).adc;
    for (uint8_t i = 0; i < sr.n_stages; i++) {
        if (sr.delivered_stages[i]) {
            float adc_val = ((float)sr.measured_amplitudes[i] / (float)sr.delivered_stages[i]) - adc_offset;
            sr.measured_amplitudes[i] = lroundf(adc_val * scale[sr.output_mode]);
        }
        if (i < sr.n_stages - 1)
            printf("  %dx Stage %d: %6ld %s\n", sr.delivered_stages[i], i, sr.measured_amplitudes[i], units[sr.output_mode]);
        else
            printf("  %dx Inter-pulse gap: %6ld %s\n\n", sr.delivered_stages[i], sr.measured_amplitudes[i], units[sr.output_mode]);
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

    queue_init(&q_tx_offsets, sizeof(uint8_t), 1);
    queue_init(&q_rx_offsets, sizeof(offsets_t) * 2, 1);
    queue_init(&q_stimulus_cmd, sizeof(pulsetrain_t), 2);
    queue_init(&q_stimulus_trigger[0], sizeof(pulsetrain_t), 5);
    queue_init(&q_stimulus_trigger[1], sizeof(pulsetrain_t), 5);
    queue_init(&q_stim_result, sizeof(stim_result_t), 32);
    queue_init(&q_manual_cmd, sizeof(manual_cmd_t), 5); 
    queue_init(&q_adc_result, sizeof(int16_t), 5);

    while (!tud_cdc_connected()) sleep_ms(100);

    multicore_launch_core1(main_core1);
    if (multicore_fifo_pop_blocking() != 0xDEADBEEF)
        return EXIT_FAILURE;

    stimjim_context_t *stimjim_ctx = stimjim_ctx_init(&q_tx_offsets, &q_rx_offsets);

    printf("StimJim %s\n\n", FIRMWARE_VERSION);
    print_offsets(stimjim_ctx);

    while (true) {
        process_cmds(stimjim_ctx);
        process_completed_pulsetrains(stimjim_ctx);
    }
}
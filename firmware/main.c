#define FIRMWARE_VERSION "v0.0.0"

#include <float.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <math.h>
#include <pico/multicore.h>
#include <pico/stdio.h>
#include <pico/types.h>
#include <pico/util/queue.h>
#include <pico/stdio_usb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tusb.h>
#include "adc.h"
#include "dac.h"
#include "stimjim_ctx.h"

#define SAMPLES 200u
// #define ACCESSCTRL_N_CORE0 0xEC
// #define ACCESSCTRL_N_CORE1 0xDC

typedef struct {
  uint8_t stage_pulse; 
  uint32_t pulse; 
  uint32_t next_transition_time; 
} waveform_status_t;

static queue_t channel_io_queue;
static queue_t manual_pulsetrain_queue;
static queue_t offsets_tx_queue;
static queue_t offsets_rx_queue;
static queue_t stim_result_queue;
static queue_t trigger_pulsetrain_queue;

typedef enum {
  OFFSETS_TX_GET_OFFSETS,
  OFFSETS_TX_SET_OFFSETS,
  OFFSETS_TX_SET_CURRENTVOLTAGE_OFFSETS,
} offsets_tx_t;

typedef struct {
  uint8_t ch, n_stages;
  int32_t measured_amplitudes[MAX_STAGES+1];
  uint32_t delivered_stages[MAX_STAGES+1];
  output_mode_t output_mode;
} stim_result_t;

// shared between cores
static inline bool __time_critical_func(advance_stim)(timer_hw_t *timer, const bool ch, const waveform_t *wf, const float adc_offset, stim_result_t *sr, waveform_status_t *wfs);
static inline bool process_trigger(const bool ch, waveform_t const *wf_trig, waveform_t *wf_active);
static inline float set_adc_offset(const bool ch);
static inline int8_t set_current_or_voltage_offset(const bool ch, const bool line, const float adc_offset);
static inline void __time_critical_func(reset_stim)(const bool ch, const waveform_t *wf, const float adc_offset, waveform_status_t *wfs, stim_result_t *sr);
static inline void __time_critical_func(start_stim)(timer_hw_t *timer, const bool ch, const waveform_t *wf);

// core 1
static inline void process_channel_io_queue(void);
static inline bool process_manual_pulsetrain_queue(waveform_t *wf, const offsets_t off);
static inline void process_offsets_tx_queue(offsets_t *offsets);
static inline void process_trigger_pulsetrain_queue(waveform_t *wf, const offsets_t off);

static void __time_critical_func(core1_entry)(void) {
  offsets_t offsets = { 0 };
  stim_result_t sr = { 0 };
  waveform_t wf_trigger = { 0 };
  waveform_t wf_active = { 0 };
  waveform_status_t wfs = { 0 };
  bool active_stim = 0;
  dac_init(1);

  timer1_hw->pause = 1;
  timer1_hw->timelw = 0;
  timer1_hw->timehw = 0;
  set_output_mode(1, OUTPUT_MODE_FLOAT);

  for (;;) {
    process_offsets_tx_queue(&offsets);
    if ((active_stim = process_manual_pulsetrain_queue(&wf_active, offsets)));
    else ((active_stim = process_trigger(1, &wf_trigger, &wf_active)));
    if (active_stim) {
      sio_hw->doorbell_in_clr = 1;
      start_stim(timer1_hw, 1, &wf_active);
    }
    while (active_stim) {
      active_stim = advance_stim(timer1_hw, 1, &wf_active, offsets.adc, &sr, &wfs);
      if (sio_hw->doorbell_in_clr) {
        sio_hw->doorbell_in_clr = 1;
        active_stim = false;
      } 
      if (!active_stim)
        reset_stim(1, &wf_active, offsets.adc, &wfs, &sr);
    }
    process_channel_io_queue();
    process_trigger_pulsetrain_queue(&wf_trigger, offsets);
  }
}

// core 0
static bool process_cmds(stimjim_ctx_t *ctx, offsets_t *off, waveform_t *wf_trigger, waveform_t *wf_active);
static inline void process_completed_pulsetrains(const stimjim_ctx_t *ctx);
static inline void print_offsets(offsets_t off0);
static inline void set_channel1_offsets(void);

int32_t __time_critical_func(main)(void) {
  if (!stdio_init_all())
    return EXIT_FAILURE;

  uint64_t spi_pin_mask = (1LL << SPI_SCK_A) | (1LL << SPI_MOSI_A) | (1LL << SPI_MISO_A) | 
                          (1LL << SPI_SCK_B) | (1LL << SPI_MOSI_B) | (1LL << SPI_MISO_B); 

  spi_init(SPI_A, 10u * 1000u * 1000u);
  spi_init(SPI_B, 10u * 1000u * 1000u);
  gpio_set_function_masked64(spi_pin_mask, GPIO_FUNC_SPI);

  uint64_t stim_pin_mask = (1LL << NLDAC_A) | (1LL << CSA_A) | (1LL << CSB_A) |
                           (1LL << OE0_A)   | (1LL << OE1_A) | (1LL << LED_A) |
                           (1LL << NLDAC_B) | (1LL << CSA_B) | (1LL << CSB_B) |
                           (1LL << OE0_B)   | (1LL << OE1_B) | (1LL << LED_B);
  
  gpio_set_dir_out_masked64(stim_pin_mask);
  gpio_set_mask64(stim_pin_mask);
  gpio_set_function_masked64(stim_pin_mask, GPIO_FUNC_SIO);

  // omitted from mask because these should default to 0
  gpio_init(CHANNEL_IO_A);
  gpio_init(CHANNEL_IO_B);

  // accessctrl_hw->timer[0] = ACCESSCTRL_N_CORE1;
  // accessctrl_hw->spi[1] = ACCESSCTRL_N_CORE1;
  // accessctrl_hw->uart[0] = ACCESSCTRL_N_CORE1;
  // accessctrl_hw->timer[1] = ACCESSCTRL_N_CORE0;
  // accessctrl_hw->spi[0] = ACCESSCTRL_N_CORE0;

  queue_init(&channel_io_queue, sizeof(channel_io_t), 5);
  queue_init(&offsets_tx_queue, sizeof(offsets_tx_t), 5);
  queue_init(&offsets_rx_queue, sizeof(offsets_t), 5);
  queue_init(&manual_pulsetrain_queue, sizeof(pulsetrain_t), 5);
  queue_init(&stim_result_queue, sizeof(stim_result_t), 5);
  queue_init(&trigger_pulsetrain_queue, sizeof(pulsetrain_t), 5);

  // ctx defined globally but only accessible through getters and setters
  stimjim_ctx_t *ctx = stimjim_ctx_init(&channel_io_queue); 
  multicore_launch_core1(core1_entry);

  offsets_t offsets = { 0 };
  stim_result_t sr = { 0 };
  waveform_t wf_trigger = { 0 };
  waveform_t wf_active = { 0 };
  waveform_status_t wfs = { 0 };
  bool active_stim = 0;
  dac_init(0);

  set_channel1_offsets();
  offsets.adc     = set_adc_offset(0);
  offsets.current = set_current_or_voltage_offset(0, true,  offsets.adc);
  offsets.voltage = set_current_or_voltage_offset(0, false, offsets.adc);

  while(!tud_cdc_connected());

  printf("StimJim %s %s\r\n", FIRMWARE_VERSION);
  print_offsets(offsets);

  gpio_put(LED_A, false);
  gpio_put(LED_B, false);

  printf("Ready to go!\r\n\r\n");
  while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);

  timer0_hw->pause = 1;
  timer0_hw->timelw = 0;
  timer0_hw->timehw = 0;
  
  uint32_t saved_interrupts;

  for (;;) {
    process_completed_pulsetrains(ctx);
    if ((active_stim = process_cmds(ctx, &offsets, &wf_trigger, &wf_active))) 
      tud_task(); 
      // tud_task() is tinyusb interrupt ISR. Call it now manually because
      // tinyusb has a longer task to fulfill immediately after sending a
      // command to start a pulse train, and we don't want this longer
      // tud_task() function call to interrupt our pulse train because it will
      // lag one of the first few stages. 
    else if ((active_stim = process_trigger(0, &wf_trigger, &wf_active)));
    if (active_stim) {
      saved_interrupts = save_and_disable_interrupts(); 
      // stop tinyusb interrupts, we will instead manually call tud_task() function
      // during an active pulse train. A tud_task() isr that coincides
      // with when a stimulus is transitioning stages has the capability to
      // delay the transition. However, a tud_task() that is conveniently called
      // at the end of every stage transition will not cause a lag.
      start_stim(timer0_hw, 0, &wf_active);
    }
    while (active_stim) {
      active_stim = advance_stim(timer0_hw, 0, &wf_active, offsets.adc, &sr, &wfs);
      if (tud_cdc_available()) {
        sio_hw->doorbell_out_set = 1;
        active_stim = false;
      } 
      if (!active_stim) {
        reset_stim(0, &wf_active, offsets.adc, &wfs, &sr);
        restore_interrupts_from_disabled(saved_interrupts);
      }
    }
  }
}

//

static inline void set_channel1_offsets(void) {
  offsets_tx_t off_tx = OFFSETS_TX_SET_OFFSETS;
  queue_add_blocking(&offsets_tx_queue, &off_tx);
}

static inline void __time_critical_func(reset_stim)(const bool ch, const waveform_t *wf, const float adc_offset, waveform_status_t *wfs, stim_result_t *sr) {
  static const uint8_t ch_io_pins[2] = { CHANNEL_IO_A, CHANNEL_IO_B };
  static const uint8_t led_pins[2] = { LED_A, LED_B };
  float conversion_factor[2] = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
  timer_hw_t *timer[2] = { timer0_hw, timer1_hw };

  timer[ch]->pause = 1;
  timer[ch]->timelw = 0;
  timer[ch]->timehw = 0;
  wfs->stage_pulse = 0;
  wfs->pulse = 0;
  wfs->next_transition_time = 0;

  set_output_mode(ch, OUTPUT_MODE_FLOAT);
  gpio_put(ch_io_pins[ch],  false);
  gpio_put(led_pins[ch], false);
  dac_write(ch, 0, wf->stage_amplitude[wf->n_stages-1]);

  sr->ch = ch;
  sr->n_stages = wf->n_stages;
  
  for (uint8_t i = 0; i < sr->n_stages; i++) 
    if (sr->delivered_stages[i]) { // avoid divide by zero
      float adc_val = ((float)sr->measured_amplitudes[i] / (float)sr->delivered_stages[i]) - adc_offset;
      sr->measured_amplitudes[i] = lroundf(adc_val * conversion_factor[sr->output_mode]); 
    }

  queue_add_blocking(&stim_result_queue, sr);
  memset(sr, 0, sizeof(*sr));
  dac_latch(ch);

  // ignore any triggers that occurred during pulse train
  io_bank0_hw->intr[4] = RISING_EDGE_INTERRUPT_BIT(ch_io_pins[ch] , 1);
}

static inline bool __time_critical_func(advance_stim)(timer_hw_t *timer, const bool ch, const waveform_t *wf, const float adc_offset, stim_result_t *sr, waveform_status_t *wfs) {
  if (timer->timerawl >= wfs->next_transition_time) {
    dac_latch(ch);
    const uint8_t current_stage = wfs->stage_pulse;
    if (++wfs->stage_pulse >= wf->n_stages)
      wfs->stage_pulse = 0;
    wfs->next_transition_time += wf->stage_duration[current_stage];
    dac_write(ch, 0, wf->stage_amplitude[wfs->stage_pulse]);
    sr->delivered_stages[current_stage]++;
    // perform this last, give dac as much time to settle as possible before reading
    sr->measured_amplitudes[current_stage] += adc_read(ch, wf->output_mode); 
    if (!wfs->stage_pulse) {
      if (++wfs->pulse == wf->n_pulses) {
        return false;
      }
    }
    if (!ch) {
      tud_task();
  }
  }
  return true;
}

static inline void __time_critical_func(start_stim)(timer_hw_t *timer, const bool ch, const waveform_t *wf) {
  static const uint8_t ch_io_pins[2] = { CHANNEL_IO_A, CHANNEL_IO_B };
  static const uint8_t led_pins[2] = { LED_A, LED_B };
  dac_write(ch, 0, wf->stage_amplitude[0]);
  set_output_mode(ch, wf->output_mode);
  gpio_put(led_pins[ch], true);
  gpio_put(ch_io_pins[ch], true);
  timer->pause = 0;
}

static inline bool process_manual_pulsetrain_queue(waveform_t *wf, const offsets_t off) {
  bool start_stim_flag = false;
  pulsetrain_t pt;
  while (queue_try_remove(&manual_pulsetrain_queue, &pt)) {
    start_stim_flag = true;
    *wf = pt_to_wf(1, pt, off);
  }
  return start_stim_flag;
}

static inline void process_trigger_pulsetrain_queue(waveform_t *wf, const offsets_t off) {
  pulsetrain_t pt;
  while (queue_try_remove(&trigger_pulsetrain_queue, &pt))
    *wf = pt_to_wf(1, pt, off);
}

static inline float set_adc_offset(const bool ch) {
  set_output_mode(ch, OUTPUT_MODE_GND);
  float accumulator = 0;
  for (uint8_t i = 0; i < SAMPLES; i++)
    accumulator += adc_read(ch, 0);
  return accumulator / SAMPLES;
}

static inline int8_t set_current_or_voltage_offset(const bool ch, const bool line, const float adc_offset) {
  const int8_t sweep_range = 50;
  int8_t min_i = -sweep_range;
  float min = FLT_MAX;

  set_output_mode(ch, line ? OUTPUT_MODE_GND : OUTPUT_MODE_VOLTAGE);
  for (int8_t i = -sweep_range; i <= sweep_range; i++) {
    dac_update_output(ch, i);
    sleep_us(10);
    float accumulator = 0;
    for (uint16_t j = 0; j < SAMPLES; j++) 
      accumulator += adc_read(ch, line);
    accumulator -= (adc_offset* (float)SAMPLES);
    if (fabsf(accumulator) < min) {
      min = fabsf(accumulator);
      min_i = i;
    }
  }
  set_output_mode(ch, OUTPUT_MODE_FLOAT);  
  return min_i;
}

static inline void process_offsets_tx_queue(offsets_t *offsets) {
  offsets_tx_t off_tx;
  while (queue_try_remove(&offsets_tx_queue, &off_tx)) {
    switch(off_tx) {
      case OFFSETS_TX_GET_OFFSETS:
        queue_add_blocking(&offsets_rx_queue, offsets);
        break;
      case OFFSETS_TX_SET_OFFSETS:
        offsets->adc = set_adc_offset(1);
        // intentional fall-through
      case OFFSETS_TX_SET_CURRENTVOLTAGE_OFFSETS:
        offsets->current = set_current_or_voltage_offset(1, true, offsets->adc);
        offsets->voltage = set_current_or_voltage_offset(1, false, offsets->adc);
        break;
    }
  }
}

static inline bool process_trigger(const bool ch, waveform_t const *wf_trig, waveform_t *wf_active) {
  if (wf_trig->n_pulses == 0 || wf_trig->n_stages == 0)
    return false;

  static const uint8_t ch_io_pins[2] = { CHANNEL_IO_A, CHANNEL_IO_B };
  uint32_t ints = ch ? io_bank0_hw->proc1_irq_ctrl.ints[4] : io_bank0_hw->proc0_irq_ctrl.ints[4];

  if (ints & RISING_EDGE_INTERRUPT_BIT(ch_io_pins[ch], 1)) {
    *wf_active = *wf_trig;
    return true;
  }
  return false;
}

static inline void process_channel_io_queue(void) {
  channel_io_t ch_io;
  while (queue_try_remove(&channel_io_queue, &ch_io)) {
    gpio_set_pulls(CHANNEL_IO_B, 0, !ch_io.dir);
    gpio_set_dir(CHANNEL_IO_B, ch_io.dir);
    io_bank0_hw->proc1_irq_ctrl.inte[4] = RISING_EDGE_INTERRUPT_BIT(CHANNEL_IO_B, !ch_io.dir);
  }
}

static inline offsets_t get_ch1_offsets_blocking(void) {
  offsets_tx_t off_tx = OFFSETS_TX_GET_OFFSETS;
  queue_add_blocking(&offsets_tx_queue, &off_tx);
  offsets_t off;
  queue_remove_blocking(&offsets_rx_queue, &off);
  return off;
}

static inline void print_offsets(offsets_t off0) {
  offsets_t off1 = get_ch1_offsets_blocking();
  printf("ADC offsets: %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n\r\n",
        off0.adc,     off1.adc,
        off0.current, off1.current,
        off0.voltage, off1.voltage);
}

static bool process_cmds(stimjim_ctx_t *ctx, offsets_t *off, waveform_t *wf_trigger, waveform_t *wf_active) {
  static char buf[1024];
  static uint16_t nbuf = 0;

  int32_t c = getchar_timeout_us(0);
  if (c == PICO_ERROR_TIMEOUT) return false;
  sio_hw->doorbell_out_set = 1;
  if (nbuf >= sizeof(buf) - 1) { nbuf = 0; return false; }

  buf[nbuf++] = (char)c;
  if (buf[nbuf - 1] != '\n') return false;

  buf[nbuf - 1] = '\0';
  if (nbuf >= 2 && buf[nbuf - 2] == '\r')
    buf[nbuf - 2] = '\0';

  const char cmd  = buf[0];
  char *args = buf + 1;
  nbuf = 0;

  switch (cmd) {

    case 'S': {
      const char format_error_string[] = "S usage: S<idx>,<mode0>,<mode1>,<period_us>,<duration_us>; <amp0>,<amp1>,<dur_us>; ...\r\n";
      int32_t n = -1; 
      uint32_t duration;
      pulsetrain_t pt = { 0 };
      if (sscanf(args, "%d,%u,%u,%u,%u;", 
                 &n, &pt.output_mode[0], &pt.output_mode[1], 
                 &pt.period, &duration) != 5) {
        puts(format_error_string);
        return false;
      }

      char *tok = strtok(args, ";");
      tok = strtok(NULL, ";");
      while (tok && pt.n_stages < MAX_STAGES) {
        if (sscanf(tok, "%d,%d,%u",
                   &pt.amplitude[0][pt.n_stages],
                   &pt.amplitude[1][pt.n_stages],
                   &pt.stage_dur[pt.n_stages]) != 3){
          puts(format_error_string);
          return false;
        }
        pt.n_stages++;
        tok = strtok(NULL, ";");
      }
      
      char error_string[512] = "";
      char string_buffer[128];
      if (n < 0 || n >= MAX_PULSETRAINS) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid pulse train index: %d\r\n", n);
        strcat(error_string, string_buffer);
      }
      if (pt.output_mode[0] > 3) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid output mode on channel 0: %u\r\n", pt.output_mode[0]);
        strcat(error_string, string_buffer);
      }
      if (pt.output_mode[1] > 3) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid output mode on channel 1: %u\r\n", pt.output_mode[1]);
        strcat(error_string, string_buffer);
      }
      if (pt.period <= 0) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid period: %d\r\n", pt.period);
        strcat(error_string, string_buffer); 
      }

      uint32_t accumulator = 0;
      for (uint8_t i = 0; i < pt.n_stages; i++) 
        accumulator += pt.stage_dur[i];

      if (accumulator > pt.period) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid pulse train: summed stage durations exceed period.\r\n");
        strcat(error_string, string_buffer);
      }

      if (error_string[0] != '\0') {
        puts(error_string);
        return false;
      }

      pt.n_pulses = duration / pt.period;

      bool short_pulse = false;
      for (uint8_t i = 0; i < pt.n_stages; i++) 
        if (pt.stage_dur[i] < 20)
          short_pulse = true;
      if (pt.period - accumulator < 20)
          short_pulse = true;

      if (short_pulse)
        puts("Warning: <20us stage or inter-pulse gap detected. Desired pulse timings are not guaranteed.");

      stimjim_ctx_set_pulsetrain(ctx, n, &pt);
      
      channel_io_t ch_io[2] = { stimjim_ctx_get_channel_io(ctx, 0), stimjim_ctx_get_channel_io(ctx, 1) };

      // update trigger waveforms if trigger pulse train changed 
      if (ch_io[0].idx == n) 
        *wf_trigger = pt_to_wf(0, pt, *off); 
      if (ch_io[1].idx == n) 
        queue_add_blocking(&trigger_pulsetrain_queue, &pt);

      printf("PulseTrain[%d]: mode[%d,%d], period=%u us, pulses=%u, %d stages\r\n",
            n, pt.output_mode[0], pt.output_mode[1], pt.period, pt.n_pulses, pt.n_stages);
      for (int32_t j = 0; j < pt.n_stages; j++)
        printf("  Stage %d: amp[%d,%d], dur=%u us\r\n",
              j, pt.amplitude[0][j], pt.amplitude[1][j], pt.stage_dur[j]);
      puts(""); // blank line
      
      return false;
    }

    case 'T':
    case 'U': {
      int32_t idx = -1;
      if (sscanf(args, "%d", &idx) != 1) {
        puts("T/U usage: T<idx> or U<idx>\r\n");
        return false;
      }

      if (idx < 0 || idx >= MAX_PULSETRAINS) { 
        printf("Invalid pulse train index: %d\r\n\r\n", idx); return false; 
      }

      pulsetrain_t pt = stimjim_ctx_get_pulsetrain_parameters(ctx, idx); 
      if (pt.output_mode[1] < 2)
        queue_add_blocking(&manual_pulsetrain_queue, &pt);
      if (pt.output_mode[0] < 2) {
        *wf_active = pt_to_wf(0, pt, *off);
      }
      printf("Started PulseTrain[%d].\r\n\r\n", idx);
      return pt.output_mode[0] < 2;
    }

    case 'B': {
      const offsets_tx_t off_tx = OFFSETS_TX_SET_OFFSETS;
      queue_add_blocking(&offsets_tx_queue, &off_tx);
      off->adc = set_adc_offset(0);
      puts("ADC offsets updated.\r\n");
      // intentional fall-through
    }
    
    case 'C': {
      puts("Updating current and voltage offsets...");
      const offsets_tx_t off_tx = OFFSETS_TX_SET_CURRENTVOLTAGE_OFFSETS;
      queue_add_blocking(&offsets_tx_queue, &off_tx);
      off->current = set_current_or_voltage_offset(0, true,  off->adc);
      off->voltage = set_current_or_voltage_offset(0, false, off->adc);
      puts("Current and voltage offsets updated.\r\n");
      return false;
    }

    case 'D': {
      print_offsets(*off);
      return false;
    }

    case 'R': {
      int32_t ch, idx, out = 0;
      uint32_t parsed = sscanf(args, "%d,%d,%d", &ch, &idx, &out);
      if (!(parsed == 2 || parsed == 3)) {
        puts("R usage: R<ch>,<pt_idx|-1>,<0=trigger|1=sync> or R<ch>,<pt_idx|-1>\r\n");
        return false;
      }

      char error_string[256] = "";
      char string_buffer[100];
      if (ch < 0 || ch > 1) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid channel: %d\r\n", ch);
        strcat(error_string, string_buffer); 
      }
      if (idx >= MAX_PULSETRAINS || idx < -1) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid pulse train index: %d\r\n", idx);
        strcat(error_string, string_buffer);
      }
      if (out < 0 || out > 1) {
        snprintf(string_buffer, sizeof(string_buffer), "Invalid trigger/sync setting: %d (must be 0 or 1)\r\n", out);
        strcat(error_string, string_buffer);
      }
      if (error_string[0] != '\0') {
        puts(error_string);
        return false;
      }

      stimjim_ctx_set_channel_io(ctx, ch, idx, out);

      pulsetrain_t pt = (idx == -1) ? (pulsetrain_t){ 0 } : stimjim_ctx_get_pulsetrain_parameters(ctx, idx);

      if (idx == -1 || (pt.output_mode[ch] < 2 && pt.n_pulses > 0)) {
        if (ch == 1) 
          queue_add_blocking(&trigger_pulsetrain_queue, &pt);
        else 
          *wf_trigger = (idx == -1) ? (waveform_t){ 0 } : pt_to_wf(0, pt, *off);
      }

      if (idx == -1) 
        printf("IN%d -> Disabled\r\n\r\n", ch);
      else if (out) 
        printf("IN%d -> output marker\r\n\r\n", ch);
      else 
        printf("IN%d -> PulseTrain[%d]\r\n\r\n", ch, idx);
      return false;
    }

    case 'V': {
      int32_t ch; float mv;
      if (sscanf(args, "%d,%f", &ch, &mv) != 2) {
        puts("V usage: V<ch>,<mV>\r\n");
        return false;
      }

      if (ch < 0 || ch > 1) { printf("Invalid channel: %d\r\n\r\n", ch); return false; }
      offsets_t o    = ch ? get_ch1_offsets_blocking() : *off; 
      int32_t       code = (int)(mv / MILLIVOLTS_PER_DAC) + o.voltage;
      dac_update_output(ch, code);
      printf("Set channel %d to amplitude %f mV (DAC value %d).\r\n\r\n", ch, mv, code);
      return false;
    }

    case 'A': {
      int32_t ch = -1, code = 0;
      if (sscanf(args, "%d,%d", &ch, &code) != 2) {
        puts("A usage: A<ch>,<dac_code>\r\n");
        return false;
      }
      if (ch < 0 || ch > 1) { printf("Invalid channel: %d\r\n\r\n", ch); return false; }
      dac_update_output(ch, code);
      printf("Set channel %d to amplitude %d.\r\n\r\n", ch, code);
      return false;
    }

    case 'E': {
      int32_t ch = -1, line = -1;
      if (sscanf(args, "%d,%d", &ch, &line) != 2) {
        puts("E usage: E<ch>,<line>\r\n");
        return false;
      }

      char error_string[256] = "";
      char string_buffer[100];
      if (ch < 0 || ch > 1) { 
        snprintf(string_buffer, sizeof(string_buffer), "Invalid channel: %d\r\n", ch); 
        strcat(error_string, string_buffer);
      }
      if (line < 0 || line > 1) { 
        snprintf(string_buffer, sizeof(string_buffer), "Invalid line: %d\r\n", line); 
        strcat(error_string, string_buffer);      
      }
      if (error_string[0] != '\0') {
        puts(error_string);
        return false;
      }
      offsets_t o    = ch ? get_ch1_offsets_blocking() : *off;
      int16_t   val  = adc_read(ch, line);
      int16_t   real = (val - o.adc) * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
      printf("Read value: %d (%hd %s)\r\n\r\n", val, real, line ? "uA" : "mV");
      return false;
    }

    case 'M': {
      int32_t ch = -1, mode = -1;
      if (sscanf(args, "%d,%d", &ch, &mode) != 2) {
        puts("M usage: E<ch>,<mode>\r\n");
        return false;
      }
      char error_string[256] = "";
      char string_buffer[100];
      if (ch < 0 || ch > 1) { 
        snprintf(string_buffer, sizeof(string_buffer), "Invalid channel: %d\r\n", ch); 
        strcat(error_string, string_buffer);
      }
      if (mode < 0 || mode > 3) { 
        snprintf(string_buffer, sizeof(string_buffer), "Invalid mode: %d\r\n", mode); 
        strcat(error_string, string_buffer);      
      }
      if (error_string[0] != '\0') {
        puts(error_string);
        return false;
      }
      set_output_mode(ch, mode);
      printf("Set channel %d to mode %d\r\n\r\n", ch, mode);
      return false;    
    }

    default:
      printf("Unknown command: '%c'\r\n\r\n", cmd);
  }
}

static inline void process_completed_pulsetrains(const stimjim_ctx_t *ctx) {
  stim_result_t sr;
  if (!queue_try_remove(&stim_result_queue, &sr)) return;
  char units[2][3] = {"mV", "uA"};
  float conversion_factor[2] = { MILLIVOLTS_PER_ADC, MICROAMPS_PER_ADC };
  printf("Ch%d pulse train complete. Delivered:\r\n", sr.ch);
  for (int32_t i = 0; i < sr.n_stages - 1; i++)
    printf(" %dx Stage %d: %6ld %s\r\n", sr.delivered_stages[i], i,
           sr.measured_amplitudes[i], units[sr.output_mode]);
  printf(" %dx Inter-pulse gap: %6ld %s\r\n\r\n", sr.delivered_stages[sr.n_stages-1],
           sr.measured_amplitudes[sr.n_stages-1], units[sr.output_mode]);
}
- 40 mV and 2.5 μA max error

Extra credit:
- adc read every 10us

Design notes:

Cores:
- Core0: USB/serial and stimjim_context (command parsing, calibration state).
  Hands work to core1 over queues and reports results from core1 back over
  queues; never touches the real-time hardware during normal operation.
- Core1: all real-time work — pulse-train delivery, trigger handling, and every
  ADC/DAC/GPIO interaction after core1 initialization.

Core1 main loop (no interrupts):
- Forever alternates handle_cmd_queue() then handle_triggers(). There are no
  ISRs — both user commands and trigger inputs are polled — so the command path
  and the trigger path are mutually exclusive and can't race.
- Commands from core0 (q_core1_cmd): stimulus request (T/U), trigger config
  (R/S), offset calibration (B/C/D), and manual DAC/ADC/output-mode (A/M/V/E).
- Cancel ('X') command is raised by core0 on the inter-core doorbell and polled
  inside the stage loop.

Triggers (polled, no ISR):
- The R command arms a CHANNEL_IO pin as a trigger by setting its rising-edge
  enable bit (proc1_irq_ctrl.inte); the NVIC IRQ is never enabled so there is no
  ISR. Core1 polls the hardware edge latch (proc1_irq_ctrl.ints).

Commands and triggers are ignored when:
- Commands that queue up while a stimulus runs are drained afterward by
  handle_cmd_queue_during_stimulus(). Only config commands are applied: trigger
  config (R/S) and offsets (B/C/D). Stimulus requests (T/U) and manual commands
  (A/M/V/E) received during a stimulus are discarded.
- Latched edges/triggers are cleared after every stimulus, so triggers arriving
  during command handling or stimulus delivery are ignored.

Timing:
- Stage timing uses the core1 DWT cycle counter (clk_sys, 150 MHz).
- DAC/ADC SPI runs on PIO (two state machines per PIO: DAC @ 30 MHz,
  ADC @ 10 MHz). The hot path uses blocking wrappers.

Jitter avoidance (the stage-0 pulse width must be exact):
- The entire core1 hot path is SRAM-resident (.time_critical / __always_inline)
  so it is immune to XIP flash-cache misses, which can stall the core for
  several µs.

Changes from stimjim_teensy:
- 'X' cancels active stimulus
- Input validation
- Removed 2.5V calibration (2.5V and 10V offsets differ)

Suggested changes:
- Period = sum of stage durations
- Specify n_pulses not total duration
- B should auto-run C (voltage/current offsets depend on ADC offsets)?
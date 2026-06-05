# StimJim Firmware Architecture

## Design decisions

This table describes some decisions made about the stimjim firmware architecture.

|  |  |  |
|---|---|---|
| Dual-core | Core 0 = USB/UI, Core 1 = stimulus timing | Isolates real-time work from blocking USB I/O |
| Inter-core queues & doorbell | Most cross-core communication goes through queues; cancellation uses the doorbell | Core 0 can never corrupt Core 1 timing-critical state |
| PIO-based SPI | Two state machines per SPI bus | Non-blocking 30 MHz DAC writes and 10 MHz ADC reads, allows asynchronous stimuli delivery |
| Timer alarms | alarm[0] ch0, alarm[1] ch1, alarm[2] sync | Allows fully independent or synchronized dual-channel stimulus |
| Pin interrupt | rising edge on either channel input incurs ISR | Allows low latency between trigger and start of stimulus delivery |

## Data & peripheral ownership

> [!NOTE] 
> GPIO and hardware listed under Core 1 is exclusively managed by Core 1 during
> normal operation. However, Core 0 also initializes and configures some of
> these GPIO pins before Core 1 is launched. Additionally, Core 0 writes
> `io_bank0_hw->proc1_irq_ctrl.inte` at runtime (via the R command) to enable or
> disable the rising-edge interrupt on CHANNEL_IO pins; Core 1 processes these
> interrupts via `isr_trigger`.

```mermaid
flowchart LR
  subgraph C0["Core 0 — main.c"]
    direction TB
    subgraph c0data["Data"]
      subgraph ctx["stimjim_ctx_t"]
        pt["pulse train array\npulsetrain_t[100]"]
        off["calibration offsets\noffsets_t[2]"]
        chio["channel I/O config\nchannel_io_t[2]"]
      end
      ibuf["serial input buffer"]
    end
    subgraph c0hw["Peripherals"]
      usb["USB CDC\n(TinyUSB)"]
    end
  end

  subgraph C1["Core 1 — core1.c"]
    direction TB
    subgraph c1data["Data"]
      subgraph sctx["stimulus_context_t[2]"]
        sm["stimulus_state\nIDLE | WAIT_STAGE_TRANSITION\nDAC_SETTLING | ADC_MEASURING"]
        counters["stage_counter\npulse_counter\nnext_alarm_us"]
        pts["pt_active (pulsetrain_t)\npt_trigger (pulsetrain_t)"]
        result["sr (stimulus_result_t)"]
      end
      sync_flag["sync_active (bool)"]
    end
    subgraph c1hw["Peripherals"]
      pio0["PIO0 — ch 0\nDAC (SM0, 30 MHz) + ADC (SM1, 10 MHz)"]
      pio1["PIO1 — ch 1\nDAC (SM0, 30 MHz) + ADC (SM1, 10 MHz)"]
      timer["Timer0\nalarm[0] ch 0 / alarm[1] ch 1 / alarm[2] sync"]
      c1_gpio["GPIO out: NLDAC, CS_DAC, CS_ADC, OE0/OE1, LED\nCHANNEL_IO (× 2 ch) — sync output or trigger input\n  rising-edge IRQ configured via proc1_irq_ctrl.inte"]
      irq["Interrupt handlers (Core 1 exclusive)\nIO_IRQ_BANK0 → isr_trigger (CHANNEL_IO rising edge)\nTIMER0_IRQ_0 → isr_ch0 (ch0A stage transition)\nTIMER0_IRQ_1 → isr_ch1 (ch 1 stage transition)\nTIMER0_IRQ_2 → isr_sync (sync stage transition)"]
    end
  end

  C0 ~~~ C1
```

## Startup sequence

This chart is supposed to show how core 0 and core 1 interact during startup.

```mermaid
sequenceDiagram
  participant C0 as Core 0 (main.c)

  Note over C0: init stdio (USB)<br/>configure GPIOs<br/>init inter-core queues<br/>set bus priority → Core 1
  create participant C1 as Core 1 (core1.c)
  C0->>C1: multicore_launch_core1()
  Note over C0: wait for handshake
  Note over C1: init PIO SPI, DACs, ADCs<br/>ISRs, Timer0 alarms, GPIO IRQ
  C1-->>C0: handshake (multicore FIFO)
  Note over C1: while(1): process queues,<br/>advance stimulus state machines
  Note over C0: stimjim_ctx_init()
  C0->>C1: q_offsets_tx (calibration request)
  C1-->>C0: q_offsets_rx (calibration result)
  Note over C0: wait for USB CDC connection<br/>print firmware version & offsets
  Note over C0: while(1): parse serial commands,<br/>queue to Core 1, print command<br/>feedback and stimulus readbacks
```

## Command flow

This chart is supposed to show the flow of data whenever a user sends a command.
Some data gets passed back and forth between cores through thread-safe queues
before being saved and printed.

```mermaid
flowchart LR
  subgraph cmds["Serial commands to Core 0"]
    direction TB
    S["S — set pulse train"]
    TU["T / U — trigger stimulus"]
    R["R — configure channel I/O"]
    M["M — set output mode"]
    VA["V / A — manual DAC output"]
    E["E — read ADC"]
    BC["B / C — calibrate"]
    X["X — cancel stimulus"]
    D["D — display offsets"]
  end

  subgraph data["Core 0 data (stimjim_ctx_t)"]
    direction TB
    pt["pulsetrain_t[100]"]
    off["offsets_t[2]"]
    chio["channel_io_t[2]"]
  end

  subgraph c1["Core 1 (queue → action)"]
    direction TB
    qsc["q_stimulus_cmd\nrun stimulus state machine"]
    qst["q_stimulus_trigger[ch]\nupdate pt_trigger"]
    qmc_dac["q_manual_cmd\nset DAC output"]
    qmc_adc["q_manual_cmd\nread ADC"]
    qmc_mode["q_manual_cmd\nset output mode (OE0/OE1)"]
    qotx["q_offsets_tx\nmeasure_offsets()"]
    door["doorbell register\ncancel_stimulus()"]
  end

  subgraph c1data["Core 1 data (stimulus_context_t[2])"]
    direction TB
    c1_state["stimulus_state"]
    c1_counters["stage_counter\npulse_counter\nnext_alarm_us"]
    c1_pts["pt_active (pulsetrain_t)\npt_trigger (pulsetrain_t)"]
    c1_result["sr (stimulus_result_t)"]
  end

  subgraph resp["Core 0 (queue → action)"]
    direction TB
    qsr["q_stimulus_result\nprint delivered amplitudes\nper stage"]
    qadc["q_adc_result\nprint ADC reading"]
    qorx["q_offsets_rx\nprint calibration results"]
  end

  S --> pt
  D --> off
  M --> qmc_mode
  R --> chio
  R --> qst
  TU --> qsc --> c1_state & c1_counters & c1_pts & c1_result
  qsc --> qsr
  VA --> qmc_dac
  E --> qmc_adc --> qadc
  BC --> qotx --> qorx --> off
  X --> door --> c1_state
  qst --> c1_pts
```

## PIO assembly

This section explains some of the PIO stuff.

PIO communicates with the ARM cores through TX/RX FIFOs, and uses two shift
registers internally: the **OSR** (Output Shift Register) shifts bits out onto
MOSI one at a time; the **ISR** (Input Shift Register) accumulates incoming bits
from MISO.

### PIO instruction reference

| Instruction | What it does |
|---|---|
| `pull block` | Wait until ARM puts a word in TX FIFO, then load it into OSR |
| `out pins, 1` | Shift 1 bit from OSR onto MOSI |
| `in pins, 1` | Sample 1 bit from MISO into ISR |
| `push block` | Send ISR contents to RX FIFO for ARM to read |
| `set x, N` | Set counter register x to N |
| `jmp x-- label` | Decrement x and jump to label if x was non-zero; fall through if x was 0. Used as a loop counter: `set x, N` then `jmp x-- loop` runs the loop body N+1 times |
| `label:` | A named anchor — not an instruction, just a target for `jmp` |
| `side N` | Simultaneously set SCK to N while executing this instruction — no extra cycle cost. The pin is wired to SCK at init time via `sm_config_set_sideset_pins(&c, sck)` |
| `.wrap / .wrap_target` | When execution reaches `.wrap`, loop back to `.wrap_target` automatically |

### Mode 1 — DAC write (SM0), 30 MHz SCK

CPOL=0, CPHA=1: SCK idles **low**. The master drives MOSI on the rising edge;
the DAC latches it on the falling edge. The ARM sends a 32-bit word left-aligned
with the 16-bit DAC code in the top 16 bits; the PIO shifts out the top 24 bits
(16-bit code + 8 zero command bits).

```asm
.program pio_spi_mode1
.side_set 1                       ; one side-set pin = SCK, idles low (0)

.wrap_target
    pull  block       side 0      ; wait for ARM to write a word; load it into OSR; hold SCK low
    set   x, 23       side 0      ; set loop counter to 23 (will count 24 iterations: 23 down to 0)
bitloop_m1:
    out   pins, 1     side 1      ; shift MSB of OSR onto MOSI; simultaneously raise SCK
    jmp   x-- bitloop_m1  side 0  ; lower SCK; decrement x and loop (exits when x was 0)
.wrap                             ; go back to pull — SM stalls here waiting for next word
                                  ; the stall sets the TXSTALL flag, used to detect completion
```

### Mode 2 — ADC read (SM1), 10 MHz SCK

CPOL=1, CPHA=0: SCK idles **high**. The ADC drives MISO just before the falling edge; the master samples it on the falling edge, then drives the next MOSI bit on the rising edge. The PIO sends a dummy 16-bit word to generate SCK cycles while simultaneously capturing 16 bits from MISO. One bit of MOSI is driven before the loop to keep the bit counts balanced at 16 each.

```asm
.program pio_spi_mode2
.side_set 1                       ; one side-set pin = SCK, idles high (1)

.wrap_target
    pull  block       side 1      ; wait for ARM word; load into OSR; hold SCK high
    set   x, 14       side 1      ; loop counter: 14 (15 iterations), plus 1 bit before + 1 after = 16 total
    out   pins, 1     side 1      ; drive MOSI[15] before the first falling edge

bitloop_m2:
    in    pins, 1     side 0      ; lower SCK (falling edge): sample MISO into ISR
    out   pins, 1     side 1      ; raise SCK (rising edge): drive next MOSI bit
    jmp   x-- bitloop_m2  side 1  ; loop (side 1 holds SCK high between iterations)

    in    pins, 1     side 0      ; final falling edge: sample last MISO bit
    push  block       side 1      ; send 16-bit ISR to RX FIFO; return SCK to idle high
.wrap                             ; go back to pull
```

Total: 16 MISO samples, 16 MOSI bits. The received word is sign-extended by `adc_get_value()` to recover the 13-bit ADC result.

## PIO SPI state machine

SM0 and SM1 share the same SCK and MOSI pins and can never run simultaneously. `pio_spi_select_*` stops the SM, resets it, and jumps to the correct program entry before asserting CS.

```mermaid
flowchart LR
  subgraph DAC["SM0 — DAC write\nMode 1 (CPOL=0 CPHA=1), 30 MHz, 24-bit TX"]
    direction TB
    D1["pio_spi_select_dac()\nCS_DAC low\nrestart SM0, jump to offset_dac\nclear TXSTALL flag"]
    D2["dac_write_output()\nput src << 8 in TX FIFO\nenable SM0"]
    D3["SM0 shifts 24 bits out\nMSB-first on rising SCK edge"]
    D4["SM0 stalls on pull\nTXSTALL set → pio_spi_is_done()"]
    D5["pio_spi_deselect_dac()\nCS_DAC high, disable SM0"]
    D1 --> D2 --> D3 --> D4 --> D5
  end

  subgraph ADC["SM1 — ADC read\nMode 2 (CPOL=1 CPHA=0), 10 MHz, 16-bit TX+RX"]
    direction TB
    A1["pio_spi_select_adc()\nflush RX FIFO\nCS_ADC low\nrestart SM1, jump to offset_adc\nclear TXSTALL flag"]
    A2["adc_read()\nput 0 in TX FIFO\nenable SM1"]
    A3["SM1 shifts 16 bits out (MOSI)\nsamples 16 bits in (MISO)\nsimultaneously"]
    A4["SM1 pushes RX word to FIFO\nTXSTALL set → pio_spi_is_done()"]
    A5["adc_get_value()\nread RX FIFO\nsign-extend 13-bit result"]
    A6["pio_spi_deselect_adc()\nCS_ADC high, disable SM1"]
    A1 --> A2 --> A3 --> A4 --> A5 --> A6
  end

  DAC ~~~ ADC
```

## Stimulus state machine

This state machine runs per channel (`stimulus_context_t[2]`), driven by
`advance_stimulus()` in Core 1's `while(1)` loop. When both channels are
supposed to operate synchronously, the 

```mermaid
flowchart TD
  IDLE([IDLE])
  PDW[WAIT_STAGE_TRANSITION]
  DS[DAC_SETTLING]
  AM[ADC_MEASURING]

  IDLE -->|"initiate_stimulus()\nsingle: latch ch DAC with stage[0]\nsync: latch both DACs atomically"| PDW
  PDW -->|"stage_transitioned (set by ISR)\npreload next stage DAC via SPI"| DS
  DS -->|"10 µs after stage start\ninitiate ADC read"| AM
  AM -->|"ADC received, accumulate\nsingle: schedule_latch(ch)\nsync: schedule_latch(2, MIN(...))\nonly when both channels sync_ready"| PDW

  PDW -->|"stage_transitioned (set by ISR)+ stimulus_ending"| IDLE
  PDW -->|"cancel_stimulus()"| IDLE
  DS -->|"cancel_stimulus()"| IDLE
  AM -->|"cancel_stimulus()"| IDLE
```

**ISR role:** `isr_ch0/1` (single) or `isr_sync` (sync) fires at the Timer0
alarm — latches the DAC value preloaded after a stage transition (single: one
NLDAC pulse; sync: both NLDACs on the same clock cycle), then sets
`stage_transitioned`. `stimulus_ending` causes the ISR to drive output to GND
instead of latching.

### PIO SPI state during stimulus delivery

| Stimulus state | Active PIO SM | What the SPI is doing |
|---|---|---|
| IDLE | none | idle; DAC SM stalled with trigger pulse train's first stage preloaded |
| WAIT_STAGE_TRANSITION (waiting for ISR) | none | idle, waiting for Timer0 alarm to fire |
| WAIT_STAGE_TRANSITION (ISR fired, stage_transitioned=true) | SM0 (DAC) | transmitting next stage amplitude to DAC |
| DAC_SETTLING | SM0 stalled | transmission complete; waiting 10 µs for analog output to stabilize |
| ADC_MEASURING | SM1 (ADC) | clocking out dummy bits to generate SCK, sampling MISO |

## Stimulus lifecycle pt2

```mermaid
flowchart TD

  START(( ))
  IDLE([IDLE])
  PDW([WAIT_STAGE_TRANSITION])
  DS([DAC_SETTLING])
  AM([ADC_MEASURING])

  subgraph INIT["initiate_stimulus()"]
    i1["set output mode (OE0/OE1);\nLED + CHANNEL_IO → high;\nspin to µs boundary;\nlatch preloaded stage[0];\nstart ADC config SPI async;\nstage_counter = 0; pulse_counter = 0;\nnext_alarm_us = now (µs at latch)"]
  end

  subgraph PRELOAD["begin_dac_preload()"]
    p1["deassert CS_ADC;\nassert CS_DAC;\ndac_write_output(stage[n+1]);"]
  end

  subgraph ADC_GO["begin_adc_read()"]
    a1["deassert CS_DAC;\nassert CS_ADC;\nadc_read();"]
  end

  subgraph PROC["process_adc_result()"]
    s1["read ADC from PIO FIFO;\naccumulate measurement;\nnext_alarm_us += stage_duration[stage_counter];\nstage_counter++;\nif stage_counter ≥ n_stages:\n  stage_counter = 0; pulse_counter++;\nif stage_counter == 0 && pulse_counter ≥ n_pulses:\n  stimulus_ending = true"]
  end

  subgraph SCHED["schedule_latch()"]
    s2["Timer0 alarm[ch] = next_alarm_us;"]
  end

  subgraph COMPLETE["complete_stimulus()"]
    c1["q_stimulus_result → Core 0\nflush q_stimulus_cmd"]
    c2["dac_write_output(stage[0] of next trigger)\nwait until SPI done; leave CS_DAC asserted"]
    c1 --> c2
  end

  START --> IDLE
  IDLE -->|"trigger interrupt or T/U command"| INIT --> PDW
  PDW -->|"stage_transitioned
  (set by timer ISR)"| PRELOAD --> DS
  DS -->|"≥10 µs + DAC write done"| ADC_GO --> AM
  AM -->|"ADC read done"| PROC --> SCHED --> PDW
  PDW -->|"stage_transitioned 
  (set by timer ISR) + stimulus_ending"| COMPLETE --> IDLE
```

**Timer0 ISR role:** `isr_ch0/1` (single) or `isr_sync` (sync) fires when
`alarm[ch]` expires — clears `intf[ch]`, re-arms `intr[ch]`, then sets
`stage_transitioned`. If `stimulus_ending` is true, it drives `OE0/OE1 → GND`
and clears LED + CHANNEL_IO instead of pulsing NLDAC.

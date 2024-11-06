# StimJimBIST
A simple console application that performs built in evaluation of the funcationility
of both stimulation channels.

Limitations:
- Assumes that StimJimPulser is loaded onto the Teensy
- Does not test physical connectivity (solder joints) of BNCS, screw-terminals
- Does not test trigger or GPIO capabilities
- Assumes there is nothing connected to the output channels of the device

### Usage
Usage:
```
StimJimBIST COM3
```

Example result:
```
Running StimJim BIST.
Testing voltage mode.

Channel 0:
| Test V. | Meas V. | Err. mV. | Pass / Fail | Test uA. | Meas uA. | Err. uA | Pass / Fail |
|---------|---------|----------|-------------|----------|----------|---------|-------------|
| -9.00   | -8.99   | 7        | PASS        | -1993.36 | -1987.00 | 6.36    | PASS        |
| 0.00    | 0.01    | 7        | PASS        | 0.00     | 0.00     | 0.00    | PASS        |
| 9.00    | 9.01    | 5        | PASS        | 1993.36  | 1982.00  | -11.36  | PASS        |

Channel 1:
| Test V. | Meas V. | Err. mV. | Pass / Fail | Test uA. | Meas uA. | Err. uA | Pass / Fail |
|---------|---------|----------|-------------|----------|----------|---------|-------------|
| -9.00   | -9.00   | 3        | PASS        | -1993.36 | -1985.00 | 8.36    | PASS        |
| 0.00    | 0.01    | 5        | PASS        | 0.00     | 0.00     | 0.00    | PASS        |
| 9.00    | 9.02    | 19       | PASS        | 1993.36  | 1984.00  | -9.36   | PASS        |

Testing current mode.

Channel 0:
| Pol. | Meas V. | Err. mV. | Pass / Fail | Meas uA. | Err. uA | Pass / Fail |
|------|---------|----------|-------------|----------|---------|-------------|
| -    | -9.98   | 21       | PASS        | -4.00    | -4.00   | PASS        |
| +    | 10.01   | 6        | PASS        | 0.00     | 0.00    | PASS        |

Channel 1:
| Pol. | Meas V. | Err. mV. | Pass / Fail | Meas uA. | Err. uA | Pass / Fail |
|------|---------|----------|-------------|----------|---------|-------------|
| -    | -9.97   | 27       | PASS        | -4.00    | -4.00   | PASS        |
| +    | 10.01   | 12       | PASS        | 0.00     | 0.00    | PASS        |

GLOBAL RESULT: PASS
```

### Explanation

#### Voltage mode test
This test examines the functionality of the DAC, ADC, output mux, current source, and current sense circuit simultaneously.

- Put both channels in voltage mode ()
	- This this ties V_OUT to CHANNEL_OUT, which is open
	- This ties I_OUT to ground through a 1k resistor
	- I_OUT = V_OUT / 3kOhm
	
- For each channel, apply -9V, 0V, and 9V to V_OUT
	- This sets V_DAC = V_OUT / 1.5
	- V_DAC is still applied to Howland current source which passed current through the 1k dischange resistor
  - Current measurement can therefore be performed with transimpedance gain of 0.33 mA per volt of V_DAC
	- Read the voltage at CHANNEL_OUT to see if the voltage is set to the test voltage
	- Read I_SENSE to see that its equal to V_DAC / 1.5 * 0.33 mA/V

#### Current mode test
This test examines the functionality of the DAC, ADC, and output mux, and current sense circuit simultaneously.
It is largely redundant with the first test but does ensure that the output mux can switch to a non default state.

- Put both channels in current mode 
	- This ties V_OUT to floating
    - This ties I_OUT to CHANNEL_OUT, which is open
	
- For each channel, attempt to apply  -1V, 1V to V_OUT
	- This sets V_DAC = V_OUT / 1.5
	- This attempts to deliver V_DAC * 0.33 mA/V to CHANNEL_OUT
	- Read the voltage at CHANNEL_OUT (should saturate at +/-10V)
	- Read current through CHANNEL_OUT (should be 0 since its open)

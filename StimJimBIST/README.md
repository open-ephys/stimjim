# StimJimBIST
A simple console application that performs built in evaluation of the funcationility
of both stimulation channels.

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

Limitations:
- Assumes that StimJimPulser is loaded onto the Teensy
- Does not test physical connectivity (solder joints) of BNCS, screw-terminals
- Does not test trigger or GPIO capabilities
Microstim - a flexible, precise, and inexpensive open-source stimulator
-------------------------------
Microstim is a current and voltage stimulator for stimulating neural tissue (as with stimulating electrodes). 

Specifications:

 - Current range: -833uA to +833uA. (for resistances up to 14k)
 - Pulse widths down to 50 us.
 - Maximum output voltage: 12V
 - Onboard ADC measurement of actual output current
 

Getting started:

To generate a pulse train, you need to send a serial command, terminated by a newline (\n). An example command would be
   T0,3,200,-200,50,2000,1000000\n
   
The "T" indicates to generate a pulse train. If any comma-separated numbers appear following the T, they are the pulse train parameters. 

![Alt text](./pulseTrainParametrization.svg)
<img src="./pulseTrainParametrization.svg">

The first two parameters are the mode of each output channels. Modes are as follows:

0. Voltage output, ADC reads voltage of output
1. Current output, ADC reads voltage of output 
2. Current output, ADC reads output current
3. Output is grounded, ADC reads output current (shunted through a 1k resistor to ground).

In this example, output 0 is generating a voltage output, and output 1 is grounded (not doing anything). 

The third and fourth parameters are the _amplitude_ of the two phases of the pulse, 
in millivolts if mode is 0 and microamps if mode is non-zero. In the example given above, the pulse would be biphasic,
first rising to 200 mV and then falling to -200 mV before returning to 0. 

The fifth parameter is the the duration of each phase of the pulse, in microseconds. In the example, the 200 mV phase 
would last 50 us, followed by 50 us at -200 mV.

The sixth parameter is the period of the pulse train. In this example, biphasic pulses are delivered every 2000 ms.

The seventh and last parameter is the duration of the pulse train, in microseconds. 
In the example, the pulse train lasts for one second (1000000 microseconds).


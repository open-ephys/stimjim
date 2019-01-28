Microstim - a flexible, precise, and inexpensive open-source stimulator
-------------------------------
Microstim is a current and voltage stimulator for stimulating neural tissue (as with stimulating electrodes). 

# Specifications:

 - Two independently-controllable output channels, each with both current and voltage output modes
 - Total cost: $131 USD.
 - Output range in current mode: -833uA to +833uA. 
 - Pulse width in current mode: down to 50 us.
 - Output range in voltage mode: -12V to 12V.
 - Pulse width in current mode: down to 10 us.
 - In current mode, +-11V compliance voltage (+/-833uA output possible for resistances up to 13k, +-110uA possible for resistances up to 100k)
 - Channels are isolated from power supply (but not from each other).
 - Onboard ADC measurement of actual output current or voltage
 
# Building your own microstim:

Here are the steps to build your very own microstim box. 

1. Order [PCBs](./PCBs/CombinedPCB_fabricationFiles_20181210/)  (from [Seeedstudio](https://www.seeedstudio.com/fusion.html) or any other PCB manufacturer). 
2. Order [components](./microstim_BOM.xlsx) (entirely from [Digikey](https://www.digikey.com/), except for the enclosure).
3. Order an [enclosure](https://www.newark.com/box-enclosures/b5-080bk/enclosure-instrument-aluminium/dp/39M5412?ost=B5-080BK) (optional, but it protects the circuit and looks pretty) from Newark.
4. Solder components onto the PCB, using the [schematic](./schematic.pdf) and [layout](./pcb.pdf) files for reference. This may take a few hours, depending on your soldering ability.
5. Compile and download the [firmware](./microstim_teensy/microstim_teensy.ino) onto the Teensy, using the [Arduino IDE](https://www.arduino.cc/en/main/software) with [Teensyduino](https://www.pjrc.com/teensy/td_download.html) installed (or write your own!)
6. Stimulate something!

# Getting started using microstim:

To generate a pulse train, you first need to send a serial command, terminated by a newline (\n) in order to define a pulse train. An example command would be:

   S0,2,0,2000,1000000;100,0,150;-100,-100,200
   
The "S" is to *s*pecify the parameters of a pulse train, and the zero immediately following the S implies that we are going to set parameters for PulseTrain #0. microstim can store 10 different PulseTrain parameter sets. If any comma- and semicolon-separated numbers appear following the S, they are the pulse train parameters. A command that simply consists of "S0" will result in simply printing out the current parameters of PulseTrain #0. 

Once a PulseTrain is defined, to run it you would send a command of the form:

   T0

This tells microstim to start running PulseTrain #0. If we had parameterized PulseTrain #3, we could just as easily run PulseTrain #3 with the command "T3".

##  Parameterization 

![Alt text](./pulseTrainParametrization.svg)

The paramerization of a PulseTrain via an "S" command is as follows:

S[PulseTrain number],[output 0 mode],[output 1 mode],[period],[duration];[mV or uA for stage 0 output 0],[mV or uA for stage 0 output 1],[stage 0 duration (ms)]; [mV or uA for stage 1 output 0],[mV or uA for stage 1 output 1],[stage 1 duration (ms)]; (etc for additional stages, up to 10 total stages).

After the "S", the first parameter is the number of the PulseTrain that we are defining. The next two parameters are the modes of each output channels. Modes are as follows:

 - 0 = Voltage output, ADC reads voltage of output
 - 1 = Current output, ADC reads voltage of output 
 - 2 = Current output, ADC reads output current
 - 3 = Output is grounded, ADC reads output current (shunted through a 1k resistor to ground).

In this example, output 1 is generating a current output (mode 2), and output 1 is generating a voltage output (mode 0). To specify that a channel should not output anything during a pulse train, set the mode to 3 (ground).  

The fourth parameter is the _period_ of the pulse train. In this example, pulses are delivered every 2000 ms.

The fifth parameter is the _duration_ of the pulse train, in microseconds. In the example, the pulse train lasts for one second (1000000 microseconds).

Parameters after the 5th come in sets of 3, and describe a "stage" of a pulse in the train. Note that a semi-colon is before the beginning of each set of 3, and the three parameters themselves are separated from each other by commas. The first two parameters are the amplitude (in mV or uA, depending on whether the output is voltage or current) of each channel during that stage, and the third is the duration (in microseconds) of that stage. 

In the example given above, there are two stages to each pulse. In the first, channel 0 outputs 100uA, channel 1 outputs nothing, and that lasts for 150 usec. Then channel 0 switches to outputting -100uA, and channel 2 outputs -100mV for 200 usec. 






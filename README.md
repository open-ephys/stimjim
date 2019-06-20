Stimjim - a flexible, precise, and inexpensive open-source stimulator
-------------------------------
Stimjim is a current and voltage stimulator for stimulating neural tissue (as with stimulating electrodes). 

![Stimjim picture](photo.png)

# Specifications:

 - Two independently-controllable output channels, each with both current and voltage output modes
 - Total cost: $200 USD.
 - Output range in current mode: -3.33mA to +3.33mA. 
 - Output range in voltage mode: -15V to 15V.
 - Pulse width down to 0.1 ms (as configured with default firmware; down to 0.01 ms voltage pulses with custom firmware)
 - Compliance voltage is +-13.7V in current mode (+/-3.33uA output possible for resistances up to 4k, +-137uA possible for resistances up to 100k)
 - Powered by USB
 - Channels are isolated from power supply and from each other.
 - Onboard ADC measurement of actual output current or voltage
 
# Building your own Stimjim:

Here are the steps to build your very own Stimjim box. 

1. Order [printed circuit boards (PCBs)](./PCB/stimjim_fabricationFiles.zip)  (from [JLCPCB](https://jlcpcb.com/), [Seeedstudio](https://www.seeedstudio.com/fusion.html) or any other PCB manufacturer). 
2. Order [components](./stimjim_BOM.xlsx) (entirely from [Digikey](https://www.digikey.com/)).
3. Solder components onto the PCB, using the [schematic](./schematic.pdf) and [layout](./pcb.pdf) files for reference. This may take a few hours, depending on your soldering ability.
4. Compile and download the [firmware](./stimjimPulser/) onto the Teensy, using the [Arduino IDE](https://www.arduino.cc/en/main/software) with [Teensyduino](https://www.pjrc.com/teensy/td_download.html) installed (or write your own!). Before compiling stimjimPulser.ino, You will need to install the Stimjim library by copying the "lib" folder to the appropriate arduino "libraries" folder on your computer.
5. Stimulate something!

# Getting started using Stimjim:

To generate a pulse train, you first need to send a serial command, terminated by a newline (\n) in order to define a pulse train. An example command would be:

   S0,0,1,2000,1000000; 100,0,150; -100,-100,200
   
The "S" is to *s*pecify the parameters of a pulse train, and the zero immediately following the S implies that we are going to set parameters for PulseTrain #0. The code provided here allows Stimjim to store 10 different PulseTrain parameter sets. If any comma- and semicolon-separated numbers appear following the S, they are the pulse train parameters. A command that simply consists of "S0" will result in simply printing out the current parameters of PulseTrain #0. 

Once a PulseTrain is defined, to run it you would send a command of the form:

   T0

This tells Stimjim to start running PulseTrain #0. If we had parameterized PulseTrain #3, we could just as easily run PulseTrain #3 with the command "T3".

##  Parameterization 

![Alt text](./pulseTrainParametrization.svg)

The numbers that come after an "S" indicate parameters as denoted below:

1. [PulseTrain number], 
2. [output 0 mode],
3. [output 1 mode],
4. [period],
5. [duration];
6. [mV or uA for stage 0 output 0],
7. [mV or uA for stage 0 output 1],
8. [stage 0 duration (ms)]; 
9. [mV or uA for stage 1 output 0],
10. [mV or uA for stage 1 output 1],
11. [stage 1 duration (ms)];
12. (etc for additional stages, up to 10 total stages).

After the "S", the first parameter is the number of the PulseTrain that we are defining. The next two parameters are the modes of each output channels. Modes are as follows:

 - 0 = Voltage output
 - 1 = Current output
 - 2 = Output is disconnected
 - 3 = Output is grounded

Our example command above was: 
   
   S0,0,1,2000,1000000; 100,0,150; -100,-100,200
   
In this example, output 0 is generating a voltage output (mode 0), and output 1 is generating a current output (mode 1). To specify that a channel should not output anything during a pulse train, set the mode to 3 (ground).  

The fourth parameter is the _period_ of the pulse train. In this example, pulses are delivered every 2000 ms.

The fifth parameter is the _duration_ of the pulse train, in microseconds. In the example, the pulse train lasts for one second (1000000 microseconds).

Parameters after the 5th come in sets of 3, and describe a "stage" of a pulse in the train.  The first two parameters are the amplitude (in mV or uA, depending on whether the output is voltage or current) of each channel during that stage, and the third is the duration (in microseconds) of that stage. Note that a semi-colon is before the beginning of each set of 3, and the three parameters themselves are separated from each other by commas.

In the example given above, there are two stages to each pulse. In the first, channel 0 outputs 100mV, channel 1 outputs nothing, and that lasts for 150 usec. Then channel 0 switches to outputting -100mV, and channel 2 outputs -100uA for 200 usec. 






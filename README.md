Microstim - a flexible, precise, and inexpensive open-source stimulator
-------------------------------
Microstim is a current and voltage stimulator for stimulating neural tissue (as with stimulating electrodes). 

# Specifications:

 - Cost is approximately $131 USD per stimulator
 - Two independently-controllable output channels
 - Output range in current mode: -833uA to +833uA. 
 - Pulse width in current mode: down to 50 us.
 - Output range in voltage mode: -12V to 12V.
 - Pulse width in current mode: down to 10 us.
 - In current mode, +-11V compliance voltage (+/-833uA output possible for resistances up to 13k, +-110uA possible for resistances up to 100k)
 - Channels are isolated from power supply (but not from each other).
 - Onboard ADC measurement of actual output current or voltage
 
# Building your own microstim:

Here are the steps to build your very own microstim box. 

1. Order [PCBs](./PCBs/CombinedPCB_fabricationFiles_20181210/)  (from Seeedstudio or any other PCB manufacturer). 
2. Order [components](./microstim_BOM.xlsx) (entirely from digikey, except for the enclosure).
3. Order an enclosure (optional, but it protects the circuit and looks pretty) from Newark.
4. Solder components onto the PCB, using the [schematic](./schematic.pdf) and [layout](./pcb.pdf) files for reference. This may take a few hours, depending on your soldering ability.
5. Compile and download the [firmware](./microstim_teensy/microstim_teensy.ino) onto the Teensy, using the [Arduino IDE](https://www.arduino.cc/en/main/software) with [Teensyduino](https://www.pjrc.com/teensy/td_download.html) installed (or write your own!)
6. Shock something!

# Getting started using microstim:

To generate a pulse train, you need to send a serial command, terminated by a newline (\n). An example command would be:

   T0,3,200,-200,50,2000,1000000
   
The "T" indicates to generate a pulse train. If any comma-separated numbers appear following the T, they are the pulse train parameters. (Any unspecified parameters will use the most recent specification of those parameters, or the defaults if no pulse trains have yet been run.) Pulse train parameters 1-7 are shown graphically below.

![Alt text](./pulseTrainParametrization.svg)

The first two parameters are the mode of each output channels. Modes are as follows:

 - 0 = Voltage output, ADC reads voltage of output
 - 1 = Current output, ADC reads voltage of output 
 - 2 = Current output, ADC reads output current
 - 3 = Output is grounded, ADC reads output current (shunted through a 1k resistor to ground).

In this example, output 0 is generating a voltage output, and output 1 is grounded (not doing anything). 

The third and fourth parameters are the _amplitude_ of the two phases of the pulse, in millivolts if mode is 0 and microamps if mode is non-zero. In the example given above, the pulse would be biphasic, first rising to 200 mV and then falling to -200 mV before returning to 0. 

The fifth parameter is the the _duration of each phase_ of the pulse, in microseconds. In the example, the 200 mV phase would last 50 us, followed by 50 us at -200 mV.

The sixth parameter is the _period_ of the pulse train. In this example, biphasic pulses are delivered every 2000 ms.

The seventh and last parameter is the _duration_ of the pulse train, in microseconds. In the example, the pulse train lasts for one second (1000000 microseconds).




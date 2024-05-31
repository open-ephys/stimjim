/*    Copyright (c) 2019 Nathan Cermak <cerman07 at protonmail.com>.
 *
 *    This file is part of the Stimjim Library for Arduino
 *
 *    The Stimjim Library is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef __STIMJIM_H__
#define __STIMJIM_H__


#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <SPI.h>

#define MICROAMPS_PER_DAC 0.1017            // 20V * (1/(3000 V/A)) / 2^16 = 0.1uA / DAC unit
#define MICROAMPS_PER_ADC 0.85              // (1/(100*(1+49.9k/1.8k) V/A)) * 20V / 2^13 = 0.85
#define MILLIVOLTS_PER_DAC 0.4574           // 20V / 2^16 * gain of 1.505 ~= 0.44 mV / DAC unit
#define MILLIVOLTS_PER_ADC 2.44             // 20V / 2^13 = 2.44 mV / ADC unit

#define OE0_1 6
#define OE1_1 5
#define OE0_0 1
#define OE1_0 0
#define CS0_1 9
#define CS1_1 7
#define CS0_0 3
#define CS1_0 2
#define NLDAC_0 4
#define NLDAC_1 10
#define IN0 22
#define IN1 23
#define LED0 21
#define LED1 20
#define MISO_1 8
#define MISO_0 12
#define GPIO_1 36
#define GPIO_2 36
#define GPIO_3 36
#define GPIO_4 36
#define GPIO_5 36
#define GPIO_6 36
#define GPIO_7 36
#define GPIO_7 36
#define GPIO_9 36
#define GPIO_10 36
#define GPIO_11 36

class StimJim {

  public:
	// begin() should be run before using any other StimJim methods.
	// It initializes the SPI, creates appropriate SPISettings objects for DACs and ADCs,
	// sets all necessary pins to outputs, sets ADC range to +-10V, and measures current offsets.
    void begin();

	// setOutputMode() sets the output mode for the selected channel (0/1).
	// mode 0-3 are: VOLTAGE, CURRENT, HIGH-Z, GROUND.
    void setOutputMode(byte channel, byte mode);

	// Write a 16-bit signed int value to each channel. Outputs will be nearly
	// synchronously updated (sub-microsecond).
    void writeToDacs(int16_t amp0, int16_t amp1);

	// Write a 16-bit signed int value to a single channel.
    void writeToDac(byte channel, int16_t amp);

	// Sets ADC range on both channels to +-2.5V, +-5V, or +-10V (use range=2.5, 5
	// or 10 to select, respectively).
    void setAdcRange(float range);

	void setAdcLine(byte channel, byte line);

	// read the ADC on for a channel. line=0 means measure voltage at output,
	// line=1 means read the value from the current sense instrument amplifier.
    int readAdc(byte channel, byte line);

	// get ADC readings from ground (OUT_0 or OUT_1), when OUT_0 or OUT_1 is grounded.
	// this accounts for intrinsic "bipolar zero error" in AD7321 ADC.
	void getAdcOffsets();


    // getCurrentOffsets() sweeps through a range of DAC values while measuring the
	// current shunted through a 1k resistor (output remains grounded). This enables
	// compensation for imperfect resistor matching and subsequent DC offset in the
	// Howland current pump.
    void getCurrentOffsets();

    // getVoltageOffsets() sweeps through a range of DAC values while measuring the
	// output voltage.
	// Caution!! Anything connected to the output will see a voltage ramp!
	// This enables compensation for any DC offset in the DAC or amplifier.
    void getVoltageOffsets();
	float adcOffset25[2];   // read voltage on both channels at 2.5V range
	float adcOffset10[2];    // read voltage on both channels at 10V range

    int currentOffsets[2];  // These store the compensation for various DC offsets from
    int voltageOffsets[2];  // getVoltageOffsets() and getCurrentOffsets()

  private:
    void setupDacs();
    volatile bool adcSelectedInput[2]; // states of ADCs (are they set to read current or voltage?)
    SPISettings SpiSettingsDac, SpiSettingsAdc;

};

extern StimJim Stimjim;

#endif /* ! defined __STIMJIM_H__ */

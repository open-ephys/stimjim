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
#include "Stimjim.h"

void StimJim::begin(){

  // ------------- SPI setup ------------------------------------- //
  SPI.begin();

  //AD5752 DAC runs at max 30MHz, SPI mode 1 or 2
  SpiSettingsDac = SPISettings(30000000, MSBFIRST, SPI_MODE1);
  //AD7321 settings - clock starts high, data latch on falling edge
  SpiSettingsAdc = SPISettings(10000000, MSBFIRST, SPI_MODE2);

  // ------------- Setup output pins, turn on LEDs ------------------------------------ //
  int pins[] = {NLDAC_0, NLDAC_1, CS0_0, CS1_0, CS0_1, CS1_1, OE0_0, OE1_0, OE0_1, OE1_1, LED0, LED1};
  for (int i = 0; i < 12; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWriteFast(pins[i], HIGH);
  }

  // These follow LED state
  pinMode(GPIO_10, OUTPUT);
  pinMode(GPIO_11, OUTPUT);
  digitalWriteFast(GPIO_10, LOW);
  digitalWriteFast(GPIO_11, LOW);

  setupDacs();

  setAdcLine(0,0);
  setAdcLine(1,0);

  getAdcOffsets();
  getCurrentOffsets();
  //getVoltageOffsets();

  delay(300);
  digitalWriteFast(LED0, LOW);
  digitalWriteFast(LED1, LOW);


}

void StimJim::setOutputMode(byte channel, byte mode) {
  // mode 0-3 are: VOLTAGE, CURRENT, HIGH-Z, GROUND
  digitalWriteFast((channel) ? OE0_1 : OE0_0, (0b00000001 & mode) ? HIGH : LOW);
  digitalWriteFast((channel) ? OE1_1 : OE1_0, (0b00000010 & mode) ? HIGH : LOW);
}



void StimJim::setupDacs() {
  SPI.beginTransaction(SpiSettingsDac);
  for (int i = 0; i < 2; i++) {
    int cs = (i == 0) ? CS0_0 : CS0_1; //CS0_x is DAC for channel x
    for (int j = 0; j < 2; j++) { // first write may be ignored due to undefined powerup state, so repeat 2x
      digitalWrite(cs, LOW);
      SPI.transfer(8); // select "output range" register
      SPI.transfer16(4); // set mode as "+-10V mode (xxx...100)
      digitalWrite(cs, HIGH);
    }
    digitalWrite(cs, LOW);
    SPI.transfer(16); // select "power control" register
    SPI.transfer16(1); // set mode as dac_A powered up
    digitalWrite(cs, HIGH);
    delayMicroseconds(10);
  }
  SPI.endTransaction();
}


void StimJim::writeToDacs(int16_t amp0, int16_t amp1) {
  /* The input shift register is 24 bits wide, MSB first. The input register consists of
     a read/write bit, three register select bits, three DAC address bits, and 16 data bits.
     StimJim's AD5752 uses two's complement. */
  SPI.beginTransaction(SpiSettingsDac);
  digitalWrite(CS0_0, LOW);
  SPI.transfer(0);
  SPI.transfer16(amp0);
  digitalWrite(CS0_0, HIGH);
  digitalWrite(CS0_1, LOW);
  SPI.transfer(0);
  SPI.transfer16(amp1);
  digitalWrite(CS0_1, HIGH);
  SPI.endTransaction();
  // latch data on both DACs
  digitalWrite(NLDAC_0, LOW);
  digitalWrite(NLDAC_1, LOW);
  digitalWrite(NLDAC_0, HIGH);
  digitalWrite(NLDAC_1, HIGH);
}


void StimJim::writeToDac(byte channel, int16_t amp) {
  SPI.beginTransaction(SpiSettingsDac);
  digitalWrite((channel) ? CS0_1 : CS0_0, LOW);
  SPI.transfer(0);     // write to AD5752 is 24-bit, first 8 low means output 0
  SPI.transfer16(amp);
  digitalWrite((channel) ? CS0_1 : CS0_0, HIGH);
  SPI.endTransaction();

  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, LOW);
  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, HIGH);
}



void StimJim::setAdcRange(float range) {
  SPI.beginTransaction(SpiSettingsAdc);
  for (int i = 0; i < 2; i++) {
    int cs = (i == 0) ? CS1_0 : CS1_1; //CS1_x is ADC for channel x
    digitalWrite(cs, LOW); //[15] = write, [13] = range register
    if (range == 2.5) {
      SPI.transfer16(0b1011000100000000); // write range register +-2.5V on both channels
    } else if (range == 5) {
      SPI.transfer16(0b1010100010000000); // write range register +-5V on both channels)
    } else {
      SPI.transfer16(32768 + 8192);       // write range register, +-10V on both channels
    }
    digitalWrite(cs, HIGH);
  }
  SPI.endTransaction();
}


void StimJim::setAdcLine(byte channel, byte line) {
  int cs = (channel) ? CS1_1 : CS1_0;
  SPI.beginTransaction(SpiSettingsAdc);
  digitalWrite(cs, LOW);
  SPI.transfer16(32768 + 1024 * line + 16); //[15-13] = 100 means write control reg; [10] = ADD0; [4] = use internal ref;
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}


int StimJim::readAdc(byte channel, byte line) {

  int cs = (channel) ? CS1_1 : CS1_0;

  SPI.setMISO((channel) ? MISO_1 : MISO_0); // channels 0 & 1 use different MISO lines because isolators dont do tristate
  SPI.beginTransaction(SpiSettingsAdc);
  digitalWrite(cs, LOW);
  if (adcSelectedInput[channel] != line) { // make sure we're reading the correct input (voltage or current)
    adcSelectedInput[channel] = line;
    SPI.transfer16(32768 + 16 + 1024 * line);
    digitalWrite(cs, HIGH);
    digitalWrite(cs, LOW);
  }

  int data = SPI.transfer16(0) - (line ? 8192 : 0);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  if (data > 4095)
    data -= 8192;
  return (data);
}


void StimJim::getAdcOffsets(){
  setOutputMode(0,3);
  setOutputMode(1,3);

  adcOffset25[0] = 0;
  adcOffset25[1] = 0;
  adcOffset10[0] = 0;
  adcOffset10[1] = 0;

  setAdcRange(2.5);
  int x0,x1;
  for (int i = 0; i < 150; i++){
    x0 = readAdc(0,0);
    x1 = readAdc(1,0);
    if (i >= 50) { // sometimes there is an initial transient for first reads - ignore it
	  adcOffset25[0] += x0;
	  adcOffset25[1] += x1;
    }
  }
  setAdcRange(10);
  for (int i = 0; i < 150; i++){
    x0 = readAdc(0,0);
    x1 = readAdc(1,0);
    if (i >= 50) { // sometimes there is an initial transient for first reads - ignore it
	  adcOffset10[0] += x0;
	  adcOffset10[1] += x1;
    }
  }
  adcOffset25[0] /= 100;
  adcOffset25[1] /= 100;
  adcOffset10[0] /= 100;
  adcOffset10[1] /= 100;
}



void StimJim::getCurrentOffsets() {
  float adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setOutputMode(1, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setAdcRange(2.5);
  for (int i = -100; i < 100; i++) {
    writeToDacs(i, i);
    delayMicroseconds(50);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 100; j++) {
      adcReadSum[0] += readAdc(0, 1) - adcOffset25[0]; // channel(0,1), line(V/I)
      adcReadSum[1] += readAdc(1, 1) - adcOffset25[1];
    }
    // Serial.print(i); Serial.print(",");
    // Serial.print(adcReadSum[0]); Serial.print(","); Serial.println(adcReadSum[1]);
    for (int channel = 0; channel < 2; channel++) {
      if (abs(adcReadSum[channel]) < bestDcVal[channel]) {
        currentOffsets[channel] = i;
        bestDcVal[channel] = abs(adcReadSum[channel]);
      }
    }
  }
  writeToDacs(currentOffsets[0], currentOffsets[1]);
  setAdcRange(10);
  //char str[40];
  //sprintf(str, "Best current offsets: %d, %d\n", currentOffsets[0], currentOffsets[1]);
  //Serial.print(str);
}


void StimJim::getVoltageOffsets() {
  float adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 0); // VOLTAGE OUTPUT - DANGEROUS
  setOutputMode(1, 0); // VOLTAGE OUTPUT - DANGEROUS
  setAdcRange(2.5);
  for (int i = -100; i < 100; i++) {
    writeToDacs(i, i);
    delayMicroseconds(30);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 100; j++) {
      adcReadSum[0] += readAdc(0, 0) - adcOffset25[0]; // channel(0,1), line(V/I)
      adcReadSum[1] += readAdc(1, 0) - adcOffset25[1];
    }
    // Serial.print(i); Serial.print(",");
    // Serial.print(adcReadSum[0]); Serial.print(","); Serial.println(adcReadSum[1]);
    for (int channel = 0; channel < 2; channel++) {
      if (abs(adcReadSum[channel]) < bestDcVal[channel]) {
        voltageOffsets[channel] = i;
        bestDcVal[channel] = abs(adcReadSum[channel]);
      }
    }
  }

  writeToDacs(voltageOffsets[0], voltageOffsets[1]);
  setOutputMode(0, 3);
  setOutputMode(1, 3);
  setAdcRange(10);
  //char str[40];
  //sprintf(str, "Best voltage offsets: %d, %d\n", voltageOffsets[0], voltageOffsets[1]);
  //Serial.print(str);
}

StimJim Stimjim;




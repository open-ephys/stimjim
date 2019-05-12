#include "Stimjim.h"

// ------------- SPI setup ------------------------------------- //
//AD5752 DAC runs at max 30MHz, SPI mode 1 or 2
SPISettings SpiSettingsDAC(10000000, MSBFIRST, SPI_MODE1); // Does not seem to work at 20MHz!
//AD7321 settings - clock starts high, data latch on falling edge
SPISettings SpiSettingsADC(5000000, MSBFIRST, SPI_MODE2);  // Can't run SPI clock faster than 1MHz without errors


// ------------- Compensation for various DC offsets ---------- //
int currentOffsets[2];
int voltageOffsets[2];

// --------- States of ADCs (set to read current or voltage?) ---//
volatile bool adcSelectedInput[2];


void setOutputMode(byte channel, byte mode) {
  // mode 0-3 are: VOLTAGE, CURRENT, HIGH-Z, GROUND
  digitalWrite((channel) ? OE0_1 : OE0_0, (0b00000001 & mode) ? HIGH : LOW);
  digitalWrite((channel) ? OE1_1 : OE1_0, (0b00000010 & mode) ? HIGH : LOW);
}



void setADCrange(float range) {
  SPI.beginTransaction(SpiSettingsADC);
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

void setupDACs() {
  SPI.beginTransaction(SpiSettingsDAC);
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


void writeToDacs(int16_t amp0, int16_t amp1) {
  /* The input shift register is 24 bits wide, MSB first. The input register consists of
     a read/write bit, three register select bits, three DAC address bits, and 16 data bits.
     Stimjim's AD5752 uses two's complement. */
  SPI.beginTransaction(SpiSettingsDAC);
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


void writeToDac(byte channel, int16_t amp) {
  SPI.beginTransaction(SpiSettingsDAC);
  digitalWrite((channel) ? CS0_1 : CS0_0, LOW);
  SPI.transfer(0);     // write to AD5752 is 24-bit, first 8 low means output 0
  SPI.transfer16(amp);
  digitalWrite((channel) ? CS0_1 : CS0_0, HIGH);
  SPI.endTransaction();

  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, LOW);
  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, HIGH);
}



int readADC(byte channel, byte line) {

  int cs = (channel) ? CS1_1 : CS1_0;

  SPI.setMISO((channel) ? MISO_1 : MISO_0); // channels 0 & 1 use different MISO lines because isolators dont do tristate
  SPI.beginTransaction(SpiSettingsADC);
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


void getCurrentOffsets() {
  int adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setOutputMode(1, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setADCrange(2.5);
  for (int i = -100; i < 100; i++) {
    writeToDacs(i, i);
    delayMicroseconds(50);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 100; j++) {
      adcReadSum[0] += readADC(0, 1); // channel(0,1), line(V/I)
      adcReadSum[1] += readADC(1, 1);
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
  setADCrange(10);
  char str[40];
  sprintf(str, "Best current offsets: %d, %d\n", currentOffsets[0], currentOffsets[1]);
  Serial.print(str);
}


void getVoltageOffsets() {
  int adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 0); // VOLTAGE OUTPUT - DANGEROUS
  setOutputMode(1, 0); // VOLTAGE OUTPUT - DANGEROUS
  setADCrange(2.5);
  for (int i = -100; i < 100; i++) {
    writeToDacs(i, i);
    delayMicroseconds(30);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 100; j++) {
      adcReadSum[0] += readADC(0, 0); // channel(0,1), line(V/I)
      adcReadSum[1] += readADC(1, 0);
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
  setADCrange(10);
  char str[40];
  sprintf(str, "Best voltage offsets: %d, %d\n", voltageOffsets[0], voltageOffsets[1]);
  Serial.print(str);
}



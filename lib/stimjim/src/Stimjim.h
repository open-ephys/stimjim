/* Stimjim library (c) Nathan Cermak 2019. */

#ifndef __STIMJIM_H__
#define __STIMJIM_H__


#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <SPI.h>

#define MICROAMPS_PER_DAC 0.1017            // 20V * (1/(3000 V/A)) / 2^16 = 0.1uA / DAC unit
#define MICROAMPS_PER_ADC 0.85              //  (1/(100*(1+49.9k/1.8k) V/A)) * 20V / 2^13 = 0.85
#define MILLIVOLTS_PER_DAC 0.442            // 20V / 2^16 * gain of 1.45 ~= 0.44 mV / DAC unit
#define MILLIVOLTS_PER_ADC 2.44             // 20V / 2^13 = 2.44 mV / ADC unit

#define OE0_0 6
#define OE1_0 5
#define OE0_1 1
#define OE1_1 0
#define CS0_0 9
#define CS1_0 7
#define CS0_1 3
#define CS1_1 2
#define NLDAC_1 4
#define NLDAC_0 10
#define IN0 22
#define IN1 23
#define LED0 21
#define LED1 20
#define MISO_0 8
#define MISO_1 12





// ------------ Function prototypes --------------------------- //
void setOutputMode(byte channel, byte mode);
void setADCrange(float range);
void setupDACs();
void writeToDacs(int16_t amp0, int16_t amp1);
void writeToDac(byte channel, int16_t amp);
int readADC(byte channel, byte line);
void getCurrentOffsets();
void getVoltageOffsets();


// ------------- Compensation for various DC offsets ---------- //
extern int currentOffsets[2];
extern int voltageOffsets[2];

// --------- States of ADCs (set to read current or voltage?) ---//
extern volatile bool adcSelectedInput[2];


#endif /* ! defined __STIMJIM_H__ */

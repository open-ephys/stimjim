/* stimjim_teensy (c) Nathan Cermak 2019. */

#include <SPI.h>

#define MICROAMPS_PER_DAC 0.1017            // 20V * (1/(3000 V/A)) / 2^16 = 0.1uA / DAC unit
#define MICROAMPS_PER_ADC 0.85              //  (1/(100*(1+49.9k/1.8k) V/A)) * 20V / 2^13 = 0.85
#define MILLIVOLTS_PER_DAC 0.442            // 20V / 2^16 * gain of 1.45 ~= 0.44 mV / DAC unit
#define MILLIVOLTS_PER_ADC 2.44             // 20V / 2^13 = 2.44 mV / ADC unit

#define PT_ARRAY_LENGTH 10
#define MAX_NUM_STAGES 10

#define OE0_0 7
#define OE1_0 6
#define OE0_1 2
#define OE1_1 1
#define CS0_0 9
#define CS1_0 8
#define CS0_1 4
#define CS1_1 3
#define NLDAC_1 5
#define NLDAC_0 10
#define IN0 22
#define IN1 23
#define LED0 21
#define LED1 20

// ------------- SPI setup ------------------------------------- //
//AD5752 DAC runs at max 30MHz, SPI mode 1 or 2
SPISettings settingsDAC(10000000, MSBFIRST, SPI_MODE1); // Does not seem to work at 20MHz!
//AD7321 settings - clock starts high, data latch on falling edge
SPISettings settingsADC(5000000, MSBFIRST, SPI_MODE2);  // Can't run SPI clock faster than 1MHz without errors

// ------------- Serial setup ---------------------------------- //
char comBuf[1000];
int bytesRecvd;

// ------------- Compensation for various DC offsets ---------- //
int currentOffsets[2];
int voltageOffsets[2];

// ------------- States of ADCs --------------------------------//
volatile bool adcSelectedInput[2];

// ------------- PulseTrain parameter setup -------------------- //
struct PulseTrain {
  unsigned int mode[2];
  unsigned int period;                          // usec
  unsigned long duration;                       // usec

  int nStages;
  int amplitude[2][MAX_NUM_STAGES];             // mV or uA, depending on mode
  unsigned int stageDuration[MAX_NUM_STAGES];   // usec

  unsigned long trainStartTime;                 // usec
  int nPulses;
  int measuredAmplitude[4][MAX_NUM_STAGES];     // Ch0_V, Ch0_I, Ch1_V, Ch1_I
};
volatile PulseTrain PTs[PT_ARRAY_LENGTH];
volatile PulseTrain *activePT0, *activePT1;
IntervalTimer IT0, IT1;

// ------------ Globals for trigger status -------------------- //
int triggerTargetPTs[2];

// ------------ Function prototypes --------------------------- //
void setupADCs();
void setupDACs();
void setOutputMode(byte channel, byte mode);
void writeToDacs(int16_t amp0, int16_t amp1);
void writeToDac(byte channel, int16_t amp);
int readADC(byte channel, byte line);

void getCurrentOffsets();
void getVoltageOffsets();

int pulse (volatile PulseTrain* PT);
void printTrainResultSummary(volatile PulseTrain* PT);
void pulse0();
void pulse1();
void startIT0(int ptIndex);
void startIT0ViaInputTrigger();
void startIT1(int ptIndex);
void startIT1ViaInputTrigger();

void printPulseTrainParameters(int i);
volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT);








void setOutputMode(byte channel, byte mode) {
  // mode 0-3 are: VOLTAGE, CURRENT, HIGH-Z, GROUND
  digitalWrite((channel) ? OE0_1 : OE0_0, (0b00000001 & mode) ? HIGH : LOW);
  digitalWrite((channel) ? OE1_1 : OE1_0, (0b00000010 & mode) ? HIGH : LOW);
}


void writeToDacs(int16_t amp0, int16_t amp1) {
  /* The input shift register is 24 bits wide, MSB first. The input register consists of 
   * a read/write bit, three register select bits, three DAC address bits, and 16 data bits.
   * Stimjim's AD5752 uses two's complement. */
  SPI.beginTransaction(settingsDAC);
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
  SPI.beginTransaction(settingsDAC);
  digitalWrite((channel) ? CS0_1 : CS0_0, LOW);
  SPI.transfer(0);
  SPI.transfer16(amp);
  digitalWrite((channel) ? CS0_1 : CS0_0, HIGH);
  SPI.endTransaction();

  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, LOW);
  digitalWrite((channel) ? NLDAC_1 : NLDAC_0, HIGH);
}


void setupADCs() {
  SPI.beginTransaction(settingsADC);
  for (int i = 0; i < 2; i++) {
    int cs = (i == 0) ? CS1_0 : CS1_1; //CS1_x is ADC for channel x
    digitalWrite(cs, LOW); //[15] = write, [13] = range register
    SPI.transfer16(32768 + 8192); // write range register, all zeros (+-10V on both channels)
    digitalWrite(cs, HIGH);
    delayMicroseconds(100);
    digitalWrite(cs, LOW);
    SPI.transfer16(32768 + 16); // write control register, read channel 0 (voltage), use internal ref, 
    digitalWrite(cs, HIGH);
  }
  SPI.endTransaction();
}

void setupDACs() {
  SPI.beginTransaction(settingsDAC);
  for (int i = 0; i < 2; i++) {
    int cs = (i == 0) ? CS0_0 : CS0_1; //CS0_x is DAC for channel x
    for (int j=0; j<2; j++){ // first write may be ignored due to undefined powerup state, so repeat 2x
      digitalWrite(cs, LOW); 
      SPI.transfer(8); // select "output range" register
      SPI.transfer16(4); // set mode as "+-10V mode (xxx...100)
      digitalWrite(cs, HIGH);
    }
    digitalWrite(cs, LOW);
    SPI.transfer(16); // select "power control" register
    SPI.transfer16(1); // set mode as dac_A powered up
    digitalWrite(cs, HIGH);
    delayMicroseconds(100);
  }
  SPI.endTransaction();
}



int readADC(byte channel, byte line) {
  SPI.beginTransaction(settingsADC);
  digitalWrite((channel) ? CS1_1 : CS1_0, LOW);
  delayMicroseconds(10);     // TODO - figure out why exactly this is necessary. 10us = NO errors, 3us = rare errors
  if(adcSelectedInput[channel] != line){
    adcSelectedInput[channel] = line;
    SPI.transfer16(32768 + 16 + 1024*line);
    digitalWrite((channel) ? CS1_1 : CS1_0, HIGH);
    digitalWrite((channel) ? CS1_1 : CS1_0, LOW);
  }

  int data = SPI.transfer16(0) - (line ? 8192 : 0);
  digitalWrite((channel) ? CS1_1 : CS1_0, HIGH);
  SPI.endTransaction();
  if (data > 4095)
    data -= 8192;
  return(data);
}


void getCurrentOffsets() {
  int adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setOutputMode(1, 3); // output grounded, Iout goes to ground via 1k on-board resistor

  for (int i = -500; i < 500; i++) {
    writeToDacs(i, i);
    delayMicroseconds(30);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 10; j++) {
      adcReadSum[0] += readADC(0, 1);
      delayMicroseconds(10);      
      adcReadSum[1] += readADC(1, 1);
      delayMicroseconds(10);     
    } 
    Serial.print(i); Serial.print(","); 
    Serial.print(adcReadSum[0]); Serial.print(","); Serial.println(adcReadSum[1]); 
    for (int channel = 0; channel < 2; channel++) {
      if (abs(adcReadSum[channel]) < bestDcVal[channel]) {
        currentOffsets[channel] = i;
        bestDcVal[channel] = abs(adcReadSum[channel]);
      }
    }
  }
  writeToDacs(currentOffsets[0], currentOffsets[1]);

  char str[40];
  sprintf(str, "Best current offsets: %d, %d\n", currentOffsets[0], currentOffsets[1]);
  Serial.print(str);
}


void getVoltageOffsets() {
  int adcReadSum[2];
  int bestDcVal[2] = {10000, 10000};

  setOutputMode(0, 0); // VOLTAGE OUTPUT - DANGEROUS
  setOutputMode(1, 0); // VOLTAGE OUTPUT - DANGEROUS

  for (int i = -500; i < 500; i++) {
    writeToDacs(i, i);
    delayMicroseconds(20);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j = 0; j < 10; j++) {
      adcReadSum[0] += readADC(0, 0);
      delayMicroseconds(10);
      adcReadSum[1] += readADC(1, 0);
    }
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

  char str[40];
  sprintf(str, "Best voltage offsets: %d, %d\n", voltageOffsets[0], voltageOffsets[1]);
  Serial.print(str);
}


int pulse (volatile PulseTrain* PT) {
  if (PT->nPulses == 0)
    PT->trainStartTime = micros();

  if (micros() - PT->trainStartTime >= PT->duration)
    return 0;

  setOutputMode(0, PT->mode[0]);
  setOutputMode(1, PT->mode[1]);

  for (int i = 0; i < PT->nStages; i++) {
    // minimum time is 5us, seems to actually be 10us
    writeToDacs(PT->amplitude[0][i] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? currentOffsets[0] : voltageOffsets[0]), //* (PT->mode[0] != 3),
                PT->amplitude[1][i] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? currentOffsets[1] : voltageOffsets[1])); //* (PT->mode[1] != 3));

    // add 10 if no read, 47 if read (4x)
    delayMicroseconds(max( ((long int) PT->stageDuration[i])-67, 0));

    // note: this limits bandwidth for voltage pulses
    PT->measuredAmplitude[0][i] += readADC(0, PT->mode[0] > 0) * ((PT->mode[0])?MICROAMPS_PER_ADC:MILLIVOLTS_PER_ADC); 
    PT->measuredAmplitude[1][i] += readADC(0, PT->mode[1] > 0) * ((PT->mode[1])?MICROAMPS_PER_ADC:MILLIVOLTS_PER_ADC); 

    //Serial.print(round(adcValue[0] * MILLIVOLTS_PER_ADC)); Serial.print("\t");
    //Serial.print(round(adcValue[1] * MICROAMPS_PER_ADC));  Serial.print("\t");
    //Serial.print(round(adcValue[0] * MILLIVOLTS_PER_ADC)); Serial.print("\t");
    //Serial.print(round(adcValue[1] * MICROAMPS_PER_ADC));  Serial.println(""); 
  }
  writeToDacs((PT->mode[0]) ? currentOffsets[0] : voltageOffsets[0],
              (PT->mode[1]) ? currentOffsets[1] : voltageOffsets[1]);
  setOutputMode(0, 3);
  setOutputMode(1, 3);
  PT->nPulses++;

  return 1;
}


void printTrainResultSummary(volatile PulseTrain* PT) {
  Serial.print("Train complete. Delivered "); Serial.print(PT->nPulses);
  Serial.println(" pulses.\nCurrent/Voltage by stage: ");
  Serial.println("           Ch0                Ch1 ");
  char str[200];
  for (int i = 0; i < PT->nStages; i++) {
    Serial.print("Stage "); Serial.print(i);
    sprintf(str, "%6d%s,          ", PT->measuredAmplitude[0][i] / PT->nPulses, (PT->mode[0])?"uA":"mV");
    Serial.print(str);
    sprintf(str, "%6d%s,          ", PT->measuredAmplitude[1][i] / PT->nPulses, (PT->mode[1])?"uA":"mV");
    Serial.println(str);
  }
}

void pulse0() {
  if (!pulse(activePT0)) {
    IT0.end();
    printTrainResultSummary(activePT0);
    if(activePT0->mode[0] < 2)  digitalWrite(LED0, LOW);
    if(activePT0->mode[1] < 2)  digitalWrite(LED1, LOW);
  }
}

void pulse1() {
  if (!pulse(activePT1)) {
    IT1.end();
    printTrainResultSummary(activePT1);
    if(activePT1->mode[0] < 2)  digitalWrite(LED0, LOW);
    if(activePT1->mode[1] < 2)  digitalWrite(LED1, LOW);
  }
}

void startIT0ViaInputTrigger() {
  if (triggerTargetPTs[0] >= 0)
    startIT0(triggerTargetPTs[0]);
}

void startIT0(int ptIndex) {
  activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
  Serial.print("\nStarting T train with parameters of PulseTrain "); Serial.println(ptIndex);
  if(activePT0->mode[0] < 2)  digitalWrite(LED0, HIGH);
  if(activePT0->mode[1] < 2)  digitalWrite(LED1, HIGH);
  if (!IT0.begin(pulse0, activePT0->period))
    Serial.println("startIT0: failure to initiate IntervalTimer IT0");
  pulse0(); //intervalTimer starts with delay - we want to start with pulse!
}

void startIT1ViaInputTrigger() {
  if (triggerTargetPTs[1] >= 0)
    startIT1(triggerTargetPTs[1]);
}

void startIT1(int ptIndex) {
  activePT1 = clearPulseTrainHistory(&PTs[ptIndex]);
  Serial.print("\nStarting U train with parameters of PulseTrain "); Serial.println(ptIndex);
  if(activePT1->mode[0] < 2)  digitalWrite(LED0, HIGH);
  if(activePT1->mode[1] < 2)  digitalWrite(LED1, HIGH);

  if (!IT1.begin(pulse1, activePT1->period))
    Serial.println("startIT1: failure to initiate IntervalTimer IT1");
  pulse1(); //intervalTimer starts with delay - we want to start with pulse!
}


void printPulseTrainParameters(int i) {
  if (i < 0 || i >= PT_ARRAY_LENGTH) {
    Serial.println("Invalid PulseTrain array index.");
    return;
  }

  const char modeStrings[4][40] = {"Voltage output","Current output", "No output (high-Z)", "No output (grounded)"};
  Serial.println("----------------------------------");
  char str[200];
  sprintf(str, "Parameters for PulseTrain[%d]\n  mode[ch0]: %d (%s)\n  mode[ch1]: %d (%s)\n",
    i, PTs[i].mode[0], modeStrings[PTs[i].mode[0]], PTs[i].mode[1], modeStrings[PTs[i].mode[1]]);
  Serial.print(str);
  sprintf(str, "  period:    %d usec (%0.3f sec, %0.3f Hz)\n  duration: %lu usec (%0.3f sec)\n",
          PTs[i].period, 0.000001 * PTs[i].period, 1000000.0 / PTs[i].period, PTs[i].duration, 0.000001 * PTs[i].duration);
  Serial.print(str);
  Serial.println("\n  stage    duration     output0   output1");
  for (int j = 0; j < PTs[i].nStages; j++) {
    sprintf(str, "   %2d  %7d usec %8d%s %8d%s\n", j, PTs[i].stageDuration[j],
            PTs[i].amplitude[0][j], (PTs[i].mode[0] == 0) ? "mV" : "uA",
            PTs[i].amplitude[1][j], (PTs[i].mode[1] == 0) ? "mV" : "uA");
    Serial.print(str);
  }
  Serial.println("----------------------------------\n");
}


volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT) {
  PT->nPulses = 0;
  for (int i = 0; i < PT->nStages; i++)
    for (int j=0; j < 4; j++)
      PT->measuredAmplitude[j][i] = 0;
  return (PT);
}









void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  delay(1000);
  
  Serial.begin(112500);
  bytesRecvd = 0;
  delay(1000);
  Serial.println("Booting StimJim on Teensy 3.5!");

  Serial.println("Initializing pins...");
  int pins[] = {NLDAC_0, NLDAC_1, CS0_0, CS1_0, CS0_1, CS1_1, OE0_0, OE1_0, OE0_1, OE1_1, LED0, LED1};
  for (int i = 0; i < 12; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);

  Serial.println("Initializing SPI...");
  SPI.begin();
  
  Serial.println("Initializing DACs and ADCs...");
  setupADCs();
  setupDACs();

  
  Serial.println("Measuring DC offsets...");
  getCurrentOffsets();
  getVoltageOffsets();
  
  for (int i = 0; i < PT_ARRAY_LENGTH; i++) {
    PTs[i].mode[0] = 0;
    PTs[i].mode[1] = 0;
    PTs[i].period = 10000;
    PTs[i].duration = 10000000;
    PTs[i].nStages = 2;
    for (int j = 0; j < MAX_NUM_STAGES; j++) {
      PTs[i].amplitude[0][j] = j ? -1000 : 1000;
      PTs[i].amplitude[1][j] = j ? 1000 : -1000;
      PTs[i].stageDuration[j] = 100;
    }
  }

  PTs[1].mode[0] = 1;
  PTs[1].mode[1] = 1;
  for (int j = 0; j < MAX_NUM_STAGES; j++) {
    PTs[1].amplitude[0][j] = j ? -100 : 100;
    PTs[1].amplitude[1][j] = j ? 100 : 1 - 00;
    PTs[1].stageDuration[j] = 100;
  }

  Serial.println("Initializing inputs...");
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);
  triggerTargetPTs[0] = -1; // initialize target to -1 so that triggers do nothing
  triggerTargetPTs[1] = -1;

  adcSelectedInput[0] = 0;
  adcSelectedInput[1] = 0;
  
  Serial.println("Ready to go!\n\n");
}





void loop() {

  if (Serial.available() > 0) {
    Serial.readBytes(comBuf + bytesRecvd, 1); // read one byte into the buffer
    bytesRecvd++; // keep track of the number of characters we've read!

    if (comBuf[bytesRecvd - 1] == '\n') { // termination character for string - means we've recvd a full command!
      unsigned int ptIndex = 0;

      if (comBuf[0] == 'S') {
        sscanf(comBuf + 1, "%u,", &ptIndex);
        if (ptIndex >= PT_ARRAY_LENGTH) {
          Serial.println("Invalid PulseTrain index.");
          bytesRecvd = 0;
          return;
        }

        int m = sscanf(comBuf + 1, "%*d,%u,%u,%u,%lu;",
                       &(PTs[ptIndex].mode[0]),
                       &(PTs[ptIndex].mode[1]),
                       &(PTs[ptIndex].period),
                       &(PTs[ptIndex].duration));
        if (PTs[ptIndex].mode[0] < 0 || PTs[ptIndex].mode[0] > 3) PTs[ptIndex].mode[0] = 3;
        if (PTs[ptIndex].mode[1] < 0 || PTs[ptIndex].mode[1] > 3) PTs[ptIndex].mode[1] = 3;
        
        if (m == 4) {
          PTs[ptIndex].nStages = 0;
          char *token = strtok(comBuf + 1, ";");
          token = strtok(NULL, ";"); //move to the 2nd segment delimited by ";"

          while (token != NULL) {
            m = sscanf(token, "%d,%d,%u",
                       &(PTs[ptIndex].amplitude[0][PTs[ptIndex].nStages]),
                       &(PTs[ptIndex].amplitude[1][PTs[ptIndex].nStages]),
                       &(PTs[ptIndex].stageDuration[PTs[ptIndex].nStages]));
            if (m != 3)
              break;
            PTs[ptIndex].nStages++;
            token = strtok(NULL, ";");
          }
        }
        printPulseTrainParameters(ptIndex);
      }

      else if (comBuf[0] == 'T' || comBuf[0] == 'U') {
        ptIndex = atoi(comBuf + 1);
        if (ptIndex >= PT_ARRAY_LENGTH) {
          Serial.println("Invalid PulseTrain index.");
          bytesRecvd = 0;
          return;
        }
        if (comBuf[0] == 'T') startIT0(ptIndex);
        if (comBuf[0] == 'U') startIT1(ptIndex);
      }

      else if (comBuf[0] == 'D') {
        //sscanf(comBuf + 1, "%d,%d", &(currentOffsets[0]), &(currentOffsets[1]) );
        Serial.print("currentOffsets[0]: "); Serial.println(currentOffsets[0]);
        Serial.print("currentOffsets[1]: "); Serial.println(currentOffsets[1]);
      }

      else if (comBuf[0] == 'C') {
        getCurrentOffsets();
        getVoltageOffsets();
      }

      else if (comBuf[0] == 'R') {
        int trigSrc = 0, ptIndex = 0, falling = 0;
        sscanf(comBuf + 1, "%d,%d,%d", &trigSrc, &ptIndex, &falling );
        if (ptIndex >= PT_ARRAY_LENGTH) {
          Serial.println("Invalid PulseTrain index.");
          bytesRecvd = 0;
          return;
        }
        if (ptIndex >= 0) {
          Serial.print("Attaching interrupt to IN"); Serial.print(trigSrc);
          Serial.print(" to run PulseTrain["); Serial.print(ptIndex); Serial.println("]");
          triggerTargetPTs[trigSrc] = ptIndex;
          attachInterrupt( (trigSrc) ? IN1 : IN0, (trigSrc) ? startIT1ViaInputTrigger : startIT0ViaInputTrigger,
                           (falling) ? FALLING : RISING);
        }
        else {
          Serial.print("Detaching interrupt to IN"); Serial.println(trigSrc);
          detachInterrupt( (trigSrc) ? IN1 : IN0);
          triggerTargetPTs[trigSrc] = -1;
        }
      }

      bytesRecvd = 0; // reset the pointer!
    }
  }
}

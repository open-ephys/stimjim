/* microstim_teensy (c) Nathan Cermak 2019.
 *  
 */

#include <SPI.h> 

// In voltage mode, Vout = 4.758*Vin - 12.08. 

#define MICROAMPS_PER_DAC 0.40690104166     // 5V * (1/(3000 V/A)) / 2^12 = 0.4uA / DAC unit
#define MICROAMPS_PER_ADC 0.2325            // 20V * (1/(10500 V/A)) / 2^13 = 0.2325
#define MILLIVOLTS_PER_DAC 5.81             // 5V / 2^12 * gain of 4.758 ~= 6 mV / DAC unit
#define MILLIVOLTS_PER_ADC 2.44             // 20V / 2^13 = 2.44 mV / ADC unit
#define SLEW_FUDGE 10                       // microseconds
#define OE0 6
#define OE1 5
#define OE2 4
#define OE3 3
#define CS0 8
#define CS1 7
#define NLDAC 9
#define PT_ARRAY_LENGTH 10
#define MAX_NUM_PHASES 10


// ------------- SPI setup ------------------------------------- //
//MCP4922 DAC runs at max 20MHz, mode 0,0 or 1,1 acceptable
SPISettings settingsDAC(16000000, MSBFIRST, SPI_MODE0);
//AD7321 settings - clock starts high, data latch on falling edge
SPISettings settingsADC(5000000, MSBFIRST, SPI_MODE2);  

// ------------- Serial setup ---------------------------------- //
char comBuf[1000];
int nChar;

// ------------- Compensation for imperfect resistors ---------- //
int currentOffsets[2];
int voltageOffsets[2];

// ------------- PulseTrain parameter setup -------------------- //
struct PulseTrain {
  unsigned int mode[2];
  unsigned int period;                          // usec
  unsigned long duration;                       // usec

  int nPhases;
  int amplitude[2][MAX_NUM_PHASES];             // mV or uA, depending on mode
  unsigned int phaseDuration[MAX_NUM_PHASES];   // usec

  unsigned long trainStartTime;                 // usec 
  int nPulses;
  int measuredAmplitude[2][MAX_NUM_PHASES];
};
volatile PulseTrain PTs[PT_ARRAY_LENGTH];
volatile PulseTrain *activePT0, *activePT1;
IntervalTimer IT0, IT1;

// ------------ Function prototypes --------------------------- //
void writeToDacs(int amp0, int amp1);
void writeToDac(int ch, int amp);
void setupADC();
void readADC(int *data);
void pulse0();
void pulse1();
void getCurrentOffsets();





void setOutputMode(byte channel, byte mode) {
  /* Output Input
      VOUT   VOUT       mode=0
      IOUT   VOUT       mode=1
      IOUT   ISENSE     mode=2
      GND    ISENSE     mode=3  */
  if (channel == 0) {
    digitalWrite(OE0, (0b00000001 & mode) ? HIGH : LOW);
    digitalWrite(OE1, (0b00000010 & mode) ? HIGH : LOW);
  } else if (channel == 1) {
    digitalWrite(OE2, (0b00000001 & mode) ? HIGH : LOW);
    digitalWrite(OE3, (0b00000010 & mode) ? HIGH : LOW);
  } else {
    Serial.println("Invalid channel specified!");
  }
}


// input arguments amp0 and amp1 go from -2048 to 2047.
void writeToDacs(int amp0, int amp1) {
  // convert to unsigned value
  amp0 = amp0 + 2048;
  amp1 = amp1 + 2048;
  // make sure values saturate and don't exceed [0-4095]
  if (amp0 > 4095) amp0 = 4095;
  if (amp1 > 4095) amp1 = 4095;
  if (amp0 < 0) amp0 = 0;
  if (amp1 < 0) amp1 = 0;

  SPI.beginTransaction(settingsDAC);
  //bit order for DAC (MCP4922) is notA, buf, notGA, notShutdown, Data11,Data10...Data0
  digitalWrite(CS0, LOW);
  SPI.transfer16(8192 + 4096 + amp0);
  digitalWrite(CS0, HIGH);
  digitalWrite(CS0, LOW);
  SPI.transfer16(32768 + 8192 + 4096 + amp1);
  digitalWrite(CS0, HIGH);
  SPI.endTransaction();

  // latch data
  digitalWrite(NLDAC, LOW);
  digitalWrite(NLDAC, HIGH);
}


// amp goes from -2048 to 2047.
void writeToDac(int channel, int amp) {
  amp = amp + 2048;
  if (amp > 4095) amp = 4095;
  if (amp < 0) amp = 0;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(CS0, LOW);
  SPI.transfer16( (channel ? 32768 : 0) + 8192 + 4096 + amp);
  digitalWrite(CS0, HIGH);
  SPI.endTransaction();

  digitalWrite(NLDAC, LOW);
  digitalWrite(NLDAC, HIGH);
}


/*15: write
  13: control register (0) or range register(1)
  10: add0 - selects which channel to read on next conversion. should use 1 if using sequencer.
  9: mode1 (0 for single-ended)
  8: mode0 (0 for single-ended)
  7: PM1 (0 for on all time)
  6: PM0 (0 for on all time)
  5: coding (0 = two's complement, 1 = binary)
  4: reference (0 = external; 1=internal)
  3: sequencer - if (1,0), then alternate channels
  2: sequencer */
void setupADC() {
  SPI.beginTransaction(settingsADC);
  digitalWrite(CS1, LOW);
  int data = 32768 + 8192; // write range register, all zeros (+-10V on both channels)
  SPI.transfer16(data);
  digitalWrite(CS1, HIGH);
  delayMicroseconds(100);
  digitalWrite(CS1, LOW);
  data = 32768 + 1024 + 16 + 8; // write control register, ch1, use internal ref, use sequencer
  SPI.transfer16(data);
  digitalWrite(CS1, HIGH);
  SPI.endTransaction();
}


void readADC(int *data) {
  SPI.beginTransaction(settingsADC);
  for (int i =0; i<2; i++){
    digitalWrite(CS1, LOW);
    data[i] = SPI.transfer16(0) - (i?8192:0);
    digitalWrite(CS1, HIGH);
    if (data[i] > 4095)
      data[i] -= 8192;
    //Serial.print(data[i]); Serial.print(" ");
  }
  //Serial.println("");
  SPI.endTransaction();
}




void getCurrentOffsets() {
  int lastAdcRead[2], adcReadSum[2];
  int bestDcVal[2] = {10000,10000};
  
  setOutputMode(0, 3); // no output, Iout goes to ground via 1k on-board resistor, ADC reads Isense
  setOutputMode(1, 3); // no output, Iout goes to ground via 1k on-board resistor, ADC reads Isense
  
  for (int i = -500; i < 500; i++) {
    writeToDacs(i, i);
    delayMicroseconds(100);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j=0; j<100; j++){
      readADC(lastAdcRead);
      adcReadSum[0]+=lastAdcRead[0];
      adcReadSum[1]+=lastAdcRead[1];
    }
    for (int channel=0; channel < 2; channel++){
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
  int lastAdcRead[2], adcReadSum[2];
  int bestDcVal[2] = {10000,10000};
  
  setOutputMode(0, 0); // VOLTAGE OUTPUT - DANGEROUS
  setOutputMode(1, 0); // VOLTAGE OUTPUT - DANGEROUS
  
  for (int i = -500; i < 500; i++) {
    writeToDacs(i, i);
    delayMicroseconds(20);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j=0; j<100; j++){
      readADC(lastAdcRead);
      adcReadSum[0]+=lastAdcRead[0];
      adcReadSum[1]+=lastAdcRead[1];
    }
    for (int channel=0; channel < 2; channel++){
      if (abs(adcReadSum[channel]) < bestDcVal[channel]) {
        voltageOffsets[channel] = i;
        bestDcVal[channel] = abs(adcReadSum[channel]);
      }
    }
  }

  writeToDacs(voltageOffsets[0],voltageOffsets[1]);
  setOutputMode(0, 3);
  setOutputMode(1, 3);

  char str[40];
  sprintf(str, "Best voltage offsets: %d, %d\n", voltageOffsets[0], voltageOffsets[1]);
  Serial.print(str);
}





int pulse (volatile PulseTrain* PT) {
  if (PT->nPulses==0)
    PT->trainStartTime = micros();

  if (micros() - PT->trainStartTime >= PT->duration) {
    Serial.print("Train complete. Delivered "); Serial.print(PT->nPulses); 
    Serial.println(" pulses.\nCurrent/Voltage by phase: ");
    char str[100];
    for (int i = 0; i < PT->nPhases; i++) {
      sprintf(str, "Phase %d: %6d%s, %6d%s,    (target %6d%s, %6d%s)\n", i,
              PT->measuredAmplitude[0][i] / PT->nPulses,
              (PT->mode[0] < 2)?"mV":"uA",
              PT->measuredAmplitude[1][i] / PT->nPulses,
              (PT->mode[1] < 2)?"mV":"uA", 
              PT->amplitude[0][i], 
              (PT->mode[0]==0)?"mV":"uA", 
              PT->amplitude[1][i],
              (PT->mode[1]==0)?"mV":"uA");
      Serial.print(str);
      
    }
    return 0;
  }
  
  setOutputMode(0, PT->mode[0]);
  setOutputMode(1, PT->mode[1]);

  for (int i = 0; i < PT->nPhases; i++) {
    writeToDacs(PT->amplitude[0][i] / ((!PT->mode[0])?MILLIVOLTS_PER_DAC:MICROAMPS_PER_DAC) + ((PT->mode[0])?currentOffsets[0]:voltageOffsets[0]), //* (PT->mode[0] != 3),
                PT->amplitude[1][i] / ((!PT->mode[1])?MILLIVOLTS_PER_DAC:MICROAMPS_PER_DAC) + ((PT->mode[1])?currentOffsets[1]:voltageOffsets[1])); //* (PT->mode[1] != 3));

    delayMicroseconds(max( ((long int) PT->phaseDuration[i]) - SLEW_FUDGE, 0));
    
    int lastAdcRead[2];
    readADC(lastAdcRead); // note: this limits bandwidth for voltage pulses
    PT->measuredAmplitude[0][i] += lastAdcRead[0] * ((PT->mode[0] < 2)?MILLIVOLTS_PER_ADC:MICROAMPS_PER_ADC);
    PT->measuredAmplitude[1][i] += lastAdcRead[1] * ((PT->mode[1] < 2)?MILLIVOLTS_PER_ADC:MICROAMPS_PER_ADC);
  }

  setOutputMode(0, 3);
  setOutputMode(1, 3);
  writeToDacs((PT->mode[0])?currentOffsets[0]:voltageOffsets[0],
              (PT->mode[1])?currentOffsets[1]:voltageOffsets[1]);


  PT->nPulses++;
  return 1;
}


void pulse0() {
  if (!pulse(activePT0))
    IT0.end();
}
void pulse1() { 
  if (!pulse(activePT1))
    IT1.end();
}



void printPulseTrainParameters(int i){
  if (i < 0) i=0;
  if (i >= PT_ARRAY_LENGTH) i = PT_ARRAY_LENGTH;

  const char modeStrings[4][40] = {"Voltage output, measure voltage", 
  "Current output, measure voltage", "Current output, measure current", "No output"};

  Serial.println("----------------------------------");
  Serial.print("Parameters for PulseTrain["); Serial.print(i); Serial.println("]");
  Serial.print("  mode[ch0]: "); Serial.print(PTs[i].mode[0]);
  Serial.print(" ("); Serial.print(modeStrings[PTs[i].mode[0]]); Serial.println(")");
  Serial.print("  mode[ch1]: "); Serial.print(PTs[i].mode[1]);
  Serial.print(" ("); Serial.print(modeStrings[PTs[i].mode[1]]); Serial.println(")");
  char str[100];
  sprintf(str, "  period:    %d usec (%0.3f sec, %0.3f Hz)\n  duration: %lu usec (%0.3f sec)\n",
    PTs[i].period, 0.000001*PTs[i].period, 1000000.0/PTs[i].period, PTs[i].duration, 0.000001*PTs[i].duration);
  Serial.print(str);
  
  Serial.println("\n  phase    duration     output0   output1");
  for (int j = 0; j< PTs[i].nPhases; j++){
    sprintf(str, "   %2d  %7d usec %8d%s %8d%s\n", j, PTs[i].phaseDuration[j],
            PTs[i].amplitude[0][j], (PTs[i].mode[0] == 0)?"mV":"uA",
            PTs[i].amplitude[1][j], (PTs[i].mode[1] == 0)?"mV":"uA");
    Serial.print(str);
    
  }
  Serial.println("----------------------------------\n");
}


volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT){
  PT->nPulses = 0;
  for (int i = 0; i < PT->nPhases; i++){
    PT->measuredAmplitude[0][i] = 0;
    PT->measuredAmplitude[1][i] = 0;
  }
  return(PT);
}









void setup() {
  delay(2000);
  Serial.begin(9600);

  Serial.println("Booting microstim on Teensy 3.5!");

  Serial.println("Initializing DAC and ADC...");
  pinMode(NLDAC, OUTPUT);
  digitalWrite(NLDAC, HIGH);
  pinMode(CS0, OUTPUT);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS0, HIGH);
  digitalWrite(CS1, HIGH);

  Serial.println("Setting initial outputs off...");
  pinMode(OE0, OUTPUT);
  pinMode(OE1, OUTPUT);
  pinMode(OE2, OUTPUT);
  pinMode(OE3, OUTPUT);
  setOutputMode(0,3);
  setOutputMode(1,3);

  Serial.println("Initializing SPI...");
  SPI.begin();

  Serial.println("Initializing ADC...");
  setupADC();

  Serial.println("Measuring DC offset...");
  getCurrentOffsets();
  getVoltageOffsets();
   
  
  for (int i=0; i < PT_ARRAY_LENGTH; i++){
    PTs[i].mode[0] = 0;
    PTs[i].mode[1] = 0;
    PTs[i].period = 1000;
    PTs[i].duration = 10000000;
    PTs[i].nPhases = 2;
    for (int j = 0; j < MAX_NUM_PHASES; j++){
      PTs[i].amplitude[0][j] = j?-1000:1000;
      PTs[i].amplitude[1][j] = j?1000:-1000;
      PTs[i].phaseDuration[j] = 100;
    }
  } 

  PTs[1].mode[0] = 2;
  PTs[1].mode[1] = 2;
  for (int j = 0; j < MAX_NUM_PHASES; j++){
    PTs[1].amplitude[0][j] = j?-100:100;
    PTs[1].amplitude[1][j] = j?100:1-00;
    PTs[1].phaseDuration[j] = 100;
  }


  nChar = 0;
 
  Serial.println("Ready to go!\n\n");

}





void loop() {
  
  if (Serial.available() > 0) {
    Serial.readBytes(comBuf + nChar, 1); // read one byte into the buffer
    nChar++; // keep track of the number of characters we've read!

    if (comBuf[nChar - 1] == '\n') { // termination character for string - means we've recvd a full command!
      unsigned int ptIndex=0;
      
      if (comBuf[0] == 'S') {
        sscanf(comBuf + 1, "%u,", &ptIndex);
        if (ptIndex >= PT_ARRAY_LENGTH){
          Serial.println("Invalid PulseTrain index.");
          nChar=0;
          return;
        }
        
        int m = sscanf(comBuf + 1, "%*d,%u,%u,%u,%lu;",  
               &(PTs[ptIndex].mode[0]), 
               &(PTs[ptIndex].mode[1]),
               &(PTs[ptIndex].period), 
               &(PTs[ptIndex].duration));
        if (m==4){
          PTs[ptIndex].nPhases=0;
          char *token = strtok(comBuf+1, ";");
          token = strtok(NULL, ";"); //move to the 2nd segment delimited by ";"
          
          while (token != NULL){
             m = sscanf(token, "%d,%d,%u", 
                       &(PTs[ptIndex].amplitude[0][PTs[ptIndex].nPhases]),
                       &(PTs[ptIndex].amplitude[1][PTs[ptIndex].nPhases]),
                       &(PTs[ptIndex].phaseDuration[PTs[ptIndex].nPhases]));
             if (m != 3)
                break;
             PTs[ptIndex].nPhases++;
             token = strtok(NULL, ";");
          }
        }
        printPulseTrainParameters(ptIndex);
      }
      
      else if (comBuf[0] == 'T' || comBuf[0] == 'U') {
        ptIndex = atoi(comBuf+1);
        if (ptIndex >= PT_ARRAY_LENGTH){
          Serial.println("Invalid PulseTrain index.");
          nChar=0;
          return;
        }
        Serial.print("\nStarting "); Serial.print(comBuf[0]); 
        Serial.print(" train with parameters of PulseTrain "); Serial.println(ptIndex);
        if (comBuf[0] == 'T'){
          activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
          if (!IT0.begin(pulse0, activePT0->period))
            Serial.println("loop: failure to initiate IntervalTimer IT0");
        } else {
          activePT1 = clearPulseTrainHistory(&PTs[ptIndex]);
          if (!IT1.begin(pulse1, activePT1->period))
            Serial.println("loop: failure to initiate IntervalTimer IT1");
        }
      }
      
      else if (comBuf[0] == 'D') {
        sscanf(comBuf + 1, "%d,%d", &(currentOffsets[0]), &(currentOffsets[1]) );
        Serial.print("currentOffsets[0]: "); Serial.println(currentOffsets[0]);
        Serial.print("currentOffsets[1]: "); Serial.println(currentOffsets[1]);
      }
      
      else if (comBuf[0] == 'C') {
        getCurrentOffsets();
        getVoltageOffsets();
      }
      
      nChar = 0; // reset the pointer!
    }
  }
}

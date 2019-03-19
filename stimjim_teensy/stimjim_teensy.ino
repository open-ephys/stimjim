/* microstim_teensy (c) Nathan Cermak 2019.
 *  
 */

#include <SPI.h> 

#define MICROAMPS_PER_DAC 0.1017            // 20V * (1/(3000 V/A)) / 2^16 = 0.4uA / DAC unit
#define MICROAMPS_PER_ADC 0.2325            // 20V * (1/(10500 V/A)) / 2^13 = 0.2325
#define MILLIVOLTS_PER_DAC 0.442             // 20V / 2^16 * gain of 1.45 ~= 6 mV / DAC unit
#define MILLIVOLTS_PER_ADC 2.44              // 20V / 2^13 = 2.44 mV / ADC unit

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
//AD5752 DAC runs at max 30MHz, mode 0,0 or 1,1 acceptable
SPISettings settingsDAC(10000000, MSBFIRST, SPI_MODE1T0);
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

  int nStages;
  int amplitude[2][MAX_NUM_STAGES];             // mV or uA, depending on mode
  unsigned int stageDuration[MAX_NUM_STAGES];   // usec

  unsigned long trainStartTime;                 // usec 
  int nPulses;
  int measuredAmplitude[2][MAX_NUM_STAGES];
};
volatile PulseTrain PTs[PT_ARRAY_LENGTH];
volatile PulseTrain *activePT0, *activePT1;
IntervalTimer IT0, IT1;

// ------------ Globals for trigger status -------------------- //
int triggerTargetPTs[2];

// ------------ Function prototypes --------------------------- //
void setOutputMode(byte channel, byte mode);
void writeToDacs(int16_t amp0, int16_t amp1);
void writeToDac(byte channel, int16_t amp);
void setupADCs();
void readADC(byte channel, int *data);
void getCurrentOffsets();
void getVoltageOffsets();
int pulse (volatile PulseTrain* PT);
void pulse0();
void pulse1();
void trigISR0();
void trigISR1();
void printPulseTrainParameters(int i);
volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT);


void setOutputMode(byte channel, byte mode) {
  /* mode = 0, output = VOLTAGE
     mode = 1, output = CURRENT
     mode = 2, output = HIGH-Z
     mode = 3, output = GROUND */
  digitalWrite((channel)?OE0_1:OE0_0, (0b00000001 & mode) ? HIGH : LOW);
  digitalWrite((channel)?OE1_1:OE1_0, (0b00000010 & mode) ? HIGH : LOW);
}


void writeToDacs(int16_t amp0, int16_t amp1) {
  /* The input shift register is 24 bits wide. Data is loaded into the
   * device MSB first as a 24-bit word. The input register consists of a read/write
   * bit, three register select bits, three DAC address bits, and 16 data bits. 
   * Stimjim AD5752 uses two's complement. */

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
  digitalWrite((channel)?CS0_1:CS0_0, LOW);
  SPI.transfer(0);
  SPI.transfer16(amp);
  digitalWrite((channel)?CS0_1:CS0_0, HIGH);
  SPI.endTransaction();

  digitalWrite((channel)?NLDAC_1:NLDAC_0, LOW);
  digitalWrite((channel)?NLDAC_1:NLDAC_0, HIGH);
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
void setupADCs() {
  SPI.beginTransaction(settingsADC);
  for (int i = 0; i < 2; i++){
    int cs = (i==0)?CS1_0:CS1_1;
    digitalWrite(cs, LOW);
    SPI.transfer16(32768 + 8192); // write range register, all zeros (+-10V on both channels)
    digitalWrite(cs, HIGH);
    delayMicroseconds(100);
    digitalWrite(cs, LOW);
    SPI.transfer16(32768 + 1024 + 16 + 8); // write control register, ch1, use internal ref, use sequencer
    digitalWrite(cs, HIGH);
  }
  SPI.endTransaction();
}

void setupDACs() {
  SPI.beginTransaction(settingsDAC);
  for (int i = 0; i < 2; i++){
    int cs = (i==0)?CS0_0:CS0_1;
    digitalWrite(cs, LOW);
    SPI.transfer(8); // select "output range" register
    SPI.transfer16(4); // set mode as "+-10V mode (xxx...100)
    digitalWrite(cs, HIGH);
    digitalWrite(cs, LOW);
    SPI.transfer(16); // select "power control" register
    SPI.transfer16(1); // set mode as dac_A powered up
    digitalWrite(cs, HIGH);
    delayMicroseconds(100);
  }
  SPI.endTransaction();
}



void readADC(byte channel, int *data) {
  SPI.beginTransaction(settingsADC);
  for (int i =0; i<2; i++){
    digitalWrite((channel)?CS1_1:CS1_0, LOW);
    data[i] = SPI.transfer16(0) - (i?8192:0);
    digitalWrite((channel)?CS1_1:CS1_0, HIGH);
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
  
  setOutputMode(0, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  setOutputMode(1, 3); // output grounded, Iout goes to ground via 1k on-board resistor
  
  for (int i = -500; i < 500; i++) {
    writeToDacs(i, i);
    delayMicroseconds(100);
    adcReadSum[0] = adcReadSum[1] = 0;
    for (int j=0; j<100; j++){
      readADC(0,lastAdcRead);
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
      readADC(0,lastAdcRead);
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
    /*Serial.print("Train complete. Delivered "); Serial.print(PT->nPulses); 
    Serial.println(" pulses.\nCurrent/Voltage by stage: ");
    char str[100];
    for (int i = 0; i < PT->nStages; i++) {
      sprintf(str, "Stage %d: %6d%s, %6d%s,    (target %6d%s, %6d%s)\n", i,
              PT->measuredAmplitude[0][i] / PT->nPulses,
              (PT->mode[0] < 2)?"mV":"uA",
              PT->measuredAmplitude[1][i] / PT->nPulses,
              (PT->mode[1] < 2)?"mV":"uA", 
              PT->amplitude[0][i], 
              (PT->mode[0]==0)?"mV":"uA", 
              PT->amplitude[1][i],
              (PT->mode[1]==0)?"mV":"uA");
      Serial.print(str);
      
    }*/
    return 0;
  }
  
  setOutputMode(0, PT->mode[0]);
  setOutputMode(1, PT->mode[1]);

  for (int i = 0; i < PT->nStages; i++) {
    writeToDacs(PT->amplitude[0][i] / ((!PT->mode[0])?MILLIVOLTS_PER_DAC:MICROAMPS_PER_DAC) + ((PT->mode[0])?currentOffsets[0]:voltageOffsets[0]), //* (PT->mode[0] != 3),
                PT->amplitude[1][i] / ((!PT->mode[1])?MILLIVOLTS_PER_DAC:MICROAMPS_PER_DAC) + ((PT->mode[1])?currentOffsets[1]:voltageOffsets[1])); //* (PT->mode[1] != 3));

    delayMicroseconds(max( ((long int) PT->stageDuration[i]), 0));
    // TESTING
    /*int lastAdcRead[2];
    readADC(0,lastAdcRead); // note: this limits bandwidth for voltage pulses
    PT->measuredAmplitude[0][i] += lastAdcRead[0] * ((PT->mode[0] < 2)?MILLIVOLTS_PER_ADC:MICROAMPS_PER_ADC);
    PT->measuredAmplitude[1][i] += lastAdcRead[1] * ((PT->mode[1] < 2)?MILLIVOLTS_PER_ADC:MICROAMPS_PER_ADC);
    */
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

void trigISR0(){
  if (triggerTargetPTs[0] >= 0){
    activePT0 = clearPulseTrainHistory(&PTs[triggerTargetPTs[0]]);
    if (!IT0.begin(pulse0, activePT0->period))
      Serial.println("trigISR0: failure to initiate IntervalTimer IT0");
  }
}

void trigISR1(){
  if (triggerTargetPTs[1] >= 0){
    activePT1 = clearPulseTrainHistory(&PTs[triggerTargetPTs[1]]);
    if (!IT1.begin(pulse1, activePT1->period))
      Serial.println("trigISR1: failure to initiate IntervalTimer IT1");  
  }
}

void printPulseTrainParameters(int i){
  if (i < 0) i=0;
  if (i >= PT_ARRAY_LENGTH) i = PT_ARRAY_LENGTH;

  const char modeStrings[4][40] = {"Voltage output", 
  "Current output", "No output (high-Z)", "No output (grounded)"};

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
  
  Serial.println("\n  stage    duration     output0   output1");
  for (int j = 0; j< PTs[i].nStages; j++){
    sprintf(str, "   %2d  %7d usec %8d%s %8d%s\n", j, PTs[i].stageDuration[j],
            PTs[i].amplitude[0][j], (PTs[i].mode[0] == 0)?"mV":"uA",
            PTs[i].amplitude[1][j], (PTs[i].mode[1] == 0)?"mV":"uA");
    Serial.print(str);
    
  }
  Serial.println("----------------------------------\n");
}


volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT){
  PT->nPulses = 0;
  for (int i = 0; i < PT->nStages; i++){
    PT->measuredAmplitude[0][i] = 0;
    PT->measuredAmplitude[1][i] = 0;
  }
  return(PT);
}









void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);
  delay(2000);
  Serial.begin(112500);

  Serial.println("Booting microstim on Teensy 3.5!");
  
  Serial.println("Initializing SPI...");
  SPI.begin();

  Serial.println("Initializing pins...");
  int pins[] = {NLDAC_0, NLDAC_1, CS0_0, CS1_0, CS0_1, CS1_1, OE0_0, OE1_0, OE0_1, OE1_1, LED0, LED1};
  for (int i =0; i < 12; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  setOutputMode(0,3); // redundant to set this, but safe in case of future changes
  setOutputMode(1,3); 

  Serial.println("Initializing ADC...");
  setupADCs();
  setupDACs();
  
  Serial.println("Measuring DC offset...");
  //getCurrentOffsets();
  //getVoltageOffsets();
  currentOffsets[0] = 0;
  currentOffsets[1] = 0;
  voltageOffsets[0] = 0;
  voltageOffsets[1] = 0;
  
  
   
  
  for (int i=0; i < PT_ARRAY_LENGTH; i++){
    PTs[i].mode[0] = 0;
    PTs[i].mode[1] = 0;
    PTs[i].period = 1000;
    PTs[i].duration = 10000000;
    PTs[i].nStages = 2;
    for (int j = 0; j < MAX_NUM_STAGES; j++){
      PTs[i].amplitude[0][j] = j?-1000:1000;
      PTs[i].amplitude[1][j] = j?1000:-1000;
      PTs[i].stageDuration[j] = 100;
    }
  } 

  PTs[1].mode[0] = 2;
  PTs[1].mode[1] = 2;
  for (int j = 0; j < MAX_NUM_STAGES; j++){
    PTs[1].amplitude[0][j] = j?-100:100;
    PTs[1].amplitude[1][j] = j?100:1-00;
    PTs[1].stageDuration[j] = 100;
  }

  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);
  // initialize inputs so that triggers do nothing
  triggerTargetPTs[0] = -1;
  triggerTargetPTs[1] = -1;
  
  nChar = 0;

  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
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
          PTs[ptIndex].nStages=0;
          char *token = strtok(comBuf+1, ";");
          token = strtok(NULL, ";"); //move to the 2nd segment delimited by ";"
          
          while (token != NULL){
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
        //sscanf(comBuf + 1, "%d,%d", &(currentOffsets[0]), &(currentOffsets[1]) );
        Serial.print("currentOffsets[0]: "); Serial.println(currentOffsets[0]);
        Serial.print("currentOffsets[1]: "); Serial.println(currentOffsets[1]);
      }
      
      else if (comBuf[0] == 'C') {
        getCurrentOffsets();
        getVoltageOffsets();
      }

      else if (comBuf[0] == 'R') {
        int trigSrc = 0;
        int ptIndex = 0;
        int falling = 0;
        sscanf(comBuf + 1, "%d,%d,%d", &trigSrc, &ptIndex, &falling );
        if (ptIndex >= PT_ARRAY_LENGTH){
          Serial.println("Invalid PulseTrain index.");
          nChar=0;
          return;
        }
        if (ptIndex>=0){
          Serial.print("Attaching interrupt to IN"); Serial.print(trigSrc); 
          Serial.print(" to run PulseTrain["); Serial.print(ptIndex); Serial.println("]");
          triggerTargetPTs[trigSrc] = ptIndex;
          // 
          attachInterrupt( (trigSrc)?IN1:IN0, (trigSrc)?trigISR1:trigISR0, RISING);
        }
        else {
          Serial.print("Detaching interrupt to IN"); Serial.println(trigSrc); 
          detachInterrupt( (trigSrc)?IN1:IN0);
          triggerTargetPTs[trigSrc] = -1;
        }
      }
      
      
      nChar = 0; // reset the pointer!
    }
  }
}

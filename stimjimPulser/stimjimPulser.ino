/* stimjimPulser (c) Nathan Cermak 2019. */

#include <Stimjim.h>

#define PT_ARRAY_LENGTH 10
#define MAX_NUM_STAGES 10


// ------------- Serial setup ---------------------------------- //
char comBuf[1000];
int bytesRecvd;

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
int pulse (volatile PulseTrain* PT);
void pulse0();
void pulse1();
void startIT0(int ptIndex);
void startIT0ViaInputTrigger();
void startIT1(int ptIndex);
void startIT1ViaInputTrigger();

void printPulseTrainParameters(int i);
void printTrainResultSummary(volatile PulseTrain* PT);
volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT);








int pulse (volatile PulseTrain* PT) {
  if (PT->nPulses == 0)
    PT->trainStartTime = micros();

  //check if the pulseTrain is finished; if so, exit
  if (micros() - PT->trainStartTime >= PT->duration)
    return 0;

  if (PT->nStages == 0) {
    PT->nPulses++;
    return 1;
  }
  if (PT->mode[0] < 2)   setOutputMode(0, PT->mode[0]);
  if (PT->mode[1] < 2)   setOutputMode(1, PT->mode[1]);

  int dac0val, dac1val;
  float adcReadTime = 3.5 * (PT->mode[0] < 2) + 3.5 * (PT->mode[1] < 2);   //16 bits at 5MHz
  float dacWriteTime = 3 * (PT->mode[0] < 2) + 3 * (PT->mode[1] < 2); //24 bits at 10MHz, with delay, should take 2.5us

  dac0val = PT->amplitude[0][0] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? currentOffsets[0] : voltageOffsets[0]);
  dac1val = PT->amplitude[1][0] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? currentOffsets[1] : voltageOffsets[1]);

  if (PT->mode[0] < 2 && PT->mode[1] < 2) {
    writeToDacs(dac0val, dac1val);
  } else if (PT->mode[0] < 2) {
    writeToDac(0, dac0val);
  } else if (PT->mode[1] < 2) {
    writeToDac(1, dac1val);
  }

  for (int i = 0; i < PT->nStages; i++) {
    delayMicroseconds(PT->stageDuration[i] - dacWriteTime - adcReadTime);

    // read ADCs
    if (PT->mode[0] < 2)
      PT->measuredAmplitude[0][i] += readADC(0, PT->mode[0] > 0) * ((PT->mode[0]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
    if (PT->mode[1] < 2)
      PT->measuredAmplitude[1][i] += readADC(1, PT->mode[1] > 0) * ((PT->mode[1]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);

    if ( i + 1 < PT->nStages) {
      dac0val = PT->amplitude[0][i + 1] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? currentOffsets[0] : voltageOffsets[0]);
      dac1val = PT->amplitude[1][i + 1] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? currentOffsets[1] : voltageOffsets[1]);
    } else { // we're in the last stage, set DACs back to zero
      dac0val = (PT->mode[0]) ? currentOffsets[0] : voltageOffsets[0];
      dac1val = (PT->mode[1]) ? currentOffsets[1] : voltageOffsets[1];
    }
    // write to dacs
    if (PT->mode[0] < 2 && PT->mode[1] < 2) {
      writeToDacs(dac0val, dac1val);
    } else if (PT->mode[0] < 2) {
      writeToDac(0, dac0val);
    } else if (PT->mode[1] < 2) {
      writeToDac(1, dac1val);
    }

  }

  // switch outputs to ground
  if (PT->mode[0] < 2)     setOutputMode(0, 3);
  if (PT->mode[1] < 2)     setOutputMode(1, 3);

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
    sprintf(str, "%6d%s,          ", PT->measuredAmplitude[0][i] / PT->nPulses, (PT->mode[0]) ? "uA" : "mV");
    Serial.print(str);
    sprintf(str, "%6d%s,          ", PT->measuredAmplitude[1][i] / PT->nPulses, (PT->mode[1]) ? "uA" : "mV");
    Serial.println(str);
  }
}

void pulse0() {
  if (!pulse(activePT0)) {
    IT0.end();
    printTrainResultSummary(activePT0);
    if (activePT0->mode[0] < 2)  digitalWrite(LED0, LOW);
    if (activePT0->mode[1] < 2)  digitalWrite(LED1, LOW);
  }
}

void pulse1() {
  if (!pulse(activePT1)) {
    IT1.end();
    printTrainResultSummary(activePT1);
    if (activePT1->mode[0] < 2)  digitalWrite(LED0, LOW);
    if (activePT1->mode[1] < 2)  digitalWrite(LED1, LOW);
  }
}

void startIT0ViaInputTrigger() {
  if (triggerTargetPTs[0] >= 0)
    startIT0(triggerTargetPTs[0]);
}

void startIT0(int ptIndex) {
  activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
  Serial.print("\nStarting T train with parameters of PulseTrain "); Serial.println(ptIndex);
  if (activePT0->mode[0] < 2)  digitalWrite(LED0, HIGH);
  if (activePT0->mode[1] < 2)  digitalWrite(LED1, HIGH);
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
  if (activePT1->mode[0] < 2)  digitalWrite(LED0, HIGH);
  if (activePT1->mode[1] < 2)  digitalWrite(LED1, HIGH);

  if (!IT1.begin(pulse1, activePT1->period))
    Serial.println("startIT1: failure to initiate IntervalTimer IT1");
  pulse1(); //intervalTimer starts with delay - we want to start with pulse!
}


void printPulseTrainParameters(int i) {
  if (i < 0 || i >= PT_ARRAY_LENGTH) {
    Serial.println("Invalid PulseTrain array index.");
    return;
  }

  const char modeStrings[4][40] = {"Voltage output", "Current output", "No output (high-Z)", "No output (grounded)"};
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
    for (int j = 0; j < 4; j++)
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
  setADCrange(10);
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
    PTs[1].amplitude[1][j] = j ?  100 : -100;
    PTs[1].stageDuration[j] = 100;
  }

  Serial.println("Initializing inputs...");
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);
  triggerTargetPTs[0] = -1; // initialize target to -1 so that triggers do nothing
  triggerTargetPTs[1] = -1;

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
        char str[100];
        sprintf(str, "current offsets: %d, %d\nvoltage offsets: %d, %d\n",
                currentOffsets[0], currentOffsets[1], voltageOffsets[0], voltageOffsets[1]);
        Serial.println(str);
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
      else if (comBuf[0] == 'M') {
        int channel = 0, mode = 0;
        sscanf(comBuf + 1, "%d,%d", &channel, &mode );
        setOutputMode(channel, mode);
        Serial.print("Set channel "); Serial.print(channel); Serial.print(" to mode "); Serial.println(mode);
      }
      else if (comBuf[0] == 'A') {
        int channel = 0, amp = 0;
        sscanf(comBuf + 1, "%d,%d", &channel, &amp );
        writeToDac(channel, amp);
        Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.println(amp);
      }
      else if (comBuf[0] == 'E') {
        int channel = 0, line = 0;
        sscanf(comBuf + 1, "%d,%d", &channel, &line );
        int val = readADC(channel, line);
        Serial.print("Read value: "); Serial.println(val);
      }

      bytesRecvd = 0; // reset the pointer!
    }
  }
}

/* stimjimPulser (c) Nathan Cermak 2019. */

#include <Stimjim.h>

#define PT_ARRAY_LENGTH 100
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
  //check if the pulseTrain is finished; if so, exit
  if (micros() - PT->trainStartTime >= PT->duration)
    return 0;

  if (PT->nStages == 0) {
    PT->nPulses++;
    return 1;
  }

  int dac0val, dac1val;
  float adcReadTime = 6.9 * (PT->mode[0] < 2) + 6.9 * (PT->mode[1] < 2);   //16 bits at 5MHz, calibrated time is 6.9us
  float dacWriteTime = 4.8 * (PT->mode[0] < 2) + 4.8 * (PT->mode[1] < 2);  //24 bits at 10MHz, calibrated time is 4.8us

  dac0val = PT->amplitude[0][0] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
  dac1val = PT->amplitude[1][0] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

  if (PT->mode[0] < 2 && PT->mode[1] < 2) {
    Stimjim.writeToDacs(dac0val, dac1val);
  } else if (PT->mode[0] < 2) {
    Stimjim.writeToDac(0, dac0val);
  } else if (PT->mode[1] < 2) {
    Stimjim.writeToDac(1, dac1val);
  }
  if (PT->mode[0] < 2)   Stimjim.setOutputMode(0, PT->mode[0]);
  if (PT->mode[1] < 2)   Stimjim.setOutputMode(1, PT->mode[1]);
  for (int i = 0; i < PT->nStages; i++) {
    delayMicroseconds(PT->stageDuration[i] - dacWriteTime - adcReadTime - 1.5); // empirically calibrated!

    // read ADCs
    if (PT->mode[0] < 2)
      PT->measuredAmplitude[0][i] += Stimjim.readAdc(0, PT->mode[0] > 0) * ((PT->mode[0]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
    if (PT->mode[1] < 2)
      PT->measuredAmplitude[1][i] += Stimjim.readAdc(1, PT->mode[1] > 0) * ((PT->mode[1]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);

    if ( i + 1 < PT->nStages) {
      dac0val = PT->amplitude[0][i + 1] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
      dac1val = PT->amplitude[1][i + 1] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);
    } else { // we're in the last stage, set DACs back to zero
      dac0val = (PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0];
      dac1val = (PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1];
    }
    // write to dacs
    if (PT->mode[0] < 2 && PT->mode[1] < 2) {
      Stimjim.writeToDacs(dac0val, dac1val);
    } else if (PT->mode[0] < 2) {
      Stimjim.writeToDac(0, dac0val);
    } else if (PT->mode[1] < 2) {
      Stimjim.writeToDac(1, dac1val);
    }

  }

  // switch outputs to ground
  if (PT->mode[0] < 2)     Stimjim.setOutputMode(0, 3);
  if (PT->mode[1] < 2)     Stimjim.setOutputMode(1, 3);

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
    if (activePT0->mode[0] < 2)  digitalWriteFast(LED0, LOW);
    if (activePT0->mode[1] < 2)  digitalWriteFast(LED1, LOW);
  }
}

void pulse1() {
  if (!pulse(activePT1)) {
    IT1.end();
    printTrainResultSummary(activePT1);
    if (activePT1->mode[0] < 2)  digitalWriteFast(LED0, LOW);
    if (activePT1->mode[1] < 2)  digitalWriteFast(LED1, LOW);
  }
}

void startIT0ViaInputTrigger() {
  if (triggerTargetPTs[0] >= 0)
    startIT0(triggerTargetPTs[0]);
}

void startIT0(int ptIndex) {
  activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
  activePT0->trainStartTime = micros();
  if (!IT0.begin(pulse0, activePT0->period))
    Serial.println("startIT0: failure to initiate IntervalTimer IT0");
  pulse0(); //intervalTimer starts with delay - we want to start with pulse!
  
  Serial.print("\nStarted T train with parameters of PulseTrain "); Serial.println(ptIndex);
  if (activePT0->mode[0] < 2)  digitalWriteFast(LED0, HIGH);
  if (activePT0->mode[1] < 2)  digitalWriteFast(LED1, HIGH);
}

void startIT1ViaInputTrigger() {
  if (triggerTargetPTs[1] >= 0)
    startIT1(triggerTargetPTs[1]);
}

void startIT1(int ptIndex) {
  activePT1 = clearPulseTrainHistory(&PTs[ptIndex]);
  activePT1->trainStartTime = micros();
  if (!IT1.begin(pulse1, activePT1->period))
    Serial.println("startIT1: failure to initiate IntervalTimer IT1");
  pulse1(); //intervalTimer starts with delay - we want to start with pulse!
  
  Serial.print("\nStarted U train with parameters of PulseTrain "); Serial.println(ptIndex);
  if (activePT1->mode[0] < 2)  digitalWriteFast(LED0, HIGH);
  if (activePT1->mode[1] < 2)  digitalWriteFast(LED1, HIGH);
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
  sprintf(str, "  period:    %d usec (%0.3f sec, %0.3f Hz)\n  duration:  %lu usec (%0.3f sec)\n",
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
  memset((void *) PT->measuredAmplitude, 0, 4*MAX_NUM_STAGES*sizeof(int));
  return (PT);
}









void setup() {
  Serial.begin(112500);
  bytesRecvd = 0;
  delay(1000);

  Serial.println("Booting StimJim on Teensy 3.5!");

  Stimjim.begin();

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

  IT0.priority(64);
  IT1.priority(64);
  Serial.flush();
  Serial.println("Ready to go!\n\n");


  Serial.print("Stimjim adcOffset25 = ");
  Serial.print(Stimjim.adcOffset25[0]); Serial.print(" , ");
  Serial.println(Stimjim.adcOffset25[1]);

  Serial.print("Stimjim adcOffset10 = ");
  Serial.print(Stimjim.adcOffset10[0]); Serial.print(" , ");
  Serial.println(Stimjim.adcOffset10[1]);
  

}





void loop() {

  if (Serial.available() > 0) {
    Serial.readBytes(comBuf + bytesRecvd, 1); // read one byte into the buffer
    bytesRecvd++; // keep track of the number of characters we've read!

    if (comBuf[bytesRecvd - 1] == '\n') { // termination character for string - means we've recvd a full command!
      unsigned int ptIndex = 0;
      comBuf[bytesRecvd - 1] = '\0'; // make sure we dont accidentally read into the rest of the string!
      
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
                Stimjim.currentOffsets[0], Stimjim.currentOffsets[1], Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1]);
        Serial.println(str);
      }

      else if (comBuf[0] == 'C') {
        Stimjim.getCurrentOffsets();
        Stimjim.getVoltageOffsets();
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
        Stimjim.setOutputMode(channel, mode);
        Serial.print("Set channel "); Serial.print(channel); Serial.print(" to mode "); Serial.println(mode);
      }
      else if (comBuf[0] == 'A') {
        int channel = 0, amp = 0;
        sscanf(comBuf + 1, "%d,%d", &channel, &amp );
        Stimjim.writeToDac(channel, amp);
        Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.println(amp);
      }
      else if (comBuf[0] == 'E') {
        int channel = 0, line = 0;
        sscanf(comBuf + 1, "%d,%d", &channel, &line );
        int val = Stimjim.readAdc(channel, line);
        Serial.print("Read value: "); Serial.println(val);
      }

      bytesRecvd = 0; // reset the pointer!
    }
  }
}

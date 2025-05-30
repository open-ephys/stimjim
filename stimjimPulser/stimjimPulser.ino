//    stimjimPulser (c) Nathan Cermak <cerman07 at protonmail.com>.
//
//    This file is part of stimjimPulser.
//
//    stimjimPulser is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

// Valid serial commands:
//    S - Set pulseTrain parameters. Example:
//        S0,0,1,1000,100000; 100,-100,100; -50,50,20;
//        1st argument (0) means set parameters for pulseTrain 0.
//        2nd argument (0) - mode 0 (voltage) on output channel 0 (see modes below under M)
//        3rd argument (1) - mode 1 (current) on output channel 1
//        4th argument (1000) - period of pulsetrain in microseconds. In example, run 1 pulse every ms.
//        5th argument (100000) - duration of pulsetrain in microseconds. In example, duration is 100 ms.
//        6th, 7th and 8th arguments - pulse stage 0 parameters
//            amplitudes for both channels (in uA and mV, depending on mode), and duration in usec.
//            In this case, sets amplitudes to 100uA, -100mV, for 100 microseconds
//        9th, 10th and 11th arguments - pulse stage 1 parameters
//            amplitudes for both channels (in uA and mV, depending on mode), and duration in usec.
//            In this case, sets amplitudes to 100uA, -100mV, for 100 microseconds
//        etc... for trios of arguments, up to 10 stages total.
//
//    T, U - T0 means start PulseTrain[0]. U0 also means start PulseTrain[0]. T and U can be
//           used to run two pulse train simultaneously.
//    B - measure ADC offset value (by grounding output and measuring ADC value on output).
//    C - measure current and voltage offsets by sweeping DAC values and reading output.
//    D - Print current values of all offsets (ADC, current, voltage)
//    R - R0,<n>,0 means that logic high on "input" 0 starts PulseTrain n.
//        R0,0,1 means that "input" 0 is reprogrammed as an output that marks stimulus start time of whatever pulsetrain is being delivered
//    M - M0,0 means set output mode for channel 0 to 0. output modes are as follows:
//        0 - voltage
//        1 - current
//        2 - disconnected (hi-z)
//        3 - grounded
//    A - A0,1000 means set amplitude on channel 0 to 1000 (dac units; -32,768 to +32,767)
//    V - V0,100 means set amplitude on channel 0 to 100mV
//    E - E0,1 means read channel zero, line 1. Line 0 is voltage out, line 1 is current sense.
//        Returns (prints over serial) value in raw adc units.
//    V - V0 means default serial reporting, V1 means verbose


#include <Stimjim.h>

#define PT_ARRAY_LENGTH 100
#define MAX_NUM_STAGES 10

// ------------- Serial setup ---------------------------------- //
char comBuf[1000];
int bytesRecvd;
bool verbose = false;

// ------------- PulseTrain parameter setup -------------------- //
struct PulseTrain {
    unsigned int mode[2];
    unsigned long period;                          // usec
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
bool trigOutput[2];

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

int pulse (volatile PulseTrain* PT)
{
    //check if the pulseTrain is finished; if so, exit
    if (micros() - PT->trainStartTime >= PT->duration)
        return 0;

    if (PT->nStages == 0) {
        PT->nPulses++;
        return 1;
    }

    int dac0val, dac1val;
    float adcReadTime =  4.50 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //16 bits at 10MHz, calibrated time is 4.5us
    float dacWriteTime = 2.75 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //24 bits at 30MHz, calibrated time is 2.75us
    float totalDelayTime = dacWriteTime + adcReadTime + 0.5;
    dac0val = PT->amplitude[0][0] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
    dac1val = PT->amplitude[1][0] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

    if (PT->mode[0] < 2 && PT->mode[1] < 2) {
        Stimjim.writeToDacs(dac0val, dac1val);
    } else if (PT->mode[0] < 2) {
        Stimjim.writeToDac(0, dac0val);
    } else if (PT->mode[1] < 2) {
        Stimjim.writeToDac(1, dac1val);
    }
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, PT->mode[0]);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, PT->mode[1]);
    for (int i = 0; i < PT->nStages; i++) {
        delayMicroseconds(PT->stageDuration[i] - totalDelayTime); // empirically calibrated!

        // read ADCs
        if (PT->mode[0] < 2)
            PT->measuredAmplitude[0][i] += (Stimjim.readAdc(0, PT->mode[0] > 0)-Stimjim.adcOffset10[0]) * ((PT->mode[0]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
        if (PT->mode[1] < 2)
            PT->measuredAmplitude[1][i] += (Stimjim.readAdc(1, PT->mode[1] > 0)-Stimjim.adcOffset10[1]) * ((PT->mode[1]) ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);

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
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, 3);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, 3);

    PT->nPulses++;

    return 1;
}


void printTrainResultSummary(volatile PulseTrain* PT)
{
    Serial.print("Train complete. Delivered "); Serial.print(PT->nPulses);
    Serial.println(" pulses.\r\nCurrent/Voltage by stage: ");
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

void pulse0()
{
    if (!pulse(activePT0)) {

        IT0.end();
        printTrainResultSummary(activePT0);

        if (activePT0->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
                digitalWriteFast(IN0, LOW);
        }

        if (activePT0->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }
    }
}

void pulse1()
{
    if (!pulse(activePT1)) {

        IT1.end();
        printTrainResultSummary(activePT1);

        if (activePT1->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
               digitalWriteFast(IN0, LOW);
        }

        if (activePT1->mode[1] < 2)
        {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }
    }
}

void startIT0ViaInputTrigger()
{
    if (triggerTargetPTs[0] >= 0)
        startIT0(triggerTargetPTs[0]);
}

void startIT1ViaInputTrigger()
{
    if (triggerTargetPTs[1] >= 0)
        startIT1(triggerTargetPTs[1]);
}

void startIT0(int ptIndex)
{
    if (ptIndex < 0) {

        Serial.println("Forcing T train to stop");
        IT0.end();

        if (activePT0->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
              digitalWriteFast(IN0, LOW);
        }

        if (activePT0->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }

        return;
    }

    activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
    activePT0->trainStartTime = micros();
    if (!IT0.begin(pulse0, activePT0->period))
        Serial.println("startIT0: failure to initiate IntervalTimer IT0");

    Serial.print("\r\nStarted T train with parameters of PulseTrain "); Serial.println(ptIndex);

    if (activePT0->mode[0] < 2) {
        digitalWriteFast(LED0, HIGH);
        digitalWriteFast(GPIO_10, HIGH);
        if (trigOutput[0])
          digitalWriteFast(IN0, HIGH);
    }

    if (activePT0->mode[1] < 2){
        digitalWriteFast(LED1, HIGH);
        digitalWriteFast(GPIO_11, HIGH);
        if (trigOutput[1])
          digitalWriteFast(IN1, HIGH);
    }

    pulse0(); //intervalTimer starts with delay - we want to start with pulse!
}

void startIT1(int ptIndex)
{
    if (ptIndex < 0) {

        Serial.println("Forcing U train to stop");

        if (activePT1->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
                digitalWriteFast(IN0, LOW);

        }

        if (activePT1->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }

        IT1.end();
        return;
    }

    activePT1 = clearPulseTrainHistory(&PTs[ptIndex]);
    activePT1->trainStartTime = micros();

    if (!IT1.begin(pulse1, activePT1->period))
        Serial.println("startIT1: failure to initiate IntervalTimer IT1");

    Serial.print("\r\nStarted U train with parameters of PulseTrain "); Serial.println(ptIndex);

    if (activePT1->mode[0] < 2) {
        digitalWriteFast(LED0, HIGH);
        digitalWriteFast(GPIO_10, HIGH);
        if (trigOutput[0])
            digitalWriteFast(IN0, HIGH);

    }
    if (activePT1->mode[1] < 2) {
        digitalWriteFast(LED1, HIGH);
        digitalWriteFast(GPIO_11, HIGH);
        if (trigOutput[1])
            digitalWriteFast(IN1, HIGH);
    }

    pulse1(); //intervalTimer starts with delay - we want to start with pulse!
}

void printPulseTrainParameters(int i)
{
    if (i < 0 || i >= PT_ARRAY_LENGTH) {
        Serial.println("Invalid PulseTrain array index.");
        return;
    }

    const char modeStrings[4][40] = {"Voltage output", "Current output", "No output (high-Z)", "No output (grounded)"};
    Serial.println("----------------------------------");
    char str[200];
    sprintf(str, "Parameters for PulseTrain[%d]\r\n  mode[ch0]: %d (%s)\r\n  mode[ch1]: %d (%s)\r\n",
        i, PTs[i].mode[0], modeStrings[PTs[i].mode[0]], PTs[i].mode[1], modeStrings[PTs[i].mode[1]]);
    Serial.print(str);
    sprintf(str, "  period:    %lu usec (%0.3f sec, %0.3f Hz)\r\n  duration:  %lu usec (%0.3f sec)\r\n",
          PTs[i].period, 0.000001 * PTs[i].period, 1000000.0 / PTs[i].period, PTs[i].duration, 0.000001 * PTs[i].duration);
    Serial.print(str);
    Serial.println("\r\n  stage    duration     output0   output1");

    for (int j = 0; j < PTs[i].nStages; j++) {
        sprintf(str, "   %2d  %7d usec %8d%s %8d%s\r\n", j, PTs[i].stageDuration[j],
            PTs[i].amplitude[0][j], (PTs[i].mode[0] == 0) ? "mV" : "uA",
            PTs[i].amplitude[1][j], (PTs[i].mode[1] == 0) ? "mV" : "uA");
        Serial.print(str);
    }

    Serial.println("----------------------------------\r\n");
}

volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT)
{
      PT->nPulses = 0;
      memset((void *) PT->measuredAmplitude, 0, 4*MAX_NUM_STAGES*sizeof(int));
      return (PT);
}

void setup()
{
    Serial.begin(112500);
    bytesRecvd = 0;
    delay(1000);

    Serial.println("Booting StimJim on Teensy 3.5!");

    Stimjim.begin();

    for (int i = 0; i < PT_ARRAY_LENGTH; i++) {
        PTs[i].mode[0] = 3;
        PTs[i].mode[1] = 3;
        PTs[i].period = 10000;
        PTs[i].duration = 500000;
        PTs[i].nStages = 0;
    }

      Serial.println("Initializing triggers inputs...");
      pinMode(IN0, OUTPUT);
      pinMode(IN1, OUTPUT);
      triggerTargetPTs[0] = -1; // initialize target to -1 so that triggers do nothing
      triggerTargetPTs[1] = -1;
      trigOutput[0] = true;
      trigOutput[1] = true;

      // GPIO
      pinMode(GPIO_10, OUTPUT);

      IT0.priority(64);
      IT1.priority(64);
      Serial.flush();

      // print offset values for user reference
      char str[200];
      sprintf(str, "ADC offsets (+-2.5V): %f, %f\r\nADC offsets (+-10V): %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n",
        Stimjim.adcOffset25[0],Stimjim.adcOffset25[1], Stimjim.adcOffset10[0],Stimjim.adcOffset10[1],
        Stimjim.currentOffsets[0], Stimjim.currentOffsets[1], Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1] );
      Serial.println(str);

      Serial.println("Ready to go!\r\n\r\n");
}

void loop()
{
    if (Serial.available() > 0) {

        Serial.readBytes(comBuf + bytesRecvd, 1); // read one byte into the buffer
        bytesRecvd++; // keep track of the number of characters we've read!

        if (comBuf[bytesRecvd - 1] == '\n') { // termination character for string - we received a full command!
            int ptIndex = 0;

            // remove \n and possibly \r from end of comBuf
            comBuf[bytesRecvd - 1] = '\0';
            if (bytesRecvd >= 2 && comBuf[bytesRecvd - 2] == '\r')
                comBuf[bytesRecvd - 2] = '\0';

            if (comBuf[0] == 'S') {
                sscanf(comBuf + 1, "%d,", &ptIndex);
                if (ptIndex < 0 || ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                int m = sscanf(comBuf + 1, "%*d,%u,%u,%lu,%lu;",
                    &(PTs[ptIndex].mode[0]),
                    &(PTs[ptIndex].mode[1]),
                    &(PTs[ptIndex].period),
                    &(PTs[ptIndex].duration));
                if (PTs[ptIndex].mode[0] < 0 || PTs[ptIndex].mode[0] > 3)
                    PTs[ptIndex].mode[0] = 3;
                if (PTs[ptIndex].mode[1] < 0 || PTs[ptIndex].mode[1] > 3)
                    PTs[ptIndex].mode[1] = 3;

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

            } else if (comBuf[0] == 'T' || comBuf[0] == 'U') {

                ptIndex = atoi(comBuf + 1);
                if (ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                if (comBuf[0] == 'T')
                    startIT0(ptIndex);
                if (comBuf[0] == 'U')
                    startIT1(ptIndex);

            } else if (comBuf[0] == 'B') {

                Stimjim.getAdcOffsets();

            } else if (comBuf[0] == 'C') {

                Stimjim.getCurrentOffsets();
                Stimjim.getVoltageOffsets();

            } else if (comBuf[0] == 'D') { // print offset values for user reference
                char str[100];
                sprintf(str, "ADC offsets (+-2.5V): %f, %f\r\nADC offsets (+-10V): %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n",
                    Stimjim.adcOffset25[0],Stimjim.adcOffset25[1], Stimjim.adcOffset10[0],Stimjim.adcOffset10[1],
                    Stimjim.currentOffsets[0], Stimjim.currentOffsets[1], Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1] );
                Serial.println(str);

            } else if (comBuf[0] == 'R') {

                int trigSrc = 0, output = 0;
                sscanf(comBuf + 1, "%d,%d,%d", &trigSrc, &ptIndex, &output );

                if (ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                if (ptIndex >= 0 && !output) {
                      Serial.print("Attaching interrupt to IN"); Serial.print(trigSrc);
                      Serial.print(" to run PulseTrain["); Serial.print(ptIndex); Serial.println("]");
                      pinMode((trigSrc) ? IN1 : IN0, INPUT);
                      triggerTargetPTs[trigSrc] = ptIndex;
                      attachInterrupt( (trigSrc) ? IN1 : IN0, (trigSrc) ? startIT1ViaInputTrigger : startIT0ViaInputTrigger, RISING);
                      trigOutput[trigSrc] = false;
                } else {
                      Serial.print("Detaching interrupt to IN"); Serial.println(trigSrc);
                      Serial.print("Programming IN"); Serial.print(trigSrc); Serial.print(" as output that indicates activity on output ");Serial.println(trigSrc);
                      detachInterrupt( (trigSrc) ? IN1 : IN0);
                      triggerTargetPTs[trigSrc] = -1;
                      pinMode((trigSrc) ? IN1 : IN0, OUTPUT);
                      trigOutput[trigSrc] = true;
                }

            } else if (comBuf[0] == 'M') {

                int channel = 0, mode = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &mode );
                Stimjim.setOutputMode(channel, mode);
                Serial.print("Set channel "); Serial.print(channel); Serial.print(" to mode "); Serial.println(mode);

            } else if (comBuf[0] == 'V') {

                int channel = 0, amp = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &amp );
                int dacVal = 1.0 * amp / MILLIVOLTS_PER_DAC + Stimjim.voltageOffsets[channel];
                if (dacVal <= 32767 && dacVal >= -32768) {
                    Stimjim.writeToDac(channel, dacVal);
                    Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.print(amp);
                    Serial.print(" mV (dac value "); Serial.print(dacVal); Serial.println(").");
                } else {
                    Serial.print(dacVal); Serial.println(" is out of range.");
                }

            } else if (comBuf[0] == 'A') {

                int channel = 0, amp = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &amp );

                if (amp <= 32767 && amp >= -32768) {
                    Stimjim.writeToDac(channel, amp);
                    Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.println(amp);
                } else {
                    Serial.print(amp); Serial.println(" is out of range.");
                }

            } else if (comBuf[0] == 'E') {
                int channel = 0, line = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &line );
                int val = Stimjim.readAdc(channel, line);
                int valRealUnits = (val - Stimjim.adcOffset10[channel]) * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
                Serial.print("Read value: "); Serial.print(val); Serial.print(" ("); Serial.print(valRealUnits);
                Serial.println((line)?"uA)":"mV)");
            }

            bytesRecvd = 0; // reset the pointer!
        }
    }
}

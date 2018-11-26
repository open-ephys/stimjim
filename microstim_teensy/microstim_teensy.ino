#include <SPI.h>  // include the SPI library:

#define MICROAMPS_PER_DAC 0.40690104166     // 5V * (1/(3000 V/A)) / 2^12 = 0.4uA / DAC unit
#define MICROAMPS_PER_ADC 0.2325            // 20V * (1/(10500 V/A)) / 2^13 = 0.2325
#define SLEW_FUDGE 10                         // microseconds
const int OEpin[] = {8, 7};

// DAC and ADC setup
const int DacSelectPin = 10;
const int DacLdacPin = 9;
const int AdcSelectPin = 31;

// set up the speed, mode and endianness of each device
SPISettings settingsDAC(16000000, MSBFIRST, SPI_MODE0); //max 20MHz, mode 0,0 or 1,1 acceptable
SPISettings settingsADC(5000000, MSBFIRST, SPI_MODE2);  // clock starts high, data latch on falling edge

// Serial setup
char comBuf[100];
char *pch;
int nChar;

// data from adc;
int lastAdcRead[2];
volatile long avgCurrent[3];
volatile int nPulses;

int dcOffset[2];

//T0,1,0,100,-100,100,1000,1000000

// PulseTrain parameter setup
struct PulseTrain {
  int current;
  unsigned int ch[2];     
  int amplitude[3];       // uV
  unsigned int pulseWidth;       // usec
  unsigned int period;           // usec
  unsigned long duration;         // usec
  
  unsigned long trainStartTime;
};

volatile PulseTrain PT;
IntervalTimer IT;


void pulse();
void getDcOffsets();
void writeToDacs(int amp0, int amp1);
void writeToDac(int ch, int amp);
void setupADC();
void readADC(int *data);






void getDcOffsets(){
  for (int ch=0; ch<2; ch++){
    digitalWrite(OEpin[ch], HIGH); // make sure there's no output!
    int bestOffset = 0;
    int bestDcVal = 10000;
    for (int i= -500; i<500; i++){
      writeToDac(ch, i);
      delayMicroseconds(100);
      readADC(lastAdcRead);
      if ( abs(lastAdcRead[ch]) < abs(bestDcVal)){
        bestOffset=i;
        bestDcVal = lastAdcRead[ch];
      }
    }
    dcOffset[ch] = bestOffset;
  }
  char str[40];
  sprintf(str, "best offsets: %d, %d\n", dcOffset[0],dcOffset[1]);
  Serial.print(str);
}

void pulse() {
  if (micros() - PT.trainStartTime > PT.duration) {
    char str[40];
    sprintf(str, "Train complete. Average current in 3 phases: %d, %d, %d.\n", 
            avgCurrent[0]/nPulses, avgCurrent[1]/nPulses, avgCurrent[2]/nPulses);
    Serial.print(str);
    IT.end();
    return;
  }
  if (PT.ch[0])   digitalWrite(OEpin[0], LOW);
  if (PT.ch[1])   digitalWrite(OEpin[1], LOW);

  for (int i=0; i<3; i++){
    writeToDacs(1.0*PT.amplitude[i]/MICROAMPS_PER_DAC * (PT.ch[0]>0), 
                1.0*PT.amplitude[i]/MICROAMPS_PER_DAC * (PT.ch[1]>0));            
    unsigned long t = micros();
    delayMicroseconds(PT.pulseWidth-SLEW_FUDGE);
    readADC(lastAdcRead); 
    avgCurrent[i] += lastAdcRead[0]*MICROAMPS_PER_ADC;
  }
  if (PT.ch[0])   digitalWrite(OEpin[0], HIGH);
  if (PT.ch[1])   digitalWrite(OEpin[1], HIGH);
  nPulses++;
}





// input arguments amp0 and amp1 go from -2048 to 2047.
void writeToDacs(int amp0, int amp1) {
  amp0 = amp0 + 2048 + dcOffset[0];
  amp1 = amp1 + 2048 + dcOffset[1];
  if (amp0 > 4095) amp0 = 4095;
  if (amp1 > 4095) amp1 = 4095;
  if (amp0 < 0) amp0 = 0;
  if (amp0 < 0) amp1 = 0;
  

  SPI.beginTransaction(settingsDAC);
  //bit order for DAC (MCP4922) is notA, buf, notGA, notShutdown, Data11,Data10...Data0
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer16( 8192+4096+amp0);
  digitalWrite(DacSelectPin, HIGH);
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer16(32768+ 8192+4096+amp1);
  digitalWrite(DacSelectPin, HIGH);
  SPI.endTransaction();

  // latch data
  digitalWrite(DacLdacPin, LOW);    
  digitalWrite(DacLdacPin, HIGH);
}



// amp goes from -2048 to 2047.
void writeToDac(int ch, int amp) {
  amp = amp + 2048 + dcOffset[ch];
  if (amp > 4095) amp = 4095;
  if (amp < 0) amp = 0;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer16( (ch ? 32768 : 0) + 8192 + 4096 + amp);
  digitalWrite(DacSelectPin, HIGH);
  SPI.endTransaction();
  
  digitalWrite(DacLdacPin, LOW);   
  digitalWrite(DacLdacPin, HIGH);
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
  2: sequencer
*/
void setupADC() {
  SPI1.beginTransaction(settingsADC);
  digitalWrite(AdcSelectPin, LOW);
  int data = 32768 + 8192; // write range register, all zeros (+-10V on both channels)
  SPI1.transfer(data / 256);
  SPI1.transfer(data % 256);
  digitalWrite(AdcSelectPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(AdcSelectPin, LOW);
  data = 32768 + 1024 + 16 + 8; // write control register, ch1, use internal ref, use sequencer
  SPI1.transfer(data / 256);
  SPI1.transfer(data % 256);
  digitalWrite(AdcSelectPin, HIGH);
  SPI1.endTransaction();
}

void readADC( int *data) {
  SPI1.beginTransaction(settingsADC);

  digitalWrite(AdcSelectPin, LOW);
  data[0] = 256 * SPI1.transfer(0);
  data[0] += SPI1.transfer(0);
  if (data[0] > 4095)
    data[0] += - 8192; 
  digitalWrite(AdcSelectPin, HIGH);

  digitalWrite(AdcSelectPin, LOW);
  data[1] = 256 * SPI1.transfer(0);
  data[1] += SPI1.transfer(0);
  digitalWrite(AdcSelectPin, HIGH);

  SPI1.endTransaction();
}





void setup() {
  delay(500);
  Serial.begin(9600);
  Serial.println("Booting microstim on Teensy 3.5!");
  
  Serial.println("Initializing DAC and ADC...");
  pinMode(DacLdacPin, OUTPUT);
  digitalWrite(DacLdacPin, HIGH);
  pinMode(DacSelectPin, OUTPUT);
  pinMode(AdcSelectPin, OUTPUT);
  digitalWrite(DacSelectPin, HIGH);
  digitalWrite(AdcSelectPin, HIGH);

  Serial.println("Setting initial outputs off...");
  for (int i = 0; i < 2; i++) {
    pinMode(OEpin[i], OUTPUT);
    digitalWrite(OEpin[i], HIGH); // LOW=ENABLE, HIGH=DISABLE OUTPUTS ON BOOT
  }

  nChar = 0;

  Serial.println("Initializing SPI...");
  SPI.begin();
  SPI1.begin();
  
  Serial.println("Initializing ADC...");
  setupADC();
  
  Serial.println("Measuring DC offset...");
  getDcOffsets();
  
  PT.trainStartTime = millis();
  PT.ch[0] = 0;
  PT.ch[1] = 0;
  for (int i=0; i < 3; i++)
    PT.amplitude[i] = 0;
  PT.pulseWidth=100;
  PT.period=1000; 
  PT.duration = 0;

  Serial.println("Ready to go!");
}






void loop() {

  if (Serial.available() > 0) {
    Serial.readBytes(comBuf + nChar, 1); // read one byte into the buffer
    nChar++; // keep track of the number of characters we've read!

    if (comBuf[nChar - 1] == '\n') { // termination character for string - means we've recvd a full command!
      if (comBuf[0] == 'T' || comBuf[0] == 'S') {
        int m = sscanf(comBuf+1, "%d,%d,%d,%d,%d,%d,%d,%ld", &(PT.current), &(PT.ch[0]), &(PT.ch[1]),
                 &(PT.amplitude[0]),&(PT.amplitude[1]), &(PT.pulseWidth), &(PT.period), &(PT.duration) );
        Serial.print("current: "); Serial.println(PT.current);
        Serial.print("ch[0]: "); Serial.println(PT.ch[0]);
        Serial.print("ch[1]: "); Serial.println(PT.ch[1]);
        Serial.print("amplitude[0]: "); Serial.println(PT.amplitude[0]);
        Serial.print("amplitude[1]: "); Serial.println(PT.amplitude[1]);
        Serial.print("pulseWidth: "); Serial.println(PT.pulseWidth);
        Serial.print("period: "); Serial.println(PT.period);
        Serial.print("duration: "); Serial.println(PT.duration);
        
        if (comBuf[0] == 'T') {
          nPulses=0;
          for (int i=0;i<3;i++) avgCurrent[i]=0;
          PT.trainStartTime = micros();
          if (!IT.begin(pulse, PT.period))
            Serial.println("loop: failure to initiate intervalTimer");
        }
      }
      else if (comBuf[0] == 'D'){
         int m = sscanf(comBuf+1, "%d,%d", &(dcOffset[0]), &(dcOffset[1]) );
         Serial.print("dcOffset[0]: "); Serial.println(dcOffset[0]);
         Serial.print("dcOffset[1]: "); Serial.println(dcOffset[1]);
      }
      else if (comBuf[0] == 'C'){
         getDcOffsets();
      }
      nChar = 0; // reset the pointer!
    }
  }
}

#include <SPI.h>  // include the SPI library:

const int OEpin[] = {8, 7};


const int DacSelectPin = 10;
const int DacLdacPin = 9;
const int AdcSelectPin = 31;

// set up the speed, mode and endianness of each device
SPISettings settingsDAC(16000000, MSBFIRST, SPI_MODE0); //max 20MHz, mode 0,0 or 1,1 acceptable
SPISettings settingsADC(5000000, MSBFIRST, SPI_MODE2);  // clock starts high, data latch on falling edge

//bit order for DAC (MCP4922) is notA, buf, notGA, notShutdown, Data11,Data10...Data0
//bit order for DAC (MCP4922) is 0, zero, channel, sign,  Data11,Data10...Data0

// have to send (1) write bit (0=do-nothing;1=write), (2) zero (3) register select bit (0=control reg; 1=range reg), 12-bit data, dont-care






// amp0 and amp1 go from -2048 to 2047.
void writeToDacs(int amp0, int amp1) {
  amp0 = amp0 + 2048;
  amp1 = amp1 + 2048;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer( (byte) (32 + 16 + amp0 / 256));
  SPI.transfer(amp0 % 256);
  digitalWrite(DacSelectPin, HIGH);
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer( (byte) (128 + 32 + 16 + amp1 / 256 ));
  SPI.transfer(amp1 % 256);
  digitalWrite(DacSelectPin, HIGH);
  SPI.endTransaction();

  // latch data
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);
}



// amp0 go from -2048 to 2047.
void writeToDac(int dac, int amp) {
  amp = amp + 2048;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin, LOW);
  SPI.transfer( (byte) ( ((dac) ? 128 : 0) + 32 + 16 + amp / 256));
  SPI.transfer(amp % 256);
  digitalWrite(DacSelectPin, HIGH);
  SPI.endTransaction();
  // latch data
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);
}


/*15: write
14: 0 
13: control register (0) or range register(1)
12: 0
11: 0 
10: add0 - selects which channel to read on next conversion. should use 1 if using sequencer.
9: mode1 (0 for single-ended)
8: mode0 (0 for single-ended)
7: PM1 (0 for on all time)
6: PM0 (0 for on all time) 
5: coding (0 = two's complement, 1 = binary)
4: reference (0 = external; 1=internal)
3: sequencer - if (1,0), then alternate channels
2: sequencer
1: 0 
*/
void setupADC(){
  SPI1.beginTransaction(settingsADC);
  digitalWrite(AdcSelectPin, LOW);
  int data = 32768+8192; // write range register, all zeros (+-10V on both channels)
  SPI1.transfer(data/256);
  SPI1.transfer(data%256);
  digitalWrite(AdcSelectPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(AdcSelectPin, LOW);
  data = 32768 + 1024 + 16 + 8; // write control register, ch1, use internal ref, use sequencer
  SPI1.transfer(data/256);
  SPI1.transfer(data%256);
  digitalWrite(AdcSelectPin, HIGH);
  SPI1.endTransaction();
}

int readADC() {
  SPI1.beginTransaction(settingsADC);
  
    digitalWrite(AdcSelectPin, LOW);
    int a = 256*SPI1.transfer(0);
    a += SPI1.transfer(0);
    digitalWrite(AdcSelectPin, HIGH);
    
    digitalWrite(AdcSelectPin, LOW);  
    int b = 256*SPI1.transfer(0);
    b += SPI1.transfer(0);
    digitalWrite(AdcSelectPin, HIGH);
    
  SPI1.endTransaction();
  int ch0 = a/2048;
  Serial.print(a); Serial.print(","); 
  Serial.println(b); 
  return (a);
}





void setup() {
  pinMode(DacLdacPin, OUTPUT);
  digitalWrite(DacLdacPin, HIGH);

  pinMode(DacSelectPin, OUTPUT);
  pinMode(AdcSelectPin, OUTPUT);
  digitalWrite(DacSelectPin, HIGH);
  digitalWrite(AdcSelectPin, HIGH);

  for (int i = 0; i < 2; i++) {
    pinMode(OEpin[i], OUTPUT);
    digitalWrite(OEpin[i], HIGH); // LOW=ENABLE, HIGH=DISABLE OUTPUTS ON BOOT
  }

  Serial.begin(9600);
  SPI.begin();
  SPI1.begin();
  delay(100);
  setupADC();
}






void loop() {
  float a = sin(0.002 * millis());

  //digitalWrite(OEpin[0], LOW);
  /*writeToDacs(900*a+1000, 0);
  delayMicroseconds(100);
  writeToDacs(-900*a-1000, 0);
  delayMicroseconds(100);
  writeToDacs(0,0);
  delayMicroseconds(400);*/
  //  digitalWrite(OEpin[0], HIGH); 
  
  writeToDacs(2047 * a, 0);
  if (millis() % 10 == 0) {
    readADC();
  }
  delay(1);

}

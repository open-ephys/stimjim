#include <SPI.h>  // include the SPI library:

const int OEpin[] = {8,7};


const int DacSelectPin = 10;
const int DacLdacPin = 9;
const int AdcSelectPin = 21;

// set up the speed, mode and endianness of each device
SPISettings settingsDAC(16000000, MSBFIRST, SPI_MODE0); //max 20MHz, mode 0,0 or 1,1 acceptable
SPISettings settingsADC(1000000, MSBFIRST, SPI_MODE1); 

//bit order for DAC (MCP4922) is notA, buf, notGA, notShutdown, Data11,Data10...Data0
//bit order for DAC (MCP4922) is 0, zero, channel, sign,  Data11,Data10...Data0

// have to send (1) write bit (0=do-nothing;1=write), (2) zero (3) register select bit (0=control reg; 1=range reg), 12-bit data, dont-care




void setup() {                
  pinMode(DacLdacPin, OUTPUT); 
  digitalWrite(DacLdacPin, HIGH); 

  pinMode(DacSelectPin, OUTPUT); 
  pinMode(AdcSelectPin, OUTPUT);     
  digitalWrite(DacSelectPin, HIGH);
  digitalWrite(AdcSelectPin, HIGH);
      
  for (int i=0; i<2; i++){
    pinMode(OEpin[i], OUTPUT);
    digitalWrite(OEpin[i], HIGH); // FOR TESTING!
  }

  Serial.begin(9600);
  SPI.begin();
}


void loop() {
  /*SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin,LOW);
  double sinphase = micros()/1000.0*6.28;
  int amp = 4095*(sin(sinphase)+1)/2;
  //amp = (micros() % 1000000 < 100)?4095:0;
  //Serial.println(amp);
  SPI.transfer( (byte) (32+16 + amp/256));
  SPI.transfer(amp % 256);
  digitalWrite(DacSelectPin,HIGH); 
  SPI.endTransaction();
  digitalWrite(DacLdacPin, LOW);
  delayMicroseconds(1);
  digitalWrite(DacLdacPin, HIGH);  */

  //digitalWrite(OEpin[0], LOW);
  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (32+16 + 15));
  SPI.transfer(255);
  digitalWrite(DacSelectPin,HIGH); 
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (128+32+16 ));
  SPI.transfer(0);
  digitalWrite(DacSelectPin,HIGH); 
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);  
  
  delayMicroseconds(10);
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (32+16 + 0/256));
  SPI.transfer(0 % 256);
  digitalWrite(DacSelectPin,HIGH); 
  digitalWrite(DacLdacPin, LOW);     digitalWrite(DacLdacPin, HIGH);  
  delayMicroseconds(15);
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (32+16 + 8));
  SPI.transfer(0 % 256);
  digitalWrite(DacSelectPin,HIGH); 
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);  
  SPI.endTransaction();
  delayMicroseconds(400);
  //digitalWrite(OEpin[0], HIGH);
  delay(1);
  
}

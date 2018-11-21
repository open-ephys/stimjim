#include <SPI.h>  // include the SPI library:

const int OEpin[] = {8,7};


const int DacSelectPin = 10;
const int DacLdacPin = 9;
const int AdcSelectPin = 31;

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
    digitalWrite(OEpin[i], HIGH); // DISABLE OUTPUTS ON BOOT
  }

  Serial.begin(9600);
  SPI.begin();
  SPI1.begin();
}


// amp0 and amp1 go from -2048 to 2047.
void writeToDacs(int amp0, int amp1){
  amp0 = amp0+2048;
  amp1 = amp1+2048;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (32+16 + amp0/256));
  SPI.transfer(amp0%256);
  digitalWrite(DacSelectPin,HIGH); 
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) (128+32+16 + amp1/256 ));
  SPI.transfer(amp1%256);
  digitalWrite(DacSelectPin,HIGH); 
  SPI.endTransaction();

  // latch data
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);  
}



// amp0 go from -2048 to 2047.
void writeToDac(int dac, int amp){
  amp = amp+2048;

  SPI.beginTransaction(settingsDAC);
  digitalWrite(DacSelectPin,LOW);
  SPI.transfer( (byte) ( ((dac)?128:0)+32+16 + amp/256));
  SPI.transfer(amp/256);
  digitalWrite(DacSelectPin,HIGH); 
  SPI.endTransaction();
  // latch data
  digitalWrite(DacLdacPin, LOW);    digitalWrite(DacLdacPin, HIGH);  
}

int readADC(){
  SPI1.beginTransaction(settingsADC);
  digitalWrite(AdcSelectPin, LOW);
  int a =   SPI.transfer(0);
  int b =   SPI.transfer(0);
  digitalWrite(AdcSelectPin, HIGH);
  Serial.print(a); Serial.print(","); Serial.println(b);

  SPI1.endTransaction();
  return(a*256+b);
}


void loop() {

  float a = sin(0.002*millis());
  
  /* writeToDacs(900*a+1000, 0);
  digitalWrite(OEpin[0], LOW);  
  delayMicroseconds(100);
  writeToDacs(-900*a-1000, 0);
  delayMicroseconds(100);
  writeToDacs(0,0);
  delayMicroseconds(400);
  digitalWrite(OEpin[0], HIGH); */
  writeToDacs(1000*a,1000*a);
  delay(100);
  readADC();
  
}

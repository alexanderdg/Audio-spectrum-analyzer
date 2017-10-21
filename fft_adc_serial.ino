/*
fft_adc_serial.pde
guest openmusiclabs.com 7.7.14
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.
*/

#define LOG_OUT 1 // use the log output function

#define FFT_N 256 // set to 256 point fft

#include <FFT.h> // include the library
#include <SPI.h>

#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss

byte clr;
uint16_t lut[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
int levels[16];

int fftFast[128];

void setup() {
  Serial.begin(115200); // use the serial port
  // put your setup code here, to run once:
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  digitalWrite(SLAVESELECT,HIGH);
  SPCR = (1<<SPE)|(1<<MSTR);
  clr=SPSR;
  clr=SPDR;
  initDisplay();
  intensity(15);
  selfTestOn();
  delay(1000);
  selfTestOff();
}

void loop() {
  
  while(1) { // reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0
    //Take the slow samples
    setSampleSlow();
    takeSampleSlow();
    processFFT();
    setDecade(0,fft_log_out[3]/12);
    setDecade(1,fft_log_out[7]/12);
    setDecade(2,fft_log_out[10]/12);
    setDecade(3,fft_log_out[15]/12);
    setDecade(4,fft_log_out[22]/12);
    setDecade(5,fft_log_out[32]/12);
    setDecade(6,fft_log_out[47]/12);
    setDecade(7,fft_log_out[70]/12);
    setDecade(8,fft_log_out[104]/12);
    //Take the fast samples
    setSampleFast();
    takeSampleFast();
    processFFT();
    setDecade(9,fft_log_out[10]/12);
    setDecade(10,fft_log_out[14]/12);
    setDecade(11,fft_log_out[21]/12);
    setDecade(12,fft_log_out[31]/12);
    setDecade(13,fft_log_out[46]/12);
    setDecade(14,fft_log_out[68]/12);
    setDecade(15,fft_log_out[100]/12);
    sei();
    /*
    Serial.println("start");
    for (byte i = 0 ; i < FFT_N/2 ; i++) { 
      Serial.print(fft_log_out[i]);
      Serial.print(" i:");
      Serial.println(i);
      //setDecade(i/6,fft_log_out[i + 5]/10);
    }
    */
    refresh();
    //delay(10000);
  }
}

void processFFT(void) {
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_log(); // take the output of the fft
}

//--------------------------------------------------------------------------------------------//
//                                                                                            //
//                      Code for the internal ADC and Sampling                                //
//                                                                                            //
//--------------------------------------------------------------------------------------------//

bool copyFastSample(void) {
  for(int i = 0; i < FFT_N/2; i++)
  {
    fftFast[i] = fft_log_out[i];
  }
  return true;
}

bool setSampleFast(void) {
  //set sample rate to 38.46KHz
  TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe5; // set the adc to free running mode with prescaler 32
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
  return true;
}

bool setSampleSlow(void) {
  //set sample rate to 9.61KHz
  TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe7; // set the adc to free running mode with prescaler 128
  ADMUX = 0x41; // use adc1
  DIDR0 = 0x01; // turn off the digital input for adc0
  return true;
}

bool takeSampleFast(void) {
  for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf5; // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fft_input[i] = k; // put real data into even bins
      fft_input[i+1] = 0; // set odd bins to 0
    }
  return true;
}

bool takeSampleSlow(void) {
  for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
     byte m, j;
     for(int i = 0; i < 16; i++)
     {   
        while(!(ADCSRA & 0x10)); // wait for adc to be ready
        ADCSRA = 0xf5; // restart adc
        m = ADCL; // fetch adc data
        j = ADCH;
      //throw one sample away because the prescaler is at his maximum
     }
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fft_input[i] = k; // put real data into even bins
      fft_input[i+1] = 0; // set odd bins to 0
    }
  return true;
}



//--------------------------------------------------------------------------------------------//
//                                                                                            //
//                      Code for the Max7219 display                                          //
//                                                                                            //
//--------------------------------------------------------------------------------------------//
bool initDisplay(void) {
    sendCommand(0x0900);
    sendCommand(0x0A0F);
    sendCommand(0x0B07);
    sendCommand(0x0C01);
    return true;
}
bool selfTestOn(void) {
    sendCommand(0x0F01);
    return true;   
}

bool selfTestOff(void) {
    sendCommand(0x0F00);
    return true;   
} 

bool setDecade(uint8_t decade, uint8_t level) {
    levels[decade] = level;
    return true;   
}

uint16_t decode(uint8_t value) {
    uint16_t temp = 0;
    for(int i = 1; i <= value; i++) {
        temp = temp + lut[i-1];   
    }
    return temp;   
}

bool refresh(void) {
   for(int i = 0; i < 8; i++) {
        uint8_t u1 = decode(levels[i]) & 0xFF;
        uint8_t u3 = decode(levels[7-i]) >> 8;
        uint8_t u2 = decode(levels[i+8]) & 0xFF;
        uint8_t u4 = decode(levels[15-i]) >> 8;
        uint8_t ru1 = reverse(u1);
        uint8_t ru2 = reverse(u2);
        uint8_t ru3 = reverse(u3);
        uint8_t ru4 = reverse(u4);
        //pc.printf("%d, U1:%u  U2:%u  U3:%u  U4:%u \n", i+1 << 8, u1, u2, ru3, ru4);
        sendCommand(((i+1) << 8) + ru1, ((i+1) << 8) + ru2, ((i+1) << 8) + u4, ((i+1) << 8) + u3);
   }
   return true;     
}

bool intensity(uint8_t level) {
    if(level <= 15) {
        sendCommand(((0x0A) << 8) + ((level*2)+1));
        return true;
    }
    else {
        return false;
    }  
}

uint8_t reverse(uint8_t b) {
   // Reverse the top and bottom nibble then swap them.
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

bool sendCommand(int command)
{
  sendCommand(command, command, command, command);
}

bool sendCommand(int command1,int command2,int command3,int command4)
{
  digitalWrite(SLAVESELECT,LOW);
  spi_transfer(command4>>8); //write enable
  spi_transfer(command4);
  spi_transfer(command3>>8); //write enable
  spi_transfer(command3);
  spi_transfer(command2>>8); //write enable
  spi_transfer(command2);
  spi_transfer(command1>>8); //write enable
  spi_transfer(command1);
  digitalWrite(SLAVESELECT,HIGH);
  return true;
}

char spi_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

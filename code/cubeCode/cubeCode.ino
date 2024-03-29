#define DEV_ADDR   103
#define RF_FREQ    433.200
#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3
#define RREF       4300.0
#define RNOMINAL   1000.0
#define commLEDPin 13
#define trip1Pin 5
#define trip2Pin 6
#define reedPin 18
#define currentMonPin A0
#define vbatPin A7

#include "Adafruit_MAX31865.h"

#include <SPI.h>
#include "RH_RF95.h"

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);

int sigPower = 20;
int modemConfigIndex = 0;
float rfFreq = RF_FREQ;
struct RadioPacketSend
{
  uint16_t iaddr = DEV_ADDR;
  uint16_t irssi = 0;
  uint16_t ideltat = 0;
  uint16_t idata[11] = {0,0,0,0,0,0,0,0,0,0,1};
};
RadioPacketSend radioPacketSend;
uint8_t sizeOfRadioPacketSend = sizeof(radioPacketSend);

struct RadioPacketRecv
{
  uint16_t iaddr = 0;
  unsigned long msgTime;
};
RadioPacketRecv radioPacketRecv;
uint8_t sizeOfRadioPacketRecv = sizeof(radioPacketRecv);

unsigned long tnow;
unsigned long tsent;

float avgAdc = 0.0;
float nsamples = 5000.0;
float nsamplesCnt = 1.0;
float avgRms = 0.0;
float adcValue;
float rmsAdc;
float nfilter = 5.0;
float filteredAdc = 0.0;
float nfilterCnt = 1.0;
int icnt = 0;
float rmsMax = 0.0;
float rmsSum = 0.0;
float rmsCnt = 0.0;

void setup() 
{
  Serial.begin(9600);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(rfFreq);
  rf95.setModemConfig(modeConfig[modemConfigIndex]); 
  rf95.setTxPower(sigPower, false);

  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary  
 
  pinMode(commLEDPin, OUTPUT);
  pinMode(trip1Pin, INPUT);
  pinMode(trip2Pin, INPUT);
  pinMode(reedPin, INPUT);
  pinMode(currentMonPin, INPUT);
  pinMode(vbatPin, INPUT);

  digitalWrite(commLEDPin, LOW);
  tnow  = millis();
  tsent = tnow;
  delay(1000);
}


void loop() 
{
  float frms = 0.0;
  adcValue = (float) analogRead(currentMonPin);
  filteredAdc = filteredAdc + (adcValue - filteredAdc) / nfilterCnt;
  avgAdc = avgAdc + (adcValue - avgAdc) / nsamplesCnt;
  rmsAdc = (filteredAdc - avgAdc);
  rmsAdc = rmsAdc * rmsAdc;
  avgRms = avgRms + (rmsAdc - avgRms) / nsamplesCnt;
  if (nsamplesCnt < nsamples) nsamplesCnt = nsamplesCnt + 1.0;
  if (nfilterCnt < nfilter) nfilterCnt = nfilterCnt + 1.0;
 
  frms = 32.0 * sqrt(avgRms);
  if (rmsMax < frms ) rmsMax = frms;
  rmsSum = rmsSum + frms;
  rmsCnt = rmsCnt + 1;
  
  radioPacketSend.idata[3] = (uint16_t) (32.0 * avgAdc);
  radioPacketSend.idata[4] = (uint16_t) frms;

  while (rf95.available())
  {
    if (rf95.recv((uint8_t *)&radioPacketRecv, &sizeOfRadioPacketRecv))
    {
      if (radioPacketRecv.iaddr == radioPacketSend.iaddr)
      {
        
        radioPacketSend.idata[0] = (uint16_t) digitalRead(trip1Pin);
        radioPacketSend.idata[1] = (uint16_t) digitalRead(trip2Pin);
        radioPacketSend.idata[2] = readTemperature();
        radioPacketSend.idata[5] = (uint16_t) rmsMax;
        radioPacketSend.idata[6] = (uint16_t) rmsSum;
        radioPacketSend.idata[7] = (uint16_t) digitalRead(reedPin);
        radioPacketSend.idata[8] = (uint16_t) analogRead(vbatPin);
        rmsMax = 0.0;
        rmsSum = 0.0;
        rmsCnt = 0.0;
        for (int ii = 0; ii < 10; ++ii)
        {
          Serial.print(radioPacketSend.idata[ii]);
          Serial.print(",  ");
        }
        Serial.println(radioPacketSend.idata[10]);
        
        radioPacketSend.irssi = (uint16_t) rf95.lastRssi();
        tnow  = millis();
        radioPacketSend.ideltat = (uint16_t) ((tnow - tsent) / 100); // 0.1 secs
        tsent = tnow;
        
        delay(200);
        digitalWrite(commLEDPin, HIGH);
        rf95.send((uint8_t *)&radioPacketSend, sizeOfRadioPacketSend);
        delay(10);
        rf95.waitPacketSent();
        digitalWrite(commLEDPin, LOW);
        delay(100);
      }
    }
  }

//  uint16_t temperature = readTemperature();
//  Serial.print("Temperature = "); Serial.println(temperature);
//  delay(1000);
}

uint16_t readTemperature()
{
  uint16_t rtd = thermo.readRTD();
  float temperature  = thermo.temperature(RNOMINAL, RREF);

  uint8_t fault = thermo.readFault();
  if (fault) 
  {
//    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    thermo.clearFault();
  }
  temperature = round((temperature + 273.15) * 100);
  uint16_t utemperature = (uint16_t) temperature;
  return utemperature;
}

#include <SPI.h>
#include "RH_RF95.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int sigPower = 20;
int modemConfigIndex = 1;
float rfFreq = 433.550;

const int currentMonPin = A0;
const int vbatPin = A7;
const int commLEDPin = 13;
const int trip1Pin = 11;
const int trip2Pin = 12;
const int tempPin = A1;

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
struct RadioPacketSend
{
  uint16_t iaddr = 102;
  uint16_t vbat = 0;
  uint16_t iavg = 0;
  uint16_t irms = 0;
  uint16_t itrip1 = 0;
  uint16_t itrip2 = 0;
  uint16_t itemp = 0;
  uint16_t irssi = 0;
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

void setup() 
{
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

  pinMode(currentMonPin, INPUT);
  pinMode(vbatPin, INPUT);
  pinMode(commLEDPin, OUTPUT);
  pinMode(trip1Pin, INPUT);
  pinMode(trip2Pin, INPUT);
  pinMode(tempPin, INPUT);
  digitalWrite(commLEDPin, LOW);
//  Serial.begin(9600);
  
}
void loop() 
{
  radioPacketSend.vbat = (uint16_t) analogRead(vbatPin);
  radioPacketSend.itrip1 = (uint16_t) digitalRead(trip1Pin);
  radioPacketSend.itrip2 = (uint16_t) digitalRead(trip2Pin);
  radioPacketSend.itemp = (uint16_t) analogRead(tempPin);
  
  adcValue = (float) analogRead(currentMonPin);
  filteredAdc = filteredAdc + (adcValue - filteredAdc) / nfilterCnt;
  avgAdc = avgAdc + (adcValue - avgAdc) / nsamplesCnt;
  rmsAdc = (filteredAdc - avgAdc);
  rmsAdc = rmsAdc * rmsAdc;
  avgRms = avgRms + (rmsAdc - avgRms) / nsamplesCnt;
  if (nsamplesCnt < nsamples) nsamplesCnt = nsamplesCnt + 1.0;
  if (nfilterCnt < nfilter) nfilterCnt = nfilterCnt + 1.0;
 
  radioPacketSend.iavg = (uint16_t) (avgAdc * 100.0);
  radioPacketSend.irms = (uint16_t) (sqrt(avgRms) * 100.0);

  while (rf95.available())
  {
    if (rf95.recv((uint8_t *)&radioPacketRecv, &sizeOfRadioPacketRecv))
    {
      if (radioPacketRecv.iaddr == radioPacketSend.iaddr)
      {
        
        radioPacketSend.irssi = (uint16_t) (-rf95.lastRssi());
        
/*
        Serial.println(radioPacketRecv.msgTime);
        Serial.print(radioPacketSend.iavg);
        Serial.print(",");
        Serial.print(radioPacketSend.irms);
        Serial.print(",");
        Serial.print(radioPacketSend.vbat);
        Serial.print(",");
        Serial.print(radioPacketSend.itrip1);
        Serial.print(",");
        Serial.print(radioPacketSend.itrip2);
        Serial.print(",");
        Serial.print(radioPacketSend.itemp);
        Serial.print(",");
        Serial.println(radioPacketSend.irssi);
*/   
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
}

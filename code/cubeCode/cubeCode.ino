#define DEV_ADDR 102
#define RF_FREQ  433.600
/*
 * idata[0] = trip1
 * idata[1] = trip2
 * idata[2] = temp raw
 * idata[3] = current offset raw * 32
 * idata[4] = current stdev raw  * 32
 * idata[5] = current rmsMax * 32
 * idata[6] = current rmsAvg over int  * 32
 */
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
int modemConfigIndex = 0;
float rfFreq = RF_FREQ;

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
float rmsMax = 0.0;
float rmsSum = 0.0;
float rmsCnt = 0.0;
struct RadioPacketSend
{
  uint16_t iaddr = DEV_ADDR;
  uint16_t irssi = 0;
  uint16_t ideltat = 0;
  uint16_t idata[11] = {0,0,0,0,0,0,0,0,0,0,0};
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

  tnow  = millis();
  tsent = tnow;
//  Serial.begin(9600);
//  delay(5000);
  
}
void loop() 
{
  float frms = 0.0;
  radioPacketSend.idata[0] = (uint16_t) digitalRead(trip1Pin);
  radioPacketSend.idata[1] = (uint16_t) digitalRead(trip2Pin);
  radioPacketSend.idata[2] = (uint16_t) analogRead(tempPin);
  
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
        
        radioPacketSend.irssi = (uint16_t) rf95.lastRssi();
        tnow  = millis();
        radioPacketSend.ideltat = (uint16_t) ((tnow - tsent) / 100); // 0.1 secs
        tsent = tnow;
        if (rmsCnt > 1.0) rmsSum = rmsSum / rmsCnt;
        radioPacketSend.idata[5] = (uint16_t) rmsMax;
        radioPacketSend.idata[6] = (uint16_t) rmsSum;
        rmsMax = 0.0;
        rmsSum = 0.0;
        rmsCnt = 0.0;
        
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

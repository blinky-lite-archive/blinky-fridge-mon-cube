#include <SPI.h>
#include "RH_RF95.h"
#include "ModbusRtu.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define TIMEOUTMILLIS 2000

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int sigPower = 20;
int modemConfigIndex = 1;
float rfFreq = 433.550;

const int commLEDPin = 13;

struct RadioPacketSend
{
  uint16_t iaddr = 0;
  unsigned long msgTime;

};
RadioPacketSend radioPacketSend;
uint8_t sizeOfRadioPacketSend = sizeof(radioPacketSend);

struct RadioPacketRecv
{
  uint16_t iaddr = 0;
  uint16_t vbat = 0;
  uint16_t iavg = 0;
  uint16_t irms = 0;
  uint16_t itrip1 = 0;
  uint16_t itrip2 = 0;
  uint16_t itemp = 0;
  uint16_t irssi = 0;
};
RadioPacketRecv radioPacketRecv;
uint8_t sizeOfRadioPacketRecv = sizeof(radioPacketRecv);

Modbus slave(1,Serial,0); // this is slave @1 and RS-232 or USB-FTDI
uint16_t deviceAddr[2] = {101, 102};
int numDevices = 2;
int deviceBufLen = 8;
uint16_t modbusBuffer[16];
int waitBetweenDevices = 3000;

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

  pinMode(commLEDPin, OUTPUT);
  digitalWrite(commLEDPin, LOW);
  for (int ii = 0; ii < numDevices * deviceBufLen; ++ii) modbusBuffer[ii] = 0;
//  Serial.begin(9600);
  Serial.begin(9600); // baud-rate at 19200
  slave.start();
}
void loop() 
{
  for (int ii = 0; ii < numDevices; ++ii)
  {
    wait(waitBetweenDevices);
    if (!sendRequest(ii))
    {
//      Serial.println("Booger");
    }
  }
}
boolean sendRequest(int idev)
{
  boolean waitForData = true;
  boolean timeOut = false;
  unsigned long lastWriteTime = millis();
  int bufferStart = idev * deviceBufLen;

  radioPacketSend.iaddr = deviceAddr[idev];
  radioPacketSend.msgTime = millis();
  digitalWrite(commLEDPin, HIGH);
  rf95.send((uint8_t *)&radioPacketSend, sizeOfRadioPacketSend);
  delay(10);
  rf95.waitPacketSent();
  digitalWrite(commLEDPin, LOW);
  waitForData = true;
  timeOut = false;
  lastWriteTime = millis();

  while (waitForData && !timeOut)
  {
    while (rf95.available())
    {
      if (rf95.recv((uint8_t *)&radioPacketRecv, &sizeOfRadioPacketRecv))
      {
        if (radioPacketRecv.iaddr == radioPacketSend.iaddr)
        {
          waitForData = false;
/*
          Serial.print(radioPacketRecv.iavg);
          Serial.print(",");
          Serial.print(radioPacketRecv.irms);
          Serial.print(",");
          Serial.print(radioPacketRecv.vbat);
          Serial.print(",");
          Serial.print(radioPacketRecv.itrip1);
          Serial.print(",");
          Serial.print(radioPacketRecv.itrip2);
          Serial.print(",");
          Serial.print(radioPacketRecv.itemp);
          Serial.print(",");
          Serial.println(radioPacketRecv.irssi);
*/
        }
      }
    }
    if ((millis() - lastWriteTime) > TIMEOUTMILLIS)
    {
      timeOut = true;
      radioPacketRecv.iaddr = deviceAddr[idev];
      radioPacketRecv.iavg = 0;
      radioPacketRecv.irms = 0;
      radioPacketRecv.vbat = 0;
      radioPacketRecv.itrip1 = 2;
      radioPacketRecv.itrip2 = 2;
      radioPacketRecv.itemp = 0;
      radioPacketRecv.irssi = 0;
      
    }

    modbusBuffer[bufferStart + 0] = radioPacketRecv.iaddr;
    modbusBuffer[bufferStart + 1] = radioPacketRecv.iavg;
    modbusBuffer[bufferStart + 2] = radioPacketRecv.irms;
    modbusBuffer[bufferStart + 3] = radioPacketRecv.vbat;
    modbusBuffer[bufferStart + 4] = radioPacketRecv.itrip1;
    modbusBuffer[bufferStart + 5] = radioPacketRecv.itrip2;
    modbusBuffer[bufferStart + 6] = radioPacketRecv.itemp;
    modbusBuffer[bufferStart + 7] = radioPacketRecv.irssi;

  }
  return !timeOut;
}
void wait(int milliSecs)
{
  unsigned long beginTime = millis();
  unsigned long now = beginTime;
  int bufLen = numDevices * deviceBufLen;
  while ((now - beginTime) < ((unsigned long) (milliSecs)) )
  {
    now = millis();
    delay(10);
    slave.poll( modbusBuffer,bufLen );
  }
  return;  
}

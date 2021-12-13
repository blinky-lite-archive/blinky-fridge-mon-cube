#define RF_FREQ  433.600
#define BAUD_RATE  57600
#include <SPI.h>
#include "RH_RF95.h"
#include "ModbusRtu.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define TIMEOUTMILLIS 2000
#define MODBUS_POLL_INTERVAL 200

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int sigPower = 20;
int modemConfigIndex = 0;
float rfFreq = RF_FREQ;

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
  uint16_t irssi = 0;
  uint16_t ideltat = 0;
  uint16_t idata[11] = {0,0,0,0,0,0,0,0,0,0,0};
};
RadioPacketRecv radioPacketRecv;
uint8_t sizeOfRadioPacketRecv = sizeof(radioPacketRecv);

int numDevices = 8;
int deviceBufLen = 14;
int numData = 11;
uint16_t modbusBuffer[113];
int modbusBufferLength = 113;

unsigned long lastModbusPollTime = 0;

Modbus slave(1,Serial,0); // this is slave @1 and RS-232 or USB-FTDI

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
  modbusBuffer[0] = 0;
  for (int ii = 0; ii < numDevices * deviceBufLen; ++ii) modbusBuffer[ii + 1] = 0;

//  modbusBuffer[0] = 10000; //waitBetweenDevices
//  modbusBuffer[1] = 102; //first device address
//  Serial.begin(9600);
   
  Serial.begin(BAUD_RATE);
  slave.start();
}
void loop() 
{
  int deviceAddressIndex = 0;
  if (modbusBuffer[0] > 0)
  {
    for (int ii = 0; ii < numDevices; ++ii)
    {
      deviceAddressIndex = ii * deviceBufLen + 1;
      if (modbusBuffer[deviceAddressIndex] > 0)
      {
        if (!sendRequest(deviceAddressIndex))
        {
        }
        wait(modbusBuffer[0]);
      }
    }
  }
  checkModbusRequest();

}
boolean sendRequest(int deviceAddressIndex)
{
  boolean waitForData = true;
  boolean timeOut = false;
  unsigned long lastWriteTime = millis();

  radioPacketSend.iaddr = modbusBuffer[deviceAddressIndex];
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
    checkModbusRequest();
    while (rf95.available())
    {
      if (rf95.recv((uint8_t *)&radioPacketRecv, &sizeOfRadioPacketRecv))
      {
        if (radioPacketRecv.iaddr == radioPacketSend.iaddr)
        {
          waitForData = false;
        }
      }
    }
    if ((millis() - lastWriteTime) > TIMEOUTMILLIS)
    {
      timeOut = true;
      radioPacketRecv.iaddr = modbusBuffer[deviceAddressIndex];
      radioPacketRecv.irssi = 0;
      radioPacketRecv.ideltat = 0;
      for (int ii = 0; ii < numData; ++ii)
      {
        radioPacketRecv.idata[ii] = 0;
      }
    }
  }

  modbusBuffer[deviceAddressIndex + 1] = radioPacketRecv.irssi;
  modbusBuffer[deviceAddressIndex + 2] = radioPacketRecv.ideltat;
  for (int ii = 0; ii < numData; ++ii)
  {
    modbusBuffer[deviceAddressIndex + 3 + ii] = radioPacketRecv.idata[ii];
  }
/*
  for (int ii = 0; ii < deviceBufLen; ++ii)
  {
    Serial.print(modbusBuffer[deviceAddressIndex + ii]);
    Serial.print(", ");
  }
  Serial.println(" ");
*/
  return !timeOut;
}
void wait(int milliSecs)
{
  unsigned long beginTime = millis();
  unsigned long now = beginTime;
  while ((now - beginTime) < ((unsigned long) (milliSecs)) )
  {
    now = millis();
    checkModbusRequest();
  }
  return;  
}
void checkModbusRequest()
{
  unsigned long now = millis();
  if ((now - lastModbusPollTime) > MODBUS_POLL_INTERVAL)
  {
    lastModbusPollTime = now;
    slave.poll( modbusBuffer,modbusBufferLength );
  }
  return;
}

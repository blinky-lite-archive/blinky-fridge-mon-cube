#define RF_FREQ  433.200
#define BAUD_RATE  9600
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

union ModbusUnion
{
  struct
  {
    uint16_t interval = 0;
    uint16_t scanAddr[8] = {0,0,0,0,0,0,0,0};
    uint16_t iaddr = 0;
    uint16_t irssi = 0;
    uint16_t ideltat = 0;
    uint16_t idata[11] = {0,0,0,0,0,0,0,0,0,0,0};
  };
  uint16_t modbusBuffer[23];
} mb;
int modbusBufferLength = 23;
int numDevices = 8;
int numDeviceData = 11;

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

//  mb.interval = 10000; //waitBetweenDevices
//  mb.scanAddr[0] = 102; //first device address
//  Serial.begin(9600);
   
  Serial.begin(BAUD_RATE);
  slave.start();
}
void loop() 
{
  int deviceAddressIndex = 0;
  if (mb.interval > 0)
  {
    for (int ii = 0; ii < numDevices; ++ii)
    {
      if (mb.scanAddr[ii] > 0)
      {
        if (!sendRequest(mb.scanAddr[ii]))
        {
        }
        wait(mb.interval);
      }
    }
  }
  checkModbusRequest();

}
boolean sendRequest(uint16_t devAddress)
{
  boolean waitForData = true;
  boolean timeOut = false;
  unsigned long lastWriteTime = millis();

  radioPacketSend.iaddr = devAddress;
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
      radioPacketRecv.iaddr = devAddress;
      radioPacketRecv.irssi = 0;
      radioPacketRecv.ideltat = 0;
      for (int ii = 0; ii < numDeviceData; ++ii)
      {
        radioPacketRecv.idata[ii] = 0;
      }
    }
  }

  mb.iaddr = radioPacketRecv.iaddr;
  mb.irssi = radioPacketRecv.irssi;
  mb.ideltat = radioPacketRecv.ideltat;
  for (int ii = 0; ii < numDeviceData; ++ii)
  {
    mb.idata[ii] = radioPacketRecv.idata[ii];
  }

/*
  for (int ii = 0; ii < 14; ++ii)
  {
    Serial.print(mb.modbusBuffer[9 + ii]);
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
    slave.poll( mb.modbusBuffer,modbusBufferLength );
  }
  return;
}

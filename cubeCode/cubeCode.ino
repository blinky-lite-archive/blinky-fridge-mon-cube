#define BAUD_RATE 115200

struct TransmitData
{
  float avgRms;
  float milliAmps = 0.0;
  float power = 0.0;
  float adcSampleRate = 0;
};
struct ReceiveData
{
  int powerOn = 0;
  float nsamples = 5000.0;
  float avgRmsOffset = 0.015;
  float adcTomA = 55.8;
  int loopDelay = 1000;
  float nfilter = 30.0;
};
const int powerOnPin = 21;
const int powerOnLedPin = 4;
const int currentMonPin = A0;
const float acVolts = 240.0;
float avgAdc = 0.0;
float nsamples = 5000.0;
float nfilter = 30.0;
float nsamplesCnt = 1.0;
float filteredAdc = 0.0;
float nfilterCnt = 1.0;
int oldPowerOn = 0.0;

void setupPins()
{
  pinMode(powerOnPin, OUTPUT);
  pinMode(powerOnLedPin, OUTPUT);
  pinMode(currentMonPin, INPUT);
  digitalWrite(powerOnPin, 0);
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->powerOn = newData->powerOn;
  rData->loopDelay = newData->loopDelay;
  rData->nsamples = newData->nsamples;
  rData->nfilter = newData->nfilter;
  rData->avgRmsOffset = newData->avgRmsOffset;
  rData->adcTomA = newData->adcTomA;
  if ((nsamples != rData->nsamples) || (nfilter != rData->nfilter))
  {
    nsamples = rData->nsamples;
    nfilter = rData->nfilter;
    avgAdc = 512.0;
    filteredAdc = 512.0;
    tData->avgRms = 0.0;
    nsamplesCnt = 1.0;
    nfilterCnt = 1.0;
  }
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  float adcValue;
  float rmsAdc;
  int icnt = 0;
  unsigned long tstart;
  unsigned long tnow;

  digitalWrite(powerOnPin, rData->powerOn);
  digitalWrite(powerOnLedPin, rData->powerOn);
  if (oldPowerOn != rData->powerOn)
  {
    avgAdc = 512.0;
    filteredAdc = 512.0;
    tData->avgRms = 0.0;
    nsamplesCnt = 1.0;
    nfilterCnt = 1.0;
  }
  oldPowerOn = rData->powerOn;

  tstart = millis();
  tnow = tstart;

  while(((int) (tnow - tstart)) < rData->loopDelay)
  { 
    adcValue = (float) analogRead(currentMonPin);
    filteredAdc = filteredAdc + (adcValue - filteredAdc) / nfilterCnt;
    avgAdc = avgAdc + (adcValue - avgAdc) / nsamplesCnt;
    rmsAdc = (filteredAdc - avgAdc);
    rmsAdc = rmsAdc * rmsAdc;
    tData->avgRms = tData->avgRms + (rmsAdc - tData->avgRms) / nsamplesCnt;
     ++icnt;
    if (nfilterCnt < nfilter) nfilterCnt = nfilterCnt + 1.0;
    if (nsamplesCnt < nsamples) nsamplesCnt = nsamplesCnt + 1.0;
    tnow = millis();
  }
  tData->milliAmps = tData->avgRms - rData->avgRmsOffset;
  if (tData->milliAmps < 0.0) tData->milliAmps = 0.0;
  tData->milliAmps = sqrt(tData->milliAmps) * rData->adcTomA;
  tData->power = tData->milliAmps * 0.001 * acVolts;
  tData->adcSampleRate = ((float) icnt) / ((float) rData->loopDelay);
  if (rData->powerOn == 0)
  {
    tData->milliAmps = 0.0;
    tData->power = 0.0;
  }

  return true;
}

const int microLEDPin = 13;
const int commLEDPin = 5;
boolean commLED = true;

struct TXinfo
{
  int cubeInit = 1;
  int newSettingDone = 0;
};
struct RXinfo
{
  int newSetting = 0;
};

struct TX
{
  TXinfo txInfo;
  TransmitData txData;
};
struct RX
{
  RXinfo rxInfo;
  ReceiveData rxData;
};
TX tx;
RX rx;
ReceiveData settingsStorage;

int sizeOfTx = 0;
int sizeOfRx = 0;

void setup()
{
  setupPins();
  pinMode(microLEDPin, OUTPUT);    
  pinMode(commLEDPin, OUTPUT);  
  digitalWrite(commLEDPin, commLED);
  digitalWrite(microLEDPin, commLED);

  sizeOfTx = sizeof(tx);
  sizeOfRx = sizeof(rx);
  Serial1.begin(BAUD_RATE);
  delay(1000);
}
void loop()
{
  boolean goodData = false;
  goodData = processData(&(tx.txData), &settingsStorage);
  if (goodData)
  {
    tx.txInfo.newSettingDone = 0;
    if(Serial1.available() > 0)
    { 
      commLED = !commLED;
      digitalWrite(commLEDPin, commLED);
      Serial1.readBytes((uint8_t*)&rx, sizeOfRx);
      
      if (rx.rxInfo.newSetting > 0)
      {
        processNewSetting(&(tx.txData), &settingsStorage, &(rx.rxData));
        tx.txInfo.newSettingDone = 1;
        tx.txInfo.cubeInit = 0;
      }
    }
    Serial1.write((uint8_t*)&tx, sizeOfTx);
  }
  
}

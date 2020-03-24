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
  float nsamples = 10000.0;
  float avgRmsOffset = 0.0;
  float adcTomA = 45.0;
  int loopDelay = 1000;
};
const int powerOnPin = 21;
const int powerOnLedPin = 4;
const int currentMonPin = A0;
const float acVolts = 240.0;
float avgAdc = 0.0;
float nsamples = 10000.0;
float avgAdcPrev = -1.0;


void setupPins()
{
  pinMode(powerOnPin, OUTPUT);
  pinMode(powerOnLedPin, OUTPUT);
  pinMode(currentMonPin, INPUT);
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->powerOn = newData->powerOn;
  rData->loopDelay = newData->loopDelay;
  rData->nsamples = newData->nsamples;
  rData->avgRmsOffset = newData->avgRmsOffset;
  rData->adcTomA = newData->adcTomA;
  if (nsamples != rData->nsamples)
  {
    nsamples = rData->nsamples;
    avgAdc = 0.0;
    tData->avgRms = 0.0;
    avgAdcPrev = -1.0;
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

  tstart = millis();
  tnow = tstart;

  while(((int) (tnow - tstart)) < rData->loopDelay)
  { 
    adcValue = (float) analogRead(currentMonPin);
    avgAdc = avgAdc + (adcValue - avgAdc) / nsamples;
    if(avgAdcPrev > 0.0)
    {
      rmsAdc = (adcValue - avgAdcPrev);
      rmsAdc = rmsAdc * rmsAdc;
      tData->avgRms = tData->avgRms + (rmsAdc - tData->avgRms) / nsamples;
    }
    ++icnt;
    tnow = millis();
  }
  avgAdcPrev = avgAdc;
  tData->milliAmps = tData->avgRms - rData->avgRmsOffset;
  if (tData->milliAmps < 0.0) tData->milliAmps = 0.0;
  tData->milliAmps = sqrt(tData->milliAmps) * rData->adcTomA;
  tData->power = tData->milliAmps * 0.001 * acVolts;
  tData->adcSampleRate = ((float) icnt) / ((float) rData->loopDelay);

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

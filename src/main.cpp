#include <Arduino.h>
#include "SoftwareSerial.h"

SoftwareSerial usbSerial;
auto &testSerial = Serial;

#define USE_GHST
//#define USE_CRSF
//#define USE_SBUS
////#define USE_SRXL2

bool testRunning = false;
#define NumOfTests 500
uint32_t testCount = 0;
uint16_t ChannelData[16];

uint32_t latencyResult[NumOfTests] = {0};

#ifdef USE_GHST
#include "ghst.h"
GHST ghst(Serial);
#endif

#ifdef USE_CRSF
#include "crsf.h"
CRSF crsf(Serial);
#endif

#ifdef USE_SBUS
#include "SBUS.h"
SBUS sbus(Serial);
bool failSafe;
bool lostFrame;
#endif

#ifdef USE_SRXL2
#include "spm_srxl.h"
#include "spm_srxl_config.h"

void uartInit(uint8_t uartNum, uint32_t baudRate)
{
  Serial.begin(baudRate, SERIAL_8N1, SERIAL_FULL, 1, false);
}

void uartSetBaud(uint8_t uartNum, uint32_t baudRate)
{
  Serial.begin(baudRate, SERIAL_8N1, SERIAL_FULL, 1, false);
}

uint8_t uartReceiveBytes(uint8_t uartNum, uint8_t *pBuffer, uint8_t bufferSize, uint8_t timeout_ms)
{
  Serial.setTimeout(timeout_ms);
  Serial.readBytes(pBuffer, bufferSize);
}

uint8_t uartTransmit(uint8_t uartNum, uint8_t *pBuffer, uint8_t bytesToSend)
{
  Serial.write(pBuffer, bytesToSend);
}

// Forward definitions of app-specific handling of telemetry and channel data -- see examples below
void userProvidedFillSrxlTelemetry(SrxlTelemetryData *pTelemetry)
{
  Serial.println("TLM");
}

void userProvidedReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafeData)
{
  Serial.println("Data");
}

void userProvidedHandleVtxData(SrxlVtxData *pVtxData)
{
  Serial.println("Data");
}

#endif

#define TRIGGER_WAIT_RAND_MIN_MS 200000
#define TRIGGER_WAIR_RAND_MAX_MS 350000
uint32_t TriggerBeginTime;

#define GPIO_OUTPUT_PIN D0

uint32_t BeginTriggerMicros;
uint32_t StopTriggerMicros;

uint32_t lastRCdataMicros;
uint32_t currRCdataMicros;

uint8_t CurrState;

void ICACHE_RAM_ATTR clear_array(uint32_t *array, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
    array[i] = 0;
}

double ICACHE_RAM_ATTR average(uint32_t *array, uint32_t len)
{
  double sum = 0; // sum will be larger than an item, long for safety.
  for (uint32_t i = 0; i < len; i++)
  {
    sum += array[i];
  }
  return ((double)sum / (double)len); // average will be fractional, so float may be appropriate.
}

void ICACHE_RAM_ATTR RCcallback(volatile uint16_t *data)
{
  uint32_t now = micros();
  if (CurrState == 2)
  {
    if (data[2] > 1000)
    {
      digitalWrite(D0, LOW);
      CurrState = 0;
      StopTriggerMicros = now;
      uint32_t result = StopTriggerMicros - BeginTriggerMicros;

      if (testRunning)
      {
        latencyResult[testCount] = result;
        usbSerial.print(testCount);
        usbSerial.print(",");
        usbSerial.print(result);
        usbSerial.print(",");
        usbSerial.println(now - lastRCdataMicros);
      }

      if (testCount >= NumOfTests && testRunning)
      {
        testRunning = false;
        testCount = 0;
        usbSerial.println("===== FINISHED =====");
        usbSerial.print("AVERAGE: ");
        usbSerial.print(average(latencyResult, NumOfTests));
        usbSerial.println(" microSeconds");
        clear_array(latencyResult, sizeof(latencyResult));
      }
      else
      {
        testCount++;
      }
    }
  }
  lastRCdataMicros = now;
}

void inline CRSF_GHST_RC_CALLBACK()
{
#if defined(USE_CRSF)
  RCcallback(crsf.ChannelDataIn);
#elif defined(USE_GHST)
  RCcallback(ghst.ChannelDataIn);
#endif
}

void inline PreTrigger()
{
  TriggerBeginTime = random(TRIGGER_WAIT_RAND_MIN_MS, TRIGGER_WAIR_RAND_MAX_MS) + micros();
  CurrState = 1;
}

void inline DoTrigger()
{
  digitalWrite(D0, HIGH);
  BeginTriggerMicros = micros();
  CurrState = 2;
}

void setup()
{
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW);
  pinMode(0, INPUT_PULLUP);
  CurrState = 0;
  clear_array(latencyResult, NumOfTests);

#if defined(USE_CRSF)
  crsf.RCdataCallback = &CRSF_GHST_RC_CALLBACK;
  crsf.Begin();
  Serial.begin(CRSF_RX_BAUDRATE, SERIAL_8N1, SERIAL_FULL, 1, false);
#elif defined(USE_GHST)
  ghst.RCdataCallback = &CRSF_GHST_RC_CALLBACK;
  ghst.Begin();
  Serial.begin(GHST_RX_BAUDRATE, SERIAL_8N1, SERIAL_FULL, 1, false);
#elif defined(USE_SBUS)
  sbus.begin();
#endif

  Serial.swap();

  usbSerial.begin(19200, SWSERIAL_8N1, 3, 1, false, 256);
  usbSerial.enableIntTx(false);
  usbSerial.println("Softserial Mon Started");
  usbSerial.println("Press Button to Begin Test");
}

#ifdef USE_SBUS
void loop_sbus()
{
  if (sbus.read(&ChannelData[0], &failSafe, &lostFrame))
  {
    RCcallback(ChannelData);
  }
}
#endif

#ifdef USE_SRXL2
void loop_srxl2()
{
  while (Serial.available())
  {
    // Try to receive UART bytes, or timeout after 5 ms
    uint8_t bytesReceived = Serial.read();
    Serial.println(bytesReceived);
    if (bytesReceived)
    {
      rxBufferIndex += bytesReceived;
      if (rxBufferIndex < 5)
        continue;

      if (rxBuffer[0] == SPEKTRUM_SRXL_ID)
      {
        uint8_t packetLength = rxBuffer[2];
        if (rxBufferIndex > packetLength)
        {
          // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
          if (srxlParsePacket(0, rxBuffer, packetLength))
          {
            // Move any remaining bytes to beginning of buffer (usually 0)
            rxBufferIndex -= packetLength;
            memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
          }
          else
          {
            rxBufferIndex = 0;
          }
        }
      }
    }
    else
    {
      // Tell SRXL state machine that 5 more milliseconds have passed since packet received
      srxlRun(0, 5);
      rxBufferIndex = 0;
    }
    // Check a bind button, and if pressed enter bind mode
    // if (bindButtonPressed)
    // {
    //   srxlEnterBind(DSMX_11MS);
    // }
  }
}
#endif

void loop()
{
  if (CurrState == 0)
  {
    PreTrigger();
  }
  else if (CurrState == 1)
  {
    if (micros() > TriggerBeginTime)
    {
      DoTrigger();
    }
  }

#if defined(USE_CRSF)
  crsf.handleUARTin();
#elif defined(USE_GHST)
  ghst.handleUARTin();
#elif defined(USE_SRXL2)
  loop_srxl2();
#elif defined(USE_SBUS)
  loop_sbus();
#endif

  if (digitalRead(0) == 0)
  {
    {
      usbSerial.println("Begin Test");
      usbSerial.print("Test will run:");
      usbSerial.print(NumOfTests);
      usbSerial.println(" times");
      testRunning = true;
      testCount = 0;
      clear_array(latencyResult, NumOfTests);
      usbSerial.println("===== BEGIN =====");
    }
  }
}
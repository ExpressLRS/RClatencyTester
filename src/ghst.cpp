#include <Arduino.h>
#include "ghst.h"
#include "HardwareSerial.h"

#include "SoftwareSerial.h"
extern SoftwareSerial usbSerial;

volatile bool GHST::frameActive = false; //since we get a copy of the serial data use this flag to know when to ignore it

void inline GHST::nullCallback(void){};

void (*GHST::RCdataCallback)() = &nullCallback; // null placeholder callback

void (*GHST::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*GHST::connected)() = &nullCallback;    // called when CRSF stream is regained

/// UART Handling ///
bool GHST::CRSFstate = false;

volatile uint8_t GHST::SerialInPacketLen = 0; // length of the CRSF packet as measured
volatile uint8_t GHST::SerialInPacketPtr = 0; // index where we are reading/writing

volatile uint8_t GHST::SerialInBuffer[64] = {0};
volatile uint16_t GHST::ChannelDataIn[16] = {0};
volatile uint16_t GHST::ChannelDataInPrev[16] = {0};

void GHST::Begin()
{
    usbSerial.println("About to start GHST task...");
    //instance->_dev->begin(420000);
}

void ICACHE_RAM_ATTR GHST::handleUARTin() //RTOS task to read and write CRSF packets to the serial port
{
    if (this->_dev->available())
    {
        char inChar = this->_dev->read();

        if (frameActive == false)
        {
            // stage 1 wait for sync byte //
            if (inChar == GHST_ADDR_FC) // we got sync, reset write pointer
            {
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                frameActive = true;
                SerialInBuffer[SerialInPacketPtr] = inChar;
                SerialInPacketPtr++;
            }
        }
        else // frame is active so we do the processing
        {
            // first if things have gone wrong //
            if (SerialInPacketPtr > GHST_MAX_PACKET_LEN - 1) // we reached the maximum allowable packet length, so start again because shit fucked up hey.
            {
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                frameActive = false;
                return;
            }

            // special case where we save the expected pkt len to buffer //
            if (SerialInPacketPtr == 1)
            {
                if (inChar <= GHST_MAX_PACKET_LEN)
                {
                    SerialInPacketLen = inChar;
                }
                else
                {
                    SerialInPacketPtr = 0;
                    SerialInPacketLen = 0;
                    frameActive = false;
                    return;
                }
            }

            SerialInBuffer[SerialInPacketPtr] = inChar;
            SerialInPacketPtr++;

            if (SerialInPacketPtr == SerialInPacketLen + 2) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
            {
                char CalculatedCRC = CalcCRC((uint8_t *)SerialInBuffer + 2, SerialInPacketPtr - 3);

                if (CalculatedCRC == inChar)
                {
                    ProcessPacket();
                }
                else
                {
                    frameActive = false;
                    SerialInPacketPtr = 0;
                    SerialInPacketLen = 0;
                    while (this->_dev->available())
                    {
                        this->_dev->read(); // empty any remaining garbled data
                    }
                }
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                frameActive = false;
            }
        }
    }
}

bool ICACHE_RAM_ATTR GHST::ProcessPacket()
{
    if (CRSFstate == false)
    {
        CRSFstate = true;
        usbSerial.println("GHST UART Connected");
        connected();
    }

    //Serial.println(SerialInBuffer[2], HEX);

    if (SerialInBuffer[2] >= GHST_UL_RC_CHANS_HS4_FIRST && SerialInBuffer[2] <= GHST_UL_RC_CHANS_HS4_LAST)
    {
        GetChannelDataIn();
        (RCdataCallback)(); // run new RC data callback
        return true;
    }
    return false;
}

void ICACHE_RAM_ATTR GHST::GetChannelDataIn() // data is packed as 11 bits per channel
{
#define SERIAL_PACKET_OFFSET 3
    const ghstPayloadPulses_t *const rcChannels = (ghstPayloadPulses_t *)&SerialInBuffer[SERIAL_PACKET_OFFSET];
    ChannelDataIn[0] = rcChannels->ch1to4.ch1 >> 1;
    ChannelDataIn[1] = rcChannels->ch1to4.ch2 >> 1;
    ChannelDataIn[2] = rcChannels->ch1to4.ch3 >> 1;
    ChannelDataIn[3] = rcChannels->ch1to4.ch4 >> 1;
}

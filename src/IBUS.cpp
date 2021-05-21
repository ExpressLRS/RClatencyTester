/**
 * Simple interface to the Fly Sky IBus RC system.
 * https://gitlab.com/timwilkinson/FlySkyIBus
 */

#include <Arduino.h>
#include "IBUS.h"
#include <SoftwareSerial.h>

IBUS::IBUS(HardwareSerial& serial)
{
    serial.begin(115200);
    begin((Stream&)serial);
}

void IBUS::begin(Stream& stream)
{
    this->stream = &stream;
    this->state = GET_LENGTH;
    this->last = 0;
    this->ptr = 0;
    this->len = 0;
    this->chksum = 0;
    this->lchksum = 0;
}

void IBUS::loop(void)
{
    extern SoftwareSerial usbSerial;
    while (stream->available() > 0)
    {
        // uint32_t now = millis();
        // if (now - last >= PROTOCOL_TIMEGAP)
        // {
        //     state = GET_LENGTH;
        // }
        // last = now;

        uint8_t v = stream->read();
        switch (state)
        {
        case GET_LENGTH:
            if (v <= PROTOCOL_LENGTH)
            {
                ptr = 0;
                len = v - PROTOCOL_OVERHEAD;
                chksum = 0xFFFF - v;
                state = GET_DATA;
            }
            else
            {
                state = GET_LENGTH;
            }
            break;

        case GET_DATA:
            buffer[ptr++] = v;
            chksum -= v;
            if (ptr == len)
            {
                state = GET_CHKSUML;
            }
            break;

        case GET_CHKSUML:
            lchksum = v;
            state = GET_CHKSUMH;
            break;

        case GET_CHKSUMH:
            // Validate checksum
            if (chksum == (v << 8) + lchksum)
            {
                // Execute command - we only know command 0x40
                switch (buffer[0])
                {
                    case PROTOCOL_COMMAND40:
                    // Valid - extract channel data
                    for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
                    {
                        ChannelDataIn[i / 2] = buffer[i] | (buffer[i + 1] << 8);
                    }
                    //usbSerial.print(ChannelDataIn[2], DEC);
                    if (RCdataCallback)
                        RCdataCallback();
                    break;

                    default:
                    break;
                }
            }
            state = GET_LENGTH;
            break;

        case DISCARD:
        default:
            break;
        }
    }
}


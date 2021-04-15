#ifndef H_GHST
#define H_GHST

#include <Arduino.h>
#include "HardwareSerial.h"

#define GHST_RX_BAUDRATE                420000
#define GHST_MAX_PACKET_LEN             64

#define GHST_TX_BAUDRATE_FAST           400000
#define GHST_TX_BAUDRATE_SLOW           115200
#define GHST_BYTE_TIME_FAST_US          ((1000000/GHST_TX_BAUDRATE_FAST)*10)      // 10 bit words (8 data, 1 start, 1 stop)
#define GHST_BYTE_TIME_SLOW_US          ((1000000/GHST_TX_BAUDRATE_SLOW)*10)
#define GHST_UART_WORDLENGTH            UART_WORDLENGTH_8B

typedef enum {
    GHST_ADDR_RADIO             = 0x80,
    GHST_ADDR_TX_MODULE_SYM     = 0x81,     // symmetrical, 400k pulses, 400k telemetry
    GHST_ADDR_TX_MODULE_ASYM    = 0x88,     // asymmetrical, 400k pulses, 115k telemetry
    GHST_ADDR_FC                = 0x82,
    GHST_ADDR_GOGGLES           = 0x83,
    GHST_ADDR_QUANTUM_TEE1      = 0x84,     // phase 2
    GHST_ADDR_QUANTUM_TEE2      = 0x85,
    GHST_ADDR_QUANTUM_GW1       = 0x86,
    GHST_ADDR_5G_CLK            = 0x87,     // phase 3
    GHST_ADDR_RX                = 0x89
} ghstAddr_e;

typedef enum {
    // frame types 0x10 - 0x1f always include 4 primary channels, plus either 4 aux channels,
    // or other type-specific data. Expect types 0x14-0x1f to be added in the future, and even though
    // not explicitly supported, the 4 primary channels should always be extracted.
    GHST_UL_RC_CHANS_HS4_FIRST  = 0x10,     // First frame type including 4 primary channels
    GHST_UL_RC_CHANS_HS4_5TO8   = 0x10,     // primary 4 channel, plus CH5-8
    GHST_UL_RC_CHANS_HS4_9TO12  = 0x11,     // primary 4 channel, plus CH9-12
    GHST_UL_RC_CHANS_HS4_13TO16 = 0x12,     // primary 4 channel, plus CH13-16
    GHST_UL_RC_CHANS_HS4_RSSI   = 0x13,     // primary 4 channel, plus RSSI, LQ, RF Mode, and Tx Power
    GHST_UL_RC_CHANS_HS4_LAST   = 0x1f      // Last frame type including 4 primary channels
} ghstUl_e;

#define GHST_UL_RC_CHANS_SIZE       12      // 1 (type) + 10 (data) + 1 (crc)

typedef enum {
    GHST_DL_OPENTX_SYNC         = 0x20,
    GHST_DL_LINK_STAT           = 0x21,
    GHST_DL_VTX_STAT            = 0x22,
    GHST_DL_PACK_STAT           = 0x23,     // Battery (Pack) Status
} ghstDl_e;

#define GHST_RC_CTR_VAL_12BIT       0x7C0   // servo center for 12 bit values (0x3e0 << 1)
#define GHST_RC_CTR_VAL_8BIT        0x7C    // servo center for 8 bit values

#define GHST_FRAME_SIZE             14      // including addr, type, len, crc, and payload

#define GHST_PAYLOAD_SIZE_MAX           14

#define GHST_FRAME_SIZE_MAX             24

typedef struct ghstFrameDef_s {
    uint8_t addr;
    uint8_t len;
    uint8_t type;
    uint8_t payload[GHST_PAYLOAD_SIZE_MAX + 1];         // CRC adds 1
} ghstFrameDef_t;

typedef union ghstFrame_u {
    uint8_t bytes[GHST_FRAME_SIZE];
    ghstFrameDef_t frame;
} ghstFrame_t;


/* Pulses payload (channel data), for 4x 12-bit channels */
typedef struct ghstPayloadServo4_s {
    // 48 bits, or 6 bytes
    unsigned int ch1: 12;
    unsigned int ch2: 12;
    unsigned int ch3: 12;
    unsigned int ch4: 12;
} __attribute__ ((__packed__)) ghstPayloadServo4_t;

/* Pulses payload (channel data). Includes 4x high speed control channels, plus 4 channels from CH5-CH12 */
typedef struct ghstPayloadPulses_s {
    // 80 bits, or 10 bytes
    ghstPayloadServo4_t ch1to4;

    unsigned int cha: 8;
    unsigned int chb: 8;
    unsigned int chc: 8;
    unsigned int chd: 8;
} __attribute__ ((__packed__)) ghstPayloadPulses_t;

/* Pulses payload (channel data), with RSSI/LQ, and other related data */
typedef struct ghstPayloadPulsesRssi_s {
    // 80 bits, or 10 bytes
   ghstPayloadServo4_t ch1to4;

    unsigned int lq: 8;                 // 0-100
    unsigned int rssi: 8;               // 0 - 128 sign inverted, dBm
    unsigned int rfProtocol: 8;
    signed int txPwrdBm: 8;             // tx power in dBm, use lookup table to map to published mW values
} __attribute__ ((__packed__)) ghstPayloadPulsesRssi_t;

/* CRC8 implementation with polynom = x​7​+ x​6​+ x​4​+ x​2​+ x​0 ​(0xD5) */
static const unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

static inline uint8_t ICACHE_RAM_ATTR CalcCRC(volatile uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

static inline uint8_t ICACHE_RAM_ATTR CalcCRC(uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

class GHST
{

public:
    GHST(Stream *dev) : _dev(dev)
    {
    }

    GHST(Stream &dev) : _dev(&dev) {}

    static HardwareSerial Port;

    static volatile uint16_t ChannelDataIn[16];
    static volatile uint16_t ChannelDataInPrev[16]; // Contains the previous RC channel data RX side only

    static void (*RCdataCallback)(); //function pointer for new RC data callback

    static void (*disconnected)();
    static void (*connected)();

    /////Variables/////
    static void Begin(); //setup timers etc

    /// UART Handling ///

    static bool CRSFstate;
    static bool CRSFstatePrev;
    static uint32_t UARTcurrentBaud;
    static uint32_t UARTrequestedBaud;

    static uint32_t GoodPktsCount;
    static uint32_t BadPktsCount;

    void ICACHE_RAM_ATTR initUART();
    void ICACHE_RAM_ATTR UARTwdt();
    void ICACHE_RAM_ATTR handleUARTin();

    static bool ICACHE_RAM_ATTR ProcessPacket();
    static void ICACHE_RAM_ATTR GetChannelDataIn();
    static void ICACHE_RAM_ATTR updateSwitchValues();
    static void inline nullCallback(void);

private:
    Stream *_dev;

    static volatile uint8_t SerialInPacketLen; // length of the CRSF packet as measured
    static volatile uint8_t SerialInPacketPtr; // index where we are reading/writing
    static volatile uint8_t SerialInBuffer[GHST_MAX_PACKET_LEN];

    static volatile bool frameActive; //since we get a copy of the serial data use this flag to know when to ignore it
};

#endif
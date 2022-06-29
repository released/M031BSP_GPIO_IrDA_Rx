/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "IRProtocol.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/
#ifndef bool
typedef unsigned char bool;
#endif

#ifndef true
#define true                    (1)
#endif

#ifndef false
#define false                   (0)
#endif

// #define ENABLE_UART_DBG

// see: https://www.sbprojects.net/knowledge/ir/nec.php
// for Apple see https://en.wikipedia.org/wiki/Apple_Remote
// ONKYO like NEC but 16 independent command bits
// LSB first, 1 start bit + 16 bit address (or 8 bit address and 8 bit inverted address) + 8 bit command + 8 bit inverted command + 1 stop bit.
//
#define NEC_ADDRESS_BITS        16 // 16 bit address or 8 bit address and 8 bit inverted address
#define NEC_COMMAND_BITS        16 // Command and inverted command

#define NEC_BITS                (NEC_ADDRESS_BITS + NEC_COMMAND_BITS)
#define NEC_UNIT                560             // 21.28 periods of 38 kHz, 11.2 ticks TICKS_LOW = 8.358 TICKS_HIGH = 15.0

#define NEC_HEADER_MARK         (16 * NEC_UNIT) // 9000 / 180
#define NEC_HEADER_SPACE        (8 * NEC_UNIT)  // 4500 / 90

#define NEC_BIT_MARK            NEC_UNIT
#define NEC_ONE_SPACE           (3 * NEC_UNIT)  // 1690 / 33.8  TICKS_LOW = 25.07 TICKS_HIGH = 45.0
#define NEC_ZERO_SPACE          NEC_UNIT

#define NEC_REPEAT_HEADER_SPACE (4 * NEC_UNIT)  // 2250

#define NEC_AVERAGE_DURATION    62000 // NEC_HEADER_MARK + NEC_HEADER_SPACE + 32 * 2,5 * NEC_UNIT + NEC_UNIT // 2.5 because we assume more zeros than ones
#define NEC_REPEAT_DURATION     (NEC_HEADER_MARK  + NEC_REPEAT_HEADER_SPACE + NEC_BIT_MARK) // 12 ms
#define NEC_REPEAT_PERIOD       110000 // Commands are repeated every 110 ms (measured from start to start) for as long as the key on the remote control is held down.
#define NEC_REPEAT_SPACE        (NEC_REPEAT_PERIOD - NEC_AVERAGE_DURATION) // 48 ms

#define APPLE_ADDRESS           0x87EE

#define IRDATA_FLAGS_EMPTY              0x00
#define IRDATA_FLAGS_IS_REPEAT          0x01
#define IRDATA_FLAGS_IS_AUTO_REPEAT     0x02
#define IRDATA_FLAGS_PARITY_FAILED      0x04 ///< the current (autorepeat) frame violated parity check
#define IRDATA_TOGGLE_BIT_MASK          0x08
#define IRDATA_FLAGS_EXTRA_INFO         0x10 ///< there is unexpected extra info not contained in address and data (e.g. Kaseikyo unknown vendor ID)
#define IRDATA_FLAGS_WAS_OVERFLOW       0x40 ///< irparams.rawlen is 0 in this case to avoid endless OverflowFlag
#define IRDATA_FLAGS_IS_LSB_FIRST       0x00
#define IRDATA_FLAGS_IS_MSB_FIRST       0x80 ///< Just for info. Value is simply determined by the protocol

#define MARK_EXCESS_MICROS              10 // Adapt it to your IR receiver module. See also IRremote.h.

#define RAW_BUFFER_LENGTH               100 

#define MARK   1
#define SPACE  0

#define IR_REC_STATE_IDLE               0
#define IR_REC_STATE_MARK               1
#define IR_REC_STATE_SPACE              2
#define IR_REC_STATE_STOP               3 // set to IR_REC_STATE_IDLE only by resume()

#if ! defined(MICROS_PER_TICK)
#define MICROS_PER_TICK                 50
#endif

#if !defined(RECORD_GAP_MICROS)
// To change this value, you simply can add a line #define "RECORD_GAP_MICROS <My_new_value>" in your ino file before the line "#include <IRremote.hpp>"
#define RECORD_GAP_MICROS               5000 // FREDRICH28AC / LG2 header space is 9700, NEC header space is 4500
#endif

/** Minimum gap between IR transmissions, in MICROS_PER_TICK */
#define RECORD_GAP_TICKS        (RECORD_GAP_MICROS / MICROS_PER_TICK) // 221 for 1100

/*
 * Activate this line if your receiver has an external output driver transistor / "inverted" output
 */
// #define IR_INPUT_IS_ACTIVE_HIGH
#if defined(IR_INPUT_IS_ACTIVE_HIGH)
// IR detector output is active high
#define INPUT_MARK   1 ///< Sensor output for a mark ("flash")
#else
// IR detector output is active low
#define INPUT_MARK   0 ///< Sensor output for a mark ("flash")
#endif

/*
 * Pulse parms are ((X*50)-MARK_EXCESS_MICROS) for the Mark and ((X*50)+MARK_EXCESS_MICROS) for the Space.
 * First MARK is the one after the long gap
 * Pulse parameters in microseconds
 */
#if !defined(TOLERANCE_FOR_DECODERS_MARK_OR_SPACE_MATCHING)
#define TOLERANCE_FOR_DECODERS_MARK_OR_SPACE_MATCHING    25 // Relative tolerance (in percent) for matchTicks(), matchMark() and matchSpace() functions used for protocol decoding.
#endif

/** Lower tolerance for comparison of measured data */
//#define LTOL            (1.0 - (TOLERANCE/100.))
#define LTOL            (100 - TOLERANCE_FOR_DECODERS_MARK_OR_SPACE_MATCHING)
/** Upper tolerance for comparison of measured data */
//#define UTOL            (1.0 + (TOLERANCE/100.))
#define UTOL            (100 + TOLERANCE_FOR_DECODERS_MARK_OR_SPACE_MATCHING)

//#define TICKS_LOW(us)   ((int)(((us)*LTOL/MICROS_PER_TICK)))
//#define TICKS_HIGH(us)  ((int)(((us)*UTOL/MICROS_PER_TICK + 1)))
#if MICROS_PER_TICK == 50 && TOLERANCE_FOR_DECODERS_MARK_OR_SPACE_MATCHING == 25           // Defaults
#define TICKS_LOW(us)   ((us)/67 )     // (us) / ((MICROS_PER_TICK:50 / LTOL:75 ) * 100)
#define TICKS_HIGH(us)  (((us)/40) + 1)  // (us) / ((MICROS_PER_TICK:50 / UTOL:125) * 100) + 1
#else
#define TICKS_LOW(us)   ((unsigned int ) ((long) (us) * LTOL / (MICROS_PER_TICK * 100) ))
#define TICKS_HIGH(us)  ((unsigned int ) ((long) (us) * UTOL / (MICROS_PER_TICK * 100) + 1))
#endif


/**
 * This struct contains the data and control used for receiver static functions and the ISR (interrupt service routine)
 * Only StateForISR needs to be volatile. All the other fields are not written by ISR after data available and before start/resume.
 */
typedef struct{
    // The fields are ordered to reduce memory over caused by struct-padding
    volatile uint8_t StateForISR;   ///< State Machine state
    uint8_t IRReceivePin;           ///< Pin connected to IR data from detector

    uint16_t TickCounterForISR;     ///< Counts 50uS ticks. The value is copied into the rawbuf array on every transition.

    bool OverflowFlag;              ///< Raw buffer OverflowFlag occurred
#if RAW_BUFFER_LENGTH <= 254        // saves around 75 bytes program memory and speeds up ISR
    uint8_t rawlen;                 ///< counter of entries in rawbuf
#else
    uint16_t rawlen;            ///< counter of entries in rawbuf
#endif
    unsigned int rawbuf[RAW_BUFFER_LENGTH]; ///< raw data / tick counts per mark/space, first entry is the length of the gap between previous and current command
}irparams_struct;

/**
 * Data structure for the user application, available as decodedIRData.
 * Filled by decoders and read by print functions or user application.
 */
typedef struct {
    decode_type_t protocol;  ///< UNKNOWN, NEC, SONY, RC5, ...
    uint16_t address;        ///< Decoded address
    uint16_t command;        ///< Decoded command
    uint16_t extra;          ///< Used by MagiQuest and for Kaseikyo unknown vendor ID. Ticks used for decoding Distance protocol.
    uint16_t numberOfBits; ///< Number of bits received for data (address + command + parity) - to determine protocol length if different length are possible.
    uint8_t flags;               ///< See IRDATA_FLAGS_* definitions above
    uint32_t decodedRawData;     ///< Up to 32 bit decoded raw data, used for sendRaw functions.
    irparams_struct *rawDataPtr; ///< Pointer of the raw timing data to be decoded. Mainly the data buffer filled by receiving ISR.
}IRData;


typedef struct {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } UByte;
    struct {
        int8_t LowByte;
        int8_t HighByte;
    } Byte;
    uint8_t UBytes[2];
    int8_t Bytes[2];
    uint16_t UWord;
    int16_t Word;
    uint8_t *BytePointer;
}WordUnion;

typedef struct {
    struct {
        uint8_t LowByte;
        uint8_t MidLowByte;
        uint8_t MidHighByte;
        uint8_t HighByte;
    } UByte;
    struct {
        int8_t LowByte;
        int8_t MidLowByte;
        int8_t MidHighByte;
        int8_t HighByte;
    } Byte;
    struct {
        uint8_t LowByte;
        WordUnion MidWord;
        uint8_t HighByte;
    } ByteWord;
    struct {
        int16_t LowWord;
        int16_t HighWord;
    } Word;
    struct {
        WordUnion LowWord;
        WordUnion HighWord;
    } WordUnion;
    struct {
        uint16_t LowWord;
        uint16_t HighWord;
    } UWord;
    uint8_t UBytes[4]; // seems to have the same code size as using struct UByte
    int8_t Bytes[4];
    uint16_t UWords[2];
    int16_t Words[2];
    uint32_t ULong;
    int32_t Long;
}LongUnion;


/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void IRReceive_Polling(void);
void IRReceive_irq(void);
void IRReceive_Init(void);


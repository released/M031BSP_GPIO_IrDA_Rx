/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "IRReceive.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

irparams_struct irparams;
IRData decodedIRData;

decode_type_t lastDecodedProtocol;
uint32_t lastDecodedAddress;
uint32_t lastDecodedCommand;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

/**
 * Is internally called by decode before calling decoders.
 * Must be used to setup data, if you call decoders manually.
 */
void initDecodedIRData(void) {

    if (irparams.OverflowFlag) {
        // Copy overflow flag to decodedIRData.flags and reset it
        irparams.OverflowFlag = false;
        irparams.rawlen = 0; // otherwise we have OverflowFlag again at next ISR call
        decodedIRData.flags = IRDATA_FLAGS_WAS_OVERFLOW;
        // IR_DEBUG_PRINTLN(F("Overflow happened"));
        printf("Overflow happened\r\n");

    } else {
        decodedIRData.flags = IRDATA_FLAGS_EMPTY;
        // save last protocol, command and address for repeat handling (where the are copied back :-))
        lastDecodedProtocol = decodedIRData.protocol; // repeat patterns can be equal between protocols (e.g. NEC and LG), so we must keep the original one
        lastDecodedCommand = decodedIRData.command;
        lastDecodedAddress = decodedIRData.address;

    }
    decodedIRData.protocol = UNKNOWN;
    decodedIRData.command = 0;
    decodedIRData.address = 0;
    decodedIRData.decodedRawData = 0;
    decodedIRData.numberOfBits = 0;
}


/**
 * Compensate for spaces shortened by demodulator hardware
 */
bool matchSpace(unsigned int aMeasuredTicks, unsigned int aMatchValueMicros) {
    bool passed = false;
#if defined(TRACE)
    Serial.print(F("Testing space (actual vs desired): "));
    Serial.print(aMeasuredTicks * MICROS_PER_TICK, DEC);
    Serial.print(F("us vs "));
    Serial.print(aMatchValueMicros, DEC);
    Serial.print(F("us: "));
    Serial.print(TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS) * MICROS_PER_TICK, DEC);
    Serial.print(F(" <= "));
    Serial.print(aMeasuredTicks * MICROS_PER_TICK, DEC);
    Serial.print(F(" <= "));
    Serial.print(TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS) * MICROS_PER_TICK, DEC);
#endif

    #if defined (ENABLE_UART_DBG)   // debug
    printf("\r\nTesting space (actual vs desired): \r\n");
    printf("%dus vs %dus: ",aMeasuredTicks * MICROS_PER_TICK,aMatchValueMicros);
    printf("%d <= %d <= %d\r\n",
        TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS) * MICROS_PER_TICK,
        aMeasuredTicks * MICROS_PER_TICK,
        TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS) * MICROS_PER_TICK);
    #endif

    // compensate for spaces shortened by demodulator hardware
    passed = ((aMeasuredTicks >= TICKS_LOW(aMatchValueMicros - MARK_EXCESS_MICROS))
            && (aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros - MARK_EXCESS_MICROS)));
#if defined(TRACE)
    if (passed) {
        Serial.println(F("?; passed"));
    } else {
        Serial.println(F("?; FAILED"));
    }
#endif

    #if defined (ENABLE_UART_DBG)   // debug
    printf("matchSpace passed : %d\r\n" , passed);
    #endif

    return passed;
}


/**
 * Compensate for marks exceeded by demodulator hardware
 */
bool matchMark(unsigned int aMeasuredTicks, unsigned int aMatchValueMicros) {
    bool passed = false;
#if defined(TRACE)
    Serial.print(F("Testing mark (actual vs desired): "));
    Serial.print(aMeasuredTicks * MICROS_PER_TICK, DEC);
    Serial.print(F("us vs "));
    Serial.print(aMatchValueMicros, DEC);
    Serial.print(F("us: "));
    Serial.print(TICKS_LOW(aMatchValueMicros + MARK_EXCESS_MICROS) * MICROS_PER_TICK, DEC);
    Serial.print(F(" <= "));
    Serial.print(aMeasuredTicks * MICROS_PER_TICK, DEC);
    Serial.print(F(" <= "));
    Serial.print(TICKS_HIGH(aMatchValueMicros + MARK_EXCESS_MICROS) * MICROS_PER_TICK, DEC);
#endif

    #if defined (ENABLE_UART_DBG)   // debug
    printf("\r\nTesting mark (actual vs desired): \r\n");
    printf("%dus vs %dus\r\n",aMeasuredTicks * MICROS_PER_TICK,aMatchValueMicros);
    printf("%d <= %d <= %d\r\n",
        TICKS_LOW(aMatchValueMicros + MARK_EXCESS_MICROS) * MICROS_PER_TICK,
        aMeasuredTicks * MICROS_PER_TICK,
        TICKS_HIGH(aMatchValueMicros + MARK_EXCESS_MICROS) * MICROS_PER_TICK);   
    #endif

    // compensate for marks exceeded by demodulator hardware
    passed = ((aMeasuredTicks >= TICKS_LOW(aMatchValueMicros + MARK_EXCESS_MICROS))
            && (aMeasuredTicks <= TICKS_HIGH(aMatchValueMicros + MARK_EXCESS_MICROS)));
#if defined(TRACE)
    if (passed) {
        Serial.println(F("?; passed"));
    } else {
        Serial.println(F("?; FAILED"));
    }
#endif

    #if defined (ENABLE_UART_DBG)    // debug
    printf("matchMark passed : %d\r\n" , passed);
    #endif

    return passed;
}

bool decodePulseDistanceData(uint8_t aNumberOfBits, uint8_t aStartOffset, unsigned int aBitMarkMicros,
        unsigned int aOneSpaceMicros, unsigned int aZeroSpaceMicros, bool aMSBfirst) {

    unsigned int *tRawBufPointer = &decodedIRData.rawDataPtr->rawbuf[aStartOffset];
    uint32_t tDecodedData = 0;
    uint8_t i = 0;       
    uint32_t tMask = 1UL; 

    if (aMSBfirst) {
        for (i = 0; i < aNumberOfBits; i++) {
            // Check for constant length mark
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) {
                // IR_DEBUG_PRINT(F("Mark="));
                // IR_DEBUG_PRINT(*tRawBufPointer * MICROS_PER_TICK);
                // IR_DEBUG_PRINT(F(" is not "));
                // IR_DEBUG_PRINT(aBitMarkMicros);
                // IR_DEBUG_PRINT(' ');
                printf("Mark=0x%4X, is not 0x%4X\r\n" , *tRawBufPointer * MICROS_PER_TICK,aBitMarkMicros);
                return false;
            }
            tRawBufPointer++;

            // Check for variable length space indicating a 0 or 1
            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) {
                tDecodedData = (tDecodedData << 1) | 1;
                // IR_TRACE_PRINT('1');
                #if defined (ENABLE_UART_DBG)   // debug
                printf("1");
                #endif
            } else if (matchSpace(*tRawBufPointer, aZeroSpaceMicros)) {
                tDecodedData = (tDecodedData << 1) | 0;
                // IR_TRACE_PRINT('0');
                #if defined (ENABLE_UART_DBG)   // debug                
                printf("0");
                #endif
            } else {
                // IR_DEBUG_PRINT(F("Space="));
                // IR_DEBUG_PRINT(*tRawBufPointer * MICROS_PER_TICK);
                // IR_DEBUG_PRINT(F(" is not "));
                // IR_DEBUG_PRINT(aOneSpaceMicros);
                // IR_DEBUG_PRINT(F(" or "));
                // IR_DEBUG_PRINT(aZeroSpaceMicros);
                // IR_DEBUG_PRINT(' ');
                printf("Space=0x%4X, is not 0x%4X or 0x%4X\r\n",*tRawBufPointer * MICROS_PER_TICK,aOneSpaceMicros,aZeroSpaceMicros);
                return false;
            }
            tRawBufPointer++;
        }
        // IR_TRACE_PRINTLN(F(""));
        #if defined (ENABLE_UART_DBG)   // debug           
        printf("\r\n");
        #endif

    } else {
        for (tMask = 1UL; aNumberOfBits > 0; tMask <<= 1, aNumberOfBits--) {
            // Check for constant length mark
            if (!matchMark(*tRawBufPointer, aBitMarkMicros)) {
                // IR_DEBUG_PRINT(F("Mark="));
                // IR_DEBUG_PRINT(*tRawBufPointer * MICROS_PER_TICK);
                // IR_DEBUG_PRINT(F(" is not "));
                // IR_DEBUG_PRINT(aBitMarkMicros);
                // IR_DEBUG_PRINT(' ');
                printf("Mark=0x%4X, is not 0x%4X\r\n" , *tRawBufPointer * MICROS_PER_TICK,aBitMarkMicros);                
                return false;
            }
            tRawBufPointer++;

            // Check for variable length space indicating a 0 or 1
            if (matchSpace(*tRawBufPointer, aOneSpaceMicros)) {
                tDecodedData |= tMask; // set the bit
                // IR_TRACE_PRINT('1');
                #if defined (ENABLE_UART_DBG)   // debug   
                printf("1");
                #endif
            } else if (matchSpace(*tRawBufPointer, aZeroSpaceMicros)) {
                // do not set the bit
                // IR_TRACE_PRINT('0');
                #if defined (ENABLE_UART_DBG)   // debug   
                printf("0");
                #endif
            } else {
                // IR_DEBUG_PRINT(F("Space="));
                // IR_DEBUG_PRINT(*tRawBufPointer * MICROS_PER_TICK);
                // IR_DEBUG_PRINT(F(" is not "));
                // IR_DEBUG_PRINT(aOneSpaceMicros);
                // IR_DEBUG_PRINT(F(" or "));
                // IR_DEBUG_PRINT(aZeroSpaceMicros);
                // IR_DEBUG_PRINT(' ');
                printf("Space=0x%4X, is not 0x%4X or 0x%4X\r\n",*tRawBufPointer * MICROS_PER_TICK,aOneSpaceMicros,aZeroSpaceMicros);
                return false;
            }
            tRawBufPointer++;
        }
        // IR_TRACE_PRINTLN(F(""));
        #if defined (ENABLE_UART_DBG)   // debug           
        printf("\r\n");
        #endif
    }
    decodedIRData.decodedRawData = tDecodedData;
    return true;
}

bool decodeNEC(void) {
    LongUnion tValue;
    // Check we have the right amount of data (68). The +4 is for initial gap, start bit mark and space + stop bit mark.
    if (decodedIRData.rawDataPtr->rawlen != ((2 * NEC_BITS) + 4) && (decodedIRData.rawDataPtr->rawlen != 4)) {
        // IR_DEBUG_PRINT(F("NEC: "));
        // IR_DEBUG_PRINT(F("Data length="));
        // IR_DEBUG_PRINT(decodedIRData.rawDataPtr->rawlen);
        // IR_DEBUG_PRINTLN(F(" is not 68 or 4"));
        printf("NEC: Data length= %2d ,  is not 68 or 4\r\n",decodedIRData.rawDataPtr->rawlen);

        return false;
    }

    // Check header "mark" this must be done for repeat and data
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[1], NEC_HEADER_MARK)) {
        printf("Check header mark this must be done for repeat and data [fail]\r\n");
        return false;
    }

    // Check for repeat - here we have another header space length
    if (decodedIRData.rawDataPtr->rawlen == 4) {
        if (matchSpace(decodedIRData.rawDataPtr->rawbuf[2], NEC_REPEAT_HEADER_SPACE)
                && matchMark(decodedIRData.rawDataPtr->rawbuf[3], NEC_BIT_MARK)) {
            decodedIRData.flags = IRDATA_FLAGS_IS_REPEAT | IRDATA_FLAGS_IS_LSB_FIRST;
            decodedIRData.address = lastDecodedAddress;
            decodedIRData.command = lastDecodedCommand;
            decodedIRData.protocol = lastDecodedProtocol;
            return true;
        }
        return false;
    }

    // Check command header space
    if (!matchSpace(decodedIRData.rawDataPtr->rawbuf[2], NEC_HEADER_SPACE)) {
        // IR_DEBUG_PRINT(F("NEC: "));
        // IR_DEBUG_PRINTLN(F("Header space length is wrong"));
        printf("NEC: Header space length is wrong\r\n");
        return false;
    }

    if (!decodePulseDistanceData(NEC_BITS, 3, NEC_BIT_MARK, NEC_ONE_SPACE, NEC_ZERO_SPACE, PROTOCOL_IS_LSB_FIRST)) {
        // IR_DEBUG_PRINT(F("NEC: "));
        // IR_DEBUG_PRINTLN(F("Decode failed"));
        printf("NEC: Decode failed\r\n");        
        return false;
    }

    // Stop bit
    if (!matchMark(decodedIRData.rawDataPtr->rawbuf[3 + (2 * NEC_BITS)], NEC_BIT_MARK)) {
        // IR_DEBUG_PRINT(F("NEC: "));
        // IR_DEBUG_PRINTLN(F("Stop bit mark length is wrong"));
        printf("NEC: Stop bit mark length is wrong\r\n");             
        return false;
    }

    // Success
//    decodedIRData.flags = IRDATA_FLAGS_IS_LSB_FIRST; // Not required, since this is the start value
    // LongUnion tValue;
    tValue.ULong = decodedIRData.decodedRawData;
    decodedIRData.command = tValue.UByte.MidHighByte; // 8 bit
    // Address
    if (tValue.UWord.LowWord == APPLE_ADDRESS) {
        /*
         * Apple
         */
        decodedIRData.protocol = APPLE;
        decodedIRData.address = tValue.UByte.HighByte;

    } else {
        /*
         * NEC LSB first, so first sent bit is also LSB of decodedIRData.decodedRawData
         */
        if (tValue.UByte.LowByte == (uint8_t) (~tValue.UByte.MidLowByte)) {
            // standard 8 bit address NEC protocol
            decodedIRData.address = tValue.UByte.LowByte; // first 8 bit
        } else {
            // extended NEC protocol
            decodedIRData.address = tValue.UWord.LowWord; // first 16 bit
        }
        // Check for command if it is 8 bit NEC or 16 bit ONKYO
        if (tValue.UByte.MidHighByte == (uint8_t) (~tValue.UByte.HighByte)) {
            decodedIRData.protocol = NEC;
        } else {
            decodedIRData.protocol = ONKYO;
            decodedIRData.command = tValue.UWord.HighWord; // 16 bit command

            /*
             * Old NEC plausibility check below, now it is just ONKYO :-)
             */
//            IR_DEBUG_PRINT(F("NEC: "));
//            IR_DEBUG_PRINT(F("Command=0x"));
//            IR_DEBUG_PRINT(tValue.UByte.MidHighByte, HEX);
//            IR_DEBUG_PRINT(F(" is not inverted value of 0x"));
//            IR_DEBUG_PRINTLN(tValue.UByte.HighByte, HEX);
//            decodedIRData.flags = IRDATA_FLAGS_PARITY_FAILED | IRDATA_FLAGS_IS_LSB_FIRST;
        }
    }
    decodedIRData.numberOfBits = NEC_BITS;

    return true;
}


void resume(void) 
{
    // check allows to call resume at arbitrary places or more than once
    if (irparams.StateForISR == IR_REC_STATE_STOP) 
    {
        irparams.StateForISR = IR_REC_STATE_IDLE;
    }
}

bool decode(void) {
    // uint16_t i = 0;

    if (irparams.StateForISR != IR_REC_STATE_STOP) {
        return false;
    }

    initDecodedIRData(); // sets IRDATA_FLAGS_WAS_OVERFLOW

    decodedIRData.rawDataPtr = &irparams;

    #if defined (ENABLE_UART_DBG)   // debug
    for ( i = 0 ; i < 10 ; i++ )
    {
        printf("%2d:0x%2X (%4d)\r\n" , i,irparams.rawbuf[i],irparams.rawbuf[i]);
    }
    printf("\r\n");
    #endif

    if (decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
        /*
         * Set OverflowFlag flag and return true here, to let the loop call resume or print raw data.
         */
        decodedIRData.protocol = UNKNOWN;
        return true;
    }

    if (decodeNEC()) {
        return true;
    }

    /*
     * Return true here, to let the loop decide to call resume or to print raw data.
     */
    return true;

}

void printIRResultAsCVariables(void) {
// Now dump "known" codes
    if (decodedIRData.protocol != UNKNOWN) {

        /*
         * New decoders have address and command
         */
        // aSerial->print(F("uint16_t"));
        // aSerial->print(F(" address = 0x"));
        // aSerial->print(decodedIRData.address, HEX);
        // aSerial->println(';');
        printf("address = 0x%2X\r\n" , decodedIRData.address);

        // aSerial->print(F("uint16_t"));
        // aSerial->print(F(" command = 0x"));
        // aSerial->print(decodedIRData.command, HEX);
        // aSerial->println(';');
        printf("command = 0x%2X\r\n" , decodedIRData.command);

        // All protocols have data
        // aSerial->print(F("uint32_t data = 0x"));
        // aSerial->print(decodedIRData.decodedRawData, HEX);
        // aSerial->println(';');
        // aSerial->println();
        printf("raw data = 0x%8X\r\n" , decodedIRData.decodedRawData);    
        printf("split = [0x%2X],[0x%2X],[0x%2X],[0x%2X]\r\n" , 
                    GET_BYTE0(decodedIRData.decodedRawData) ,
                    GET_BYTE1(decodedIRData.decodedRawData) ,
                    GET_BYTE2(decodedIRData.decodedRawData) ,
                    GET_BYTE3(decodedIRData.decodedRawData) );          
        printf("\r\n");
    }
}

void IRReceive_Polling(void)
{
    if (decode())
    {
        printIRResultAsCVariables();
        resume(); 
    }
}

void IRReceive_irq(void)        // 50 us timer IRQ
{
// 7 - 8.5 us for ISR body (without pushes and pops) for ATmega328 @16MHz
    uint8_t tIRInputLevel = PB2;

    /*
     * Increase TickCounter and clip it at maximum 0xFFFF / 3.2 seconds at 50 us ticks
     */
    if (irparams.TickCounterForISR < 0xFFFF /*UINT16_MAX*/) {
        irparams.TickCounterForISR++;  // One more 50uS tick
    }

    /*
     * Due to a ESP32 compiler bug https://github.com/espressif/esp-idf/issues/1552 no switch statements are possible for ESP32
     * So we change the code to if / else if
     */
//    switch (irparams.StateForISR) {
//......................................................................
    if (irparams.StateForISR == IR_REC_STATE_IDLE) { // In the middle of a gap or just resumed (and maybe in the middle of a transmission
        if (tIRInputLevel == INPUT_MARK) {
            // check if we did not start in the middle of a transmission by checking the minimum length of leading space
            if (irparams.TickCounterForISR > RECORD_GAP_TICKS) {
                // Gap just ended; Record gap duration + start recording transmission
                // Initialize all state machine variables

                irparams.OverflowFlag = false;
                irparams.rawbuf[0] = irparams.TickCounterForISR;
                irparams.rawlen = 1;
                irparams.StateForISR = IR_REC_STATE_MARK;
            } // otherwise stay in idle state
            irparams.TickCounterForISR = 0;// reset counter in both cases
        }

    } else if (irparams.StateForISR == IR_REC_STATE_MARK) {  // Timing mark
        if (tIRInputLevel != INPUT_MARK) {   // Mark ended; Record time

            irparams.rawbuf[irparams.rawlen++] = irparams.TickCounterForISR;
            irparams.StateForISR = IR_REC_STATE_SPACE;
            irparams.TickCounterForISR = 0;
        }

    } else if (irparams.StateForISR == IR_REC_STATE_SPACE) {  // Timing space
        if (tIRInputLevel == INPUT_MARK) {  // Space just ended; Record time
            if (irparams.rawlen >= RAW_BUFFER_LENGTH) {
                // Flag up a read OverflowFlag; Stop the state machine
                irparams.OverflowFlag = true;
                irparams.StateForISR = IR_REC_STATE_STOP;
            } else {

                irparams.rawbuf[irparams.rawlen++] = irparams.TickCounterForISR;
                irparams.StateForISR = IR_REC_STATE_MARK;
            }
            irparams.TickCounterForISR = 0;

        } else if (irparams.TickCounterForISR > RECORD_GAP_TICKS) {
            /*
             * Current code is ready for processing!
             * We received a long space, which indicates gap between codes.
             * Switch to IR_REC_STATE_STOP
             * Don't reset TickCounterForISR; keep counting width of next leading space
             */
            irparams.StateForISR = IR_REC_STATE_STOP;
        }
    } else if (irparams.StateForISR == IR_REC_STATE_STOP) {
        /*
         * Complete command received
         * stay here until resume() is called, which switches state to IR_REC_STATE_IDLE
         */
        
        if (tIRInputLevel == INPUT_MARK) {
            // Reset gap TickCounterForISR, to prepare for detection if we are in the middle of a transmission after call of resume()
            irparams.TickCounterForISR = 0;            
        }
    }


}

void IRReceive_Init(void)
{
    irparams.IRReceivePin = PB2;

    resume();

}


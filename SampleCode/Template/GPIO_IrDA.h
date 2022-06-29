/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

// #define STEP                                        (400)   //us
#define STEP                                        (50)    //us

#define Boot_Limit                                  ((9000+4500 +1000)/STEP)    // 36.25
#define Boot_Lower                                  ((9000+4500 -1000)/STEP)    // 31.25   
#define Bit1_Limit                                  ((2250 +800)/STEP)          // 7.625
#define Bit0_Limit                                  ((1125 +400)/STEP)          // 3.8125

#define USER_H                                      (0x00)  //user code
#define USER_L                                      (0xFF)
#define Check_EN                                    (0) //need to check user code ?
#define CA_S                                        (8) //buttoned pressed timing , unit : 108ms

#define IrDA_IO                                     (PB2)

#define IrDA_Decode_Invaild                         (0)
#define IrDA_Decode_Vaild                           (1)
#define IrDA_Decode_PressShort                      (2)
#define IrDA_Decode_PressLong                       (3)

#define IrDA_PressShort_Timing                      (120*1000/STEP)  //120 ms

/*
    test remote (NEC) user code : 0x00 0xFF

    digit 1 : 0x0 0xFF 0x45

    digit   1       2       3
    code :  45      46      47

    digit   4       5       6
    code :  44      40      43

    digit   7       8       9
    code :  07      15      09

    digit   *       0       #
    code :  16      19      0D

    digit           UP       
    code :          18     

    digit   L       OK       R
    code :  08      1C       5A

    digit           DOWN       
    code :          52      
*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void IrDA_DecodePressedLong(void);
void IrDA_DecodePressedShort(void);
void IrDA_DecodePolling(void);
void IrDA_DecodeIRQ(void);

void IrDATimer_Init(void);
void IrDAIO_Init(void);

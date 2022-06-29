/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "GPIO_IrDA.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

uint8_t IR_BT;     //decode result : 0 (invalid) ,1 (vaild) , 2(short) , 3(long)
uint8_t NEC[4];    //decode buffer , NEC[0]:user code HIGH , NEC[1]:user code LOW , NEC[2] : operation code , NEC[3] : ~operation code
uint8_t cntCA;     //pressed long counter
uint16_t cntStep;   
uint8_t IRa,IRb;   //receive I/O high/low
uint8_t IRsync;    
uint8_t BitN;      


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void IrDA_DecodePressedLong(void)
{
    printf("long,operation code:0x%2X\r\n" , NEC[2]);
}

void IrDA_DecodePressedShort(void)
{
    printf("short,operation code:0x%2X\r\n" , NEC[2]);
}

void IrDA_Bufferlog(void)
{
    uint8_t i = 0;
    for (i = 0 ; i < sizeof(NEC) ; i++)
    {
        printf("0x%2X," , NEC[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");    
}

void IrDA_DecodePolling(void)
{

    if((IR_BT == IrDA_Decode_PressShort)||(IR_BT == IrDA_Decode_PressLong))                          
    {
        if(IR_BT == IrDA_Decode_PressShort)
        {
            IrDA_DecodePressedShort();                
        }
        else
        {
            IrDA_DecodePressedLong();  
        }        
     
        IR_BT = IrDA_Decode_Invaild;

        printf("%s:",__FUNCTION__);          
        IrDA_Bufferlog();
    }
}

void IrDA_DecodeIRQ(void)
{
   cntStep++;
   if(IR_BT == IrDA_Decode_Vaild)
   {
        if(cntStep > IrDA_PressShort_Timing)
        {
            IR_BT = IrDA_Decode_PressShort;
        }
   }

   IRb = IRa;
   IRa = IrDA_IO;
        
   if (IRb && !IRa)
   {        
        if(cntStep > Boot_Limit)
        {        
            if (IR_BT == IrDA_Decode_Vaild)
            {
                if (++cntCA > CA_S)
                {
                    IR_BT = IrDA_Decode_PressLong;
                }
            }
            IRsync = 0;
        }
        else if (cntStep > Boot_Lower)
        { 
            IRsync = 1; 
            BitN = 32; 
        }                       
        else if(IRsync)
        {
            if (cntStep > Bit1_Limit)
            {          
                IRsync = 0;                  
            }
            else
            {   
                NEC[3] >>= 1;                                
                if (cntStep > Bit0_Limit)
                {
                    NEC[3] |= 0x80;
                }
                if (--BitN == 0)                                
                {
                    IRsync = 0;

                    #if (Check_EN == 1)                                       
                    if( (NEC[0] == USER_H) && 
                        (NEC[1] == USER_L) &&
                        (NEC[2] == ~NEC[3]))
                    {  
                        IR_BT = IrDA_Decode_Vaild; 
                        cntCA = 0;  
                    }
                    #else
                    // if (NEC[2] == ~NEC[3])
                    { 
                        IR_BT = IrDA_Decode_Vaild; 
                        cntCA = 0; 
                    }
                    #endif  
                    // printf("under process(%d)\r\n" , IR_BT);
                    // IrDA_Bufferlog();                                     
                }
                else if ((BitN & 0x07) == 0)
                {   
                    NEC[0] = NEC[1]; 
                    NEC[1] = NEC[2]; 
                    NEC[2] = NEC[3];   
                }
            }
        }
        cntStep = 0;
   }
}




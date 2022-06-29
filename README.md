# M031BSP_GPIO_IrDA_Rx
 M031BSP_GPIO_IrDA_Rx

update @ 2022/06/29

1. use GPIO (PB2) + TIMER 3 (50us , 20000Hz) , to decode remote control data (NEC)

2. by enable below define , to see different method to decode RC data

- #define ENABLE_Method_I_IrDA

this sample refer from http://www.51hei.com/bbs/dpj-58436-1.html 

- #define ENABLE_Method_II_Arduino

this sample refer from https://github.com/Arduino-IRremote/Arduino-IRremote

3. below is log message , 

ENABLE_Method_I_IrDA
![image](https://github.com/released/M031BSP_GPIO_IrDA_Rx/blob/main/log_ENABLE_Method_I_IrDA.jpg)	

ENABLE_Method_II_Arduino
![image](https://github.com/released/M031BSP_GPIO_IrDA_Rx/blob/main/log_ENABLE_Method_II_Arduino.jpg)	

below is the code about remote control (NEC) , used for this project
![image](https://github.com/released/M031BSP_GPIO_IrDA_Rx/blob/main/remote.jpg)	


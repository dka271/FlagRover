/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    navigation.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "navigation.h"
#include "navigation_public.h"
#include "ms2test.h"

NAVIGATION_DATA navigationData;

static QueueHandle_t navQueue;

void NAVIGATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    navigationData.state = NAVIGATION_STATE_INIT;
    
    //Initialize the navigation queue
    navQueue = xQueueCreate(10, sizeof(unsigned char[NAV_QUEUE_BUFFER_SIZE]));
    if(navQueue == 0){
        dbgPauseAll();
    }
}

void navSendMsgFromISR(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBackFromISR(navQueue, msg, NULL);
}

void navSendMsg(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}
    
unsigned char navCalculateChecksum(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < NAV_QUEUE_BUFFER_SIZE - 1; i++){
        sum += msg[i];
    }
    return sum;
}

void sendEdgeToNavigationThread(unsigned char seqNum, unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY) {
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    char msg[7];
    msg[0] = seqNum;
    msg[1] = startX;
    msg[2] = startY;
    msg[3] = endX;
    msg[4] = endY;
    msg[5] = NAV_PATHFINDING_ID;
    unsigned char i;
    for (i=0; i < 6; i++) {
        msg[6] += msg[i];
    }

    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}


/******************************************************************************
  Function:
    void NAVIGATION_Tasks ( void )

  Remarks:
    See prototype in navigation.h.
 */

void NAVIGATION_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_NAV_ENTER);
    
    unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
    unsigned int previousValue1 = 0;
    unsigned int speed1;
    unsigned int previousValue2 = 0;
    unsigned int speed2;
    unsigned int pwmCount = 0;
    unsigned int desiredSpeed = 0;
    int m1PID;
    int m2PID;
    Motor1SetPWM(0);
    Motor2SetPWM(0);
    Motor1SetDirection(MOTOR_1_FORWARDS);
    Motor2SetDirection(MOTOR_2_FORWARDS);

    dbgOutputLoc(DBG_LOC_NAV_BEFORE_WHILE);
    while(1){
        //Block until a message is received
        dbgOutputLoc(DBG_LOC_NAV_BEFORE_RECEIVE);
        BaseType_t receiveCheck = xQueueReceive(navQueue, receivemsg, portMAX_DELAY);
        dbgOutputLoc(DBG_LOC_NAV_AFTER_RECEIVE);

        //Handle the message
        if(receiveCheck == pdTRUE){
            //Convert the message into integer format
            unsigned int receivemsgint = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
            //Get the message ID
            int msgId = (receivemsg[NAV_SOURCE_ID_IDX] & NAV_SOURCE_ID_MASK) >> NAV_SOURCE_ID_OFFSET;
            //Handle a specific message
            if (msgId == NAV_TIMER_COUNTER_3_ID){
                //Motor 2 Encoder Message Handler
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);
                speed2 = (receivemsgint & 0x0000ffff) - previousValue2;
                //dbgUARTVal(speed2);
                previousValue2 = receivemsgint & 0x0000ffff;

                //Handle PWM stuff
                m2PID = PID2(desiredSpeed, speed2);
            }else if (msgId == NAV_TIMER_COUNTER_5_ID){
                //Motor 2 Encoder Message Handler
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);
                speed1 = (receivemsgint & 0x0000ffff) - previousValue1;
//                        dbgUARTVal(speed1);
//                        dbgOutputVal(speed1);
                previousValue1 = receivemsgint & 0x0000ffff;

                //Handle PWM stuff
                m1PID = PID1(desiredSpeed, speed1);

                if (UNIT_TESTING){
                    encoderSpeedTest(speed1);
                }
            }else if (msgId == NAV_COLOR_SENSOR_1_ID){
                //Handle stuff from color sensor 1
            }else if (msgId == NAV_COLOR_SENSOR_2_ID){
                //Handle stuff from color sensor 2
            }else if (msgId == NAV_COLOR_SENSOR_3_ID){
                //Handle stuff from color sensor 3
                if (UNIT_TESTING){
                    navQueueReceiveTest(receivemsg);
                }
            }else if (msgId == NAV_PATHFINDING_ID){
                //Handle stuff from the mapping queue
                //dbgOutputVal((receivemsgint & 0x00ff0000) >> 16);
                //dbgOutputVal((receivemsgint & 0x0000ff00) >> 8);
                //dbgOutputVal(receivemsgint & 0x000000ff);
            }else if (msgId == NAV_PWM_TIMER_ID){
                //int ticksTarget = TEST_SPEED_TICKS;
                Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
//                        dbgOutputVal(m1PID);
                //dbgOutputVal(m2PID);
                pwmCount++;
                if (pwmCount >= 25){
                    pwmCount = 0;
                }
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */

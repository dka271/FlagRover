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
    msg[5] = NAV_PATHFINDING_ID << NAV_SOURCE_ID_OFFSET;
    msg[6] = navCalculateChecksum(msg);
    
    //Nop();
    
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

void sendLocToNavigationThread(unsigned char x, unsigned char y) {
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    char msg[7];
    unsigned char seqNum = 130;
    msg[0] = seqNum;
    msg[1] = x;
    msg[2] = y;
    msg[5] = NAV_PATHFINDING_ID << NAV_SOURCE_ID_OFFSET;
    msg[6] = navCalculateChecksum(msg);
    
    //Nop();
    
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

/******************************************************************************
  Function:
    void NAVIGATION_Tasks ( void )

  Remarks:
    See prototype in navigation.h.
 */
//int HELPMEIMTRAPPED=0;

unsigned char pathMsg[PATH_QUEUE_BUFFER_SIZE];

int TIMER=0;

void NAVIGATION_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_NAV_ENTER);
    
    //Message queue stuff
    unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE];
    //Motor speed control stuff
    unsigned int previousValue1 = 0;
    unsigned int speed1;
    unsigned int previousValue2 = 0;
    unsigned int speed2;
    unsigned int pwmCount = 0;
    unsigned int desiredSpeed = ROVER_SPEED_STOPPED;
    int ticksRemaining = ROVER_TICKS_REMAINING_NONE;
    int m1PID;
    int m2PID;
    //Location stuff
    SetDirectionForwards();
    //Path stuff
    unsigned int moveAmount[100];
    unsigned char moveType[100];
    unsigned int moveFirstIdx = 0;
    unsigned int moveCurrentIdx = 0;
    unsigned int moveLastIdx = 0;
    unsigned int moveGoalIdx = 0xff;
    unsigned int moveMaxIdx = 99;
    int roverStopped = 0;

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
                speed2 = (receivemsgint & 0x0000ffff) - previousValue2;
                previousValue2 = receivemsgint & 0x0000ffff;
                
                //Handle path controls
                if (speed2 == 0){
                    roverStopped++;
                }else{
                    roverStopped = 0;
                }
                if (roverStopped >= 10 && moveLastIdx > moveCurrentIdx){
                    //There is a command, and the rover is not moving
                    //Interpret the command and control the rover
                
                    //FOR TESTING, REMOVE LATER
                    unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                    //END TESTING SECTION
                    
                    if (moveType[moveCurrentIdx] == ROVER_DIRECTION_LEFT){
                        SetDirectionCounterclockwise();
                        desiredSpeed = ROVER_SPEED_TURNING;
                        ticksRemaining = AdjustAngleToRotate(moveAmount[moveCurrentIdx], GetOrientation());
                        if (ticksRemaining < 0){
                            ticksRemaining *= -1;
                            SetDirectionClockwise();
                        }
                        sprintf(testMsg, "*Command: Left Turn, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_RIGHT){
                        SetDirectionClockwise();
                        desiredSpeed = ROVER_SPEED_TURNING;
                        ticksRemaining = AdjustAngleToRotate(moveAmount[moveCurrentIdx], GetOrientation());
                        if (ticksRemaining < 0){
                            ticksRemaining *= -1;
                            SetDirectionCounterclockwise();
                        }
                        sprintf(testMsg, "*Command: Right Turn, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_FORWARDS){
                        SetDirectionForwards();
                        desiredSpeed = ROVER_SPEED_STRAIGHT;
                        ticksRemaining = moveAmount[moveCurrentIdx];
                        sprintf(testMsg, "*Command: Move Straight, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_BACKWARDS){
                        SetDirectionBackwards();
                        desiredSpeed = ROVER_SPEED_STRAIGHT;
                        ticksRemaining = moveAmount[moveCurrentIdx];
                        sprintf(testMsg, "*Command: Move Backward, Ticks = %d~", ticksRemaining);
                    }
                
                    //FOR TESTING, REMOVE LATER
                    if (MOTOR_PATHFINDING_INTEGRATION_TESTING){
                        commSendMsgToWifiQueue(testMsg);
                        sprintf(testMsg, "*Current Position = (%d,%d), Current Orientation = %d, Motor Direction = %d~", (int) GetLocationX(), (int) GetLocationY(), (int) GetOrientation(), GetMotorDirection());
                        commSendMsgToWifiQueue(testMsg);
                    }
                    //END TESTING SECTION
                    
                    
                    Nop();
                    moveCurrentIdx++;
                    roverStopped = 0;
                }else if (roverStopped > 10 && moveCurrentIdx == moveGoalIdx){
                    //Destination has been reached, reset the command queue
                    moveCurrentIdx = 0;
                    moveLastIdx = 0;
                    moveGoalIdx = 0xff;
                
                    //FOR TESTING, REMOVE LATER
                    if (MOTOR_PATHFINDING_INTEGRATION_TESTING){
                        unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                        sprintf(testMsg, "*Goal Reached, Current Position = (%d,%d), Current Orientation = %d, Motor Direction = %d~", (int) GetLocationX(), (int) GetLocationY(), (int) GetOrientation(), GetMotorDirection());
                        commSendMsgToWifiQueue(testMsg);
                    }
                    //END TESTING SECTION
                }
                
                //Handle remaining distance
                
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT){
                    
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed2);
                    
                    //Handle position and orientation
                    HandlePositionAndOrientation(speed2, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);
                    //Send position to pathfinding thread
//                    unsigned char pathMsg[PATH_QUEUE_BUFFER_SIZE];
                    if (TIMER > 30) {
                        unsigned char intX = 0xff & (int) (GetLocationX());
                        unsigned char intY = 0xff & (int) (GetLocationY());
                        pathMsg[0] = intX;
                        pathMsg[1] = intY;
                        pathMsg[PATH_SOURCE_ID_IDX] = PATH_NAVIGATION_ID << PATH_SOURCE_ID_OFFSET;
                        pathMsg[PATH_CHECKSUM_IDX] = pathCalculateChecksum(pathMsg);
                        pathSendMsg(pathMsg);
                        TIMER = 0;
                    } else {
                        TIMER++;
                    }
                    
                    //Handle Daniel's server testing
                    motorTestNavSendSpeed(speed2);
                }

                //Handle PWM stuff
                m2PID = PID2(desiredSpeed, speed2);
            }else if (msgId == NAV_TIMER_COUNTER_5_ID){
                //Motor 2 Encoder Message Handler
                speed1 = (receivemsgint & 0x0000ffff) - previousValue1;
                previousValue1 = receivemsgint & 0x0000ffff;
                
                //Handle remaining distance
                if (GetMotorDirection() != ROVER_DIRECTION_LEFT){
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed1);
                    
                    //Handle position and orientation
                    HandlePositionAndOrientation(speed1, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);
                    //Send position to pathfinding thread
//                    unsigned char pathMsg[PATH_QUEUE_BUFFER_SIZE];
                    if (TIMER > 30) {
                        unsigned char intX = 0xff & (int) (GetLocationX());
                        unsigned char intY = 0xff & (int) (GetLocationY());
                        pathMsg[0] = intX;
                        pathMsg[1] = intY;
                        pathMsg[PATH_SOURCE_ID_IDX] = PATH_NAVIGATION_ID << PATH_SOURCE_ID_OFFSET;
                        pathMsg[PATH_CHECKSUM_IDX] = pathCalculateChecksum(pathMsg);
                        pathSendMsg(pathMsg);
                        TIMER = 0;
                    } else {
                        TIMER++;
                    }
                    
                    //Handle Daniel's server testing
                    motorTestNavSendSpeed(speed1);
                }

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
                
//                ElectromagnetSetOn();
//                LedSetOn();
                //Handle stuff from the pathfinding queue
                //Messages are in the form: (high) [checksum, sourceID, endY, endX, startY, startX, seqNum] (low)
                unsigned char seqNum = receivemsg[0];
                unsigned char startX = receivemsg[1];
                unsigned char startY = receivemsg[2];
                unsigned char endX = receivemsg[3];
                unsigned char endY = receivemsg[4];
                bool addCommand = true;
                
                if (seqNum == 0){
                    //This is a new path, reset the things
                    moveCurrentIdx = 0;
                    moveLastIdx = 0;
                    moveGoalIdx = 0xff;
                    float startXF = (float) startX;
                    float startYF = (float) startY;
                    float firstNodeOffset = sqrt(pow(startXF - GetLocationX(), 2) + pow(startYF - GetLocationY(), 2));
                    if (firstNodeOffset > 5.0){
                        startX = 0xff & ((int) GetLocationX());
                        startY = 0xff & ((int) GetLocationY());
                    }
                    ticksRemaining = 0;
                    desiredSpeed = ROVER_SPEED_STOPPED;
                }else if (seqNum == 130){
                    //Update the position
                    SetLocationX(startX);
                    SetLocationY(startY);
                    addCommand = false;
                }
                
                if (addCommand){
                    //Get the number of ticks for angle and distance
                    int distanceTicks = CalculateDistanceFromPoints(startX, startY, endX, endY, CALCULATE_IN_CENTIMETERS);
                    //Negative angle means clockwise turn, positive angle means counterclockwise turn
                    int angleTicks = CalculateAngleFromPoints(startX, startY, endX, endY);

                    //FOR TESTING, REMOVE LATER
                    if (MOTOR_PATHFINDING_INTEGRATION_TESTING){
                        unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                        sprintf(testMsg, "*Received edge from (%d,%d) to (%d,%d) with seqNum(%d)~",startX,startY,endX,endY,seqNum);
                        commSendMsgToWifiQueue(testMsg);
                        sprintf(testMsg, "*Distance Ticks = %d, Angle Ticks = %d",distanceTicks,angleTicks);
                        commSendMsgToWifiQueue(testMsg);
                    }
                    //END TESTING SECTION

                    //Put the turn into the movement queue
                    if (moveLastIdx <= moveMaxIdx){
                        if (angleTicks < 0){
                            //Right turn

                            angleTicks *= -1;
                            moveAmount[moveLastIdx] = angleTicks;
                            moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
                            moveLastIdx++;
                        }else{
                            //Left turn

                            moveAmount[moveLastIdx] = angleTicks;
                            moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                            moveLastIdx++;
                        }
                    }
                    //Put the move into the movement queue
                    if (moveLastIdx <= moveMaxIdx){

                        moveAmount[moveLastIdx] = distanceTicks;
                        moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                        moveLastIdx++;
                    }

                    //Check if this is the goal
                    if (seqNum == END_OF_PATH_NUM){
                        //This is the goal

                        moveGoalIdx = moveLastIdx;
                    }
                }
            }else if (msgId == NAV_PWM_TIMER_ID){
                //Handle PWM timer messages
                Motor1SetPWM(GetPWMFromValue(m1PID, pwmCount));
                Motor2SetPWM(GetPWMFromValue(m2PID, pwmCount));
                pwmCount++;
                if (pwmCount >= 25){
                    pwmCount = 0;
                }
            }else if (msgId == NAV_OTHER_ID){
                //Handle a message from another source
                //Handle Daniel's server testing
                motorTestNavReceive(receivemsg, &desiredSpeed, &ticksRemaining);
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */

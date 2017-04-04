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
    navQueue = xQueueCreate(50, sizeof(unsigned char[NAV_QUEUE_BUFFER_SIZE]));
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
    
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

void sendLocToNavigationThread(unsigned char x, unsigned char y, unsigned char orientation) {
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    char msg[7];
    unsigned char seqNum = 130;
    msg[0] = seqNum;
    msg[1] = x;
    msg[2] = y;
    msg[3] = orientation;
    msg[5] = NAV_PATHFINDING_ID << NAV_SOURCE_ID_OFFSET;
    msg[6] = navCalculateChecksum(msg);
    
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

void sendStartMessageToNavigationThread() {
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    char msg[7];
    unsigned char seqNum = 135;
    msg[0] = seqNum;
    msg[5] = NAV_PATHFINDING_ID << NAV_SOURCE_ID_OFFSET;
    msg[6] = navCalculateChecksum(msg);
    
    xQueueSendToBack(navQueue, msg, portMAX_DELAY);
}

bool DoWeCrossLineQuestionMark(){
    switch (locationState){
        case (CROSSED_0_LINES):{
            region center = regionList[CENTRAL_ZONE];
            unsigned char centerY = (center.y - (center.length >> 1)) << 1;
            unsigned char ourY = ((unsigned char) GetLocationY());
            
                
                    //FOR TESTING, REMOVE LATER
                    unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testMsg, "*Current Position = (%d,%d), Center Y = %d, State = 0~", (int) GetLocationX(), (int) GetLocationY(), centerY);
                    commSendMsgToWifiQueue(testMsg);
                    //END TESTING SECTION
            
            if (ourY >= centerY - 10){
                locationState = CROSSED_1_LINES;
                return true;
            }else{
                return false;
            }
            break;
        }
        case (CROSSED_1_LINES):{
            locationState = CROSSED_2_LINES;
            
                
                    //FOR TESTING, REMOVE LATER
                    unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testMsg, "*Current Position = (%d,%d), State = 1~", (int) GetLocationX(), (int) GetLocationY());
                    commSendMsgToWifiQueue(testMsg);
                    //END TESTING SECTION
                    
            return true;
            break;
        }
        case (CROSSED_2_LINES):{
            region farFlag = regionList[FAR_FLAG_ZONE];
            unsigned char farFlagY = (farFlag.y - (farFlag.length >> 1)) << 1;
            unsigned char minX = ((farFlag.x - (farFlag.width >> 1)) << 1) - 10;
            unsigned char maxX = ((farFlag.x + (farFlag.width >> 1)) << 1) + 10;
            unsigned char ourY = ((unsigned char) GetLocationY());
            unsigned char ourX = ((unsigned char) GetLocationX());
            
                
                    //FOR TESTING, REMOVE LATER
                    unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testMsg, "*Current Position = (%d,%d), MinX = %d, MaxX = %d, Flag Zone Y = %d, State = 2~", (int) GetLocationX(), (int) GetLocationY(), minX, maxX, farFlagY);
                    commSendMsgToWifiQueue(testMsg);
                    //END TESTING SECTION
                    
            if (ourY >= farFlagY - 20 && (ourX <= maxX && ourX >= minX)){
//                ElectromagnetSetOn();
//                LedSetOn();
                locationState = CROSSED_3_LINES;
                return true;
            }else{
                return false;
            }
            break;
        }
        case (CROSSED_3_LINES):{
            
                
                    //FOR TESTING, REMOVE LATER
                    unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testMsg, "*Current Position = (%d,%d), State = 3~", (int) GetLocationX(), (int) GetLocationY());
                    commSendMsgToWifiQueue(testMsg);
                    //END TESTING SECTION
                    
            return false;
            break;
        }
    }
    return false;
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
    //Line Sensing stuff
    int ignoringTape = 0;
    int ignoreTapeCount = 0;
    int ignoreTapeMax = 40;
    bool cs1OnTape = false;
    bool cs2OnTape = false;
//    bool cs3OnTape = false;//color sensor 3 does not exist on the flag rover
    locationState = CROSSED_0_LINES;
    //Flag Capture stuff
    bool pixyCamFoundFlag = false;
    bool pickedUpFlag = false;
    bool sawFlagOnce = false;
    //Path stuff
    unsigned int moveAmount[100];
    unsigned char moveType[100];
    unsigned int moveFirstIdx = 0;
    unsigned int moveCurrentIdx = 0;
    unsigned int moveLastIdx = 0;
    unsigned int moveGoalIdx = 0xff;
    unsigned int moveMaxIdx = 99;
    int roverStopped = 0;
    bool pathfindingIsReady = false;
    int pathfindingCount = 0;
    int pathfindingCountMax = 60;
    unsigned char flagX = 0xff;
    unsigned char flagY = 0xff;
    bool hasFlagLocation = false;
    //I2C Initialization Stuff
    //Open the I2C
    int i2cCount = -100;//Set this low so the color sensors are guaranteed to receive power by the time we start initializing them
    while (DRV_I2C_Status(sysObj.drvI2C0) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
            unsigned char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
            sprintf(testServerMsg, "Waiting for I2C 1...");
            commSendMsgToWifiQueue(testServerMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c1_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
    while (DRV_I2C_Status(sysObj.drvI2C1) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
            unsigned char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
            sprintf(testServerMsg, "Waiting for I2C 2...");
            commSendMsgToWifiQueue(testServerMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c2_handle = DRV_I2C_Open(DRV_I2C_INDEX_1, DRV_IO_INTENT_READWRITE);
    //Init the state machine
    DRV_TCS_HandleColorSensor(NULL, COLOR_SENSOR_RESET_STATE_MACHINE);
    DRV_ADC_Open();
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_ADC_1);

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
                        ticksRemaining = moveAmount[moveCurrentIdx];
                        if (ticksRemaining < 0){
                            ticksRemaining *= -1;
                            SetDirectionClockwise();
                        }
                        int minRotation = deg2tickF(30);
                        if (ticksRemaining >= minRotation){
                            //Normal turning speed
                            desiredSpeed = ROVER_SPEED_TURNING;
                        }else{
                            //Slow turning speed
                            desiredSpeed = ROVER_SPEED_SLOW_TURNING;
                            ticksRemaining += 30;
                        }
                        sprintf(testMsg, "*Command: Left Turn, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_RIGHT){
                        SetDirectionClockwise();
                        ticksRemaining = moveAmount[moveCurrentIdx];
                        if (ticksRemaining < 0){
                            ticksRemaining *= -1;
                            SetDirectionCounterclockwise();
                        }
                        int minRotation = deg2tickF(30);
                        if (ticksRemaining >= minRotation){
                            //Normal turning speed
                            desiredSpeed = ROVER_SPEED_TURNING;
                        }else{
                            //Slow turning speed
                            desiredSpeed = ROVER_SPEED_SLOW_TURNING;
                            ticksRemaining += 30;
                        }
                        sprintf(testMsg, "*Command: Right Turn, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_FORWARDS){
                        SetDirectionForwards();
                        desiredSpeed = ROVER_SPEED_STRAIGHT;
//                        desiredSpeed = ROVER_SPEED_MEDIUM;
                        ticksRemaining = moveAmount[moveCurrentIdx];
                        sprintf(testMsg, "*Command: Move Straight, Ticks = %d~", ticksRemaining);
                    }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_BACKWARDS){
                        SetDirectionBackwards();
                        desiredSpeed = ROVER_SPEED_STRAIGHT;
//                        desiredSpeed = ROVER_SPEED_MEDIUM;
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
                    
                    //Handle movement within Flag Zone
                    if (locationState == CROSSED_3_LINES && !pickedUpFlag){
                        if (pixyCamFoundFlag){
                            //Go towards flag
                        }else{
                            //Rotate until we see flag
                            moveAmount[moveLastIdx] = deg2tick(20);
                            moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                            moveLastIdx++;
                            moveGoalIdx = moveLastIdx;
                        }
                    }
                }
                
                //Handle remaining distance
                
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT){
                    
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed2);
                    
                    //Handle position and orientation
                    HandlePositionAndOrientation(speed2, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);
                    //Send position to pathfinding thread
//                    unsigned char pathMsg[PATH_QUEUE_BUFFER_SIZE];
                    if (TIMER > 30) {
                        unsigned char intX = (0xff & (int) (GetLocationX())) >> 1;
                        unsigned char intY = (0xff & (int) (GetLocationY())) >> 1;
                        pathMsg[0] = intX;
                        pathMsg[1] = intY;
                        int tempOrientation = GetOrientation();
                        if (tempOrientation < 0){
                            tempOrientation += 360;
                        }
                        pathMsg[2] = ((unsigned char) (tempOrientation >> 1));
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
                        unsigned char intX = (0xff & (int) (GetLocationX())) >> 1;
                        unsigned char intY = (0xff & (int) (GetLocationY())) >> 1;
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
                
                //Call pathfinding every x seconds
                pathfindingCount++;
                if (pathfindingCount >= pathfindingCountMax && pathfindingIsReady && locationState != CROSSED_3_LINES){
                    unsigned char commMsg[COMM_QUEUE_BUFFER_SIZE];
                    sprintf(commMsg,"*{\"S\":\"w\",\"T\":\"f\",\"M\":\"g\",\"C\":1}~");
                    commMsg[COMM_SOURCE_ID_IDX] = COMM_UART_ID << COMM_SOURCE_ID_OFFSET;
                    commMsg[COMM_CHECKSUM_IDX] = commCalculateChecksum(commMsg);
                    commSendMsg(commMsg);
                    pathfindingCount = 0;
                }
                
                //Ignore tape for 4 seconds
                ignoreTapeCount++;
                if (ignoreTapeCount >= ignoreTapeMax){
                    ignoringTape = 0;
                    if (locationState == CROSSED_3_LINES){
                        ElectromagnetSetOn();
                        LedSetOn();
                        if (!sawFlagOnce){
                            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1);
                            sawFlagOnce = true;
                        }
                    }
                }
            }else if (msgId == NAV_COLOR_SENSOR_1_ID){
                //Handle stuff from color sensor 1 (front left)
                if (receivemsg[0] == COLOR_IS_BLUE){
                    cs1OnTape = true;
                }else{
                    cs1OnTape = false;
                }
                
                //Decide whether or not to cross tape
                if (cs1OnTape && !ignoringTape){
                    moveCurrentIdx = 0;
                    moveLastIdx = 0;
                    moveGoalIdx = 0xff;
                    ticksRemaining = 0;
                    desiredSpeed = ROVER_SPEED_STOPPED;
                    pathfindingCount = 0;
                    if (DoWeCrossLineQuestionMark()){
                        //Cross the line
                        if (locationState == CROSSED_3_LINES){
                            //Orient towards flag
                            if (hasFlagLocation){
                                int angleTicks = CalculateAngleFromPoints((unsigned char) GetLocationX(), (unsigned char) GetLocationY(), flagX, flagY);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
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
                            moveAmount[moveLastIdx] = cm2tick(10);
                            ignoreTapeCount = -30;
                        }else if (locationState == CROSSED_2_LINES){
                            moveAmount[moveLastIdx] = cm2tick(8);
                            ignoreTapeCount = -30;
                        }else if (locationState == CROSSED_1_LINES){
                            //Orient ourselves towards the line
                            if (INVERTED_X_AXIS){
                                moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                                int angleTicks = deg2tickF(94.6);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                                moveAmount[moveLastIdx] = angleTicks;
                            }else{
                                moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
                                int angleTicks = deg2tickF(97.5);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                                moveAmount[moveLastIdx] = angleTicks;
                            }
                            moveLastIdx++;
                            moveAmount[moveLastIdx] = cm2tick(10);
                            ignoreTapeCount = -40;
                        }else{
                            moveAmount[moveLastIdx] = cm2tick(20);
                            ignoreTapeCount = 0;
                        }
                        moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                        moveLastIdx++;
                        moveGoalIdx = moveLastIdx;
                        ignoringTape = 1;
                    }else if (locationState == CROSSED_3_LINES && !ignoringTape){
                        //Handle movement while in the flag zone
                        if (cs1OnTape){
                            //Just got on tape
                            moveCurrentIdx = 0;
                            moveLastIdx = 0;
                            moveGoalIdx = 0xff;
                            ticksRemaining = 0;
                            desiredSpeed = ROVER_SPEED_STOPPED;
                            if (cs2OnTape){
                                //Both on tape
                                //Drop the flag
                                ElectromagnetSetOff();
                                moveAmount[moveLastIdx] = cm2tick(1);
                                moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                                moveLastIdx++;
                                moveAmount[moveLastIdx] = cm2tick(8);
                                moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                                moveLastIdx++;
                                moveGoalIdx = moveLastIdx;
                                ignoringTape = 1;
                                ignoreTapeCount = 0;
                            }else{
                                //This one on tape
                                if (INVERTED_X_AXIS){
//                                    moveAmount[moveLastIdx] = cm2tick(1);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
//                                    moveLastIdx++;
                                    moveAmount[moveLastIdx] = deg2tick(90);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
                                    moveLastIdx++;
                                    moveGoalIdx = moveLastIdx;
                                }else{
//                                    moveAmount[moveLastIdx] = cm2tick(1);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
//                                    moveLastIdx++;
                                    moveAmount[moveLastIdx] = deg2tick(90);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                                    moveLastIdx++;
                                    moveGoalIdx = moveLastIdx;
                                }
                            }
                        }else{
                            //Just got off tape
                            if (cs2OnTape){
                                //Other one on tape
                            }else{
                                //None on tape
                            }
                        }
                    }else if (cs1OnTape && !ignoringTape){
                        //Back up
                        moveAmount[moveLastIdx] = cm2tick(1);
                        moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                        moveLastIdx++;
                        moveGoalIdx = moveLastIdx;
                        ignoringTape = 1;
                        ignoreTapeCount = 20;
                    }
                }
                //FOR TESTING
                if (COLOR_SENSOR_SERVER_TESTING){
                    unsigned char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testServerMsg, "*{Message from Color Sensor 1: %d}~", receivemsg[0]);
//                    commSendMsgToWifiQueue(testServerMsg);
                }
                //END FOR TESTING
            }else if (msgId == NAV_COLOR_SENSOR_2_ID){
                //Handle stuff from color sensor 2 (front right)
                if (receivemsg[0] == COLOR_IS_BLUE){
                    cs2OnTape = true;
                }else{
                    cs2OnTape = false;
                }
                
                //Decide whether or not to cross tape
                if (cs2OnTape && !ignoringTape){
                    moveCurrentIdx = 0;
                    moveLastIdx = 0;
                    moveGoalIdx = 0xff;
                    ticksRemaining = 0;
                    desiredSpeed = ROVER_SPEED_STOPPED;
                    pathfindingCount = 0;
                    if (DoWeCrossLineQuestionMark()){
                        //Cross the line
                        if (locationState == CROSSED_3_LINES){
                            if (hasFlagLocation){
                                int angleTicks = CalculateAngleFromPoints((unsigned char) GetLocationX(), (unsigned char) GetLocationY(), flagX, flagY);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
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
                            moveAmount[moveLastIdx] = cm2tick(10);
                            ignoreTapeCount = -30;
                        }else if (locationState == CROSSED_2_LINES){
                            moveAmount[moveLastIdx] = cm2tick(8);
                            ignoreTapeCount = -30;
                        }else if (locationState == CROSSED_1_LINES){
                            //Orient ourselves towards the line
                            if (INVERTED_X_AXIS){
                                moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                                int angleTicks = deg2tickF(82.5);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                                moveAmount[moveLastIdx] = angleTicks;
                            }else{
                                moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
                                int angleTicks = deg2tickF(85.4);
                                angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                                moveAmount[moveLastIdx] = angleTicks;
                            }
                            moveLastIdx++;
                            moveAmount[moveLastIdx] = cm2tick(10);
                            ignoreTapeCount = -40;
                        }else{
                            moveAmount[moveLastIdx] = cm2tick(20);
                            ignoreTapeCount = 0;
                        }
                        moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                        moveLastIdx++;
                        moveGoalIdx = moveLastIdx;
                        ignoringTape = 1;
                    }else if (locationState == CROSSED_3_LINES && !ignoringTape){
                        //Handle movement while in the flag zone
                        if (cs2OnTape){
                            //Just got on tape
                            moveCurrentIdx = 0;
                            moveLastIdx = 0;
                            moveGoalIdx = 0xff;
                            ticksRemaining = 0;
                            desiredSpeed = ROVER_SPEED_STOPPED;
                            if (cs1OnTape){
                                //Both on tape
                                //Drop the flag
                                ElectromagnetSetOff();
                                moveAmount[moveLastIdx] = cm2tick(1);
                                moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                                moveLastIdx++;
                                moveAmount[moveLastIdx] = cm2tick(8);
                                moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                                moveLastIdx++;
                                moveGoalIdx = moveLastIdx;
                                ignoringTape = 1;
                                ignoreTapeCount = 0;
                            }else{
                                //This one on tape
                                if (INVERTED_X_AXIS){
//                                    moveAmount[moveLastIdx] = cm2tick(1);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
//                                    moveLastIdx++;
                                    moveAmount[moveLastIdx] = deg2tick(90);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
                                    moveLastIdx++;
                                    moveGoalIdx = moveLastIdx;
                                }else{
//                                    moveAmount[moveLastIdx] = cm2tick(1);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
//                                    moveLastIdx++;
                                    moveAmount[moveLastIdx] = deg2tick(90);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
                                    moveLastIdx++;
                                    moveGoalIdx = moveLastIdx;
                                }
                            }
                        }else{
                            //Just got off tape
                            if (cs1OnTape){
                                //Other one on tape
                            }else{
                                //None on tape
                            }
                        }
                    }else if (cs2OnTape && !ignoringTape){
                        //Back up
                        moveAmount[moveLastIdx] = cm2tick(1);
                        moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                        moveLastIdx++;
                        moveGoalIdx = moveLastIdx;
                        ignoringTape = 1;
                        ignoreTapeCount = 20;
                    }
                }
                //FOR TESTING
                if (COLOR_SENSOR_SERVER_TESTING){
                    unsigned char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
                    sprintf(testServerMsg, "*{Message from Color Sensor 2: %d}~", receivemsg[0]);
//                    commSendMsgToWifiQueue(testServerMsg);
                }
                //END FOR TESTING
            }else if (msgId == NAV_PIXY_CAM_ID){
                //Handle messages with pixy cam information
                Nop();
                unsigned char wifiMsg[SEND_QUEUE_BUFFER_SIZE];
                sprintf(wifiMsg, "*{Pixy Cam Value = %d}~",receivemsg[0] + (receivemsg[1] << 8));
                commSendMsgToWifiQueue(wifiMsg);
                Nop();
                if (locationState == CROSSED_3_LINES){
                    //Only do this in the flag zone
                    //Charge le flag
                    moveCurrentIdx = 0;
                    moveLastIdx = 0;
                    moveGoalIdx = 0xff;
                    ticksRemaining = 0;
                    desiredSpeed = ROVER_SPEED_STOPPED;
                    moveAmount[moveLastIdx] = cm2tick(20);
                    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
                    moveLastIdx++;
                    moveGoalIdx = moveLastIdx;
                }
            }else if (msgId == NAV_PATHFINDING_ID){
                //Handle stuff from the pathfinding queue
                //Messages are in the form: (high) [checksum, sourceID, endY, endX, startY, startX, seqNum] (low)
                if (locationState != CROSSED_3_LINES){
                    unsigned char seqNum = receivemsg[0];
                    unsigned char startX = receivemsg[1] << 1;
                    unsigned char startY = receivemsg[2] << 1;
                    unsigned char endX = receivemsg[3] << 1;
                    unsigned char endY = receivemsg[4] << 1;
                    bool addCommand = true;

                    if (seqNum == 0 || (seqNum == 125 && (moveLastIdx == 0 || moveGoalIdx == moveLastIdx))){
                        //This is a new path, reset the things
                        moveCurrentIdx = 0;
                        moveLastIdx = 0;
                        moveGoalIdx = 0xff;
                        float startXF = (float) startX;
                        float startYF = (float) startY;
                        float firstNodeOffset = sqrt(pow(startXF - GetLocationX(), 2) + pow(startYF - GetLocationY(), 2));
                        startX = 0xff & ((int) GetLocationX());
                        startY = 0xff & ((int) GetLocationY());
                        if (firstNodeOffset > 7.5){
                            //Drop paths that have bad position
                            addCommand = false;
                        }
                        ticksRemaining = 0;
                        desiredSpeed = ROVER_SPEED_STOPPED;
                    }else if (seqNum == 130){
                        //Update the position
                        SetLocationX(startX);
                        SetLocationY(startY);
                        //Update the orientation
                        SetOrientation(endX);
                        Nop();
                        addCommand = false;
                    }else if (seqNum == 135){
                        //Start pathfinding call loop
                        pathfindingIsReady = true;
                        addCommand = false;
                    }else if (seqNum > 0 && moveLastIdx == 0){
                        //The start of this path was dropped, drop the rest of the path
                        addCommand = false;
                    }

                    if (addCommand){
                        //Get the number of ticks for angle and distance
                        int distanceTicks = CalculateDistanceFromPoints(startX, startY, endX, endY, CALCULATE_IN_CENTIMETERS);
                        //Negative angle means clockwise turn, positive angle means counterclockwise turn
                        int angleTicks = CalculateAngleFromPoints(startX, startY, endX, endY);
                        angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());

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
//                            float minRotation = deg2tickF(30);
//                            if (angleTicks >= minRotation || angleTicks <= (minRotation * -1)){
                                //One turn
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
//                            }else{
//                                //Two turns
//                                int deg90ticks = deg2tick(90);
//                                if (angleTicks < 0){
//                                    //Right turn
//                                    angleTicks *= -1;
//                                    moveAmount[moveLastIdx] = deg90ticks;
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
//                                    moveLastIdx++;
//                                    moveAmount[moveLastIdx] = (deg90ticks - angleTicks);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
//                                    moveLastIdx++;
//                                }else{
//                                    //Left turn
//                                    moveAmount[moveLastIdx] = deg90ticks;
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_LEFT;
//                                    moveLastIdx++;
//                                    moveAmount[moveLastIdx] = (deg90ticks - angleTicks);
//                                    moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
//                                    moveLastIdx++;
//                                }
//                            }
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
                            hasFlagLocation = true;
                            flagX = endX;
                            flagY = endY;
                        }
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
                
                
                //Handle color sensor communication state machine
                i2cCount++;
                if (i2cCount >= 50){
                    int currentState2 = DRV_TCS_HandleColorSensor(i2c1_handle, COLOR_SENSOR_ID_2);
                    i2cCount = 0;
                    Nop();
                }else if (i2cCount == 25){
                    int currentState1 = DRV_TCS_HandleColorSensor(i2c2_handle, COLOR_SENSOR_ID_1);
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

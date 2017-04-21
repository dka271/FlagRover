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

//Navigation communication interface functions
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

//State checker for line crossing
unsigned char testMsg[SEND_QUEUE_BUFFER_SIZE];
bool DoWeCrossLineQuestionMark(){
    //Do not cross any lines until orientation is found
    if (movementState < STATE_ORIENTATION_FOUND){
        return false;
    }
    //Decide if this is the right line to cross
    switch (locationState){
        case (CROSSED_0_LINES):{
            region center = regionList[CENTRAL_ZONE];
            unsigned char centerY = (center.y - (center.length >> 1)) << 1;
            unsigned char ourY = ((unsigned char) GetLocationY());
            
            //FOR TESTING, REMOVE LATER
            if (NAV_TESTING){
                sprintf(testMsg, "*Current Position = (%d,%d), Center Y = %d, State = 0~", (int) GetLocationX(), (int) GetLocationY(), centerY);
                commSendMsgToWifiQueue(testMsg);
            }
            //END TESTING SECTION
            
            if (ourY >= centerY - 10){
                locationState = CROSSED_1_LINES;
                flagRoverEnteredNewZone();
                return true;
            }else{
                return false;
            }
            break;
        }
        case (CROSSED_1_LINES):{
            locationState = CROSSED_2_LINES;
            
            //FOR TESTING, REMOVE LATER
            if (NAV_TESTING){
                sprintf(testMsg, "*Current Position = (%d,%d), State = 1~", (int) GetLocationX(), (int) GetLocationY());
                commSendMsgToWifiQueue(testMsg);
            }
            //END TESTING SECTION
            flagRoverEnteredNewZone();  
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
            if (NAV_TESTING){
                sprintf(testMsg, "*Current Position = (%d,%d), MinX = %d, MaxX = %d, Flag Zone Y = %d, State = 2~", (int) GetLocationX(), (int) GetLocationY(), minX, maxX, farFlagY);
                commSendMsgToWifiQueue(testMsg);
            }
            //END TESTING SECTION
                    
            if (ourY >= farFlagY - 20 && (ourX <= maxX && ourX >= minX)){
//                ElectromagnetSetOn();
//                LedSetOn();
                locationState = CROSSED_3_LINES;
                flagRoverEnteredNewZone();
                return true;
            }else{
                return false;
            }
            break;
        }
        case (CROSSED_3_LINES):{
            //FOR TESTING, REMOVE LATER
            if (NAV_TESTING){
                sprintf(testMsg, "*Current Position = (%d,%d), State = 3~", (int) GetLocationX(), (int) GetLocationY());
                commSendMsgToWifiQueue(testMsg);
            }
            //END TESTING SECTION
                    
            return false;
            break;
        }
    }
    return false;
}

/******************************************************************************
    Private Navigation related variables
 */
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
//Line Sensing stuff
int ignoringTape = 0;
int ignoreTapeCount = 0;
int ignoreTapeMax = 40;
bool cs1OnTape = false;
bool cs2OnTape = false;
//bool cs3OnTape = false;//color sensor 3 does not exist on the flag rover
//Orientation stuff
unsigned char orientAttempts = 0;
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

//This function is used to handle color sensor data sent to navigation
//This function is only for the flag rover
void HandleColorSensorData(unsigned char ColorSensorID){
    //FOR TESTING, REMOVE LATER
    region CenterZone = regionList[CENTRAL_ZONE];
    region SensorZone = regionList[CLOSE_SENSOR_ZONE];
    if (NAV_TESTING){
        sprintf(testMsg, "*Center: (%d, %d), %dx%d. Sensor: (%d, %d) %dx%d~",CenterZone.x, CenterZone.y, CenterZone.length, CenterZone.width, SensorZone.x, SensorZone.y, SensorZone.length, SensorZone.width);
        commSendMsgToWifiQueue(testMsg);
    }
    //END TESTING SECTION
    //These variables affect this function based on color sensor input
    bool csMeOnTape;
    bool csOtherOnTape;
    float orientTurn1;
    float orientTurn2;
    unsigned char turn1Direction;
    unsigned char turn2Direction;
    if (ColorSensorID == NAV_COLOR_SENSOR_1_ID){
        csMeOnTape = cs1OnTape;
        csOtherOnTape = cs2OnTape;
        orientTurn1 = 97.5;
        orientTurn2 = 94.6;
        turn1Direction = ROVER_DIRECTION_RIGHT;
        turn2Direction = ROVER_DIRECTION_LEFT;
    }else if (ColorSensorID == NAV_COLOR_SENSOR_2_ID){
        csMeOnTape = cs2OnTape;
        csOtherOnTape = cs1OnTape;
        orientTurn1 = 85.4;
        orientTurn2 = 82.5;
        turn1Direction = ROVER_DIRECTION_LEFT;
        turn2Direction = ROVER_DIRECTION_RIGHT;
    }
    
    if (movementState < STATE_ORIENTATION_FOUND && movementState > STATE_WAITING_FOR_GAME_START){
        //Handle orientation
        if (movementState == STATE_MOVING_TO_TAPE && csMeOnTape){
            //Found tape, try to orient to it
            ResetMovementQueue();
            StopMovement();
            movementState = STATE_ORIENTING_TO_TAPE;
            SetDirectionBackwards();
            unsigned char dir;
            if (INVERTED_X_AXIS){
                dir = turn1Direction;
            }else{
                dir = turn2Direction;
            }
            //Make a bunch of small turns instead of one large turn so that we can get oriented as accurately as possible
            AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
            AddMovement(deg2tick(20), dir);
            AddMovement(deg2tick(20), dir);
            AddMovement(deg2tick(20), dir);
            AddMovement(deg2tick(20), dir);
            AddMovement(deg2tick(20), dir);
            SetMovementGoal();
        }else if (movementState == STATE_ORIENTING_TO_TAPE && csMeOnTape){
            //Oriented to tape, wait for feedback
            ResetMovementQueue();
            StopMovement();
            orientAttempts++;
            if (csMeOnTape && csOtherOnTape){
                //We are aligned to the tape, wait for sensor feedback
                movementState = STATE_WAITING_FOR_SENSOR_FEEDBACK;
                sendTapeSignalToSensor();
            }else if (orientAttempts >= 4){
                //This orientation isn't working, back up and try again
                AddMovement(cm2tick(5), ROVER_DIRECTION_BACKWARDS);
                AddMovement(deg2tick(10), turn1Direction);
                GoToRandomLine(false);
                orientAttempts = 0;
            }else{
                unsigned char dir;
                if (INVERTED_X_AXIS){
                    dir = turn1Direction;
                }else{
                    dir = turn2Direction;
                }
                //Make a bunch of small turns instead of one large turn so that we can get oriented as accurately as possible
                AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                AddMovement(deg2tick(20), dir);
                AddMovement(deg2tick(20), dir);
                AddMovement(deg2tick(20), dir);
                AddMovement(deg2tick(20), dir);
                AddMovement(deg2tick(20), dir);
                SetMovementGoal();
            }
        }else if (movementState == STATE_WAITING_FOR_SENSOR_FEEDBACK && csMeOnTape){
            //We don't do anything in this state since we are waiting for sensor rover feedback
        }
    }else{
        //Decide whether or not to cross tape
        if (csMeOnTape && !ignoringTape){
            ResetMovementQueue();
            StopMovement();
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
                            if (INVERTED_X_AXIS){
                                AddMovement(angleTicks, turn1Direction);
                            }else{
                                AddMovement(angleTicks, turn2Direction);
                            }
                        }else{
                            //Left turn
                            if (INVERTED_X_AXIS){
                                AddMovement(angleTicks, turn2Direction);
                            }else{
                                AddMovement(angleTicks, turn1Direction);
                            }
                        }
                    }
                    AddMovement(cm2tick(10), ROVER_DIRECTION_FORWARDS);
                    ignoreTapeCount = -30;
                }else if (locationState == CROSSED_2_LINES){
                    AddMovement(cm2tick(8), ROVER_DIRECTION_FORWARDS);
                    ignoreTapeCount = -30;
                }else if (locationState == CROSSED_1_LINES){
                    //Orient ourselves towards the line
                    if (INVERTED_X_AXIS){
                        int angleTicks = deg2tickF(orientTurn1);
                        angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                        AddMovement(angleTicks, turn1Direction);
                    }else{
                        int angleTicks = deg2tickF(orientTurn2);
                        angleTicks = AdjustAngleToRotate(angleTicks, GetOrientation());
                        AddMovement(angleTicks, turn2Direction);
                    }
                    AddMovement(cm2tick(10), ROVER_DIRECTION_FORWARDS);
                    ignoreTapeCount = -40;
                }else{
                    AddMovement(cm2tick(20), ROVER_DIRECTION_FORWARDS);
                    ignoreTapeCount = 0;
                }
                SetMovementGoal();
                ignoringTape = 1;
            }else if (locationState == CROSSED_3_LINES && !ignoringTape){
                //Handle movement while in the flag zone
                if (csMeOnTape){
                    //Just got on tape
                    ResetMovementQueue();
                    StopMovement();
                    if (csOtherOnTape){
                        //Both on tape
                        //Drop the flag
                        ElectromagnetSetOff();
                        AddMovement(cm2tick(1), ROVER_DIRECTION_FORWARDS);
                        AddMovement(cm2tick(8), ROVER_DIRECTION_BACKWARDS);
                        SetMovementGoal();
                        ignoringTape = 1;
                        ignoreTapeCount = 0;
                    }else{
                        //This one on tape
                        if (INVERTED_X_AXIS){
                            AddMovement(deg2tick(90), turn1Direction);
                            SetMovementGoal();
                        }else{
                            AddMovement(deg2tick(90), turn2Direction);
                            SetMovementGoal();
                        }
                    }
                }
            }else if (csMeOnTape && !ignoringTape){
                //Back up
                AddMovement(cm2tick(4), ROVER_DIRECTION_BACKWARDS);
                SetMovementGoal();
                ignoringTape = 1;
                ignoreTapeCount = 20;
            }
        }
    }
}

void GoToRandomLine(bool rotate){
    //Go to a "random" line on the field, assuming no obstacles
    if (rotate){
        moveAmount[moveLastIdx] = deg2tick(105);
        moveType[moveLastIdx] = ROVER_DIRECTION_RIGHT;
        moveLastIdx++;
    }
    moveAmount[moveLastIdx] = in2tick(50);
    moveType[moveLastIdx] = ROVER_DIRECTION_FORWARDS;
    moveLastIdx++;
    moveGoalIdx = moveLastIdx;
    movementState = STATE_MOVING_TO_TAPE;
    orientAttempts = 0;
}

// *****************************************************************************
//Movement API functions

//Clears any path that is currently in the queue
void ResetMovementQueue(){
    moveCurrentIdx = 0;
    moveLastIdx = 0;
    moveGoalIdx = 0xff;
}

//Stops the current movement of the rover
void StopMovement(){
    ticksRemaining = 0;
    desiredSpeed = ROVER_SPEED_STOPPED;
}

//Adds a movement to the queue
void AddMovement(int tickAmount, int direction){
    moveAmount[moveLastIdx] = tickAmount;
    moveType[moveLastIdx] = direction;
    moveLastIdx++;
}

//Sets the goal to the movement that was just added
void SetMovementGoal(){
    moveGoalIdx = moveLastIdx;
}

//This function handles reading from the movement queue and starting movements
bool HandleMovementQueue(){
    bool toReturn = false;
    if (gameIsPaused){
        ResetMovementQueue();
        StopMovement();
        GoToRandomLine(true);
        return toReturn;
    }else{
        if (speed2 == 0){
            roverStopped++;
        }else{
            roverStopped = 0;
        }
    }
    if (roverStopped >= 10 && moveLastIdx > moveCurrentIdx){
        //There is a command, and the rover is not moving
        //Interpret the command and control the rover

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
            ticksRemaining = moveAmount[moveCurrentIdx];
            int minMove = in2tick(2);
            if (ticksRemaining <= minMove){
                desiredSpeed = ROVER_SPEED_SLOW;
            }else{
                desiredSpeed = ROVER_SPEED_STRAIGHT;
            }
            sprintf(testMsg, "*Command: Move Straight, Ticks = %d~", ticksRemaining);
        }else if (moveType[moveCurrentIdx] == ROVER_DIRECTION_BACKWARDS){
            SetDirectionBackwards();
            ticksRemaining = moveAmount[moveCurrentIdx];
            int minMove = in2tick(2);
            if (ticksRemaining <= minMove){
                desiredSpeed = ROVER_SPEED_SLOW;
            }else{
                desiredSpeed = ROVER_SPEED_STRAIGHT;
            }
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
        toReturn = true;

        //FOR TESTING, REMOVE LATER
        if (MOTOR_PATHFINDING_INTEGRATION_TESTING){
            sprintf(testMsg, "*Goal Reached, Current Position = (%d,%d), Current Orientation = %d, Motor Direction = %d~", (int) GetLocationX(), (int) GetLocationY(), (int) GetOrientation(), GetMotorDirection());
            commSendMsgToWifiQueue(testMsg);
        }
        //END TESTING SECTION

        //Handle not reaching a line during orientation
        if (movementState == STATE_MOVING_TO_TAPE){
            //Back up, then go to a new line
            moveAmount[moveLastIdx] = cm2tick(8);
            moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
            moveLastIdx++;
            GoToRandomLine(true);
        }

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
        if (sawFlagOnce){
            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1);
            sawFlagOnce = true;
        }
    }
    return toReturn;
}

/******************************************************************************
  Function:
    void NAVIGATION_Tasks ( void )
 */

unsigned char pathMsg[PATH_QUEUE_BUFFER_SIZE];

int TIMER=0;

void NAVIGATION_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_NAV_ENTER);
    
    //Location stuff
    movementState = STATE_WAITING_FOR_GAME_START;
    locationState = CROSSED_0_LINES;
    SetDirectionForwards();
    //I2C Initialization Stuff
    //Open the I2C
    int i2cCount = -100;//Set this low so the color sensors are guaranteed to receive power by the time we start initializing them
    while (DRV_I2C_Status(sysObj.drvI2C0) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
            sprintf(testMsg, "Waiting for I2C 1...");
            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c1_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
    while (DRV_I2C_Status(sysObj.drvI2C1) != SYS_STATUS_READY){
        //Wait for the I2C to be ready to be opened
        //FOR TESTING
        if (COLOR_SENSOR_SERVER_TESTING){
            sprintf(testMsg, "Waiting for I2C 2...");
            commSendMsgToWifiQueue(testMsg);
        }
        //END FOR TESTING
    }
    DRV_HANDLE i2c2_handle = DRV_I2C_Open(DRV_I2C_INDEX_1, DRV_IO_INTENT_READWRITE);
    //Init the I2C state machine
    DRV_TCS_HandleColorSensor(NULL, COLOR_SENSOR_RESET_STATE_MACHINE);
    //Open the ADC used for the flag sensing
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
                if (HandleMovementQueue()){
                    //Goal reached
                }
                
                //Handle remaining distance
                if (GetMotorDirection() == ROVER_DIRECTION_LEFT){
                    HandleDistanceRemaining(&desiredSpeed, &ticksRemaining, speed2);
                    
                    //Handle position and orientation
                    HandlePositionAndOrientation(speed2, GetMotorDirection(), CALCULATE_IN_CENTIMETERS);
                    //Send position to pathfinding thread
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
                
                //Call pathfinding every x seconds
                pathfindingCount++;
                if (pathfindingCount >= pathfindingCountMax && pathfindingIsReady && locationState != CROSSED_3_LINES && movementState >= STATE_ORIENTATION_FOUND){
                    unsigned char commMsg[COMM_QUEUE_BUFFER_SIZE];
                    sprintf(commMsg,"*{\"S\":\"w\",\"T\":\"f\",\"M\":\"g\",\"C\":1}~");
                    commMsg[COMM_SOURCE_ID_IDX] = COMM_UART_ID << COMM_SOURCE_ID_OFFSET;
                    commMsg[COMM_CHECKSUM_IDX] = commCalculateChecksum(commMsg);
                    commSendMsg(commMsg);
                    pathfindingCount = 0;
                    
                    //Update position
                    sprintf(commMsg,"*{\"S\":\"f\",\"T\":\"a\",\"M\":\"p\",\"N\":1,\"F\":[1,12,6,6,%d,%d,0],\"C\":1}~", (int) (GetLocationX() / 2), (int) (GetLocationY() / 2));
                    commSendMsgToWifiQueue(commMsg);
                }
                
                //Ignore tape for x seconds
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
                //Handle the color sensor result
                HandleColorSensorData(NAV_COLOR_SENSOR_1_ID);
                
                //FOR TESTING
                if (COLOR_SENSOR_SERVER_TESTING){
                    sprintf(testMsg, "*{Message from Color Sensor 1: %d}~", receivemsg[0]);
//                    commSendMsgToWifiQueue(testMsg);
                }
                //END FOR TESTING
            }else if (msgId == NAV_COLOR_SENSOR_2_ID){
                //Handle stuff from color sensor 2 (front right)
                if (receivemsg[0] == COLOR_IS_BLUE){
                    cs2OnTape = true;
                }else{
                    cs2OnTape = false;
                }
                //Handle color sensor data
                HandleColorSensorData(NAV_COLOR_SENSOR_2_ID);
                
                //FOR TESTING
                if (COLOR_SENSOR_SERVER_TESTING){
                    sprintf(testMsg, "*{Message from Color Sensor 2: %d}~", receivemsg[0]);
//                    commSendMsgToWifiQueue(testMsg);
                }
                //END FOR TESTING
            }else if (msgId == NAV_PIXY_CAM_ID){
                //Handle messages with pixy cam information
                if (NAV_TESTING){
                    sprintf(testMsg, "*{Pixy Cam Value = %d}~",receivemsg[0] + (receivemsg[1] << 8));
                    commSendMsgToWifiQueue(testMsg);
                }
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
                        if (movementState >= STATE_WAITING_FOR_SENSOR_FEEDBACK){
                            //Kill any path
                            moveCurrentIdx = 0;
                            moveLastIdx = 0;
                            moveGoalIdx = 0xff;
                            ticksRemaining = 0;
                            desiredSpeed = ROVER_SPEED_STOPPED;
                            //Update the position
                            SetLocationX(startX);
                            SetLocationY(startY);
                            //Update the orientation
                            bool orientationReceived = false;
                            if (movementState == STATE_WAITING_FOR_SENSOR_FEEDBACK){
                                region CenterZone = regionList[CENTRAL_ZONE];
                                region SensorZone = regionList[CLOSE_SENSOR_ZONE];
                                region OurDefenseZone = regionList[CLOSE_DEFENSE_ZONE];
                                region TheirDefenseZone = regionList[FAR_DEFENSE_ZONE];
                                if (IDENTITY_OF_THIS_ROVER == IDENTITY_OF_FRIENDLY_FLAG_ROVER){
                                    if (startX >= 14 && startX <= ((CenterZone.length - 7) << 1) && startY >= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On center zone
                                        SetOrientation(90);
                                        orientationReceived = true;
                                    }else if (startX <= 14 && startY >= 30 && startY <= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On close sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(180);
                                        }else{
                                            SetOrientation(0);
                                        }
                                        orientationReceived = true;
                                    }else if (startX >= ((CenterZone.length - 7) << 1) && startY >= 30 && startY <= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On far sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(0);
                                        }else{
                                            SetOrientation(180);
                                        }
                                        orientationReceived = true;
                                    }
                                }else if (IDENTITY_OF_THIS_ROVER == IDENTITY_OF_FRIENDLY_TAGGER_ROVER){
                                    if (startX >= 14 && startX <= ((CenterZone.length - 7) << 1) && startY >= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On center zone
                                        SetOrientation(90);
                                        orientationReceived = true;
                                    }else if (startX <= 14 && startY >= 30 && startY <= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On close sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(180);
                                        }else{
                                            SetOrientation(0);
                                        }
                                        orientationReceived = true;
                                    }else if (startX >= ((CenterZone.length - 7) << 1) && startY >= 30 && startY <= ((CenterZone.y - (CenterZone.width + 4)) << 1)){
                                        //On far sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(0);
                                        }else{
                                            SetOrientation(180);
                                        }
                                        orientationReceived = true;
                                    }
                                }else if (IDENTITY_OF_THIS_ROVER == IDENTITY_OF_FRIENDLY_COUNTER_ROVER){
                                    if (startX >= 14 && startX <= ((CenterZone.length - 7) << 1) && startY <= ((TheirDefenseZone.y << 1))){
                                        //On center zone
                                        SetOrientation(-90);
                                        orientationReceived = true;
                                    }else if (startX <= 14 && startY <= ((TheirDefenseZone.y << 1) + 10) && startY >= ((CenterZone.y + (CenterZone.width + 4)) << 1)){
                                        //On close sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(180);
                                        }else{
                                            SetOrientation(0);
                                        }
                                        orientationReceived = true;
                                    }else if (startX >= ((CenterZone.length - 7) << 1) && startY <= ((TheirDefenseZone.y << 1) + 10) && startY >= ((CenterZone.y + (CenterZone.width + 4)) << 1)){
                                        //On far sensor zone
                                        if (INVERTED_X_AXIS){
                                            SetOrientation(0);
                                        }else{
                                            SetOrientation(180);
                                        }
                                        orientationReceived = true;
                                    }
                                }else{
//                                    SetOrientation(endX);
//                                    orientationReceived = true;
                                }
                        
                                if (orientationReceived){
                                    //This was the feedback we were looking for, we can do stuff now
                                    movementState = STATE_ORIENTATION_FOUND;
                                    //Back up so we aren't on the line when we start pathfinding
                                    moveAmount[moveLastIdx] = cm2tick(6);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                                    moveLastIdx++;
                                    moveGoalIdx = moveLastIdx;
                                    pathfindingCount = 0;
                                }else{
                                    //We are probably in a corner
                                    //Back up, then go to a new line
                                    moveAmount[moveLastIdx] = cm2tick(8);
                                    moveType[moveLastIdx] = ROVER_DIRECTION_BACKWARDS;
                                    moveLastIdx++;
                                    GoToRandomLine(true);
                                }
                            }else{
//                                SetOrientation(endX);
                            }
                        }
                        addCommand = false;
                    }else if (seqNum == 135){
                        //Start pathfinding call loop
                        pathfindingIsReady = true;
                        movementState = STATE_MOVING_TO_TAPE;
                        locationState = CROSSED_0_LINES;
                        flagRoverEnteredNewZone();
                        //Choose a direction and go until we hit a line
                        //This is the first movement, so we can just go straight
                        GoToRandomLine(false);
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

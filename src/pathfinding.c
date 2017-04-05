/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pathfinding.c

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "pathfinding.h"
#include "pathfinding_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

PATHFINDING_DATA pathfindingData;

static QueueHandle_t pathQueue;
fieldItem fieldItemStack[MAXIMUM_NUMBER_OF_OBSTACLES];
crossSquare crossSquareStack[MAXIMUM_NUMBER_OF_OBSTACLES];
unsigned char fieldItemStackTop = 0;
unsigned char crossSquareStackTop = 0;
adjacencyList halfHeartedAdjacencyList;
int currentNumberOfNodes = 2;
pathFindingNode pathFindingNodeList[MAXIMUM_NUMBER_OF_NODES];
unsigned char closedSet[MAXIMUM_NUMBER_OF_NODES];
unsigned char openSet[MAXIMUM_NUMBER_OF_NODES];
unsigned char openSetTop = 0;
unsigned short gScoreArray[MAXIMUM_NUMBER_OF_NODES];
unsigned short fScoreArray[MAXIMUM_NUMBER_OF_NODES];
unsigned char cameFrom[MAXIMUM_NUMBER_OF_NODES];


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PATHFINDING_Initialize ( void )

  Remarks:
    See prototype in pathfinding.h.
 */
unsigned char uniqueIDMapper(unsigned char startingID) {
    if (IDENTITY_OF_THIS_ROVER == 1) { //1 is for the flag rover
        //Numbers designed for flag rover, so they don't need to change
        return startingID;
    } else if (IDENTITY_OF_THIS_ROVER == 3) { //3 is for the tagger rover
        if (startingID == IDENTITY_OF_FRIENDLY_FLAG) {
            return IDENTITY_OF_ENEMY_FLAG_ROVER;
        } else if (startingID == IDENTITY_OF_FRIENDLY_FLAG_ROVER) {
            return IDENTITY_OF_FRIENDLY_TAGGER_ROVER;
        } else if (startingID == IDENTITY_OF_FRIENDLY_TAGGER_ROVER) {
            return IDENTITY_OF_FRIENDLY_FLAG_ROVER;
        } else if (startingID == IDENTITY_OF_ENEMY_FLAG_ROVER) {
            return IDENTITY_OF_FRIENDLY_FLAG;
        } else { //No need to swap other elements
            return startingID;
        }
    } else if (IDENTITY_OF_THIS_ROVER == 4) { //4 is for the countermeasure rover
        if (startingID == IDENTITY_OF_FRIENDLY_FLAG) {
            return IDENTITY_OF_ENEMY_SENSOR_ROVER;
        } else if (startingID == IDENTITY_OF_ENEMY_SENSOR_ROVER) {
            return IDENTITY_OF_FRIENDLY_FLAG;
        } else if (startingID == IDENTITY_OF_FRIENDLY_FLAG_ROVER) {
            return IDENTITY_OF_FRIENDLY_COUNTER_ROVER;
        } else if (startingID == IDENTITY_OF_FRIENDLY_COUNTER_ROVER) {
            return IDENTITY_OF_FRIENDLY_FLAG;
        } else { //No need to swap other elements
            return startingID;
        }
    }
}

void PATHFINDING_Initialize(void) {
    /* Place the App state machine in its initial state. */
    pathfindingData.state = PATHFINDING_STATE_INIT;

    //Initialize the navigation queue
    
    pathQueue = xQueueCreate(10, sizeof (unsigned char[PATH_QUEUE_BUFFER_SIZE]));
    if (pathQueue == 0) {
        dbgPauseAll();
    }


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void pathSendMsg(unsigned char msg[PATH_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBack(pathQueue, msg, portMAX_DELAY);
}

unsigned char pathCalculateChecksum(unsigned char msg[PATH_QUEUE_BUFFER_SIZE]) {
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < PATH_QUEUE_BUFFER_SIZE - 1; i++) {
        sum += msg[i];
    }
    return sum;
}

crossSquare convertFieldItemToCrossSquare(fieldItem tempFieldItem) {
    short SlowX = tempFieldItem.centerX - (tempFieldItem.width / 2)-2;
    short ShighX = tempFieldItem.centerX + (tempFieldItem.width / 2)+2;
    short SlowY = tempFieldItem.centerY - (tempFieldItem.length / 2)-2;
    short ShighY = tempFieldItem.centerY + (tempFieldItem.length / 2)+2;
    
    unsigned short lowX; 
    unsigned short highX; 
    unsigned short lowY; 
    unsigned short highY;
    
    if (SlowX <= 0) {
        lowX = 0;
    } else {
        lowX = SlowX;
    }
    
    if (ShighX <= 0) {
        highX = 0;
    } else {
        highX = ShighX;
    }
    
    if (SlowY <= 0) {
        lowY = 0;
    } else {
        lowY = SlowY;
    }
    
    if (ShighY <= 0) {
        highY = 0;
    } else {
        highY = ShighY;
    }
    
    point topLeft = convertXAndYToPoint(highX, highY);
    point bottomRight = convertXAndYToPoint(lowX, lowY);
    return convertPointsToCrossSquare(topLeft, bottomRight);
}

//Takes an x and y and returns a point struct
point convertXAndYToPoint(unsigned char x, unsigned char y) {
    point tempPoint;
    tempPoint.x = x;
    tempPoint.y = y;
    return tempPoint;
}

//Converts two points into a crossSquare
crossSquare convertPointsToCrossSquare(point topLeft, point bottomRight) {
    crossSquare tempCrossSquare;
    tempCrossSquare.topLeft = topLeft;
    tempCrossSquare.bottomRight = bottomRight;
    return tempCrossSquare;
}

//Checks if a fieldItem with the same "objectType" is already in the stack
//Returns -1 if it's not in there, returns in location of the object in the list if it is
int checkIfObjectInStack(unsigned char objectType) {
    int i;

    for (i = 0; i < fieldItemStackTop; i++) {
        if (fieldItemStack[i].objectType == objectType) {
            return i;
        }
    }
    return -1;
}

//Places a fieldItem on the stack, and calculates the respective crossSquare and also places that on a stack//RIGHTHERE
void storeInFieldItemStack(fieldItem tempFieldItem) {
    int updateLoc = checkIfObjectInStack(tempFieldItem.objectType);
    
    //-1 means it was not in the stack
    if (updateLoc == -1) {
        fieldItemStack[fieldItemStackTop] = tempFieldItem;
        fieldItemStackTop++;
        if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG)) {
            //do nothing so nothing is added to crossSquare stack
        } else if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG_ROVER)) {
            Nop();
            sendLocToNavigationThread(tempFieldItem.centerX, tempFieldItem.centerY, (tempFieldItem.orientation >> 1));
        } else {
            crossSquareStack[crossSquareStackTop] = convertFieldItemToCrossSquare(tempFieldItem);
            crossSquareStackTop++;
        }
    } else {
        fieldItemStack[updateLoc] = tempFieldItem;
        if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG)) {
            //do nothing so nothing is updated in the crossSquare stack
        } else if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG_ROVER)) {
            Nop();
            sendLocToNavigationThread(tempFieldItem.centerX, tempFieldItem.centerY, (tempFieldItem.orientation >> 1));
        } else {
            crossSquareStack[updateLoc] = convertFieldItemToCrossSquare(tempFieldItem);
        }
    }
}

//Calculates the offset nodes and places them in the adjacency list
void calculateOffsetNodes(fieldItem tempFieldItem) {
    point topLeftPoint;
    point topRightPoint;
    point bottomLeftPoint;
    point bottomRightPoint;
    int offset = NODE_OFFSET;
    if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG_ROVER)) { //This is the flag rover itself CHANGE THIS FOR OTHER ROVERS
        point singleNode;
        singleNode.x = tempFieldItem.centerX;
        singleNode.y = tempFieldItem.centerY;
        halfHeartedAdjacencyList.nodes[0] = singleNode;
        
                //Calculate the four cross points offset from the rover itself
//        if (tempFieldItem.centerX >= ((tempFieldItem.width / 2) + (offset*2))) {
//            topLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - (offset*2);
//            topLeftPoint.y = tempFieldItem.centerY;
//            if (checkIfNodeValid(topLeftPoint)) {
//                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topLeftPoint;
//                currentNumberOfNodes++;
//            }
//        }
////        if (tempFieldItem.centerX <= (255 - (tempFieldItem.width / 2) + (offset*2))) {
////            topRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + (offset*2);
////            topRightPoint.y = tempFieldItem.centerY;
////            if (checkIfNodeValid(topRightPoint)) {
////                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topRightPoint;
////                currentNumberOfNodes++;
////            }
////        }
//
//        if (tempFieldItem.centerY >= ((tempFieldItem.length / 2) + (offset*2))) {
//            bottomLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - (offset*2);
//            bottomLeftPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - (offset*2);
//            if (checkIfNodeValid(bottomLeftPoint)) {
//                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomLeftPoint;
//                currentNumberOfNodes++;
//            }
//        }
//
//        if (tempFieldItem.centerY >= ((tempFieldItem.length / 2) + (offset*2))) {
//            bottomRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + (offset*2);
//            bottomRightPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - (offset*2);
//            if (checkIfNodeValid(bottomRightPoint)) {
//                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomRightPoint;
//                currentNumberOfNodes++;
//            }
//        }
        
    } else if (tempFieldItem.objectType == uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG)) { 
        point singleNode;
        singleNode.x = tempFieldItem.centerX;
        singleNode.y = tempFieldItem.centerY;
        halfHeartedAdjacencyList.nodes[1] = singleNode;
        
//        //Calculate the four cross points offset from the rover itself
//        if (tempFieldItem.centerX >= ((tempFieldItem.width / 2) + (offset*2))) {
//            topLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - (offset*2);
//            topLeftPoint.y = tempFieldItem.centerY;
//            if (checkIfNodeValid(topLeftPoint)) {
//                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topLeftPoint;
//                currentNumberOfNodes++;
//            }
//        }
//        if (tempFieldItem.centerX <= (255 - (tempFieldItem.width / 2) + (offset*2))) {
//            topRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + (offset*2);
//            topRightPoint.y = tempFieldItem.centerY;
//            if (checkIfNodeValid(topRightPoint)) {
//                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topRightPoint;
//                currentNumberOfNodes++;
//            }
//        }
//
        if (tempFieldItem.centerY >= ((tempFieldItem.length / 2) + offset)) {
            bottomLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - (offset/2);
            bottomLeftPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - (offset/2);
            if (checkIfNodeValid(bottomLeftPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomLeftPoint;
                currentNumberOfNodes++;
            }
        }
        
        if (tempFieldItem.centerX <= (255 - (tempFieldItem.width / 2) - offset) && tempFieldItem.centerY >= ((tempFieldItem.length / 2) + offset)) {
            bottomRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + (offset/2);
            bottomRightPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - (offset/2);
            if (checkIfNodeValid(bottomRightPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomRightPoint;
                currentNumberOfNodes++;
            }
        }
        
    }else {
    
        if (tempFieldItem.objectType >= 2 && tempFieldItem.objectType < 8) {
            offset *= 2;
        } else if (tempFieldItem.objectType >= 8) {
            //do nothing, offset is set correctly for this case (NODE_OFFSET)
        }

        //Calculate the four points (width + OFFSET) or (width + 2*OFFSET)
        if (tempFieldItem.centerX >= ((tempFieldItem.width / 2) + offset) && tempFieldItem.centerY <= (255 - (tempFieldItem.length / 2) - offset)) {
            topLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - offset;
            topLeftPoint.y = tempFieldItem.centerY + (tempFieldItem.length / 2) + offset;
            if (checkIfNodeValid(topLeftPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topLeftPoint;
                currentNumberOfNodes++;
            }
        }
        if (tempFieldItem.centerX <= (255 - (tempFieldItem.width / 2) + offset) && tempFieldItem.centerY <= (255 - (tempFieldItem.length / 2) - offset)) {
            topRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + offset;
            topRightPoint.y = tempFieldItem.centerY + (tempFieldItem.length / 2) + offset;
            if (checkIfNodeValid(topRightPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = topRightPoint;
                currentNumberOfNodes++;
            }
        }

        if (tempFieldItem.centerX >= ((tempFieldItem.width / 2) + offset) && tempFieldItem.centerY >= ((tempFieldItem.length / 2) + offset)) {
            bottomLeftPoint.x = tempFieldItem.centerX - (tempFieldItem.width / 2) - offset;
            bottomLeftPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - offset;
            if (checkIfNodeValid(bottomLeftPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomLeftPoint;
                currentNumberOfNodes++;
            }
        }

        if (tempFieldItem.centerX <= (255 - (tempFieldItem.width / 2) - offset) && tempFieldItem.centerY >= ((tempFieldItem.length / 2) + offset)) {
            bottomRightPoint.x = tempFieldItem.centerX + (tempFieldItem.width / 2) + offset;
            bottomRightPoint.y = tempFieldItem.centerY - (tempFieldItem.length / 2) - offset;
            if (checkIfNodeValid(bottomRightPoint)) {
                halfHeartedAdjacencyList.nodes[currentNumberOfNodes] = bottomRightPoint;
                currentNumberOfNodes++;
            }
        }
    }
}

//Calls the calculation of all nodes (4 at a time) for each fieldItem on the stack
void fillAdjacencyNodes() {
    int i;
    for (i = 0; i < fieldItemStackTop; i++) {
        calculateOffsetNodes(fieldItemStack[i]);
    }
}

//Returns false if a node is in an illegal location (within an object) or true otherwise
bool checkIfNodeValid(point tempPoint) {
    //Check for intersections or within bounds of objects
    int i;
    if (tempPoint.x > MAX_WIDTH_OF_FIELD || tempPoint.y > MAX_LENGTH_OF_FIELD) {
        return false;
    }
    for (i = 0; i < fieldItemStackTop; i++) {
        if (checkIfPointWithinFieldItem(tempPoint, fieldItemStack[i])) {
            return false;
        }
    }
    return true;
}

//Returns true if a point is within a bounding box of the fieldItem passed in
bool checkIfPointWithinFieldItem(point pointToCheck, fieldItem tempFieldItem) {
    unsigned char minX;
    unsigned char maxX;
    unsigned char minY;
    unsigned char maxY;
    minX = tempFieldItem.centerX - (tempFieldItem.width / 2);
    maxX = tempFieldItem.centerX + (tempFieldItem.width / 2);
    minY = tempFieldItem.centerY - (tempFieldItem.length / 2);
    maxY = tempFieldItem.centerY + (tempFieldItem.length / 2);
    if (pointToCheck.x <= maxX && pointToCheck.x >= minX && pointToCheck.y <= maxY && pointToCheck.y >= minX) {
        return true;
    }
    return false;
}

//Null fill the adjacency list
void clearAdjacencyList() {
    int i;
    int j;
    for (i = 0; i < currentNumberOfNodes; i++) {
        for (j = 0; j < MAXIMUM_NUMBER_OF_IN_SIGHT_NODES; j++) {
            halfHeartedAdjacencyList.edges[i][j] = '\0';
        }
    }
    currentNumberOfNodes = 2;
}

//Returns the maximum of the two numbers passed in
unsigned char getMax(unsigned char num1, unsigned char num2) {
    if (num1 > num2) {
        return num1;
    }
    return num2;
}

//Returns the minimum of the two numbers passed in
unsigned char getMin(unsigned char num1, unsigned char num2) {
    if (num1 > num2) {
        return num2;
    }
    return num1;
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(point p, point q, point r) {
    if (q.x <= getMax(p.x, r.x) && q.x >= getMin(p.x, r.x) && q.y <= getMax(p.y, r.y) && q.y >= getMin(p.y, r.y)) {
        return true;
    }
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(point p, point q, point r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) {
        return 0;  // collinear
    }
 
    return (val > 0)? 1: 2; // clock or counterclock wise
}

//Calls the line intersection function on each part of a crossSquare (p2 = topLeft, q2 = bottomRight)
bool checkIntersectionOfLineAndCrossSquare(point p1, point q1, point p2, point q2) {
    if (p2.x == NULL || p2.y == NULL || q2.x == NULL || q2.y == NULL) {
        return true;
    }
    if (checkIntersectionOfTwoLines(p1, q1, p2, q2)) {
        return true;
    }
    point bottomLeft;
    point topRight;
    bottomLeft.x = p2.x;
    bottomLeft.y = q2.y;
    
    topRight.x = q2.x;
    topRight.y = p2.y;
    
    if (checkIntersectionOfTwoLines(p1, q1, bottomLeft, topRight)) {
        return true;
    }
    return false;
}

//Returns true if two lines intersect
bool checkIntersectionOfTwoLines(point p1, point q1, point p2, point q2) {
// Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}

//Returns true if the line of sight between two nodes does not intersect with any objects
bool checkIfTwoNodesInSightOfEachOther(point point1, point point2) {
    int i;
    if (point1.x == NULL || point1.y == NULL || point2.x == NULL || point2.y == NULL) {
        return false;
    }
    
    for (i = 0; i < crossSquareStackTop; i++) { //subtract 2
        if (checkIntersectionOfLineAndCrossSquare(point1, point2, crossSquareStack[i].topLeft, crossSquareStack[i].bottomRight)) {
            return false;
        }
    }
    return true;
}

//Controller function that organizes function calls to create the adjacency edges
void createAdjacencyEdges() {
    unsigned char i;
    unsigned char j;
    unsigned char startingNumberOfEdges = 0;

    unsigned char curNumEdges;
    for (i = 0; i < currentNumberOfNodes; i++) {
        while (1) {
            if (halfHeartedAdjacencyList.edges[i][startingNumberOfEdges] == '\0') {
                break;
            } else {
                startingNumberOfEdges++;
            }
        }
        
        curNumEdges = startingNumberOfEdges;
        startingNumberOfEdges = 0;
        for (j = i; j < currentNumberOfNodes; j++) {

            if (i == j) {
                //Do nothing on purpose
            } else if (checkIfTwoNodesInSightOfEachOther(halfHeartedAdjacencyList.nodes[i], halfHeartedAdjacencyList.nodes[j])) {
                halfHeartedAdjacencyList.edges[i][curNumEdges] = j;
                while (1) {
                    if (halfHeartedAdjacencyList.edges[j][startingNumberOfEdges] == '\0') {
                        break;
                    } else {
                        startingNumberOfEdges++;
                    }
                }
                halfHeartedAdjacencyList.edges[j][startingNumberOfEdges] = i;
                startingNumberOfEdges = 0;
                curNumEdges++;
            }
        }
    }
}

//For testing purposes only.
//Sends out all current adjacency list edges in JSON format over wifly. One message per edge.
void testingSendOutEdgesOverWifly() {
    int i;
    int j;
    for (i=0; i < currentNumberOfNodes; i++) {
        j=0;
        while(1) {
            if (halfHeartedAdjacencyList.edges[i][j] == '\0') {
                break;
            } else {
//                testingSendEdgeWithCoordinatesOverWifly(i, halfHeartedAdjacencyList.edges[i][j], halfHeartedAdjacencyList.nodes[i], halfHeartedAdjacencyList.nodes[j]);
                testingSendEdgeOverWifly(i, halfHeartedAdjacencyList.edges[i][j]);
                j++;
            }
        }
    }
}

// Controller function that calls necessary functions in order to create adjacency list
void calculateAdjacencyList() {
    clearAdjacencyList();
    fillAdjacencyNodes();
    createAdjacencyEdges();
//    testingSendOutEdgesOverWifly();
}

//Resets all private AStar related structures before running the algorithm
void clearAStarLists(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES]) {
    int i;
    for (i=0; i < MAXIMUM_NUMBER_OF_NODES; i++) {
        closedSet[i] = 0;
        openSet[i] = '\0';
        gScoreArray[i] = 65535;
        fScoreArray[i] = 65535;
        cameFrom[i] = 123;
    }
    for(i=0; i < MAXIMUM_NUMBER_OF_IN_SIGHT_NODES; i++) {
        path[i] = END_OF_PATH_NUM;
    }
    openSetTop = 0;
}

//Calculates the euclidean distance between two nodes (takes in the location of the node in the adjacency list))
unsigned short calculateEuclideanDistance(unsigned char starting, unsigned char ending) {
    int curTotal;
    point startingPoint = halfHeartedAdjacencyList.nodes[starting];
    point endingPoint = halfHeartedAdjacencyList.nodes[ending];
    curTotal = sqrt(pow(startingPoint.x - endingPoint.x, 2) + pow(startingPoint.y - endingPoint.y, 2));
    if (curTotal > 65535) {
        return 65535;
    } else {
        return (unsigned short)curTotal;
    }
}

//Returns true if the open set is empty
bool isUnsortedOpenSetEmpty() {
    int i;
    for (i=0; i < openSetTop; i++) {
        if (openSet[i] != 0) {
            return false;
        }
    }
    //This means 0 is the only element in the openSet
    if (isClosedSetEmpty()) {
        return false;
    }
    return true;
}

//returns true if the closed set is empty
bool isClosedSetEmpty() {
    int i;
    for (i=0; i < openSetTop; i++) {
        if (closedSet[i] != 0) {
            return false;
        }
    }
    return true;
}

//Returns true if the passed in node is already in the closed set
bool isNodeInClosedSet(unsigned char node) {
    if (closedSet[node]) {
        return true;
    }
    return false;
}

//Returns true if the passed in node is already in the open set
bool isNodeInOpenSet(unsigned char node) {
    int i;
    for (i=0; i < openSetTop; i++) {
        if (openSet[i] == node) {
            return true;
        }
    }
    return false;
}

//Returns the node with the lowest F score currently in the open set (returns the location of the node in the adjacency list)
unsigned char getLowestFScore() {
    int currentLowestValue = 65537;
    unsigned char currentLowestNode = 0;
    
    int i;
    for (i=0; i < openSetTop; i++) {
        if (openSet[i] == '\0') {
            continue;
        }
        if (fScoreArray[openSet[i]] < currentLowestValue) {
            currentLowestValue = fScoreArray[openSet[i]];
            currentLowestNode = openSet[i];
        }
    }
    return currentLowestNode;
}

//Removes a node from the open set
void removeFromOpenSet(unsigned char current) {
    int i;
    for (i=0; i < openSetTop; i++) {
        if (openSet[i] == current) {
            openSet[i] = '\0';
            break;
        }
    }
}

//Adds a node to the closed set
void addToClosedSet(unsigned char current) {
    closedSet[current] = 1; //1 means it is in the list
}

//Adds a node to the open set
void addToOpenSet(unsigned char current) {
    openSet[openSetTop] = current;
    openSetTop++;
}

//Returns the number of edges a node has (or the number of nodes it is connected to as they are equivalent)
unsigned char getNumberOfNeighborsOfNode(unsigned char node) {
    unsigned char i=0;
    while(1) {
        if (halfHeartedAdjacencyList.edges[node][i] == '\0') {
            break;
        }
        i++;
    }
    return i;
}

//Walks the final path backwards constructing the final path
void reconstructPath(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES], unsigned char current) {
    unsigned char totalPathTop=0;
    unsigned char loc = current;
    while (loc != 0) {
        path[totalPathTop] = loc;
        totalPathTop++;
        loc = cameFrom[loc];
    }
    path[totalPathTop] = loc;
}

//Returns 1 on failure, 0 upon success
unsigned char monotonicAStar(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES], unsigned char start, unsigned char goal) {
    clearAStarLists(path);
    openSetTop = 1; //Reset top of openSet stack
    openSet[start] = start;
    gScoreArray[start] = 0; //Set the start gScore to 0
    fScoreArray[start] = calculateEuclideanDistance(start, goal);
    
    unsigned char current;
    while(!isUnsortedOpenSetEmpty()) {
        current = getLowestFScore();
        
        if (current == goal) {
            reconstructPath(path, current);
            return 0;
        }
        
        removeFromOpenSet(current);
        addToClosedSet(current);
        
        unsigned char numNeighbors = getNumberOfNeighborsOfNode(current);
        
        unsigned char i;
        for (i=0; i < numNeighbors; i++) {
            unsigned char neighbor = halfHeartedAdjacencyList.edges[current][i];
            
            if (isNodeInClosedSet(neighbor)) {
                continue;
            }
            
            int tentative_gScore = gScoreArray[current] + calculateEuclideanDistance(current, neighbor);
            if (!isNodeInOpenSet(neighbor)) {
                addToOpenSet(neighbor);
            } else if (tentative_gScore >= gScoreArray[neighbor]) {
                continue;
            }
            
            cameFrom[neighbor] = current;
            gScoreArray[neighbor] = tentative_gScore;
            fScoreArray[neighbor] = gScoreArray[neighbor] + calculateEuclideanDistance(neighbor, goal);
        }
    }
    return 1;
}

//Returns 1 upon failure, 0 upon success
unsigned char calculatePath() {
    if (halfHeartedAdjacencyList.nodes[0].x == NULL || halfHeartedAdjacencyList.nodes[0].y == NULL || halfHeartedAdjacencyList.nodes[1].x == NULL || halfHeartedAdjacencyList.nodes[1].y == NULL) {
        Nop();
        return 1;
    }
    unsigned char start = 0;
    unsigned char goal = 1;
    unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES];

    //If statement runs if AStar fails
    if (monotonicAStar(path, start, goal)) {
//        testingSendPathOverWifly(path);
        Nop();
        return 1;
    }
//    testingSendPathOverWifly(path);
    Nop();
    sendPathToNavigationThread(path);
    return 0;
}

//Sends the path (one edge at a time) to the navigation queue
void sendPathToNavigationThread(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES]) {
    unsigned char numNodesInPath;
    Nop();
    for (numNodesInPath=0; numNodesInPath < MAXIMUM_NUMBER_OF_IN_SIGHT_NODES; numNodesInPath++) {
        if (path[numNodesInPath] == END_OF_PATH_NUM) {
            break;
        }
    }
    Nop();
    unsigned char i;
//    testingSendEdgeOverWifly(numNodesInPath, numNodesInPath);
//    for (i=(numNodesInPath-1); i > 1; i--) {
    for (i=(numNodesInPath-1); i >= 2; i--) {
        Nop();
        sendEdgeToNavigationThread(numNodesInPath-i-1, halfHeartedAdjacencyList.nodes[path[i]].x, halfHeartedAdjacencyList.nodes[path[i]].y, halfHeartedAdjacencyList.nodes[path[i-1]].x, halfHeartedAdjacencyList.nodes[path[i-1]].y);
//        sendEdgeToNavigationThread(numNodesInPath-i-1, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i]].x, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i]].y, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i-1]].x, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i-1]].y);
//        testingSendEdgeOverWifly(halfHeartedAdjacencyList.nodes[path[i]].x, halfHeartedAdjacencyList.nodes[path[i]].y);
//        testingSendEdgeOverWifly(halfHeartedAdjacencyList.nodes[path[i-1]].x, halfHeartedAdjacencyList.nodes[path[i-1]].y);
    }
    sendEdgeToNavigationThread(END_OF_PATH_NUM, halfHeartedAdjacencyList.nodes[path[1]].x, halfHeartedAdjacencyList.nodes[path[1]].y, halfHeartedAdjacencyList.nodes[path[0]].x, halfHeartedAdjacencyList.nodes[path[0]].y);
//    sendEdgeToNavigationThread(END_OF_PATH_NUM, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i]].x, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i]].y, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i-1]].x, halfHeartedAdjacencyList.nodes[path[numNodesInPath-i-1]].y);
//    testingSendEdgeOverWifly(halfHeartedAdjacencyList.nodes[path[1]].x, halfHeartedAdjacencyList.nodes[path[1]].y);
//    testingSendEdgeOverWifly(halfHeartedAdjacencyList.nodes[path[0]].x, halfHeartedAdjacencyList.nodes[path[0]].y);
}


/******************************************************************************
  Function:
    void PATHFINDING_Tasks ( void )

  Remarks:
    See prototype in pathfinding.h.
 */

int PRIVATEDELETEME = 0;
point currentGoal;

void PATHFINDING_Tasks(void) {
    dbgOutputLoc(DBG_LOC_PATH_ENTER);

    /* Check the application's current state. */
    switch (pathfindingData.state) {
            /* Application's initial state. */
        case PATHFINDING_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized) {

                pathfindingData.state = PATHFINDING_STATE_SERVICE_TASKS;
                
            }
            break;
        }

        case PATHFINDING_STATE_SERVICE_TASKS:
        {
            unsigned char receivemsg[PATH_QUEUE_BUFFER_SIZE];

            dbgOutputLoc(DBG_LOC_PATH_BEFORE_WHILE);
            MAX_LENGTH_OF_FIELD = 61;//110;
            MAX_WIDTH_OF_FIELD = 26;//64;
            while (1) {
                //Block until a message is received
                dbgOutputLoc(DBG_LOC_PATH_BEFORE_RECEIVE);
                BaseType_t receiveCheck = xQueueReceive(pathQueue, receivemsg, portMAX_DELAY);
                dbgOutputLoc(DBG_LOC_PATH_AFTER_RECEIVE);
                

                //Handle the message
                if (receiveCheck == pdTRUE) {
                    //Get the message ID
                    int msgId = (receivemsg[PATH_SOURCE_ID_IDX] & PATH_SOURCE_ID_MASK) >> PATH_SOURCE_ID_OFFSET;

                    //Handle a specific message
                    if (msgId == PATH_COMMUNICATION_ID) {
                        //Handle message from communication thread
                        if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_FIELD_ITEM_SEND) {

                        } else if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_START_GAME) {

                        } else if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_PAUSE_GAME) {

                        } else if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_FLAG_CAPTURE_SEND) {

                        } else if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_NAVIGATION_SEND) {
                            //Figure out what this is
                            
                        } else if (receivemsg[PATH_MESSAGE_TYPE_IDX] == PATH_ITEM_TEST) {
                            if (receivemsg[0] == 'g') {
                                PRIVATEDELETEME++;
                                if (PRIVATEDELETEME > 1) {
                                    Nop();
                                }
                                Nop();
                                calculatePath();
                            } else if (receivemsg[0] == 'l') {
                                
                                region tempRegion;
                                tempRegion.x = receivemsg[2];
                                tempRegion.y = receivemsg[3];
                                tempRegion.width = receivemsg[4];
                                tempRegion.length = receivemsg[5];
                                regionList[receivemsg[1]] = tempRegion;
                                
//                                if (receivemsg[1] == CLOSE_DEFENSE_ZONE || receivemsg[1] == CENTRAL_ZONE || receivemsg[1] == FAR_DEFENSE_ZONE) {
//                                    MAX_WIDTH_OF_FIELD = tempRegion.width;
//                                }
//                                if (receivemsg[1] == CLOSE_SENSOR_ZONE) {
//                                    MAX_LENGTH_OF_FIELD = tempRegion.length;
//                                }
                                
                            } else {
                                fieldItem tempFieldItem;
                                constructFieldItem(&tempFieldItem, receivemsg[0], receivemsg[6], receivemsg[4], receivemsg[3], receivemsg[1], receivemsg[2], receivemsg[5]);
                                storeInFieldItemStack(tempFieldItem);
                                calculateAdjacencyList();
                            }
                            
//                            PRIVATEDELETEME++;
//                            if (PRIVATEDELETEME == 5) {
//                                Nop();
//                            }
                        }
                    } else if (msgId == PATH_FLAG_CAPTURE_ID) {
                        //Handle message from flag capture thread
                        //Only applicable on flag rover
                    } else if (msgId == PATH_NAVIGATION_ID) {
                        //Handle message from navigation thread
                        //Receiving a feedback loop from the Navigation thread
//                        point roverPosition;
//                        roverPosition.x = receivemsg[0];
//                        roverPosition.y = receivemsg[1];
                        
                        fieldItem newFieldRoverPosition;
                        Nop();
                        newFieldRoverPosition.centerX = receivemsg[0];
                        newFieldRoverPosition.centerY = receivemsg[1];
                        newFieldRoverPosition.length = 11;
                        newFieldRoverPosition.width = 10;
                        newFieldRoverPosition.objectType = uniqueIDMapper(IDENTITY_OF_FRIENDLY_FLAG_ROVER); //Change this for other types of field rovers
                        newFieldRoverPosition.orientation = receivemsg[2]; //This is not used
                        newFieldRoverPosition.versionNumber = 1; //This is not used
                        
//                        storeInFieldItemStack(newFieldRoverPosition);
                        
                        int updateLoc = checkIfObjectInStack(newFieldRoverPosition.objectType);
                        if (updateLoc == -1) {
                            fieldItemStack[fieldItemStackTop] = newFieldRoverPosition;
                            fieldItemStackTop++;
                        } else {
                                fieldItemStack[updateLoc] = newFieldRoverPosition;
                        }
                        
                        calculateAdjacencyList();

                    }
                }
            }
            break;
        }

            /* TODO: implement your application state machine.*/


            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */

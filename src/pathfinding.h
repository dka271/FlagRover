/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pathfinding.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _PATHFINDING_H
#define _PATHFINDING_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "system_config.h"
#include "system_definitions.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"
#include <sys/appio.h>
#include <GenericTypeDefs.h>
#include "system/common/sys_module.h"
#include "system/msg/sys_msg.h"
#include "debug.h"
#include "motor.h"
#include "communication_public.h"
#include "pathfinding_public.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
    
    


// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	PATHFINDING_STATE_INIT=0,
	PATHFINDING_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} PATHFINDING_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    PATHFINDING_STATES state;

    /* TODO: Define any additional data used by the application. */

} PATHFINDING_DATA;



typedef struct {
	point topLeft;
	point bottomRight;
} crossSquare;

typedef struct {
	point nodes[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES];
	unsigned char edges[MAXIMUM_NUMBER_OF_NODES][MAXIMUM_NUMBER_OF_IN_SIGHT_NODES];
} adjacencyList;

typedef struct {
    unsigned char node;
    unsigned short gScore;
    unsigned short hScore;
    int fScore;
} pathFindingNode;

//fieldItem fieldItemArray[MAXIMUM_NUMBER_OF_OBSTACLES];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PATHFINDING_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PATHFINDING_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PATHFINDING_Initialize ( void );
    
void storeInFieldItemStack(fieldItem tempFieldItem);
crossSquare convertPointsToCrossSquare(point topLeft, point bottomRight);
point convertXAndYToPoint(unsigned char x, unsigned char y);
crossSquare convertFieldItemToCrossSquare(fieldItem tempFieldItem);
int checkIfObjectInStack(unsigned char objectType);
void calculateAdjacencyList();
void clearAdjacencyList();
void calculateOffsetNodes(fieldItem tempFieldItem);
void createAdjacencyEdges();
bool checkIntersectionOfTwoLines(point line1Point1, point line1Point2, point line2Point1, point line2Point2);
bool checkIfNodeValid(point tempPoint);
bool checkIfPointWithinFieldItem(point pointToCheck, fieldItem tempFieldItem);
bool checkIfTwoNodesInSightOfEachOther(point point1, point point2);
unsigned char getMax(unsigned char num1, unsigned char num2);
unsigned char getMin(unsigned char num1, unsigned char num2);
bool onSegment(point p, point q, point r);
int orientation(point p, point q, point r);
bool checkIntersectionOfTwoLines(point p1, point q1, point p2, point q2);
bool checkIntersectionOfLineAndCrossSquare(point p1, point q1, point p2, point q2);
void testingSendOutEdgesOverWifly();
unsigned char calculatePath();
void AStar(point* path, unsigned char start, unsigned char goal);
unsigned short calculateSquaredEuclideanDistance(point startingPoint, point endingPoint);
int squareFunction(int num);
pathFindingNode getLowestFScoreNode();
void constructPath(unsigned char node);
void removeNodeFromList(pathFindingNode node);
void addNodeToList(unsigned char node);
unsigned char getNumberOfNeighborsOfNode(unsigned char node);
bool isListNotEmpty();
bool isNodeInList(unsigned char node);
pathFindingNode getPathFindingNodeFromList(unsigned char node);
void setPathFindingNodeInList(unsigned char startingPoint, unsigned char newNode);
bool checkIfPathFindingNodeExists(unsigned char node);
unsigned char monotonicAStar(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES], unsigned char start, unsigned char goal);
void clearAStarLists(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES]);
unsigned short calculateEuclideanDistance(unsigned char starting, unsigned char ending);
bool isNodeInOpenSet(unsigned char node);
bool isNodeInClosedSet(unsigned char node);
bool isUnsortedOpenSetEmpty();
unsigned char getLowestFScore();
void removeFromOpenSet(unsigned char current);
void addToOpenSet(unsigned char current);
void reconstructPath(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES], unsigned char current);
bool isClosedSetEmpty();
void sendPathToNavigationThread(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES]);
unsigned char uniqueIDMapper(unsigned char startingID);

/*******************************************************************************
  Function:
    void PATHFINDING_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PATHFINDING_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void PATHFINDING_Tasks( void );


#endif /* _PATHFINDING_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */





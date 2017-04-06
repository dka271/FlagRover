/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef NAVIGATION_PUBLIC_H    /* Guard against multiple inclusion */
#define NAVIGATION_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
#define NAV_TIMER_COUNTER_3_ID 0
#define NAV_TIMER_COUNTER_5_ID 1
#define NAV_COLOR_SENSOR_1_ID 2
#define NAV_COLOR_SENSOR_2_ID 3
#define NAV_COLOR_SENSOR_3_ID 4//not used in the Flag Rover
#define NAV_PIXY_CAM_ID 4
#define NAV_PATHFINDING_ID 5
#define NAV_PWM_TIMER_ID 6
#define NAV_OTHER_ID 7
    
#define NAV_QUEUE_BUFFER_SIZE 7
#define NAV_CHECKSUM_IDX 6
#define NAV_SOURCE_ID_IDX 5
#define NAV_SOURCE_ID_MASK 0xe0
#define NAV_SOURCE_ID_OFFSET 5
    
//Used to control the types of movements the rover makes
#define STATE_WAITING_FOR_GAME_START 0
#define STATE_MOVING_TO_TAPE 1
#define STATE_ORIENTING_TO_TAPE 2
#define STATE_WAITING_FOR_SENSOR_FEEDBACK 3
#define STATE_ORIENTATION_FOUND 4

//Define which region the flag rover is in (0: our defense, 1: center, 2: enemy defense, 3: far flag zone)
#define CROSSED_0_LINES 0
#define CROSSED_1_LINES 1
#define CROSSED_2_LINES 2
#define CROSSED_3_LINES 3
    
unsigned char movementState;
unsigned char locationState;
void sendStartMessageToNavigationThread();
    
unsigned char navCalculateChecksum(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);

void navSendMsgFromISR(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);
void navSendMsg(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);

void sendEdgeToNavigationThread(unsigned char seqNum, unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY);
void sendLocToNavigationThread(unsigned char x, unsigned char y, unsigned char orientation);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

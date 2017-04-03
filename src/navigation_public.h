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
    
unsigned char locationState;
    
unsigned char navCalculateChecksum(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);

void navSendMsgFromISR(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);
void navSendMsg(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);

void sendEdgeToNavigationThread(unsigned char seqNum, unsigned char startX, unsigned char startY, unsigned char endX, unsigned char endY);
void sendLocToNavigationThread(unsigned char x, unsigned char y);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

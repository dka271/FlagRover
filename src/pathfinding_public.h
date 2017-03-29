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

#ifndef PATHFINDING_PUBLIC_H    /* Guard against multiple inclusion */
#define PATHFINDING_PUBLIC_H


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
    
typedef struct {
	unsigned char x;
	unsigned char y;
} point;

typedef struct {
    unsigned char x;
    unsigned char y;
    unsigned char length;
    unsigned char width;
} region;
    
#define PATH_COMMUNICATION_ID 0
#define PATH_FLAG_CAPTURE_ID 1
#define PATH_NAVIGATION_ID 2
    
#define PATH_QUEUE_BUFFER_SIZE 10
#define PATH_CHECKSUM_IDX 9
#define PATH_SOURCE_ID_IDX 8
#define PATH_MESSAGE_TYPE_IDX 7
#define PATH_SOURCE_ID_MASK 0xc0
#define PATH_SOURCE_ID_OFFSET 6
    
//Message Types to be received from communication task
#define PATH_FIELD_ITEM_SEND 0
#define PATH_START_GAME 1
#define PATH_PAUSE_GAME 2
#define PATH_FLAG_CAPTURE_SEND 3
#define PATH_NAVIGATION_SEND 4
#define PATH_ITEM_TEST 5
    
//Unique identifications for rovers
#define IDENTITY_OF_FRIENDLY_FLAG 0
#define IDENTITY_OF_FRIENDLY_FLAG_ROVER 1
#define IDENTITY_OF_FRIENDLY_SENSOR_ROVER 2
#define IDENTITY_OF_FRIENDLY_TAGGER_ROVER 3
#define IDENTITY_OF_FRIENDLY_COUNTER_ROVER 4
#define IDENTITY_OF_ENEMY_FLAG_ROVER 5
#define IDENTITY_OF_ENEMY_SENSOR_ROVER 6
#define IDENTITY_OF_ENEMY_TAGGER_ROVER 7
#define IDENTITY_OF_ENEMY_COUNTER_ROVER 8

//Unique identifiers for regions
#define CLOSE_SENSOR_ZONE 0
#define CLOSE_FLAG_ZONE 1
#define CLOSE_DEFENSE_ZONE 2
#define CENTRAL_ZONE 3
#define FAR_SENSOR_ZONE 4
#define FAR_FLAG_ZONE 5
#define FAR_DEFENSE_ZONE 6

//End of path identifier
#define END_OF_PATH_NUM 125

region regionList[7];

unsigned char pathCalculateChecksum(unsigned char msg[PATH_QUEUE_BUFFER_SIZE]);
    

void pathSendMsg(unsigned char msg[PATH_QUEUE_BUFFER_SIZE]);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* PATHFINDING_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

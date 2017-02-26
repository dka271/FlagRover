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
    
#define PATH_COMMUNICATION_ID 0
#define PATH_FLAG_CAPTURE_ID 1
#define PATH_NAVIGATION_ID 2
    
#define PATH_QUEUE_BUFFER_SIZE 9
#define PATH_CHECKSUM_IDX 8
#define PATH_SOURCE_ID_IDX 7
#define PATH_SOURCE_ID_MASK 0xc0
#define PATH_SOURCE_ID_OFFSET 7
    
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

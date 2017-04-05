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

#ifndef COMMUNICATION_PUBLIC_H    /* Guard against multiple inclusion */
#define COMMUNICATION_PUBLIC_H


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
    

    
#define COMM_UART_ID 0
#define COMM_PATHFINDING_ID 1
#define COMM_SEND_ID 2
#define COMM_OTHER_ID 3 

#define SEND_QUEUE_BUFFER_SIZE 216 
#define RECEIVE_BUFFER_SIZE 216 
#define COMM_QUEUE_BUFFER_SIZE 218
#define COMM_CHECKSUM_IDX 217
#define COMM_SOURCE_ID_IDX 216
#define COMM_SOURCE_ID_MASK 0xc0
#define COMM_SOURCE_ID_OFFSET 6
    
#define MAXIMUM_NUMBER_OF_OBSTACLES 30
#define MAXIMUM_NUMBER_OF_NODES 120
#define MAXIMUM_NUMBER_OF_IN_SIGHT_NODES 80
#define NODE_OFFSET 6
    
typedef struct {
    unsigned char objectType;
    unsigned char centerX;
    unsigned char centerY;
    unsigned char width;
    unsigned char length;
    unsigned char orientation;
    unsigned char versionNumber;
} fieldItem;

unsigned char commCalculateChecksum(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
    
void commSendMsgFromISR(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
void commSendMsg(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
void commSendMsgToWifiQueue(unsigned char msg[SEND_QUEUE_BUFFER_SIZE]);
void readPublic(char* bufferToWriteTo, int size);
void readPublic2(char* bufferToWriteTo, int size);
//void readPublicIntoBuffer();
int readPublicIntoBuffer2(char bufferToWriteTo[COMM_QUEUE_BUFFER_SIZE]);
void writePublic(char* bufferToReadFrom);
void uartReceiveFromSendQueueInISR(unsigned char msg[SEND_QUEUE_BUFFER_SIZE]);
void constructFieldItem(fieldItem *object, unsigned char objectType, unsigned char versionNum, unsigned char length, unsigned char width, unsigned char centerX, unsigned char centerY, unsigned char orientation);
void testingSendEdgeOverWifly(unsigned char i, unsigned char j);
void testingSendPathOverWifly(unsigned char path[MAXIMUM_NUMBER_OF_IN_SIGHT_NODES]);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

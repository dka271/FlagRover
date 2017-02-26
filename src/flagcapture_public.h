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

#ifndef FLAGCAPTURE_PUBLIC_H    /* Guard against multiple inclusion */
#define FLAGCAPTURE_PUBLIC_H


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
    
#define FLAG_ADC_1_ID 0
#define FLAG_ADC_2_ID 1
#define FLAG_GRABBER_ID 2
#define FLAG_PIXY_CAM_ID 3
    
#define FLAG_QUEUE_BUFFER_SIZE 15
#define FLAG_CHECKSUM_IDX 14
#define FLAG_SOURCE_ID_IDX 13
#define FLAG_SOURCE_ID_MASK 0xe0
#define FLAG_SOURCE_ID_OFFSET 5
    
unsigned char flagCalculateChecksum(unsigned char msg[FLAG_QUEUE_BUFFER_SIZE]);

void flagSendMsgFromISR(unsigned char msg[FLAG_QUEUE_BUFFER_SIZE]);
void flagSendMsg(unsigned char msg[FLAG_QUEUE_BUFFER_SIZE]);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* FLAGCAPTURE_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

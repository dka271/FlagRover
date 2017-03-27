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

#ifndef _LED_H    /* Guard against multiple inclusion */
#define _LED_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "debug.h"
#include "communication_public.h"
#include "myjson.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
//The LED operates using port RF0 (pin 45 on the PIC32)
#define LED_PINS 0x0001

//Sets the electromagnet on
void LedSetOn();

//Sets the electromagnet off
void LedSetOff();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */

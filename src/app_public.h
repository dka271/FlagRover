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

#ifndef APP_PUBLIC_H    /* Guard against multiple inclusion */
#define APP_PUBLIC_H


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
    
    void app1SendMsgFromISR(unsigned int msg);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* APP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

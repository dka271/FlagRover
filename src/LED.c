#include "LED.h"

//Sets the led on
void LedSetOn(){
    TRISFCLR = LED_PINS;
    ODCFCLR  = LED_PINS;
    LATFSET  = LED_PINS;
}

//Sets the led off
void LedSetOff(){
    TRISFCLR = LED_PINS;
    ODCFCLR  = LED_PINS;
    LATFCLR  = LED_PINS;
}
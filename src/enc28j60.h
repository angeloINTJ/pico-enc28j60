#ifndef ENC28J60_ARDUINO_H
#define ENC28J60_ARDUINO_H

#include "include/pico/enc28j60-driver.h"

#if defined(FREERTOS_H) || defined(INC_FREERTOS_H)
#include "include/pico/enc28j60-freertos-driver.h"
#endif

#endif // ENC28J60_ARDUINO_H

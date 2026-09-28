#ifndef ENC28J60_ARDUINO_H
#define ENC28J60_ARDUINO_H

#if __has_include("pico/enc28j60-driver.h")
#include "pico/enc28j60-driver.h"
#else
#include "include/pico/enc28j60-driver.h"
#endif

#if defined(FREERTOS_H) || defined(INC_FREERTOS_H)
#if __has_include("pico/enc28j60-freertos-driver.h")
#include "pico/enc28j60-freertos-driver.h"
#else
#include "include/pico/enc28j60-freertos-driver.h"
#endif
#endif

#endif // ENC28J60_ARDUINO_H

/**
 * MEGADOOR HEADER
 * Christopher Bero <berocs@acedb.co>
 */

#ifndef MAIN_H
#define MAIN_H

// Include for Watchdog
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>

#include "avr-printf.h"
#include "avr-pn532.h"
#include "avr-i2c.h"
#include "avr-usart.h"
#include "avr-sha1.h"

#include "avr-keys.h"

// Application specific settings / macros
#define NFC_POLLING_NUM 100 // Number of polling periods before returning.
#define NFC_POLLING_LEN 1 // Polling period, in units of 150ms.
#define NFC_POLLING_TIMEOUT (NFC_POLLING_NUM*NFC_POLLING_LEN*150) // In milliseconds.

#endif // MAIN_H

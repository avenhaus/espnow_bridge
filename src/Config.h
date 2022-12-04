#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdio.h>

/********************************************\
|*  Pin Definitions
\********************************************/

/*
https://drive.google.com/file/d/1gbKM7DA7PI7s1-ne_VomcjOrb0bE2TPZ/view
---+------+----+-----+-----+-----------+---------------------------
No.| GPIO | IO | RTC | ADC | Default   | Function
---+------+----+-----+-----+-----------+---------------------------
25 |   0* | IO | R11 | 2_1 | Boot      |  
35 |   1  | IO |     |     | UART0_TXD | USB Programming/Debug
24 |   2* | IO | R12 | 2_2 |           | LED
34 |   3  | IO |     |     | UART0_RXD | USB Programming/Debug
26 |   4* | IO | R10 | 2_0 |           |
29 |   5* | IO |     |     | SPI0_SS   | 
14 |  12* | IO | R15 | 2_5 |           | 
16 |  13  | IO | R14 | 2_4 |           | 
13 |  14  | IO | R16 | 2_6 |           | 
23 |  15* | IO | R13 | 2_3 |           | 
27 |  16+ | IO |     |     | UART2_RXD | 
28 |  17+ | IO |     |     | UART2_TXD | 
30 |  18  | IO |     |     | SPI0_SCK  | 
31 |  19  | IO |     |     | SPI0_MISO | 
33 |  21  | IO |     |     | I2C0_SDA  |
36 |  22  | IO |     |     | I2C0_SCL  |
37 |  23  | IO |     |     | SPI0_MOSI | 
10 |  25  | IO | R06 | 2_8 |DAC1/I2S-DT| 
11 |  26  | IO | R07 | 2_9 |DAC2/I2S-WS| 
12 |  27  | IO | R17 | 2_7 | I2S-BCK   | 
8  |  32  | IO | R09 | 1_4 |           | 
9  |  33  | IO | R08 | 1_5 |           | 
6  |  34  | I  | R04 | 1_6 |           | 
7  |  35  | I  | R05 | 1_7 |           | 
4  |  36  | I  | R00 | 1_0 | SENSOR_VP | 
5  |  39  | I  | R03 | 1_3 | SENSOR_VN | 
3  |  EN  | I  |     |     | RESET     | 
---+------+----+-----+-----+-----------+---------------------------
(IO6 to IO11 are used by the internal FLASH and are not useable)
22 x I/O  + 4 x input only = 26 usable pins 
GPIO_34 - GPIO_39 have no internal pullup / pulldown.
+ GPIO 16 and 17 are not available on WROVER (PSRAM)
* Strapping pins: IO0, IO2, IO4, IO5 (HIGH), IO12 (LOW), IO15 (HIGH)
*/


#define LED_PIN 2

#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21


#define ESPNOW_CHANNEL 0
#define ESPNOW_ENCRYPTED 0 // Encryption does not work for broadcasts

#define USE_AES_ENCRYPT 1
#define AES_MAGIC_NO_ACK 0xAA01
#define AES_MAGIC_WAIT_ACK 0xAA02
#define AES_MAGIC_ACK 0xAB00
#define AES_MAGIC_OTA 0xAB01

// #define SERIAL_BAUD 115200
#define SERIAL_BAUD 460800

/* ============================================== *\
 * Constants
\* ============================================== */

extern const char EMPTY_STRING[];
extern const char NEW_LINE[];

extern const char HOSTNAME[] PROGMEM;
extern const char PROJECT_NAME[] PROGMEM;
extern const char PROJECT_VERSION[] PROGMEM;
extern const char COMPILE_DATE[] PROGMEM;
extern const char COMPILE_TIME[] PROGMEM;

#define FST (const char *)F
#define PM (const __FlashStringHelper *)

/* ============================================== *\
 * Debug
\* ============================================== */

#define SHOW_INFO 0
#if SHOW_INFO
#define INFO(...) __VA_ARGS__
#else
#define INFO(...)
#endif

#define SERIAL_DEBUG 1

#if SERIAL_DEBUG < 1
#define DEBUG_println(...) 
#define DEBUG_print(...) 
#define DEBUG_printf(...) 
#else
#define DEBUG_println(...) if (debugStream) {debugStream->println(__VA_ARGS__);}
#define DEBUG_print(...) if (debugStream) {debugStream->print(__VA_ARGS__);}
#define DEBUG_printf(...) if (debugStream) {debugStream->printf(__VA_ARGS__);}
#endif

extern Print* debugStream;

#endif // CONFIG_H
#pragma once

// Pin definitions taken from:
//    https://datasheets.raspberrypi.org/pico/pico-datasheet.pdf


// LEDs
#define LED_BUILTIN       (25u)

// Serial
#define TX (0u)
#define RX (1u)

#define PIN_SERIAL2_TX (31u)
#define PIN_SERIAL2_RX (31u)


// SPI
#define MISO  (4u)
#define MOSI  (3u)
#define SCK   (2u)
#define SS    (5u)

// static const int SS   = PIN_SPI0_SS;   // SPI Slave SS not used. Set here only for reference.
// static const int MOSI = PIN_SPI0_MOSI;
// static const int MISO = PIN_SPI0_MISO;
// static const int SCK  = PIN_SPI0_SCK;

#define PIN_SPI1_MISO  (12u)
#define PIN_SPI1_MOSI  (31u)
#define PIN_SPI1_SCK   (31u)
#define PIN_SPI1_SS    (31u)
// static const int MISO_1 = PIN_SPI1_MISO;

#define PIN_A0 (26u)
#define PIN_A1 (27u)
#define PIN_A2 (28u)
#define PIN_A3 (29u)

static const int A0  = PIN_A0;
static const int A1  = PIN_A1;
static const int A2  = PIN_A2;
static const int A3  = PIN_A3;

// D0 - D10
#define D0 (26u)  
#define D1 (27u)
#define D2 (28u)
#define D3 (29u)
#define D4 (6u)
#define D5 (7u)
#define D6 (0u)
#define D7 (1u)
#define D8 (2u)
#define D9 (4u)
#define D10 (3u)

// static const int 0  = D0;

// Wire
#define SDA  (6u)
#define SCL  (7u)

#define PIN_WIRE1_SDA  (26u)
#define PIN_WIRE1_SCL  (27u)

// #define SERIAL_HOWMANY (3u)
// #define SPI_HOWMANY    (2u)
// #define WIRE_HOWMANY   (2u)

// #include "../generic/common.h"

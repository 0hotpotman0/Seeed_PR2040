#pragma once

#pragma once
// #include <macros.h>
#include <stdint.h>

#ifndef __PINS_ARDUINO__
#define __PINS_ARDUINO__

#ifdef __cplusplus
// extern "C" unsigned int PINCOUNT_fn();
#endif

// Pin count
// ----
#define PINS_COUNT           (30u)
#define NUM_DIGITAL_PINS     (30u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (0u)

// extern PinName digitalPinToPinName(pin_size_t P);

// LEDs
// ----
#define PIN_LED     (25u)
#define LED_BUILTIN PIN_LED

// Analog pins
// -----------
#define PIN_A0 (26u)
#define PIN_A1 (27u)
#define PIN_A2 (28u)
#define PIN_A3 (29u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;

#define ADC_RESOLUTION 12

// Serial
#define TX (0ul)
#define RX (1ul)

#define PIN_SERIAL2_TX (31u)
#define PIN_SERIAL2_RX (31u)

// SPI
#define MISO  (4u)
#define MOSI  (3u)
#define SCK   (2u)
#define SS    (5u)

// static const uint8_t SS   = PIN_SPI0_SS;   // SPI Slave SS not used. Set here only for reference.
// static const uint8_t MOSI = PIN_SPI0_MOSI;
// static const uint8_t MISO = PIN_SPI0_MISO;
// static const uint8_t SCK  = PIN_SPI0_SCK;

#define PIN_SPI1_MISO  (8u)
#define PIN_SPI1_MOSI  (11u)
#define PIN_SPI1_SCK   (10u)
#define PIN_SPI1_SS    (9u)

static const uint8_t SS1   = PIN_SPI1_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Wire
#define SDA        (6u)
#define SCL        (7u)

#define SERIAL_HOWMANY		1
#define SERIAL1_TX			(digitalPinToPinName(PIN_SERIAL_TX))
#define SERIAL1_RX			(digitalPinToPinName(PIN_SERIAL_RX))

#define SERIAL_CDC			1
#define HAS_UNIQUE_ISERIAL_DESCRIPTOR
#define BOARD_VENDORID		0x2e8a
#define BOARD_PRODUCTID		0x00c0
// #define BOARD_NAME			"RaspberryPi Pico"

uint8_t getUniqueSerialNumber(uint8_t* name);
void _ontouch1200bps_();

#define SPI_HOWMANY		(2)
#define SPI_MISO		(digitalPinToPinName(MISO))
#define SPI_MOSI		(digitalPinToPinName(MOSI))
#define SPI_SCK			(digitalPinToPinName(SCK))


#define SPI_MISO1		(digitalPinToPinName(PIN_SPI1_MISO))
#define SPI_MOSI1		(digitalPinToPinName(PIN_SPI1_MOSI))
#define SPI_SCK1		(digitalPinToPinName(PIN_SPI1_SCK))

#define WIRE_HOWMANY	(1)
#define I2C_SDA			(digitalPinToPinName(SDA))
#define I2C_SCL			(digitalPinToPinName(SCL))

// #define digitalPinToPort(P)		(digitalPinToPinName(P)/32)

#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#define USB_MAX_POWER	(500)

#endif //__PINS_ARDUINO__


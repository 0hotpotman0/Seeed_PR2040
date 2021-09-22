# Arduino-Pico
[![Release](https://img.shields.io/github/v/release/earlephilhower/arduino-pico?style=plastic)](https://github.com/earlephilhower/arduino-pico/releases)
[![Gitter](https://img.shields.io/gitter/room/earlephilhower/arduino-pico?style=plastic)](https://gitter.im/arduino-pico/community)

Raspberry Pi Pico Arduino core, for all RP2040 boards

This is a port of the RP2040 (Raspberry Pi Pico processor) to the Arduino ecosystem. It uses the bare Raspberry Pi Pico SDK and a custom GCC 10.3/Newlib 4.0 toolchain.

# Documentation
See https://arduino-pico.readthedocs.io/en/latest/ along with the examples for more detailed usage information.

# Supported Boards
* XIAO RP2040
* Wio RP2040

# Installing via Arduino Boards Manager
**Windows Users**: Please do not use the Windows Store version of the actual Arduino application
because it has issues detecting attached Pico boards.  Use the "Windows ZIP" or plain "Windows"
executable (EXE)  download direct from https://arduino.cc. and allow it to install any device
drivers it suggests.  Otherwise the Pico board may not be detected.  Also, if trying out the
2.0 beta Arduino please install the release 1.8 version beforehand to ensure needed device drivers
are present.  (See #20 for more details.)

Open up the Arduino IDE and go to File->Preferences.

In the dialog that pops up, enter the following URL in the "Additional Boards Manager URLs" field:

https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

![image](https://user-images.githubusercontent.com/11875/111917251-3c57f400-8a3c-11eb-8120-810a8328ab3f.png)

Hit OK to close the dialog.

Go to Tools->Boards->Board Manager in the IDE

Type "seeed xiao rp2040" in the search box and select "Install"


# Features
* Adafruit TinyUSB Arduino (USB mouse, keyboard, flash drive, generic HID, CDC Serial, MIDI, WebUSB, others)
* Generic Arduino USB Serial, Keyboard, and Mouse emulation
* Filesystems (LittleFS and SD/SDFS)
* Multicore support (setup1() and loop1())
* Overclocking and underclocking from the menus
* digitalWrite/Read, shiftIn/Out, tone, analogWrite(PWM)/Read, temperature
* Peripherals:  SPI master, Wire(I2C) master/slave, dual UART, emulated EEPROM, I2S audio output, Servo
* printf (i.e. debug) output over USB serial 

The RP2040 PIO state machines (SMs) are used to generate jitter-free:
* Servos
* Tones
* I2S Output


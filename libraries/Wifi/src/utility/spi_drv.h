/*
  spi_drv.h - Library for Arduino Wifi shield.
  Copyright (c) 2018 Arduino SA. All rights reserved.
  Copyright (c) 2011-2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SPI_Drv_h
#define SPI_Drv_h

#include <inttypes.h>
#include "buffer.h"

#define SPI_START_CMD_DELAY 10

#define NO_LAST_PARAM 0
#define LAST_PARAM 1

#define DUMMY_DATA 0x00

#define ESP8285_MAX_ONCE_SEND 2048
#define ESP8285_BUF_SIZE 4096 

typedef struct {
  uint8_t cmd;
  uint8_t addr;
  uint8_t data[64];
} spi_trans_data;

typedef struct {
  uint8_t cmd;
  uint32_t len;
} spi_trans_len;


#define WAIT_FOR_SLAVE_SELECT() \
  if (!SpiDrv::initialized)     \
  {                             \
    SpiDrv::begin();            \
  }                             \
  SpiDrv::waitForSlaveReady();  \
  SpiDrv::spiSlaveSelect();

class SpiDrv
{
private:
  //static bool waitSlaveReady();
  static void waitForSlaveSign();
  static void getParam(uint8_t *param);
  static uint32_t _read_time;

public:
  static bool initialized;

  static void begin();

  static void end();

  static void spiDriverInit();

  static void spiSlaveSelect();

  static void spiSlaveDeselect();

  static char spiTransfer(volatile char data);

  static void waitForSlaveReady(bool const feed_watchdog = false);

  static void sendParamLen8(uint8_t param_len);

  static void sendParamLen32(uint32_t param_len);

  static uint8_t readParamLen8(uint8_t *param_len = NULL);

  static uint32_t readParamLen32(uint32_t *param_len = NULL);
  
  static bool sendCmd(const char *data, uint32_t data_size);

  static uint32_t readData(char *data);
 
  static uint8_t readCmd(char *data);
  
  static int data_find(uint8_t* src,uint32_t src_len, const char* tagert);
  
  static uint32_t recvPkg(char* out_buff, uint16_t out_buff_len, uint16_t *data_len, uint32_t timeout, char* coming_mux_id, bool* peer_closed, bool first_time_recv);
  
  static bool  get_mqttsubrecv(uint32_t LinkID, char* topic, char* msg);
  
  static int available();

  static void rx_empty();
  static char* recvString_1( const char* target1,uint32_t timeout);
  static char* recvString_2( const char* target1,  const char* target2, uint32_t timeout, int8_t* find_index);
  static char* recvString_3( const char* target1,  const char* target2, const char* target3,uint32_t timeout, int8_t* find_index);
  static bool recvFind( const char* target, uint32_t timeout);
  static bool recvFindAndFilter( const char* target,  const char* begin,  const char* end, char** data, uint32_t timeout);

};

extern SpiDrv spiDrv;
extern Buffer_t wifi_buffer;
//Buffer_t wifi_buffer;

#endif

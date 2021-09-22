/*
  spi_drv.cpp - Library for Arduino Wifi shield.
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

#include "Arduino.h"
#include <SPI.h>
#include "utility/spi_drv.h"
#include "pins_arduino.h"
#include "seeed_rp2040_wifi.h"
#include "buffer.h"

#define NINA_GPIO0 (21u)
#define SPIWIFI_SS (24u)
#define SPIWIFI_ACK (21u)
#define SPIWIFI_RESET (22u)

#define pinMode(pin, mode) pinMode(pin, mode)
#define digitalRead(pin) digitalRead(pin)
#define digitalWrite(pin, value) digitalWrite(pin, value)

//#define _DEBUG_
extern "C"
{
#include "utility/debug.h"
}

static uint8_t SLAVESELECT = 9; // ss
static uint8_t SLAVEREADY = 21;  // handshake pin
static uint8_t SLAVERESET = 22;   // reset pin
static uint8_t SLAVEPOWER = 25;
static bool inverted_reset = false;

#define DELAY_TRANSFER()

#ifndef SPIWIFI
#define SPIWIFI SPI1
#endif

#ifndef NINA_GPIOIRQ
#define NINA_GPIOIRQ NINA_GPIO0
#endif

bool SpiDrv::initialized = false;
uint32_t SpiDrv::_read_time = 0;
extern WiFiClass WiFi;
Buffer_t wifi_buffer;
void SpiDrv::begin()
{

    if (SLAVERESET > PINS_COUNT)
    {
        inverted_reset = true;
        SLAVERESET = ~SLAVERESET;
    }
    pinMode(SLAVESELECT, OUTPUT);
    pinMode(SLAVEPOWER, OUTPUT);
    pinMode(SLAVEREADY, OUTPUT);
    pinMode(SLAVERESET, OUTPUT);
    pinMode(NINA_GPIO0, OUTPUT);

    digitalWrite(SLAVEREADY, HIGH);
	digitalWrite(SLAVERESET, HIGH);
	digitalWrite(NINA_GPIO0, HIGH);
    digitalWrite(SLAVESELECT, LOW);
    digitalWrite(SLAVEPOWER, LOW);
    digitalWrite(SLAVERESET, LOW);
    delay(20);
    digitalWrite(SLAVERESET, HIGH);
	delay(50);
    pinMode(SLAVEREADY, INPUT);
    SPIWIFI.begin();

	//uint8_t *buff = (uint8_t *)malloc(ESP8285_BUF_SIZE);
	Buffer_Init(&wifi_buffer, ESP8285_BUF_SIZE);
    initialized = true;
}

void SpiDrv::end()
{
    digitalWrite(SLAVERESET, inverted_reset ? HIGH : LOW);

    pinMode(SLAVESELECT, INPUT);
    digitalWrite(SLAVEPOWER, HIGH);
    SPIWIFI.end();

    initialized = false;
}

void SpiDrv::spiSlaveSelect()
{
    SPIWIFI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    digitalWrite(SLAVESELECT, LOW);

    // wait for up to 5 ms for the NINA to indicate it is not ready for transfer
    // the timeout is only needed for the case when the shield or module is not present
    for (unsigned long start = millis(); (digitalRead(SLAVEREADY) != HIGH) && (millis() - start) < 5;)
        ;
}

void SpiDrv::spiSlaveDeselect()
{
    digitalWrite(SLAVESELECT, HIGH);
    SPIWIFI.endTransaction();
}

char SpiDrv::spiTransfer(volatile char data)
{
    char result = SPIWIFI.transfer(data);
    DELAY_TRANSFER();

    return result; // return the received byte
}




#define WAIT_START_CMD(x) waitSpiChar(START_CMD)

#define IF_CHECK_START_CMD(x)            \
    if (!WAIT_START_CMD(_data))          \
    {                                    \
        TOGGLE_TRIGGER()                 \
        WARN("Error waiting START_CMD"); \
        return 0;                        \
    }                                    \
    else

#define CHECK_DATA(check, x)          \
    if (!readAndCheckChar(check, &x)) \
    {                                 \
        TOGGLE_TRIGGER()              \
        WARN("Reply error");          \
        INFO2(check, (uint8_t)x);     \
        return 0;                     \
    }                                 \
    else

#define waitSlaveReady() (digitalRead(SLAVEREADY) == LOW)
#define waitSlaveSign() (digitalRead(SLAVEREADY) == HIGH)
#define waitSlaveSignalH()                  \
    while (digitalRead(SLAVEREADY) != HIGH) \
    {                                       \
    }
#define waitSlaveSignalL()                 \
    while (digitalRead(SLAVEREADY) != LOW) \
    {                                      \
    }

void SpiDrv::waitForSlaveSign()
{
    while (!waitSlaveSign())
        ;
}

void SpiDrv::sendParamLen8(uint8_t param_len)
{
    // Send Spi paramLen
    spiTransfer(param_len);
}

void SpiDrv::sendParamLen32(uint32_t param_len)
{
    // Send Spi paramLen
	spiTransfer((uint8_t)(param_len & 0x000000ff));
	spiTransfer((uint8_t)((param_len & 0x0000ff00) >> 8));
	spiTransfer((uint8_t)((param_len & 0x00ff0000) >> 16));
    spiTransfer((uint8_t)((param_len & 0xff000000) >> 24)); 
}

uint8_t SpiDrv::readParamLen8(uint8_t *param_len)
{
    uint8_t _param_len = spiTransfer(DUMMY_DATA);
    if (param_len != NULL)
    {
        *param_len = _param_len;
    }
    return _param_len;
}

uint32_t SpiDrv::readParamLen32(uint32_t *param_len)
{
    uint32_t _param_len = (spiTransfer(DUMMY_DATA) & 0xff) | spiTransfer(DUMMY_DATA) << 8 | spiTransfer(DUMMY_DATA) << 16 | (spiTransfer(DUMMY_DATA) << 24);
    if (param_len != NULL)
    {
        *param_len = _param_len;
    }
    return _param_len;
}

static void kmp_get_next( const char* targe, int next[])
{  
    int targe_Len = strlen(targe);  
    next[0] = -1;  
    int k = -1;  
    int j = 0;  
    while (j < targe_Len - 1)  
    {     
        if (k == -1 || targe[j] == targe[k])  
        {  
            ++j;  
            ++k;   
            if (targe[j] != targe[k])  
                next[j] = k;    
            else   
                next[j] = next[k];  
        }  
        else  
        {  
            k = next[k];  
        }  
    }  
}  
static int kmp_match(char* src,int src_len, const char* targe, int* next)  
{  
    int i = 0;  
    int j = 0;  
    int sLen = src_len;  
    int pLen = strlen(targe);  
    while (i < sLen && j < pLen)  
    {     
        if (j == -1 || src[i] == targe[j])  
        {  
            i++;  
            j++;  
        }  
        else  
        {       
            j = next[j];  
        }  
    }  
    if (j == pLen)  
        return i - j;  
    else  
        return -1;  
} 

static int kmp_find(char* src,uint32_t src_len, const char* tagert)
{
	uint32_t index = 0;
	uint32_t tag_len = strlen(tagert);
	int* next = (int*)malloc(sizeof(uint32_t) * tag_len);
	kmp_get_next(tagert,next);
	index = kmp_match(src,src_len,tagert, next);
	free(next);
	return index;
}

int SpiDrv::data_find(uint8_t* src,uint32_t src_len, const char* tagert)
{
	return kmp_find((char*)src,src_len,tagert);
}

void SpiDrv::rx_empty()
{
    char *data = NULL;

    pinMode(SLAVEREADY, INPUT);
	unsigned long start = millis();
    while (!waitSlaveReady() && millis() - start < 1000)
    {
        readCmd(data);
    }
}
char* SpiDrv::recvString_1( const char* target1,uint32_t timeout)
{
	uint32_t iter = 0;
	memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    unsigned long start = millis();
	while (millis() - start < timeout) {
        while(!waitSlaveReady()&& iter < ESP8285_BUF_SIZE){
			iter += readCmd((char *)wifi_buffer.buffer+iter);			
        }
        if (data_find(wifi_buffer.buffer,iter,target1) != -1) {
            return (char*)wifi_buffer.buffer;
        } 
	}
    return NULL;
}


char* SpiDrv::recvString_2( const char* target1, const char* target2, uint32_t timeout, int8_t* find_index)
{
	uint32_t iter = 0;
    *find_index = -1;
	memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    unsigned int start = millis();
    while (millis() - start < timeout) {
        while(!waitSlaveReady() && iter < ESP8285_BUF_SIZE) {
			iter += readCmd((char *)wifi_buffer.buffer+iter);
        }
        if (data_find(wifi_buffer.buffer,iter,target1) != -1) {
            *find_index = 0;
			if (data_find(wifi_buffer.buffer,iter,target2) != -1) {
				*find_index = 1;
			}
            return (char*)wifi_buffer.buffer;
        } else if (data_find(wifi_buffer.buffer,iter,target2) != -1) {
            *find_index = 1;
            return (char*)wifi_buffer.buffer;
        }
    }
    return NULL;
}

char* SpiDrv::recvString_3( const char* target1, const char* target2, const char* target3,uint32_t timeout, int8_t* find_index)
{

	uint32_t iter = 0;
    *find_index = -1;
	memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while(!waitSlaveReady()&& iter < ESP8285_BUF_SIZE) {
            iter += readCmd((char *)wifi_buffer.buffer+iter);
        }
        if (data_find(wifi_buffer.buffer,iter,target1) != -1) {
            *find_index = 0;
            return (char*)wifi_buffer.buffer;
        } else if (data_find(wifi_buffer.buffer,iter,target2) != -1) {
            *find_index = 1;
            return (char*)wifi_buffer.buffer;
        } else if (data_find(wifi_buffer.buffer,iter,target3) != -1) {
            *find_index = 2;
            return (char*)wifi_buffer.buffer;
        }
    }
    return NULL;
}

bool SpiDrv::recvFind(const char* target, uint32_t timeout)
{
    recvString_1(target, timeout);
    if (data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,target) != -1) {
        return true;
    }
    return false;
}

bool SpiDrv::recvFindAndFilter( const char* target, const char* begin, const char* end, char** data, uint32_t timeout)
{
    recvString_1(target, timeout);
    if (data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,target) != -1) {
        int32_t index1 = data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,begin);
        int32_t index2 = data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,end);
        if (index1 != -1 && index2 != -1) {
            index1 += strlen(begin);
			if(index2 - index1 > 0)
			{
				*data = (char *)malloc(index2 - index1);
				memcpy(*data, wifi_buffer.buffer+index1, index2 - index1);
				return true;
			}
        }
    }
    return false;
}

uint32_t SpiDrv::readData(char* data)
{
	static uint32_t read_time = 0;
	spi_trans_len trans_len;
	spi_trans_data trans_data;
	memset(&trans_len, 0x0, sizeof(trans_len));
	memset(&trans_data, 0x0, sizeof(trans_data));
	int data_len = sizeof(trans_len);
	if(read_time == 0){
		trans_len.cmd = 0x4;
		trans_len.len = 0;
		spiSlaveSelect();
		sendParamLen8(trans_len.cmd);              //8bit
        data_len = readParamLen32(&trans_len.len); //32bit//待测
		spiSlaveDeselect();
		read_time = (trans_len.len + 63) / 64;
	}
	if(read_time > 0) {
	    data_len = sizeof(trans_data);
		trans_data.cmd = 0x3;
		trans_data.addr = 0x00;	
		delayMicroseconds(50);
		spiSlaveSelect();
        sendParamLen8(trans_data.cmd);  //8bit*2?
        sendParamLen8(trans_data.addr); //8bit*2?
        for (uint8_t i = 0; i < 64; i++){
            trans_data.data[i] = readParamLen8(&trans_data.data[i]); //8*64bit*2?
		}
		spiSlaveDeselect();
		memcpy(data, trans_data.data, sizeof(trans_data.data));
		delayMicroseconds(100);
		read_time--;
	}
	return trans_len.len;
}

uint8_t SpiDrv::readCmd(char *data)
{
    static uint32_t read_last_len = 0;
    static uint32_t data_len;
    uint8_t tmp = 0;
    spi_trans_len trans_len;
    spi_trans_data trans_data;
    memset(&trans_len, 0x0, sizeof(trans_len));
    memset(&trans_data, 0x0, sizeof(trans_data));
    //int data_len = sizeof(trans_len);
    if (_read_time == 0)
    {
        trans_len.cmd = 0x4;
        trans_len.len = 0;
        spiSlaveSelect();
        sendParamLen8(trans_len.cmd);              //8bit
        data_len = readParamLen32(&trans_len.len); //32bit//待测
        spiSlaveDeselect();
        _read_time = (data_len + 63) / 64;
        read_last_len = (data_len + 64) % 64;
    }
    if (_read_time > 0)
    {
        //data_len = sizeof(trans_data);
        trans_data.cmd = 0x3;
        trans_data.addr = 0x0;
        delayMicroseconds(100);
        spiSlaveSelect();
        sendParamLen8(trans_data.cmd);  //8bit*2?
        sendParamLen8(trans_data.addr); //8bit*2?
        for (uint8_t i = 0; i < 64; i++){
            trans_data.data[i] = readParamLen8(&trans_data.data[i]); //8*64bit*2?
		}
        spiSlaveDeselect();
        memcpy(data, trans_data.data, sizeof(trans_data.data));
        delayMicroseconds(100);
        tmp = 64;
        _read_time--;
        //data += 64;
    }
    if (!_read_time)
    {
        //*(data + read_last_len) = '\0';
		if(read_last_len == 0) {
			return 1;
		}
        return read_last_len;
    }
    return tmp;
}

bool SpiDrv::sendCmd(const char *data, uint32_t data_size)
{
    uint32_t send_time = 0;
    spi_trans_data trans_data;
    spi_trans_len trans_len;
    memset(&trans_len, 0x0, sizeof(trans_len));
    memset(&trans_data, 0x0, sizeof(trans_data));
    trans_len.cmd = 0x1;
    trans_len.len = data_size;
    //int data_len = sizeof(trans_len);
    spiSlaveSelect();
    sendParamLen8(trans_len.cmd); //8bit
    sendParamLen32(trans_len.len);
    spiSlaveDeselect();
    unsigned long start;
    send_time = (data_size + 63) / 64;
    while (send_time > 0)
    {
        delayMicroseconds(100);
        memset(&trans_data, 0x0, sizeof(trans_data));
        //data_len = sizeof(trans_data);
        trans_data.cmd = 0x2;
        trans_data.addr = 0x0;
        if (send_time == 1)
            memcpy(&trans_data.data, data, strlen(data));
        else
            memcpy(&trans_data.data, data, 64);
        start = millis();
        while (!waitSlaveSign())
        {
            if (millis() - start > 1000)
                return false;
        }
        spiSlaveSelect();
        sendParamLen8((uint16_t)trans_data.cmd);  //8bit
        sendParamLen8((uint16_t)trans_data.addr); //8bit
        for (uint8_t i = 0; i < 64; i++)
            sendParamLen8((uint16_t)trans_data.data[i]); //8*64bit
        spiSlaveDeselect();
        delayMicroseconds(100);
        send_time--;
        data += 64;
    }
    start = millis();
    while (!waitSlaveSign())
    {
        if (millis() - start > 1000)
            return false;
    }
    memset(&trans_len, 0x0, sizeof(trans_len));
    trans_len.cmd = 0x1;
    trans_len.len = 0;
    //data_len = sizeof(spi_trans_len);
    spiSlaveSelect();
    sendParamLen8(trans_len.cmd); //8bit
    sendParamLen32(trans_len.len);
    spiSlaveDeselect();
    _read_time = 0;
    return 1;
}

uint32_t SpiDrv::recvPkg(char* out_buff, uint16_t out_buff_len, uint16_t *data_len, uint32_t timeout, char* coming_mux_id, bool* peer_closed, bool first_time_recv)
{
    uint8_t temp_buff[16];
    uint8_t temp_buff2[16];
	uint8_t temp_read[64];
	uint8_t temp_read_len = 0;
	uint8_t temp_read_len_end = 0;
    uint8_t temp_buff_len = 0;
    uint8_t temp_buff2_len = 0;
    uint8_t find_frame_flag_index = 0;
    static int8_t mux_id = -1;
    static int16_t frame_len = 0;
    static int32_t frame_len_sum = 0;//only for single socket TODO:
    // bool    overflow = false;
    int ret = 0;
    uint16_t size = 0;
    uint64_t start = 0, start2 = 0;
    bool no_frame_flag = false;
    bool new_frame = false;
    bool peer_just_closed = false;
    
    // parameters check
    if (out_buff == NULL) {
        return -1;
    }
    // init vars
    memset(temp_buff, 0, sizeof(temp_buff));
    memset(temp_buff, 0, sizeof(temp_buff2));
    if(first_time_recv)
    {
        frame_len = 0;
        frame_len_sum = 0;
    }

    // required data already in buf, just return data
    uint32_t buff_size = Buffer_Size(&wifi_buffer);
    if(buff_size >= out_buff_len)
    {
        Buffer_Gets(&wifi_buffer, (uint8_t*)out_buff, out_buff_len);
        if(data_len)
            *data_len = out_buff_len;
        frame_len_sum -= size;
        if(frame_len_sum <= 0)
        {
            frame_len = 0;
            frame_len_sum = 0;
            Buffer_Clear(&wifi_buffer);
            if(*peer_closed)//buffer empty, return EOF
            {
                return -2;
            }
        }
        return out_buff_len;
    }
    
    // read from spi buffer, if not frame start flag, put into nic buffer
    // and need wait for full frame flag in 200ms(can be fewer), frame format: '+IPD,id,len:data' or '+IPD,len:data'
    // wait data from spi buffer if not timeout
    start2 = millis();
    do{
        //self->read_lock = true;
        //spi_drain_rx_fifo(self);
        //self->read_lock = false;
        if(!waitSlaveReady() || (temp_read_len_end - temp_read_len) > 0)
        {
			if(temp_read_len == 0 || temp_read_len == 64){
				temp_read_len_end = readData((char *)temp_read);
				temp_read_len = 0;
			}
			temp_buff[temp_buff_len] = temp_read[++temp_read_len];
            if(find_frame_flag_index == 0 && temp_buff[temp_buff_len] == '+'){
                ++find_frame_flag_index;
                start = millis();
            }
            else if(find_frame_flag_index == 1 && temp_buff[temp_buff_len] == 'I'){
                ++find_frame_flag_index;
            }
            else if(find_frame_flag_index == 2 && temp_buff[temp_buff_len] == 'P'){
                ++find_frame_flag_index;
            }
            else if(find_frame_flag_index == 3 && temp_buff[temp_buff_len] == 'D'){
                ++find_frame_flag_index;
            }
            else if(find_frame_flag_index == 4 && temp_buff[temp_buff_len] == ','){
                ++find_frame_flag_index;
            }
            else if(find_frame_flag_index == 5){
                    if(temp_buff[temp_buff_len] == ':'){ // '+IPD,3,1452:' or '+IPD,1452:'
                        temp_buff[temp_buff_len+1] = '\0';
                        char* index = strstr((char*)temp_buff+5, ",");
                        if(index){ // '+IPD,3,1452:'
                            ret = sscanf((char*)temp_buff,"+IPD,%hhd,%hd:",&mux_id,&frame_len);
                            if (ret!=2 || mux_id < 0 || mux_id > 4 || frame_len<=0) { // format not satisfy, it's data
                                no_frame_flag = true;
                            }else{// find frame start flag, although it may also data
                                new_frame = true;
                            }
                        }else{ // '+IPD,1452:'
                            ret = sscanf((char*)temp_buff,"+IPD,%hd:",&frame_len);
                            if (ret !=1 || frame_len<=0) { // format not satisfy, it's data
                                no_frame_flag = true;
                            }else{// find frame start flag, although it may also data
                                new_frame = true;
                                // printk("new frame:%d\r\n", frame_len);
                            }
                        }
                    }
            }
            else{ // not match frame start flag, put into nic buffer
                no_frame_flag = true;
            }
            // new frame or data
            // or wait for frame start flag timeout(300ms, can be fewer), maybe they're data
            if(new_frame || no_frame_flag || temp_buff_len >= 12 ||
                (find_frame_flag_index && (millis() - start > 300) )
            ) // '+IPD,3,1452:'
            {
                if(!new_frame){
                    if(frame_len_sum > 0){
                        if(!Buffer_Puts(&wifi_buffer, temp_buff, temp_buff_len+1))
                        {
                            // overflow = true;
                            // break;//TODO:
                        }
                    }
                    else{
                        if(temp_buff[0]=='C'){
                            memset(temp_buff2, 0, sizeof(temp_buff2));
                        }
                        temp_buff2[temp_buff2_len++] = temp_buff[0];
                        // printk("%c", temp_buff[0]); //TODO: optimize spi overflow, if spi overflow, uncomment this will print some data
                        // printk("-%d:%s\r\n", temp_buff2_len, temp_buff2);
                        if(strstr((const char*)temp_buff2, "CLOSED\r\n") != NULL){
                            // printk("pear closed\r\n");
                            *peer_closed = true;
                            peer_just_closed = true;
                            break;
                        }
                    }
                }else{
                    frame_len_sum += frame_len;
                }
                find_frame_flag_index = 0;
                temp_buff_len = 0;
                new_frame = false;
                no_frame_flag = false;
                // enough data as required
                size = Buffer_Size(&wifi_buffer);
                if( size >= out_buff_len) // data enough
                    break;
                if(frame_len_sum!=0 && frame_len_sum <= size) // read at least one frame ok
                {
                    break;
                }
                continue;
            }
            ++temp_buff_len;
        }
        if(timeout!=0 && (millis() - start2 > timeout) && !find_frame_flag_index )
        {
            return -3;
        }
        //self->read_lock = true;
        //spi_drain_rx_fifo(self);
        //self->read_lock = false;
    }while( (timeout || find_frame_flag_index) && (!*peer_closed || !waitSlaveReady()) );
    size = Buffer_Size(&wifi_buffer);
    if( size == 0 && !peer_just_closed && *peer_closed)//peer closed and no data in buffer
    {
        frame_len = 0;
        frame_len_sum = 0;
        return -4;
    }
    size = size > out_buff_len ? out_buff_len : size;
    Buffer_Gets(&wifi_buffer, (uint8_t*)out_buff, size);
    if(data_len)
        *data_len = size;
    frame_len_sum -= size;
    if(frame_len_sum <= 0 || peer_just_closed)
    {
        frame_len = 0;
        frame_len_sum = 0;
        Buffer_Clear(&wifi_buffer);
        if(peer_just_closed)
        {
            return -2;
        }
    }
    return size;
}

bool  SpiDrv::get_mqttsubrecv(uint32_t LinkID, char* topic, char* msg)
{
	char* cur = NULL;
	uint32_t iter = 0;
	memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    unsigned long start = millis();
	while (millis() - start < 3000) {
        while(!waitSlaveReady()&& iter < ESP8285_BUF_SIZE) {
            iter += readCmd((char *)wifi_buffer.buffer+iter);
        }
	}
	cur = strstr((char*)wifi_buffer.buffer, "+MQTTSUBRECV");
	if(cur == NULL)
	{
		//mp_printf(MP_PYTHON_PRINTER, "test get_mqttsubrecv none\n");
		return false;
	}
	int LinkID_recv;
	int len;
	sscanf(cur, "+MQTTSUBRECV:%d,\"%[^\"]\",%d,%[^\n]", &LinkID_recv, topic, &len, msg);
	msg[len] = NULL;
	return true;
}

int SpiDrv::available()
{
    return (digitalRead(NINA_GPIOIRQ) != LOW);
}
SpiDrv spiDrv;

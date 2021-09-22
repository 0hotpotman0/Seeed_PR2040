/*
  server_drv.cpp - Library for Arduino Wifi shield.
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

//#define _DEBUG_

#include "utility/server_drv.h"

#include "Arduino.h"
#include "utility/spi_drv.h"

extern "C" {
#include "utility/wl_types.h"
#include "utility/debug.h"
}
network_socket_obj_t socket_t;

// Start server TCP on port specified
void ServerDrv::startServer(uint16_t port, uint8_t sock, uint8_t protMode)
{
	
	
}

void ServerDrv::startServer(uint32_t ipAddress, uint16_t port, uint8_t sock, uint8_t protMode)
{

}

// Start server TCP on port specified
void ServerDrv::startClient(char* ipAddress, uint16_t port, uint8_t sock, uint8_t protMode)
{
	switch (protMode)
    {
    case TCP_MODE:
    {
        if (false == sATCIPSTARTSingle("TCP",ipAddress, port))
        {   
            return ;
        }
        break;
    }
    case UDP_MODE:
    {
        if (false == sATCIPSTARTSingle("UDP",ipAddress, port))
        {
            return ;
        }
        break;
    }
    default:
    {
        if (false == sATCIPSTARTSingle("TCP",ipAddress, port))
        {
            return ;
        }
        break;
    }
    }

}

void ServerDrv::startClient(const char* host, uint8_t host_len, uint32_t ipAddress, uint16_t port, uint8_t sock, uint8_t protMode)
{

}

// Start server TCP on port specified
void ServerDrv::stopClient(uint8_t sock)
{
   eATCIPCLOSESingle();
}


uint8_t ServerDrv::getServerState(uint8_t sock)
{
    return 0;
}

uint8_t ServerDrv::getClientState(uint8_t sock)
{
	int state = 0;
    eATCIPSTATUS(&state);
    return state;
}

uint16_t ServerDrv::availData(uint8_t sock)
{
    return 0;
}

uint8_t ServerDrv::availServer(uint8_t sock)
{
    return 0;
}

bool ServerDrv::getData(uint8_t sock, uint8_t *data, uint8_t peek)
{
    return 0;
}

bool ServerDrv::getDataBuf(uint8_t sock, uint8_t *_data, uint16_t *_dataLen)
{
    int ret = 0;
    uint16_t read_len = 1500;
    ret = esp_recv((char *)_data, read_len, _dataLen, 10000, &socket_t.peer_closed, socket_t.first_read_after_write);
    socket_t.first_read_after_write = false;
    if (ret == -1)
    {
        return false;
    }
    else if (ret == -2) // EOF
    {
        return false;
    }
    else if (ret == -3) // timeout
    {
        return false;
    }
    else if (ret == -4) //peer closed
    {
        return false;
    }
    return true;
}

bool ServerDrv::insertDataBuf(uint8_t sock, const uint8_t *data, uint16_t _len)
{
    return 0;
}

bool ServerDrv::sendUdpData(uint8_t sock)
{
    return 0;
}


uint16_t ServerDrv::sendData(uint8_t sock, const uint8_t *data, uint16_t len)
{
    bool ret = false;
    uint32_t send_total_len = 0;
    uint16_t send_len = 0;
	socket_t.first_read_after_write = true;
    while(send_total_len < len)
    {
        send_len = ((len-send_total_len) > ESP8285_MAX_ONCE_SEND)?ESP8285_MAX_ONCE_SEND : (len-send_total_len);
        ret = sATCIPSENDSingle((char *)data+send_total_len, send_len, 10000);
        if(!ret)
            return false;
        send_total_len += send_len;
    }
    return true;
}

uint8_t ServerDrv::checkDataSent(uint8_t sock)
{
    return 0;
}

uint8_t ServerDrv::getSocket()
{
	int state = 255;
	state = getClientState(255);
	if(state == 2 || state == 4 || state == 5)
	{
		return  2;
	}
	return 255;
}

int esp_recv(char* buffer, uint16_t buffer_size, uint16_t* read_len, uint32_t timeout,  bool* peer_closed, bool first_time_recv)
{
    return (int)SpiDrv::recvPkg(buffer, buffer_size, read_len, timeout, NULL, peer_closed, first_time_recv);
}

bool get_mqttconn(mqttconn_obj* mqttconn)
{
	char* cur = NULL;
	cur = strstr((char*)wifi_buffer.buffer, "+MQTTCONN:");
	if(cur == NULL)
	{
		debug_println("  MQTTCONN could'n get \n");
		return false;
	}
	sscanf(cur, "+MQTTCONN:%d,%d,%d,\"%[^\"]\",%d,\"%[^\"]\",%d", &mqttconn->LinkID, &mqttconn->state, &mqttconn->scheme, mqttconn->host, &mqttconn->port, mqttconn->path, &mqttconn->reconnect);
	return true;
}

bool sATCIPSTARTSingle(const char* type, char* host, uint16_t port)
{
	const char* cmd = "AT+CIPSTART=\"";
	//char* host = netutils_format_ipv4_addr(addr,NETUTILS_BIG);
	char port_str[10] = {0};
    int8_t find_index;
	itoa(port, port_str, 10);
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd(type,strlen(type));
	SpiDrv::sendCmd("\",\"",strlen("\",\""));
	SpiDrv::sendCmd((const char*)host,strlen(host));
	SpiDrv::sendCmd("\",",strlen("\","));
	SpiDrv::sendCmd((const char*)port_str,strlen(port_str));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    if(SpiDrv::recvString_3("OK", "ERROR", "ALREADY CONNECT", 10000, &find_index)!=NULL && (find_index==0 || find_index==2)){
        return true;
	}
    return false;
}

bool eATCIPCLOSESingle()
{

    int8_t find;
	const char* cmd = "AT+CIPCLOSE\r\n";
	SpiDrv::rx_empty();
	SpiDrv::sendCmd((char*)cmd,strlen(cmd));
    if (SpiDrv::recvString_2("OK", "ERROR", 5000, &find) != NULL)
    {
        if( find == 0)
            return true;
    }
    return false;
}
bool eATCIPSTATUS(int* list)
{
	const char* cmd = "AT+CIPSTATUS\r\n";
	char *str_status= NULL;
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	if(SpiDrv::recvFind("OK",1000))
	{
		if(SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"STATUS:1\r\n")!= -1)
		{

			*list = 1;
			return true;
		}
		if(SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"STATUS:2\r\n")!= -1)
		{
			*list = 2;
			return true;
		}
		if(SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"STATUS:3\r\n")!= -1)
		{
			*list = 3;
			return true;
		}
		if(SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"STATUS:4\r\n")!= -1)
		{
			*list = 4;
			return true;
		}
		if(SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"STATUS:5\r\n")!= -1)
		{
			*list = 5;
			return true;
		}
		str_status = strstr((char*)wifi_buffer.buffer, "STATUS:");
		sscanf(str_status,"STATUS:%d",list);
		return true;
	}
    return false;
}

bool sATCIPSENDSingle(char* buffer, uint32_t len, uint32_t timeout)
{
	const char* cmd = "AT+CIPSEND=";
	char len_str[10] = {0};
	itoa(len,len_str ,10);
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd((const char*)len_str,strlen(len_str));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    if (SpiDrv::recvFind(">", 5000)) {
        SpiDrv::rx_empty();
		SpiDrv::sendCmd(buffer,len);
        return SpiDrv::recvFind("SEND OK", timeout);
    }
    return false;
}

bool sMQTTUSERCFG(int LinkID, int scheme, const char* client_id, const char* username, const char* password, int cert_key_ID, int CA_ID, const char* path)
{
    
   
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTUSERCFG=%d,%d,\"%s\",\"%s\",\"%s\",%d,%d,\"%s\"", LinkID, scheme, client_id, username, password, cert_key_ID, CA_ID, path);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }

    return true;
}
bool sMQTTUSERNAME(int LinkID, char* username)
{
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTUSERNAME=%d,\"%s\"", LinkID, username);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }

    return true;
}
bool sMQTTPASSWORD(int LinkID, char* password)
{
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTUSERNAME=%d,\"%s\"", LinkID, password);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }
    return true;
}
bool sMQTTCONNCFG(int LinkID, int keepalive, int disable_clean_session, const char* lwt_topic, const char* wt_msg, int lwt_qos, int lwt_retain)
{
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTCONNCFG=%d,%d,%d,\"%s\",\"%s\",%d,%d", LinkID, keepalive, disable_clean_session, lwt_topic, wt_msg, lwt_qos, lwt_retain);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }

    return true;
}
bool sMQTTCONN(int LinkID, const char* host, int port, int reconnect)
{
    
   
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTCONN=%d,\"%s\",%d,%d", LinkID, host, port, reconnect);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }
    return true;
}
bool qMQTTCONN()
{
	const char* cmd = "AT+MQTTCONN?";
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
	return SpiDrv::recvFind("OK",1000);
}
bool sMQTTPUB(int LinkID,  const char* topic,  const char* data, uint8_t qos, uint8_t retain)
{
    char mqtt_cmd[128] = {0};

    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);

    sprintf(mqtt_cmd, "AT+MQTTPUB=%d,\"%s\",\"%s\",%d,%d", LinkID, topic, data, qos, retain);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1( "\r\nOK", 3000) == NULL)
    {
        return false;
    }
    return true;
}
bool qMQTTSUB(int LinkID, const char* topic, uint8_t qos)
{    
    char mqtt_cmd[128] = {0};
    SpiDrv::rx_empty();
    sprintf(mqtt_cmd, "AT+MQTTSUB=%d,\"%s\",%d", LinkID, topic, qos);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    return SpiDrv::recvFind("OK",1000);
}
bool eMQTTSUB_Start()
{
	const char* cmd = "AT+MQTTSUB?";
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
	return true;
}
bool eMQTTSUB_Get(bool* end)
{
    int8_t find;
    *end = false;
    if (SpiDrv::recvString_2( "\r\n+MQTTSUB:", "\r\n\r\nOK", 10000, &find) != NULL)
    {
        if( find == 1)
            *end = true;
        return true;
    }
    return false;
}
bool sMQTTUNSUB(int LinkID,  const char* topic)
{

    char mqtt_cmd[128] = {0};
    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    sprintf(mqtt_cmd, "AT+MQTTUNSUB=%d,\"%s\"", LinkID, topic);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1("\r\nOK", 3000) == NULL)
    {
        return false;
    }
    return true;
}
bool sMQTTCLEAN(int LinkID)
{
    char mqtt_cmd[128] = {0};
    SpiDrv::rx_empty();
    memset(wifi_buffer.buffer,0,ESP8285_BUF_SIZE);
    sprintf(mqtt_cmd, "AT+MQTTCLEAN=%d", LinkID);
	SpiDrv::sendCmd(mqtt_cmd,strlen(mqtt_cmd));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));

    if (SpiDrv::recvString_1("\r\nOK", 3000) == NULL)
    {
        return false;
    }
    return true;
}


ServerDrv serverDrv;

/*
  wifi_drv.cpp - Library for Arduino Wifi shield.
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "Arduino.h"
#include "utility/spi_drv.h"
#include "utility/wifi_drv.h"

#define _DEBUG_

extern "C"
{
#include "utility/debug.h"
}



// Array of data to cache the information related to the networks discovered
char WiFiDrv::_networkSsid[][WL_SSID_MAX_LENGTH] = {{"1"}, {"2"}, {"3"}, {"4"}, {"5"}};

// Cached values of retrieved data
char WiFiDrv::_ssid[] = {0};
uint8_t WiFiDrv::_bssid[] = {0};
uint8_t WiFiDrv::_mac[] = {0};
uint8_t WiFiDrv::_localIp[] = {0};
uint8_t WiFiDrv::_subnetMask[] = {0};
uint8_t WiFiDrv::_gatewayIp[] = {0};
// Firmware version
char WiFiDrv::fwVersion[] = {0};

// Private Methods

// void WiFiDrv::getNetworkData(uint8_t *ip, uint8_t *mask, uint8_t *gwip)
// {
//     tParam params[PARAM_NUMS_3] = { {0, (char*)ip}, {0, (char*)mask}, {0, (char*)gwip}};

//     WAIT_FOR_SLAVE_SELECT();

//     // Send Command
//     SpiDrv::sendCmd(GET_IPADDR_CMD, PARAM_NUMS_1);

//     uint8_t _dummy = DUMMY_DATA;
//     SpiDrv::sendParam(&_dummy, sizeof(_dummy), LAST_PARAM);

//     // pad to multiple of 4
//     SpiDrv::readChar();
//     SpiDrv::readChar();

//     SpiDrv::spiSlaveDeselect();
//     //Wait the reply elaboration
//     SpiDrv::waitForSlaveReady();
//     SpiDrv::spiSlaveSelect();

//     // Wait for reply
//     SpiDrv::waitResponseParams(GET_IPADDR_CMD, PARAM_NUMS_3, params);

//     SpiDrv::spiSlaveDeselect();
// }

// void WiFiDrv::getRemoteData(uint8_t sock, uint8_t *ip, uint8_t *port)
// {
//     tParam params[PARAM_NUMS_2] = { {0, (char*)ip}, {0, (char*)port} };

//     WAIT_FOR_SLAVE_SELECT();

//     // Send Command
//     SpiDrv::sendCmd(GET_REMOTE_DATA_CMD, PARAM_NUMS_1);
//     SpiDrv::sendParam(&sock, sizeof(sock), LAST_PARAM);

//     // pad to multiple of 4
//     SpiDrv::readChar();
//     SpiDrv::readChar();

//     SpiDrv::spiSlaveDeselect();
//     //Wait the reply elaboration
//     SpiDrv::waitForSlaveReady();
//     SpiDrv::spiSlaveSelect();

//     // Wait for reply
//     SpiDrv::waitResponseParams(GET_REMOTE_DATA_CMD, PARAM_NUMS_2, params);

//     SpiDrv::spiSlaveDeselect();
// }

// // Public Methods

void WiFiDrv::wifiDriverInit()
{
  SpiDrv::begin();
}

void WiFiDrv::wifiDriverDeinit()
{
  SpiDrv::end();
}

bool WiFiDrv::GetOprToStation(int *mode)
{
  return qATCWMODE(mode);
}

bool WiFiDrv::setOprToStation(int set_mode)
{
  int mode;
  if (sATCWMODE(set_mode))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool WiFiDrv::joinAP(char *ssid, char *pwd)
{
  return sATCWJAP(ssid, pwd);
}

bool WiFiDrv::leaveAP()
{
    return eATCWQAP();
}

bool WiFiDrv::closeAP()
{
    return eATCWCAP();
}
bool WiFiDrv::set_ipconfig(uint8_t mode,/*IPAddress*/uint8_t* local_ip, /*IPAddress*/uint8_t* gateway, /*IPAddress*/uint8_t* subnet)
{
  if (!sATCWDHCP(mode, 1))
  {
    return false;
  }
  if (mode == 1)
  {
    if (!sATCIPSTA_CUR((char *)local_ip, (char *)gateway, (char *)subnet))
    {
      return false;
    }
  }
  else
  {
    if (!sATCIPAP_CUR((char *)local_ip, (char *)gateway, (char *)subnet))
    {
      return false;
    }
  }
  return true;
}
bool WiFiDrv::enableMUX()
{
    return sATCIPMUX(1);
}

bool WiFiDrv::disableMUX()
{
    return sATCIPMUX(0);
}

bool WiFiDrv::get_ipconfig(/*IPAddress*/uint8_t* local_ip, /*IPAddress*/uint8_t* gateway, /*IPAddress*/uint8_t* subnet)
{
	if(0 == qATCIPSTA_CUR())
		return false;
	char* cur = NULL;
	cur = strstr((char*)wifi_buffer.buffer, "ip");
	if(cur == NULL)
	{
		debug_println("esp8285_ipconfig could'n get ip\n");
		return false;
	}
	sscanf(cur,"ip:\"%u.%u.%u.%u\"",&local_ip[0],&local_ip[1],&local_ip[2],&local_ip[3]);
	cur = strstr((char*)wifi_buffer.buffer, "gateway");
	if(cur == NULL)
	{
		debug_println( "esp8285_ipconfig could'n get gateway\n");
		return false;
	}
	sscanf(cur,"gateway:\"%u.%u.%u.%u\"",&gateway[0],&gateway[1],&gateway[2],&gateway[3]);
	cur = strstr((char*)wifi_buffer.buffer, "netmask");
	if(cur == NULL)
	{
		debug_println("esp8285_ipconfig could'n get netmask\n");
		return false;
	}
	sscanf(cur,"netmask:\"%u.%u.%u.%u\"",&subnet[0],&subnet[1],&subnet[2],&subnet[3]);
	return true;
}
char* netutils_format_ipv4_addr(uint8_t *ip, char* host, netutils_endian_t endian) {
    if (endian == NETUTILS_LITTLE) {
        snprintf(host, 16, "%u.%u.%u.%u", ip[3], ip[2], ip[1], ip[0]);
    } else {
        snprintf(host, 16, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    }
    return host;
}

void netutils_parse_ipv4_addr(char* addr_in, uint8_t *out_ip, netutils_endian_t endian) 
{
    uint8_t	addr_len;
    const char *addr_str = addr_in;
	addr_len = strlen(addr_str);
    if (addr_len == 0) {
        // special case of no address given
        memset(out_ip, 0, 4);
        return;
    }
    const char *s = addr_str;
    const char *s_top = addr_str + addr_len;
    for (uint8_t i = 3; ; i--) {
        uint8_t val = 0;
        for (; s < s_top && *s != '.'; s++) {
            val = val * 10 + *s - '0';
        }
        if (endian == NETUTILS_LITTLE) {
            out_ip[i] = val;
        } else {
            out_ip[4 - 1 - i] = val;
        }
        if (i == 0 && s == s_top) {
            return;
        } else if (i > 0 && s < s_top && *s == '.') {
            s++;
        } else {
            debug_println("invalid arguments");
			memset(out_ip, 0, 4);
			return;
        }
    }
}

bool WiFiDrv::get_host_byname(const char* host,uint8_t* out_ip, uint32_t timeout_ms)
{
	int index = 0;
	if(false == sATCIPDOMAIN(host, timeout_ms))
	{
		debug_println( "get_host_byname failed\n");
		return false;
	}
	char IP_buf[16]={0};
	index = SpiDrv::data_find(wifi_buffer.buffer,ESP8285_BUF_SIZE,"+CIPDOMAIN:");
	sscanf((char*)wifi_buffer.buffer + index,"+CIPDOMAIN:\"%[^\"]\"",IP_buf);
	netutils_parse_ipv4_addr(IP_buf, out_ip, NETUTILS_BIG);
	return true;
}

bool WiFiDrv::dns_setserver(uint8_t enable, char *DNS_IP1, char *DNS_IP2)
{
  if (!sDNS(enable, (char *)DNS_IP1, (char *)DNS_IP2))
  {
    return false;
  }
  return true;
}
bool WiFiDrv::get_mac(uint8_t mode, uint8_t *macTemp)
{
	if(mode == 1)
	{
		if (!qCIPSTAMAC(macTemp))
		{	
			return false;
		}
	}
  else 
  {
	if (!qCIPAPMAC(macTemp))
	{
		return false;
	}
  }
  return true;
}

bool WiFiDrv::wifi_softap_get_config(softap_config *apconfig)
{
  if (0 == qATCWSAP())
    return false;
  char *cur = NULL;
  cur = strstr((char *)wifi_buffer.buffer, "+CWSAP:");
  if (cur == NULL)
  {
    WARN("esp8285_config could'n get ip\n");
    return false;
  }
  sscanf(cur, "+CWSAP:\"%[^\"]\",\"%[^\"]\",%d,%d,%d,%d", apconfig->ssid, apconfig->password, &apconfig->channel, &apconfig->authmode, &apconfig->max_conn, &apconfig->ssid_hidden);
  if (apconfig->authmode > 5 || apconfig->authmode < 0)
  {
    sscanf(cur, "+CWSAP:\"%[^\"]\",\"\",%d,%d,%d,%d", apconfig->ssid, &apconfig->channel, &apconfig->authmode, &apconfig->max_conn, &apconfig->ssid_hidden);
    *apconfig->password = NULL;
  }
  //nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "qian connect to 1.%s 2.%s 3.%d 4.%d 5.%d 6.%d", apconfig->ssid,apconfig->password,&apconfig->channel,&apconfig->authmode,&apconfig->max_conn,&apconfig->ssid_hidden));
  return true;
}

bool WiFiDrv::wifi_softap_set_config(softap_config *apconfig)
{
  apconfig->max_conn = 8;
  return sATCWSAP(apconfig->ssid, apconfig->password, apconfig->channel, apconfig->authmode, apconfig->max_conn, apconfig->ssid_hidden);
}

bool WiFiDrv::eATCWLAP_Start()
{
    char cmd[] = {"AT+CWLAP\r\n"};
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
    return true;
}

bool WiFiDrv::eATCWLAP_Get(bool* end)
{
    int8_t find;
    *end = false;
    if (SpiDrv::recvString_2("\r\n+CWLAP:", "\r\n\r\nOK", 10000, &find) != NULL)
    {
        if( find == 1)
            *end = true;
        return true;
    }
    return false;
}

bool WiFiDrv::eINIT(int mode)
{
  bool init_flag = 1;
  init_flag = init_flag && eAT();
  init_flag = init_flag && eATE(1);
  init_flag = init_flag && disableMUX();
  init_flag = init_flag && sATCIPMODE(0);
  init_flag = init_flag && setOprToStation(mode);
  if(mode & SOFTAP_MODE){
  	init_flag = init_flag && enableMUX();
  }
  else{
  	init_flag = init_flag && disableMUX();
  }
  if(!(mode & SOFTAP_MODE)){
  	init_flag = init_flag && leaveAP();
  }
  delay(800);
  return init_flag;
}

bool eAT()
{
  const char *cmd = "AT\r\n";
  SpiDrv::rx_empty(); // clear rx
  SpiDrv::sendCmd(cmd, strlen(cmd));
  if (SpiDrv::recvFind("OK", 1000))
    return true;
  SpiDrv::rx_empty(); // clear rx
  SpiDrv::sendCmd(cmd, strlen(cmd));
  if (SpiDrv::recvFind("OK", 1000))
    return true;
  SpiDrv::rx_empty(); // clear rx
  SpiDrv::sendCmd(cmd, strlen(cmd));
  if (SpiDrv::recvFind("OK", 1000))
    return true;
  return false;
}

bool eATE(bool enable)
{	
    SpiDrv::rx_empty();// clear rx
    if(enable)
    {
    	const char* cmd = "ATE0\r\n";
		SpiDrv::sendCmd(cmd,strlen(cmd));
    	return SpiDrv::recvFind("OK",1000);
    }
	else
	{
    	const char* cmd = "ATE1\r\n";
		SpiDrv::sendCmd(cmd,strlen(cmd));
    	return SpiDrv::recvFind("OK",1000);		
	}
}

bool qATCWMODE(int *mode)
{
  const char *cmd = "AT+CWMODE?\r\n";
  char *str_mode;
  bool ret;
  if (!mode)
  {
    return false;
  }
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  ret = SpiDrv::recvFindAndFilter("OK", "+CWMODE:", "\r\n\r\nOK", &str_mode, 1000);
  if (ret)
  {
    *mode = atoi(str_mode);
    free(str_mode);
    return true;
  }
  else
  {
    return false;
  }
}

bool sATCWMODE(int mode)
{
  const char *cmd = "AT+CWMODE=";
  char mode_str[10] = {0};
  int8_t find;
  itoa(mode, mode_str, 10);
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(mode_str, strlen(mode_str));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  if (SpiDrv::recvString_2("OK", "no change", 1000, &find) != NULL){
  return true;}
  return false;
}

bool sATCWJAP(char *ssid, char *pwd)
{
  const char *cmd = "AT+CWJAP=\"";
  int8_t find;
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(ssid, strlen(ssid));
  SpiDrv::sendCmd("\",\"", strlen("\",\""));
  SpiDrv::sendCmd(pwd, strlen(pwd));
  SpiDrv::sendCmd("\"", strlen("\""));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  if (SpiDrv::recvString_2("OK", "ERROR", 20000, &find) != NULL && find == 0)
    return true;
  return false;
}

bool sATCWDHCP(char mode, bool enabled)
{
  const char *cmd = "AT+CWDHCP=";
  int8_t find;
  char strEn[2] = {0};
  if (enabled)
  {
    strcpy(strEn, "1");
  }
  else
  {
    strcpy(strEn, "0");
  }
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(strEn, strlen(strEn));
  SpiDrv::sendCmd(",", strlen(","));
  SpiDrv::sendCmd(&mode, 1);
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  if (SpiDrv::recvString_2("OK", "FAIL", 10000, &find) != NULL && find == 0)
    return true;
  return false;
}

bool eATCWQAP()
{
	const char* cmd = "AT+CWQAP\r\n";
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
    return SpiDrv::recvFind("OK",1000);
}

bool eATCWCAP()
{
	const char* cmd = "AT+CWQIF\r\n";
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
    return SpiDrv::recvFind("OK",1000);
}

bool sATCIPDOMAIN(const char* domain_name, uint32_t timeout)
{
	const char* cmd = "AT+CIPDOMAIN=";
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd("\"",strlen("\""));
	SpiDrv::sendCmd((char *)domain_name,strlen(domain_name));
	SpiDrv::sendCmd("\"",strlen("\""));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    return SpiDrv::recvFind("OK",timeout);
}

bool qATCIPSTA_CUR()
{
  const char *cmd = "AT+CIPSTA?";
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool sATCIPSTA_CUR(char *ip, char *gateway, char *netmask)
{
  const char *cmd = "AT+CIPSTA_CUR=";
  SpiDrv::rx_empty();
  if (NULL == ip)
  {
    return false;
  }
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(ip, strlen(ip));
  if (NULL == gateway)
  {
     SpiDrv::sendCmd("\r\n", strlen("\r\n"));
    return  SpiDrv::recvFind("OK", 1000);
  }
  SpiDrv::sendCmd(",", strlen(","));
  SpiDrv::sendCmd(gateway, strlen(gateway));
  if (NULL == netmask)
  {
    SpiDrv::sendCmd("\r\n", strlen("\r\n"));
    return SpiDrv::recvFind("OK", 1000);
  }
  SpiDrv::sendCmd(",", strlen(","));
  SpiDrv::sendCmd(netmask, strlen(netmask));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool qATCIPAP_CUR()
{
  const char *cmd = "AT+CIPAP?";
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool sATCIPAP_CUR(char *ip, char *gateway, char *netmask)
{
  const char *cmd = "AT+CIPAP_CUR=";
  SpiDrv::rx_empty();
  if (NULL == ip)
  {
    return false;
  }
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(ip, strlen(ip));
  if (NULL == gateway)
  {
     SpiDrv::sendCmd("\r\n", strlen("\r\n"));
    return  SpiDrv::recvFind("OK", 1000);
  }
  SpiDrv::sendCmd(",", strlen(","));
  SpiDrv::sendCmd(gateway, strlen(gateway));
  if (NULL == netmask)
  {
    SpiDrv::sendCmd("\r\n", strlen("\r\n"));
    return SpiDrv::recvFind("OK", 1000);
  }
  SpiDrv::sendCmd(",", strlen(","));
  SpiDrv::sendCmd(netmask, strlen(netmask));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool sDNS(uint8_t enable, char *DNS_IP1, char *DNS_IP2)
{
  const char *cmd = "AT+CIPDNS=";
  SpiDrv::rx_empty();
  char enable_str[10] = {0};
  itoa(enable, enable_str, 10);
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd(enable_str, strlen(enable_str));
  if(enable)
  {
	
	SpiDrv::sendCmd(",", strlen(","));
	SpiDrv::sendCmd(DNS_IP1, strlen(DNS_IP1));
	SpiDrv::sendCmd(",", strlen(","));
	SpiDrv::sendCmd(DNS_IP2, strlen(DNS_IP2));
  }
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool qATCWSAP()
{
  const char *cmd = "AT+CWSAP?";
  SpiDrv::rx_empty();
  SpiDrv::sendCmd(cmd, strlen(cmd));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));
  return SpiDrv::recvFind("OK", 1000);
}

bool sATCWSAP(char *ssid, char *key, int chl, int ecn, int max_conn, int ssid_hidden)
{
  char ap_cmd[128] = {0};

  SpiDrv::rx_empty();
  memset(wifi_buffer.buffer, 0, ESP8285_BUF_SIZE);

  sprintf(ap_cmd, "AT+CWSAP=\"%s\",\"%s\",%d,%d,%d,%d", ssid, key, chl, ecn, max_conn, ssid_hidden);
  SpiDrv::sendCmd(ap_cmd, strlen(ap_cmd));
  SpiDrv::sendCmd("\r\n", strlen("\r\n"));

  if (SpiDrv::recvString_1("\r\nOK", 3000) == NULL)
  {
    return false;
  }

  return true;
}

bool qCIPSTAMAC(uint8_t* mac) 
{
    const char* cur = NULL;
	const char* cmd = "AT+CIPSTAMAC?\r\n";
    bool ret;
    if (!mac) {
        return false;
    }
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
    ret = SpiDrv::recvFind("OK",1000); 
    cur = strstr((char*)wifi_buffer.buffer, "+CIPSTAMAC:");
	sscanf(cur, "+CIPSTAMAC:\"%x:%x:%x:%x:%x:%x\"", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    if (ret) {
        return true;
    } else {
        return false;
    }
}

bool qCIPAPMAC(uint8_t* mac) 
{
    char* cur = NULL;
	const char* cmd = "AT+CIPAPMAC?\r\n";
    bool ret;
    if (!mac) {
        return false;
    }
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
    ret = SpiDrv::recvFind("OK",1000); 
    cur = strstr((char*)wifi_buffer.buffer, "+CIPAPMAC:");
	sscanf(cur, "+CIPAPMAC:\"%x:%x:%x:%x:%x:%x\"", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    if (ret) {
        return true;
    } else {
        return false;
    }
}
bool sATCIPMODE(char mode)
{
	const char* cmd = "AT+CIPMODE=";
	char mode_str[10] = {0};
	itoa(mode, mode_str, 10);
	SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd(mode_str,strlen(mode_str));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    return SpiDrv::recvFind("OK",1000);
}
bool sATCIPMUX(char mode)
{
	const char* cmd = "AT+CIPMUX=";
	char mode_str[10] = {0};
    int8_t find;
	itoa(mode, mode_str, 10);
    SpiDrv::rx_empty();
	SpiDrv::sendCmd(cmd,strlen(cmd));
	SpiDrv::sendCmd(mode_str,strlen(mode_str));
	SpiDrv::sendCmd("\r\n",strlen("\r\n"));
    if(SpiDrv::recvString_2("OK", "Link is builded",5000, &find) != NULL && find==0)
        return true;
    return false;
}

WiFiDrv wiFiDrv;
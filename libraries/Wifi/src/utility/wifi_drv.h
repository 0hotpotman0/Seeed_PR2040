/*
  wifi_drv.h - Library for Arduino Wifi shield.
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

#ifndef WiFi_Drv_h
#define WiFi_Drv_h

#include <inttypes.h>

#include "IPAddress.h"
#include "utility/wl_definitions.h"
#include "buffer.h"

// Key index length
#define KEY_IDX_LEN     1
// 100 msecs of delay to have the connection established
#define WL_DELAY_START_CONNECTION 100
// firmware version string length
#define WL_FW_VER_LENGTH 6

typedef enum {
  NO_MODE = 0,
  STATION_MODE,
  SOFTAP_MODE,
} wl_mode_t;

typedef enum _netutils_endian_t {
    NETUTILS_LITTLE,
    NETUTILS_BIG,
} netutils_endian_t;

typedef struct _softap_config
{
	char ssid[50];
	int ssid_len;
	int ssid_hidden;
	int authmode;
	char password[64];
	int max_conn;
	int channel;
}softap_config;

class WiFiDrv
{
private:
	// settings of requested network

	static char 	_networkSsid[WL_NETWORKS_LIST_MAXNUM][WL_SSID_MAX_LENGTH];

	// firmware version string in the format a.b.c
	static char 	fwVersion[WL_FW_VER_LENGTH];

	// settings of current selected network
	static char 	_ssid[WL_SSID_MAX_LENGTH];
	static uint8_t 	_bssid[WL_MAC_ADDR_LENGTH];
	static uint8_t 	_mac[WL_MAC_ADDR_LENGTH];
	static uint8_t  _localIp[WL_IPV4_LENGTH];
	static uint8_t  _subnetMask[WL_IPV4_LENGTH];
	static uint8_t  _gatewayIp[WL_IPV4_LENGTH];


public:

    static void wifiDriverInit();

    static void wifiDriverDeinit();

	 static bool joinAP(char *ssid,char *pwd);
	 static bool leaveAP();
	 static bool closeAP();
	 static bool get_host_byname(const char* host,uint8_t* out_ip, uint32_t timeout_ms);
	 static bool eATCWLAP_Start();
	 static bool eATCWLAP_Get(bool* end);
	static bool eINIT(int mode);
	static bool GetOprToStation(int *mode);
	static bool setOprToStation(int set_mode);
	static bool wifi_softap_get_config(softap_config* apconfig);
	static bool wifi_softap_set_config(softap_config* apconfig);
	static bool set_ipconfig(uint8_t mode,/*IPAddress*/uint8_t* local_ip, /*IPAddress*/uint8_t* gateway, /*IPAddress*/uint8_t* subnet);
	static bool get_ipconfig(/*IPAddress*/uint8_t* local_ip, /*IPAddress*/uint8_t* gateway, /*IPAddress*/uint8_t* subnet);
	static bool dns_setserver(uint8_t enable, char *DNS_IP1, char *DNS_IP2);
	static bool get_mac(uint8_t mode, uint8_t *macTemp);
    static bool enableMUX();
	static bool disableMUX();
    friend class WiFiUDP;
    friend class WiFiClient;
};

bool eAT();
bool eATE(bool enable);
bool sATCWMODE(int mode);
bool sATCWJAP(char *ssid,char *pwd);
bool qATCWMODE(int *mode);
bool sATCWDHCP(char mode, bool enabled);
bool eATCWQAP();
bool eATCWCAP();
bool sATCIPDOMAIN(const char* domain_name, uint32_t timeout);
bool qATCIPSTA_CUR();
bool sATCIPSTA_CUR(const char* ip,char* gateway,char* netmask);
bool qATCIPAP_CUR();
bool sATCIPAP_CUR(char *ip, char *gateway, char *netmask);
bool sDNS(uint8_t enable, char *DNS_IP1, char *DNS_IP2);
bool qATCWSAP();
bool sATCWSAP(char* ssid, char* key, int chl, int ecn, int max_conn, int ssid_hidden);
bool qCIPSTAMAC(uint8_t* mac);
bool qCIPAPMAC(uint8_t* mac);
bool sATCIPMODE(char mode);
bool sATCIPMUX(char mode);

void netutils_parse_ipv4_addr(char* addr_in, uint8_t *out_ip, netutils_endian_t endian);
char* netutils_format_ipv4_addr(uint8_t *ip, char* host, netutils_endian_t endian);
extern WiFiDrv wiFiDrv;
extern Buffer_t wifi_buffer;
#endif

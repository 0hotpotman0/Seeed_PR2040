/*
  WiFi.cpp - Library for Arduino Wifi shield.
  Copyright (c) 2018 Arduino SA. All rights reserved.
  Copyright (c) 2011-2014 Arduino LLC.  All right reserved.

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

#include "utility/wifi_drv.h"
#include "seeed_rp2040_wifi.h"
#include "Arduino.h"
extern "C"
{
#include "utility/debug.h"
}

WiFiClass::WiFiClass(): 
  _status(WL_NO_SHIELD),
  _interface(STATION_MODE)
{
  _staticIp = false;
  memset(&_ipInfo, 0x00, sizeof(_ipInfo));
  memset(&_dnsServers, 0x00, sizeof(_dnsServers));
  memset(&_hostname, 0x00, sizeof(_hostname));
}

void WiFiClass::init()
{
    WiFiDrv::wifiDriverInit();
    WiFiDrv::eINIT(STATION_MODE);
}
/*
const char* WiFiClass::firmwareVersion()
{
	return WiFiDrv::getFwVersion();
}
*/


int WiFiClass::hostByName(const char* hostname, /*IPAddress*/uint8_t* result)
{
    if (!WiFiDrv::get_host_byname(hostname, result,3000))
        return 0;
    return 1;	
}

int WiFiClass::begin(const char *ssid)
{
    return begin(ssid, "");
}

int WiFiClass::begin(const char *ssid, uint8_t key_idx, const char *key)
{
    return begin(ssid, key);
}

int WiFiClass::begin(const char *ssid, const char *passphrase)
{
    int mode = 0;
	
	_status = WL_NO_SSID_AVAIL;
    _interface = STATION_MODE;
	init();
    if (!WiFiDrv::GetOprToStation(&mode))
    {
        debug_println("couldn't init nic esp8285 ,try again \n");
		 _status = WL_CONNECT_FAILED;
        return false;
    }
    mode |= STATION_MODE;
    if (!WiFiDrv::setOprToStation(mode))
    {
        debug_println("couldn't init nic esp8285 ,try again please\n");
        return false;
    }
    if (!WiFiDrv::joinAP((char *)ssid, (char *)passphrase))
    {
        debug_println("could not connect wifi\n");
        return false;
    }
	 _status = WL_CONNECTED;
    return true;
}

uint8_t WiFiClass::beginAP(const char *ssid, uint8_t channel)
{
    softap_config wifiConfig;

    _status = WL_NO_SSID_AVAIL;
    _interface = SOFTAP_MODE;
	init();
    int mode = SOFTAP_MODE;
    if (!WiFiDrv::setOprToStation(mode))
    {
       debug_println("couldn't init nic esp8285 ,try again please\n");
	   _status = WL_AP_FAILED;
        return false;
    }
    if (!WiFiDrv::GetOprToStation(&mode))
    {
        debug_println("couldn't init nic esp8285 ,try again \n");
		_status = WL_AP_FAILED;
        return false;
    }
    memset(&wifiConfig, 0x00, sizeof(wifiConfig));
    if (false == WiFiDrv::wifi_softap_get_config(&wifiConfig))
    {
        debug_println("couldn't get apconfig \n");
		_status = WL_AP_FAILED;
        return false;
    }
    strncpy((char *)wifiConfig.ssid, ssid, sizeof(wifiConfig.ssid));
    wifiConfig.channel = channel;
	wifiConfig.authmode = 0;
    if (false == WiFiDrv::wifi_softap_set_config(&wifiConfig))
    {
		debug_println("couldn't set apconfig \n");
		_status = WL_AP_FAILED;
        return false;
    }
    return true;
}

uint8_t WiFiClass::beginAP(const char *ssid, uint8_t key_idx, const char *key, uint8_t channel)
{

    softap_config wifiConfig;

    _status = WL_NO_SSID_AVAIL;
    _interface = SOFTAP_MODE;
	init();
    int mode = SOFTAP_MODE;
    if (!WiFiDrv::setOprToStation(mode))
    {
        debug_println("couldn't init nic esp8285 ,try again please\n");
		_status = WL_AP_FAILED;
        return false;
    }

    if (!WiFiDrv::GetOprToStation(&mode))
    {
        debug_println("couldn't init nic esp8285 ,try again \n");
		_status = WL_AP_FAILED;
        return false;
    }

    memset(&wifiConfig, 0x00, sizeof(wifiConfig));
    if (false == WiFiDrv::wifi_softap_get_config(&wifiConfig))
    {
        debug_println("couldn't get apconfig \n");
		_status = WL_AP_FAILED;
        return false;
    }
    strncpy((char *)wifiConfig.ssid, ssid, sizeof(wifiConfig.ssid));
    strncpy((char *)wifiConfig.password, key, sizeof(wifiConfig.password));
    wifiConfig.channel = channel;
	wifiConfig.authmode = 1;
    if (false == WiFiDrv::wifi_softap_set_config(&wifiConfig))
    {
        debug_println("couldn't set apconfig \n");
		_status = WL_AP_FAILED;
        return false;
    }
    return true;
}

uint8_t WiFiClass::beginAP(const char *ssid, const char *key, uint8_t channel)
{
    return beginAP(ssid, 1, key, channel);
}

void WiFiClass::config(/*IPAddress*/uint8_t* local_ip, /*IPAddress*/uint8_t* gateway, /*IPAddress*/uint8_t* subnet)
{
  _staticIp = true;
    _ipInfo.ip = local_ip;
  _ipInfo.gateway = gateway;
  _ipInfo.netmask = subnet;
  if (_interface == SOFTAP_MODE) {
    WiFiDrv::set_ipconfig(SOFTAP_MODE, local_ip, gateway, subnet);
  } else {
    WiFiDrv::set_ipconfig(STATION_MODE, local_ip, gateway, subnet);
  }
}

void WiFiClass::setDNS(/*IPAddress*/uint32_t dns_server1, /*IPAddress*/uint32_t dns_server2)
{
  _dnsServers[0] = dns_server1;
  _dnsServers[1] = dns_server2;

  if (false == WiFiDrv::dns_setserver(1 ,(char *)dns_server1, (char *)dns_server2)) {
    debug_println("couldn't get apconfig \n");
  }
}

void WiFiClass::hostname(const char* name)
{
  strncpy(_hostname, name, HOSTNAME_MAX_LENGTH);
}

void WiFiClass::disconnect()
{
	if(_interface == STATION_MODE)
	{
		WiFiDrv::leaveAP();
	}
	else
	{
		WiFiDrv::closeAP();
	}
	
  WiFiDrv::wifiDriverDeinit();
}

void WiFiClass::end(void)
{
    WiFiDrv::wifiDriverDeinit();
}

uint8_t* WiFiClass::macAddress(uint8_t* mac)
{
  uint8_t macTemp[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  WiFiDrv::get_mac(_interface, macTemp);

  mac[0] = macTemp[5];
  mac[1] = macTemp[4];
  mac[2] = macTemp[3];
  mac[3] = macTemp[2];
  mac[4] = macTemp[1];
  mac[5] = macTemp[0];

  return mac;
}

uint8_t* WiFiClass::localIP()
{
	WiFiDrv::get_ipconfig(_ipInfo.ip,_ipInfo.netmask,_ipInfo.gateway);
  return _ipInfo.ip;
}

uint8_t* WiFiClass::subnetMask()
{
WiFiDrv::get_ipconfig(_ipInfo.ip,_ipInfo.netmask,_ipInfo.gateway);
  return _ipInfo.netmask;
}

uint8_t* WiFiClass::gatewayIP()
{
WiFiDrv::get_ipconfig(_ipInfo.ip,_ipInfo.netmask,_ipInfo.gateway);
  return _ipInfo.gateway;
}

// char* WiFiClass::SSID()
// {
  // return (char*)_apRecord.ssid;
// }

// int32_t WiFiClass::RSSI()
// {
  // if (_interface == ESP_IF_WIFI_AP) {
    // return 0;
  // } else {
    // esp_wifi_sta_get_ap_info(&_apRecord);

    // return _apRecord.rssi;
  // }
// }

// uint8_t WiFiClass::encryptionType()
// {
  // uint8_t encryptionType = _apRecord.authmode;

  // if (encryptionType == WIFI_AUTH_OPEN) {
    // encryptionType = 7;
  // } else if (encryptionType == WIFI_AUTH_WEP) {
    // encryptionType = 5;
  // } else if (encryptionType == WIFI_AUTH_WPA_PSK) {
    // encryptionType = 2;
  // } else if (encryptionType == WIFI_AUTH_WPA2_PSK || encryptionType == WIFI_AUTH_WPA_WPA2_PSK) {
    // encryptionType = 4;
  // } else {
    // encryptionType = 255;
  // }

  // return encryptionType;
// }

// uint8_t* WiFiClass::BSSID(uint8_t* bssid)
// {
  // if (_interface == ESP_IF_WIFI_AP) {
    // return macAddress(bssid);
  // } else {
    // bssid[0] = _apRecord.bssid[5];
    // bssid[1] = _apRecord.bssid[4];
    // bssid[2] = _apRecord.bssid[3];
    // bssid[3] = _apRecord.bssid[2];
    // bssid[4] = _apRecord.bssid[1];
    // bssid[5] = _apRecord.bssid[0];

    // return bssid;
  // }
// }

int8_t WiFiClass::scanNetworks()
{
	char *buf = (char *)wifi_buffer.buffer;
    bool end = 0;
	int pos = 0;
    int err_code = 0;
	int mode = 0;
	_interface = STATION_MODE;
    if (!WiFiDrv::GetOprToStation(&mode))
    {
        debug_println("couldn't init nic esp8285 ,try again \n");
        return WL_CONNECT_FAILED;
    }
    mode = STATION_MODE;
    if (!WiFiDrv::setOprToStation(mode))
    {
        debug_println("couldn't init nic esp8285 ,try again please\n");
        return WL_CONNECT_FAILED;
    }
    if (!WiFiDrv::eATCWLAP_Start())
    {
        err_code = -1;
        debug_println("scan failing -1\n");
		return err_code;
    }
	if (!WiFiDrv::eATCWLAP_Get(&end))
    {
        err_code = -2;
        debug_println("scan failing -2\n");
		return err_code;
    }
	char *index1 = buf;
    while (1)
    {
        index1 = strstr(index1, "+CWLAP:");
        if (!index1)
        {
            err_code = -3;
            debug_println("scan failing -3\n");
			return err_code;
        }
        char *index2 = strstr(index1, ")");
        if (!index2)
        {
            err_code = -4;
            debug_println("scan failing -4\n");
			return err_code;
        }
        *index2 = '\0';
		sscanf(index1, "+CWLAP:(%ld,\"%[^\"]\",%ld,\"%lx:%lx:%lx:%lx:%lx:%lx\"", &_scanResults[pos].authmode, _scanResults[pos].ssid, &_scanResults[pos].rssi, &_scanResults[pos].bssid[0], &_scanResults[pos].bssid[1], &_scanResults[pos].bssid[2], &_scanResults[pos].bssid[3], &_scanResults[pos].bssid[4], &_scanResults[pos].bssid[5]);//, &_scanResults[pos].primary);
		pos++;
		index1 = index2 + 1;
		if(pos == 10) 
			break;
		if(end){
			index2 = strstr(index1, "\r\nOK");
			if(index2 - index1 < 5)
			{	
				break;
			}
		}
    }
    return pos;

}
char* WiFiClass::SSID(uint8_t pos)
{
  return (char*)_scanResults[pos].ssid;
}

int32_t WiFiClass::RSSI(uint8_t pos)
{
  return _scanResults[pos].rssi;
}

uint8_t WiFiClass::encryptionType(uint8_t pos)
{
  uint8_t encryptionType = _scanResults[pos].authmode;

  if (encryptionType == 0) {
    encryptionType = 7;
  } else if (encryptionType == 1) {
    encryptionType = 5;
  } else if (encryptionType == 2) {
    encryptionType = 2;
  } else if (encryptionType == 3 || encryptionType == 4) {
    encryptionType = 4;
  } else {
    encryptionType = 255;
  }

  return encryptionType;
}

uint8_t* WiFiClass::BSSID(uint8_t pos, uint8_t* bssid)
{
  const uint32_t* tempBssid = _scanResults[pos].bssid;

  bssid[0] = tempBssid[5];
  bssid[1] = tempBssid[4];
  bssid[2] = tempBssid[3];
  bssid[3] = tempBssid[2];
  bssid[4] = tempBssid[1];
  bssid[5] = tempBssid[0];

  return bssid;
}

uint8_t WiFiClass::channel(uint8_t pos)
{
  return _scanResults[pos].primary;
}

// unsigned long WiFiClass::getTime()
// {
  // time_t now;

  // time(&now);

  // if (now < 946684800) {
    // return 0;
  // }

  // return now;
// }

// void WiFiClass::lowPowerMode()
// {
  // esp_wifi_set_ps(WIFI_PS_MODEM);
// }

// void WiFiClass::noLowPowerMode()
// {
  // esp_wifi_set_ps(WIFI_PS_NONE);
// }

// void WiFiClass::onReceive(void(*callback)(void))
// {
  // _onReceiveCallback = callback;
// }

// void WiFiClass::onDisconnect(void(*callback)(void))
// {
  // _onDisconnectCallback = callback;
// }

// err_t WiFiClass::staNetifInputHandler(struct pbuf* p, struct netif* inp)
// {
  // return WiFi.handleStaNetifInput(p, inp);
// }

// err_t WiFiClass::apNetifInputHandler(struct pbuf* p, struct netif* inp)
// {
  // return WiFi.handleApNetifInput(p, inp);
// }

// err_t WiFiClass::handleStaNetifInput(struct pbuf* p, struct netif* inp)
// {
  // err_t result = _staNetifInput(p, inp);

  // if (_onReceiveCallback) {
    // _onReceiveCallback();
  // }

  // return result;
// }

// err_t WiFiClass::handleApNetifInput(struct pbuf* p, struct netif* inp)
// {
  // err_t result = _apNetifInput(p, inp);

  // if (_onReceiveCallback) {
    // _onReceiveCallback();
  // }

  // return result;
// }

// void WiFiClass::init()
// {
  // tcpip_adapter_init();
  // esp_event_loop_init(WiFiClass::systemEventHandler, this);

  // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  // esp_wifi_init(&cfg);
  // esp_wifi_set_storage(WIFI_STORAGE_RAM);

  // sntp_setoperatingmode(SNTP_OPMODE_POLL);
  // sntp_setservername(0, (char*)"0.pool.ntp.org");
  // sntp_setservername(1, (char*)"1.pool.ntp.org");
  // sntp_setservername(2, (char*)"2.pool.ntp.org");
  // sntp_init();
  // _status = WL_IDLE_STATUS;
// }

// esp_err_t WiFiClass::systemEventHandler(void* ctx, system_event_t* event)
// {
  // ((WiFiClass*)ctx)->handleSystemEvent(event);

  // return ESP_OK;
// }

// void WiFiClass::handleSystemEvent(system_event_t* event)
// {
  // switch (event->event_id) {
    // case SYSTEM_EVENT_SCAN_DONE:
      // xEventGroupSetBits(_eventGroup, BIT2);
      // break;

    // case SYSTEM_EVENT_STA_START: {
      // struct netif* staNetif;
      // if (strlen(_hostname) == 0) {
        // uint8_t mac[6];
        // esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
        // sprintf(_hostname, "arduino-%.2x%.2x", mac[4], mac[5]);
      // }
      // tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, _hostname);

      // if (tcpip_adapter_get_netif(TCPIP_ADAPTER_IF_STA, (void**)&staNetif) == ESP_OK) {
        // if (staNetif->input != WiFiClass::staNetifInputHandler) {
          // _staNetifInput = staNetif->input;

          // staNetif->input = WiFiClass::staNetifInputHandler;
        // }
      // }

      // xEventGroupSetBits(_eventGroup, BIT0);
      // break;
    // }

    // case SYSTEM_EVENT_STA_STOP:
      // xEventGroupClearBits(_eventGroup, BIT0);
      // break;

    // case SYSTEM_EVENT_STA_CONNECTED:
      // _reasonCode = 0;

      // esp_wifi_sta_get_ap_info(&_apRecord);

      // if (_staticIp) {
       // //re-apply the custom DNS settings
        // setDNS(_dnsServers[0], _dnsServers[1]);

      // // static IP
        // _status = WL_CONNECTED;
      // }
      // break;

    // case SYSTEM_EVENT_STA_GOT_IP:
      // memcpy(&_ipInfo, &event->event_info.got_ip.ip_info, sizeof(_ipInfo));
      // _status = WL_CONNECTED;
      // break;

    // case SYSTEM_EVENT_STA_DISCONNECTED: {
      // uint8_t reason = event->event_info.disconnected.reason;

      // _reasonCode = reason;

      // memset(&_apRecord, 0x00, sizeof(_apRecord));

      // if (reason == 201/*NO_AP_FOUND*/ || reason == 202/*AUTH_FAIL*/) {
        // _status = WL_CONNECT_FAILED;
      // } else if (reason == 203/*ASSOC_FAIL*/) {
       // //try to reconnect
        // esp_wifi_connect();
      // } else {
        // _status = WL_DISCONNECTED;

        // if (_onDisconnectCallback) {
          // _onDisconnectCallback();
        // }
      // }
      // break;
    // }

    // case SYSTEM_EVENT_STA_LOST_IP:
      // memset(&_ipInfo, 0x00, sizeof(_ipInfo));
      // memset(&_dnsServers, 0x00, sizeof(_dnsServers));
      // _status = WL_CONNECTION_LOST;
      // break;

    // case SYSTEM_EVENT_AP_START: {
      // struct netif* apNetif;

      // if (tcpip_adapter_get_netif(TCPIP_ADAPTER_IF_AP, (void**)&apNetif) == ESP_OK) {
        // if (apNetif->input != WiFiClass::apNetifInputHandler) {
          // _apNetifInput = apNetif->input;

          // apNetif->input = WiFiClass::apNetifInputHandler;
        // }
      // }

      // wifi_config_t config;

      // esp_wifi_get_config(ESP_IF_WIFI_AP, &config);
      // memcpy(_apRecord.ssid, config.ap.ssid, sizeof(config.ap.ssid));
      // _apRecord.authmode = config.ap.authmode;

      // if (_staticIp) {
       // //custom static IP
        // tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
        // tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &_ipInfo);
        // tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP);

       // //re-apply the custom DNS settings
        // setDNS(_dnsServers[0], _dnsServers[1]);
      // } else {
        // tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &_ipInfo);
      // }

      // _status = WL_AP_LISTENING;
      // xEventGroupSetBits(_eventGroup, BIT1);
      // break;
    // }

    // case SYSTEM_EVENT_AP_STOP:
      // _status = WL_IDLE_STATUS;
      // memset(&_apRecord, 0x00, sizeof(_apRecord));
      // memset(&_ipInfo, 0x00, sizeof(_ipInfo));
      // xEventGroupClearBits(_eventGroup, BIT1);
      // break;

    // case SYSTEM_EVENT_AP_STACONNECTED:
      // _status = WL_AP_CONNECTED;
      // break;

    // case SYSTEM_EVENT_AP_STADISCONNECTED:
      // wifi_sta_list_t staList;

      // esp_wifi_ap_get_sta_list(&staList);

      // if (staList.num == 0) {
        // _status = WL_AP_LISTENING;
      // }
      // break;

    // default:
      // break;
  // }
// }

WiFiClass WiFi;

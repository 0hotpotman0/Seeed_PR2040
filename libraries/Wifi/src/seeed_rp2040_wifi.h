/*
  WiFi.h - Library for Arduino Wifi shield.
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

#ifndef seeed_rp2040_wifi
#define seeed_rp2040_wifi

#define WIFI_FIRMWARE_LATEST_VERSION "1.4.7"
#define WIFI_HAS_FEED_WATCHDOG_FUNC

#include "utility/WiFiClient.h"
#include "utility/WiFiMqtt.h"

#include <SPI.h>
#include <inttypes.h>


extern "C" {

}

#include "IPAddress.h"

typedef void(*FeedHostProcessorWatchdogFuncPointer)();


typedef struct _ipconfig_obj
{
	uint8_t *ip;
	uint8_t *gateway;
	uint8_t *netmask;
	uint8_t *MAC;
}ipconfig_obj;

typedef struct _wifi_ap_record_t
{
	uint32_t 	authmode;
	char 		ssid[128];
	int32_t 	rssi;
	uint32_t 	bssid[6];
	uint8_t		primary;
}wifi_ap_record_t;

#define MAX_SCAN_RESULTS 10
#define HOSTNAME_MAX_LENGTH 32

class WiFiClass
{
private:
  volatile uint8_t _status;
  volatile uint8_t _interface;
  bool _staticIp;
  ipconfig_obj _ipInfo;
  uint32_t _dnsServers[2];
  char _hostname[HOSTNAME_MAX_LENGTH+1];
  wifi_ap_record_t _scanResults[MAX_SCAN_RESULTS];
    static void init();
    unsigned long _timeout;
    FeedHostProcessorWatchdogFuncPointer _feed_watchdog_func;
public:
    WiFiClass();

    /*
     * Get firmware version
     */
    static const char* firmwareVersion();


    /* Start WiFi connection for OPEN networks
     *
     * param ssid: Pointer to the SSID string.
     */
    int begin(const char* ssid);

    /* Start WiFi connection with WEP encryption.
     * Configure a key into the device. The key type (WEP-40, WEP-104)
     * is determined by the size of the key (5 bytes for WEP-40, 13 bytes for WEP-104).
     *
     * param ssid: Pointer to the SSID string.
     * param key_idx: The key index to set. Valid values are 0-3.
     * param key: Key input buffer.
     */
    int begin(const char* ssid, uint8_t key_idx, const char* key);

    /* Start WiFi connection with passphrase
     * the most secure supported mode will be automatically selected
     *
     * param ssid: Pointer to the SSID string.
     * param passphrase: Passphrase. Valid characters in a passphrase
     *        must be between ASCII 32-126 (decimal).
     */
    int begin(const char* ssid, const char *passphrase);


    uint8_t beginAP(const char *ssid, uint8_t channel);
	uint8_t beginAP(const char *ssid, const char *key, uint8_t channel);
	uint8_t beginAP(const char *ssid, uint8_t key_idx, const char *key, uint8_t channel);

    /* Change Ip configuration settings disabling the dhcp client
        *
        * param local_ip: 	Static ip configuration
        */
    void config(uint8_t* local_ip, uint8_t* gateway, uint8_t* subnet);


    /* Change DNS Ip configuration
     *
     * param dns_server1: ip configuration for DNS server 1
     */

    void setDNS(uint32_t dns_server1, uint32_t dns_server2);


    /* Set the hostname used for DHCP requests
     *
     * param name: hostname to set
     *
     */
    void hostname(const char* name);
    /*
     * Disconnect from the network
     *
     * return: one value of wl_status_t enum
     */
    void disconnect(void);

    void end(void);

    /*
     * Get the interface MAC address.
     *
     * return: pointer to uint8_t array with length WL_MAC_ADDR_LENGTH
     */
    uint8_t* macAddress(uint8_t* mac);

    /*
     * Get the interface IP address.
     *
     * return: Ip address value
     */
    uint8_t* localIP();

    /*
     * Get the interface subnet mask address.
     *
     * return: subnet mask address value
     */
    uint8_t* subnetMask();

    /*
     * Get the gateway ip address.
     *
     * return: gateway ip address value
     */
   uint8_t* gatewayIP();

    /*
     * Return the current SSID associated with the network
     *
     * return: ssid string
     */
    const char* SSID();

    /*
      * Return the current BSSID associated with the network.
      * It is the MAC address of the Access Point
      *
      * return: pointer to uint8_t array with length WL_MAC_ADDR_LENGTH
      */
    uint8_t* BSSID(uint8_t* bssid);

    /*
      * Return the current RSSI /Received Signal Strength in dBm)
      * associated with the network
      *
      * return: signed value
      */
    int32_t RSSI();

    /*
      * Return the Encryption Type associated with the network
      *
      * return: one value of wl_enc_type enum
      */
    uint8_t	encryptionType();

    /*
     * Start scan WiFi networks available
     *
     * return: Number of discovered networks
     */
    int8_t scanNetworks();

    /*
     * Return the SSID discovered during the network scan.
     *
     * param networkItem: specify from which network item want to get the information
	 *
     * return: ssid string of the specified item on the networks scanned list
     */
    char* SSID(uint8_t networkItem);

    /*
     * Return the encryption type of the networks discovered during the scanNetworks
     *
     * param networkItem: specify from which network item want to get the information
	 *
     * return: encryption type (enum wl_enc_type) of the specified item on the networks scanned list
     */
    uint8_t	encryptionType(uint8_t networkItem);

    uint8_t* BSSID(uint8_t networkItem, uint8_t* bssid);
    uint8_t channel(uint8_t networkItem);

    /*
     * Return the RSSI of the networks discovered during the scanNetworks
     *
     * param networkItem: specify from which network item want to get the information
	 *
     * return: signed value of RSSI of the specified item on the networks scanned list
     */
    int32_t RSSI(uint8_t networkItem);

    /*
     * Return Connection status.
     *
     * return: one of the value defined in wl_status_t
     */
    uint8_t status();

    /*
     * Return The deauthentication reason code.
     *
     * return: the deauthentication reason code
     */
    uint8_t reasonCode();

    /*
     * Resolve the given hostname to an IP address.
     * param aHostname: Name to be resolved
     * param aResult: IPAddress structure to store the returned IP address
     * result: 1 if aIPAddrString was successfully converted to an IP address,
     *          else error code
     */
    int hostByName(const char* hostname, /*IPAddress*/uint8_t* result);
    unsigned long getTime();

    void lowPowerMode();
    void noLowPowerMode();

    int ping(const char* hostname, uint8_t ttl = 128);
    int ping(const String &hostname, uint8_t ttl = 128);
    int ping(IPAddress host, uint8_t ttl = 128);

    void setTimeout(unsigned long timeout);

    void setFeedWatchdogFunc(FeedHostProcessorWatchdogFuncPointer func);
    void feedWatchdog();
};

extern WiFiClass WiFi;

#endif

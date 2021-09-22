/*
  server_drv.h - Library for Arduino Wifi shield.
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

#ifndef Server_Drv_h
#define Server_Drv_h

#include "WiFi_Drv.h"
#include <inttypes.h>
//#include "utility/wifi_spi.h"

typedef enum eProtMode {TCP_MODE, UDP_MODE, TLS_MODE, UDP_MULTICAST_MODE, TLS_BEARSSL_MODE}tProtMode;

enum wl_tcp_state {
  NOTINTIALIZED = 0,
  INTIALIZED    = 1,
  IPOBTAINED   	= 2,
  CONNECTED  	= 3,
  DISCONNECTED 	= 4,
  OTHER  = 5,
};

typedef struct _network_socket_obj_t {
    union {
        struct {
            uint8_t domain;
            uint8_t type;
            int8_t fileno;
        } u_param;
        uint8_t u_state;
    };
    int8_t fd;
    float timeout;
    bool peer_closed;
    bool first_read_after_write;
} network_socket_obj_t;

typedef struct _mqttconn_obj
{
	int LinkID;
	int state;
	int scheme;
	char host[128];
	int port;
	char path[32];
	int reconnect;
}mqttconn_obj;

class ServerDrv
{
public:

    // Start server TCP on port specified
    static void startServer(uint16_t port, uint8_t sock, uint8_t protMode=TCP_MODE);

    static void startServer(uint32_t ipAddress, uint16_t port, uint8_t sock, uint8_t protMode=TCP_MODE);

    static void startClient(char* ipAddress, uint16_t port, uint8_t sock, uint8_t protMode=TCP_MODE);

    static void startClient(const char* host, uint8_t host_len, uint32_t ipAddress, uint16_t port, uint8_t sock, uint8_t protMode=TCP_MODE);

    static void stopClient(uint8_t sock);
                                                                                  
    static uint8_t getServerState(uint8_t sock);

    static uint8_t getClientState(uint8_t sock);

    static bool getData(uint8_t sock, uint8_t *data, uint8_t peek = 0);

    static bool getDataBuf(uint8_t sock, uint8_t *data, uint16_t *len);

    static bool insertDataBuf(uint8_t sock, const uint8_t *_data, uint16_t _dataLen);

    static uint16_t sendData(uint8_t sock, const uint8_t *data, uint16_t len);

    static bool sendUdpData(uint8_t sock);

    static uint16_t availData(uint8_t sock);

    static uint8_t availServer(uint8_t sock);

    static uint8_t checkDataSent(uint8_t sock);

    static uint8_t getSocket();
};
bool get_mqttconn(mqttconn_obj* mqttconn);
int esp_recv(char* buffer, uint16_t buffer_size, uint16_t* read_len, uint32_t timeout, bool* peer_closed,  bool first_time_recv);
bool sATCIPSTARTSingle(const char* type, char* addr, uint16_t port);
bool eATCIPCLOSESingle();
bool eATCIPSTATUS(int* list);
bool sATCIPSENDSingle(char* buffer, uint32_t len, uint32_t timeout);
bool sMQTTUSERCFG(int LinkID, int scheme, const char* client_id, const char* username, const char* password, int cert_key_ID, int CA_ID, const char* path);
bool sMQTTUSERNAME(int LinkID, char* username);
bool sMQTTPASSWORD(int LinkID, char* password);
bool sMQTTCONNCFG(int LinkID, int keepalive, int disable_clean_session, const char* lwt_topic, const char* wt_msg, int lwt_qos, int lwt_retain);
bool sMQTTCONN(int LinkID, const char* host, int port, int reconnect);
bool qMQTTCONN();
bool sMQTTPUB( int LinkID,  const char* topic,  const char* data, uint8_t qos, uint8_t retain);
bool qMQTTSUB( int LinkID,  const char* topic, uint8_t qos);
bool eMQTTSUB_Start();
bool eMQTTSUB_Get( bool* end);
bool sMQTTUNSUB( int LinkID,  const char* topic);
bool sMQTTCLEAN( int LinkID);
extern ServerDrv serverDrv;
#endif

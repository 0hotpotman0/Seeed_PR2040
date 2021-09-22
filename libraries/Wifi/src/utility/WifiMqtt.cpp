#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
extern "C" {
  #include "wl_definitions.h"
  #include "wl_types.h"
  #include "string.h"
  #include "debug.h"
}


#include "server_drv.h"
#include "wifi_drv.h"
#include "WiFiSocketBuffer.h"
#include "spi_drv.h"

#include "seeed_rp2040_wifi.h"
#include "WiFiMqtt.h"

WiFiMqtt::WiFiMqtt() {
}

int WiFiMqtt::mqtt_setcfg(const char* client_id, const char* username, const char* password, int cert_key_ID, int CA_ID, const char* path)
{
    if (false == sMQTTUSERCFG(0, 1, (char *)client_id, (char *)username, (char *)password,  cert_key_ID,  CA_ID, (char *)path))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_set_last_will(const char *name, int len, uint8_t *out_ip)
{
    return 1;
}

int WiFiMqtt::mqtt_connect(const char *server, int port, uint8_t reconnect)
{
    if (false == sMQTTCONN(0, (char *)server, port, reconnect))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_disconnect()
{
    if (false == sMQTTCLEAN(0))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_ping(const char *name, int len, uint8_t *out_ip)
{
    return 0;
}

int WiFiMqtt::mqtt_publish(const char *topic, const char *data, uint8_t qos, uint8_t retain)
{
    if (false == sMQTTPUB(0, (char *)topic, (char *)data, qos, retain))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_publish(const char *topic, int32_t data, uint8_t qos, uint8_t retain)
{
	char str[25];
	itoa(data, str, 10);
    if (false == sMQTTPUB(0, (char *)topic, str, qos, retain))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_publish(const char *topic, float data, uint8_t qos, uint8_t retain)
{
	char str[25];
	itoa(data, str, 10);
    if (false == sMQTTPUB(0, (char *)topic, str, qos, retain))
    {
        return 0;
    }  
    return 1;
}
int WiFiMqtt::mqtt_subscribe(const char *topic, uint8_t qos)
{
    if (false == qMQTTSUB(0, (char *)topic, qos))
    {
        return 0;
    }  
    return 1;
}

int WiFiMqtt::mqtt_set_callback(mqtt_callback callback_fn) {
	if (callback_fn != NULL) {
       mqtt_callback_fn = callback_fn;
    }
    return 1;
}

int WiFiMqtt::mqtt_wait_msg()
{	
	char topic[128] , msg[256];
    while (!SpiDrv::get_mqttsubrecv(0, topic, msg) > 0) ;
	mqtt_callback_fn(topic, msg);
    return 1;
}

int WiFiMqtt::mqtt_check_msg()
{
    char topic[128] , msg[256];
    if (SpiDrv::get_mqttsubrecv(0, topic, msg) > 0) 
    {
		mqtt_callback_fn(topic, msg);
    } 
    return 1;
}



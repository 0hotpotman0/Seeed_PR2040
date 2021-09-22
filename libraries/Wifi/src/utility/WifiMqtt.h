#ifndef wifimqtt_h
#define wifimqtt_h
#include "Arduino.h"	
#include "Print.h"
#include "IPAddress.h"


typedef void (*mqtt_callback)(const char * topic, const char * msg);
class WiFiMqtt 
{

public:
	WiFiMqtt();
	int mqtt_setcfg(const char* client_id, const char* username, const char* password, int cert_key_ID, int CA_ID, const char* path);
	int mqtt_set_last_will(const char *name, int len, uint8_t *out_ip);
	int mqtt_connect(const char *server, int port, uint8_t reconnect);
	int mqtt_disconnect();
	int mqtt_ping(const char *name, int len, uint8_t *out_ip);
	int mqtt_publish(const char *topic, const char *data, uint8_t qos, uint8_t retain);
	int mqtt_publish(const char *topic, int32_t data, uint8_t qos, uint8_t retain);
	int mqtt_publish(const char *topic, float data, uint8_t qos, uint8_t retain);
	int mqtt_subscribe(const char *topic, uint8_t qos);
	int mqtt_wait_msg();
	int mqtt_set_callback(mqtt_callback callback_fn);
	int mqtt_check_msg();



  friend class WiFiServer;
  friend class WiFiDrv;


private:
	mqtt_callback mqtt_callback_fn;
};

#endif


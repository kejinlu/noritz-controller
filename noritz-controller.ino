#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>


//2.4G Wifi网络信息
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

//  MQTT Broker信息
#define MQTT_HOST IPAddress(192, 168, 1, 5)
#define MQTT_PORT 1883

//MQTT Topic
#define MQTT_PUB_Output "noritz/state"
#define MQTT_SUB_Topic "noritz/cmd"

unsigned long previousMillis = 0;   
const long interval = 5000; 

const int RX_PIN = 34;
const int TX_PIN = 32;

bool Power_Prv_state = false;
bool Heating_Prv_state = false;
bool test = false;

unsigned long duration;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_Topic, 1);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void delayNow(long microSeconds)
{
   const unsigned long start = micros();
   while (true)
   {
       unsigned long now = micros();
       if (now - start >= microSeconds)
       {
           return;
       }
   }
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, messageTemp.c_str());
  const char* power = doc["power"];
  if (strcmp(power, "ON") == 0) {
    // 开机
    if(Power_Prv_state == false) {
      digitalWrite(TX_PIN, HIGH);
      delayNow(50000);
      digitalWrite(TX_PIN, LOW);
    }

  } else if(strcmp(power, "OFF") == 0) {
    // 关机
    if(Power_Prv_state == true) {
      digitalWrite(TX_PIN, HIGH);
      delayNow(50000);
      digitalWrite(TX_PIN, LOW);
      
      delayNow(50000);
      // check
      duration = pulseIn(RX_PIN, HIGH, 40000);
      if (duration != 0) {
        digitalWrite(TX_PIN, HIGH);
        delayNow(50000);
        digitalWrite(TX_PIN, LOW);
      }
    }
  }
  Serial.println(power);
  Serial.println("Publish received.");
  Serial.print("  message: ");
  Serial.println(messageTemp);
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}


void setup() {
  Serial.begin(921600);
  Serial.println();
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < interval) {
    return;
  }
  duration = pulseIn(RX_PIN, HIGH, 40000);
  if (duration == 0) {
    // 关机状态
    if (Power_Prv_state == true || test == false) {
      StaticJsonDocument<300> doc;
      doc["power"] = "OFF";
      doc["heating"] = "OFF";
      char json[300];
      serializeJson(doc, json);
      Serial.println("Sending message to MQTT topic..");
      Serial.println(json);
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Output, 1, true, json);
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Output, packetIdPub1);

      Power_Prv_state = false;
      Heating_Prv_state = false;
    }
  } else if (duration > 0 && duration < 150) {
    // 燃烧状态
    if (Heating_Prv_state == false || Power_Prv_state == false || test == false) {

      StaticJsonDocument<300> doc;
      doc["power"] = "ON";
      doc["heating"] = "ON";
      char json[300];
      serializeJson(doc, json);
      Serial.println("Sending message to MQTT topic..");
      Serial.println(json);
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Output, 1, true, json);
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Output, packetIdPub1);

      Heating_Prv_state = true;
      Power_Prv_state = true;
    }
  } else if (duration > 900) {
    // 开机状态
    if (Heating_Prv_state == true || Power_Prv_state == false || test == false) {

      StaticJsonDocument<300> doc;
      doc["power"] = "ON";
      doc["heating"] = "OFF";
      char json[300];
      serializeJson(doc, json);
      Serial.println("Sending message to MQTT topic..");
      Serial.println(json);
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Output, 1, true, json);
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Output, packetIdPub1);

      Heating_Prv_state = false;
      Power_Prv_state = true;
    }
  }
  if(test == false) {
    test = true;
  }
  delay(1000);
}
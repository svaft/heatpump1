#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "Wire.h"
#include "MQTT_strings.h"
#include <EEPROM.h>
#include <esp_task_wdt.h>  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html
#include <Preferences.h>   // this library is used to get access to Non-volatile storage (NVS) of ESP32



#define WDT_TIMEOUT 3      // define a 3 seconds WDT (Watch Dog Timer)
int BootReason = 99;

int address = 0;  // адрес памяти для записи (от 0 до 511)
byte value_w;     // значение данных (от 0 до 255)


TaskHandle_t Task1;

typedef struct {
  uint8_t header[4];
  uint16_t temp[10];
  uint32_t flow1;
  uint32_t flow2;
  uint32_t duration;
  uint8_t state_mask[2];
  uint8_t end[2];
} state_t;
state_t state;



//---- WiFi settings
const char* ssid = "svaft";
const char* password = "multipas8578";
//---- MQTT Broker settings
/*
const char* mqtt_server = "00457fac79744078af69601e59bae1e9.s2.eu.hivemq.cloud";  // replace with your broker url
const char* mqtt_username = "svaft";
const char* mqtt_password = "loadgame";
const int mqtt_port = 8883;
*/
const char* mqtt_server = "mqtt.cloud.yandex.net";  // replace with your broker url
//const char* mqtt_username = "are0jdpfnhs98n625k0g";
//const char* mqtt_password = "w9EnFKyQ-6r4znw";
const int mqtt_port = 8883;

//const char* deviceEvents = "$devices/are0jdpfnhs98n625k0g/events";
//const char* command1_topic = "$devices/are0jdpfnhs98n625k0g/commands";
//const char* command1_topic = "$devices/are0jdpfnhs98n625k0g/commands";

const char* mqtt_username = "a447gp37f80udg17uto9";
const char* command1_topic = "/my/custom/commands";
const char* deviceEvents = "/my/custom/topic";
const char* mqtt_password = "w9EnFKyQ-6r4znw";



WiFiClient myclient;
WiFiServer server(80);



WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

#define every(t) for (static unsigned long _lasttime; (unsigned long)((unsigned long)millis() - _lasttime) >= (t); _lasttime = millis())

byte stateBuf[40];

#define every(t) for (static unsigned long _lasttime; (unsigned long)((unsigned long)millis() - _lasttime) >= (t); _lasttime = millis())

uint32_t zero = 0;
uint8_t requestReady = 0;
byte commandsBuf[4];
byte* iRef = (byte*)&commandsBuf;
void onRequest() {
  if (requestReady == 1) {
    requestReady = 0;
    Wire.write(iRef, 4);
    *(int*)&commandsBuf = 0;
  } else {
    Wire.write((byte*)&zero, 4);
  }
  //  Wire.print("AA");
  //  Serial.println("onRequest");
}
volatile uint8_t receiveFlag = 0;
bool receiveFlagMessagePrint = false;
void onReceive(int len) {
  if (len != 40) {
    Serial.printf("!!!!!!!!!!!onReceive len error: [%d]", len);
    for (int a = 0; a < len; a++) {
      Serial.write(Wire.read());
    }
    return;
  }
  uint8_t* stateRef = (uint8_t*)&state;
  //byte *stateBufPtr = (byte *)&stateBuf;

  for (int a = 0; a < 40; a++) {
    uint8_t inByte = Wire.read();
    *stateRef = inByte;
    stateRef++;
  }
  receiveFlag = 1;
//  Serial.print("i2c:" + String(xPortGetCoreID()));

  //  dump_byte_array((byte *)&state, 40);
}

void dump_byte_array(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}



void setup() {
  delay(2000);
  state.header[0] = 0xde;
  state.header[1] = 0xad;
  state.header[2] = 0;
  state.header[3] = sizeof(state_t) - 4;

  state.state_mask[0] = 0;
  state.state_mask[1] = 0;

  state.flow1 = 0;
  state.flow2 = 0;
  state.duration = 0;
  state.end[0] = '\r';
  state.end[1] = '\n';

  //  Serial.begin(9600);
  Serial.begin(115200);
  //  delay(300);
  while (!Serial) delay(3);



  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                // add current thread to WDT watch

  esp_reset_reason_t resetReason = esp_reset_reason();
  switch (resetReason) {
    case ESP_RST_UNKNOWN:
      Serial.println("Reset reason can not be determined");
      break;
    case ESP_RST_POWERON:
      Serial.println("Reset due to power-on event");
      break;
    case ESP_RST_EXT:
      Serial.println("Reset by external pin (not applicable for ESP32)");
      break;
    case ESP_RST_SW:
      Serial.println("Software reset via esp_restart");
      break;
    case ESP_RST_PANIC:
      Serial.println("Software reset due to exception/panic");
      break;
    case ESP_RST_INT_WDT:
      Serial.println("Reset (software or hardware) due to interrupt watchdog");
      break;
    case ESP_RST_TASK_WDT:
      Serial.println("Reset due to task watchdog");
      break;
    case ESP_RST_WDT:
      Serial.println("Reset due to other watchdogs");
      break;
    case ESP_RST_DEEPSLEEP:
      Serial.println("Reset after exiting deep sleep mode");
      break;
    case ESP_RST_BROWNOUT:
      Serial.println("Brownout reset (software or hardware)");
      break;
    case ESP_RST_SDIO:
      Serial.println("Reset over SDIO");
      break;
    default:
      break;
  }


  EEPROM.begin(512);               // Инициализация EEPROM с размером 512 байт
  value_w = EEPROM.read(address);  // Чтение данных
  value_w++;
  EEPROM.write(address, value_w);  // Запись данных
  EEPROM.commit();                 // Сохранение изменений




  byte value_r = EEPROM.read(address);  // Чтение данных

  // Используйте значение данных (value) по вашему усмотрению
  Serial.println();
  Serial.print("Номер включения устройства: ");
  Serial.println(value_r);



#define I2C_DEV_ADDR 0x11

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)17, 21, 22, (uint32_t)100000);
#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", 0);
  Wire.slaveWrite((uint8_t*)message, strlen(message));
#endif


  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  espClient.setCACert(root_ca_yandex);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(300);
#define LED 2
  pinMode(LED, OUTPUT);


  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */   



}

int count = 0;
int delay_ms = 5000;

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  every(1000) {
    digitalWrite(LED, !digitalRead(LED));
    if (receiveFlag == 1) {
      if (receiveFlagMessagePrint == false) {
        Serial.print("core: " + String(xPortGetCoreID()));
        Serial.print("get message from stm32, wait to publish it to mqtt:");
        receiveFlagMessagePrint = true;
      } else {
        Serial.print(".");
      }
    }
    esp_task_wdt_reset();  //reset wd timer
  }
  every(delay_ms) {
    //    if(RxByte == '\n')
    //      Serial.println("i2c");

    if (count++ == 5)
      delay_ms = 60000;  // every 1 minute
    if (count == 20)
      delay_ms = 120000;  // every 1 minute
    if (receiveFlag == 1) {
      publishMessageBin(deviceEvents, (byte*)&state, sizeof(state_t), true);
      receiveFlag = 0;
      receiveFlagMessagePrint = 0;
    }
  }
}

//=======================================================================Function=================================================================================

void reconnect() {
  // Loop until we’re reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    String clientId = "ESP32Client-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(command1_topic, MQTTQOS1);  // subscribe the topics here
      //client.subscribe(command2_topic);   // subscribe the topics here
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");  // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//=======================================
// This void is called every time we have a message fro m the broker

void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) {
    incommingMessage += (char)payload[i];
    commandsBuf[i] = payload[i];
  }
  requestReady = 1;
  Serial.print("core: " + String(xPortGetCoreID()));
  Serial.println("Message arrived [" + String(topic) + "]" + incommingMessage);

  // check for other commands
  /* else if( strcmp(topic,command2_topic) == 0){
if (incommingMessage.equals(“1”)) { } // do something else
}
*/
}

//======================================= publising as string
void publishMessage(const char* topic, String payload, boolean retained) {
  if (receiveFlag == 1) {
    if (client.publish(topic, payload.c_str(), true))
      Serial.println(String(count) + "Message publised [" + String(topic) + "]: " + payload);
    receiveFlag = 0;
  }
}

void publishMessageBin(const char* topic, const uint8_t* buf, int len, boolean retained) {
  if (receiveFlag == 1) {
    if (client.publish(topic, buf, len, true)) {
      Serial.print("core: " + String(xPortGetCoreID()));
      Serial.print("message " + String(count) + " publised to [" + String(topic) + "]: ");
      dump_byte_array((byte *)buf, len);
      Serial.println();
    }
    receiveFlag = 0;
  }
}


//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
//    digitalWrite(led1, HIGH);
    delay(1000);
//    digitalWrite(led1, LOW);
//    delay(1000);
  } 
}

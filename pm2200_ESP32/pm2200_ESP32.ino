#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

ModbusMaster mb;

// Register pm2200 power meter_RS485
#define SerialRS485_RX_PIN 16
#define SerialRS485_TX_PIN 17

#define SLAVE_ADDRESS_1 1
#define RESET_BUTTON_PIN 0

const char* mqttServer = "141.98.16.17";
const int mqttPort = 1883;
const char* mqttUser = "admin";
const char* mqttPassword = "admin";
const char* device_name = "SDM_MQTT1";

WiFiClient espClient;
PubSubClient client(espClient);

int count = 0;
WiFiManager wifiManager;

// add de bounce
bool isResetButtonHeld() {
  int holdTime = 0;
  const int debounceDelay = 50;
  bool buttonState = LOW;
  bool lastButtonState = HIGH;

  while (digitalRead(RESET_BUTTON_PIN) == LOW) {
    delay(debounceDelay);
    buttonState = digitalRead(RESET_BUTTON_PIN);


    if (buttonState == LOW) {
      holdTime += debounceDelay;
    } else {
      return false;
    }

    if (holdTime >= 3000) {
      Serial.println("Reset button held for 3 seconds!");
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  delay(1000);

  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  mb.begin(SLAVE_ADDRESS_1, Serial2);
}

void setup_wifi() {
  if (isResetButtonHeld()) {
    Serial.println("Resetting WiFi settings and entering AP mode...");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }

  wifiManager.autoConnect("ESP32-Setup");

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT Connection...");
    if (client.connect(device_name, mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT Broker");
      client.subscribe("My_Topic");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);
    }
  }
}

void checkResetButton() {
  if (isResetButtonHeld()) {
    Serial.println("Reset button pressed, restarting WiFi setup...");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }
}

// Convert HEX to Float
float ToFloat(uint32_t z) {
  return (*(float*)&z);
}

void readpm2200() {
  uint32_t dat_sum;
  float amp_avg, vll_avg, vln_avg, ptotal;
  uint16_t data[2];
  uint8_t result;
  int i;

  result = mb.readHoldingRegisters(3025, 2);  // Read 2 registers starting from address 1
  delay(500);

  if (result == mb.ku8MBSuccess) {
    for (i = 0; i < 2; i++) {
      data[i] = mb.getResponseBuffer(i);
    }
    dat_sum = data[0];
    dat_sum = dat_sum << 16;
    dat_sum = dat_sum + data[1];

    vll_avg = ToFloat(dat_sum);

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["Power_meter"] = device_name;
    jsonDoc["VL2L"] = vll_avg;

    char jsonMessage[200];
    serializeJson(jsonDoc, jsonMessage);

    client.publish("VL2L", jsonMessage);

    Serial.println(jsonMessage);  // Print only JSON format output
    Serial.print("VL-L_avg : ");
    Serial.print(vll_avg);
    Serial.println("V");
  }

  result = mb.readHoldingRegisters(3035, 2);  // Read 2 registers starting from address 1
  delay(500);
  if (result == mb.ku8MBSuccess) {
    for (i = 0; i < 2; i++) {
      data[i] = mb.getResponseBuffer(i);
    }
    dat_sum = data[0];
    dat_sum = dat_sum << 16;
    dat_sum = dat_sum + data[1];

    vln_avg = ToFloat(dat_sum);

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["Power_meter"] = device_name;
    jsonDoc["VL2N"] = vln_avg;

    char jsonMessage[200];
    serializeJson(jsonDoc, jsonMessage);

    client.publish("VL2N", jsonMessage);

    Serial.println(jsonMessage);  // Print only JSON format output
    Serial.print("VL-N avg : ");
    Serial.print(vln_avg);
    Serial.println("V");
  }

  result = mb.readHoldingRegisters(3009, 2);  // Read 2 registers starting from address 1
  delay(500);
  if (result == mb.ku8MBSuccess) {
    for (i = 0; i < 2; i++) {
      data[i] = mb.getResponseBuffer(i);
    }
    dat_sum = data[0];
    dat_sum = dat_sum << 16;
    dat_sum = dat_sum + data[1];

    amp_avg = ToFloat(dat_sum);

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["Power_meter"] = device_name;
    jsonDoc["Iavg"] = amp_avg;

    char jsonMessage[200];
    serializeJson(jsonDoc, jsonMessage);

    client.publish("I_avg", jsonMessage);

    Serial.println(jsonMessage);  // Print only JSON format output
    Serial.print("I avg : ");
    Serial.print(amp_avg);
    Serial.println("A");
  }

  result = mb.readHoldingRegisters(3059, 2);  // Read 2 registers starting from address 1
  delay(500);
  if (result == mb.ku8MBSuccess) {
    for (i = 0; i < 2; i++) {
      data[i] = mb.getResponseBuffer(i);
    }
    dat_sum = data[0];
    dat_sum = dat_sum << 16;
    dat_sum = dat_sum + data[1];

    ptotal = ToFloat(dat_sum);

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["Power_meter"] = device_name;
    jsonDoc["Powertotal"] = ptotal;

    char jsonMessage[200];
    serializeJson(jsonDoc, jsonMessage);

    client.publish("PowerAll", jsonMessage);

    Serial.println(jsonMessage);  // Print only JSON format output
    Serial.print("Power total : ");
    Serial.print(ptotal);
    Serial.println("Wh");
  }

  else {
    Serial.print("FAILED. Error code: 0x");
    Serial.println(result, HEX);
  }
}

void loop() {
  checkResetButton();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  readpm2200();

  delay(1000);
}

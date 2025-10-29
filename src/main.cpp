#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "config_reader.h"
#include "WifiManager.h"
#include "MqttManager.h"
#include "ControlsManager.h"

JsonDocument     config;
Wifi             wifiManager;
MqttManager*     mqttManager = nullptr;
ControlsManager* controlsManager = nullptr;

// MQTT callback wrapper to convert from C-style to C++ style
void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  std::string topicStr(topic);
  std::string payloadStr((char*)payload, length);
  
  if (controlsManager) {
    controlsManager->handleMqttMessage(topicStr, payloadStr);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Load configuration
  if (!loadConfig("/config.json", config)) {
    Serial.println("Failed to load config - halting");
    while (true) { delay(1000); }
  }
  Serial.println("Config loaded successfully!");

  // ---- Wi-Fi ----
  WifiSettings wifiSettings;
  wifiSettings.ssid     = config["wifi"]["ssid"].as<std::string>();
  wifiSettings.password = config["wifi"]["psk"].as<std::string>();

  Serial.printf("Connecting to WiFi: %s\n", wifiSettings.ssid.c_str());
  wifiManager.begin(wifiSettings);
  if (!wifiManager.connected()) {
    Serial.println("WiFi connection failed - halting");
    while (true) { delay(1000); }
  }

  // ---- MQTT ----
  mqttManager = new MqttManager(wifiManager.client());
  if (!mqttManager->configureFromJson(config)) {
    Serial.println("MQTT configuration failed - halting");
    while (true) { delay(1000); }
  }

  // Wait until MQTT connects, then publish LWT "online"
  unsigned long start = millis();
  const unsigned long timeout_ms = 15000; // simple safety timeout
  while (!mqttManager->connected() && (millis() - start < timeout_ms)) {
    mqttManager->loop();
    delay(50);
  }

  if (mqttManager->connected()) {
    mqttManager->publishOnlineStatus();  // publishes the configured LWT "online" payload
    Serial.println("[mqtt] Online status published");
  } else {
    Serial.println("[mqtt] Could not connect within timeout; continuing to loop()");
  }

  // ---- Controls ----
  controlsManager = new ControlsManager(mqttManager);
  if (!controlsManager->loadFromConfig(config)) {
    Serial.println("Controls configuration failed - halting");
    while (true) { delay(1000); }
  }

  // Set up MQTT message callback using C-style callback
  mqttManager->setCallback(onMqttMessage);

  Serial.println("Setup complete!");
}

void loop() {
  wifiManager.loop();
  if (mqttManager) mqttManager->loop();
  if (controlsManager) controlsManager->loop();
  delay(50);
}
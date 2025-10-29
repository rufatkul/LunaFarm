#pragma once
#include <ArduinoJson.h>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <functional>
#include "base_control.h"
#include "switch.h"
#include "dummy.h"

// Forward declaration
class MqttManager;

class ControlsManager {
public:
  ControlsManager(MqttManager* mqtt) : mqttManager_(mqtt), lastStatePublish_(0) {}

  // Read the "controls" array from the loaded configuration and build controls
  bool loadFromConfig(const JsonDocument& cfg) {
    controls_.clear();

    // Use const types consistently with ArduinoJson v7
    JsonVariantConst controlsVar = cfg["controls"];
    if (controlsVar.isNull()) {
      Serial.println("[controls] Missing 'controls' in config");
      return false;
    }

    // Try to get as const array
    JsonArrayConst arr = controlsVar.as<JsonArrayConst>();
    if (arr.isNull()) {
      Serial.println("[controls] 'controls' is not an array");
      return false;
    }

    Serial.printf("[controls] Found controls array with %d items\n", arr.size());

    for (JsonVariantConst item : arr) {
      JsonObjectConst dev = item.as<JsonObjectConst>();
      const char* type = dev["type"] | "";
      const char* id   = dev["id"]   | "";

      Serial.printf("[controls] Processing: type='%s', id='%s'\n", type, id);

      if (!*type || !*id) {
        Serial.println("[controls] Skipping item: missing type or id");
        continue;
      }

      // Use the generic helper method  
      if (strcmp(type, "switch") == 0) {
        addControl(type, id, [this, dev]() { return buildSwitch_(dev); });
      } else if (strcmp(type, "dummy_switch") == 0) {
        addControl(type, id, [this, dev]() { return buildDummySwitch_(dev); });
      } else {
        Serial.printf("[controls] Skipping unsupported type '%s' (id=%s)\n", type, id);
      }
    }

    Serial.printf("[controls] Successfully loaded %d controls\n", controls_.size());

    // Subscribe to command topics
    subscribeToCommandTopics();
    
    // Publish initial states
    publishAllStates();

    return true;
  }

  // Call this from main loop() - handles periodic state publishing
  void loop() {
    unsigned long now = millis();
    
    // Publish states every 500ms
    if (now - lastStatePublish_ >= 500) {
      publishAllStates();
      lastStatePublish_ = now;
    }
  }

  // Handle incoming MQTT messages
  void handleMqttMessage(const std::string& topic, const std::string& payload) {
    Serial.printf("[controls] Received MQTT: topic='%s', payload='%s'\n", topic.c_str(), payload.c_str());
    
    for (auto& control : controls_) {
      // Check if this message is for this control's command topics
      for (const auto& cmdTopic : control->cmd_topics()) {
        if (topic == cmdTopic) {
          Serial.printf("[controls] Message matches control '%s' command topic\n", control->id().c_str());
          
          // Parse JSON payload
          JsonDocument cmdDoc;
          DeserializationError error = deserializeJson(cmdDoc, payload);
          
          if (error) {
            Serial.printf("[controls] JSON parse error: %s. Trying as plain text.\n", error.c_str());
            // Fallback to plain text for backward compatibility
            BaseControl::StateVector states = {{"state", payload}};
            bool stateChanged = control->update_state(states);
            
            if (stateChanged) {
              Serial.printf("[controls] State changed via plain text for '%s'\n", control->id().c_str());
              publishControlState(*control);
            }
          } else {
            // Successfully parsed JSON
            Serial.println("[controls] Successfully parsed JSON command");
            
            // Extract all key-value pairs from JSON
            BaseControl::StateVector states;
            for (JsonPair kv : cmdDoc.as<JsonObject>()) {
              std::string key = kv.key().c_str();
              std::string value;
              
              // Convert different JSON types to string
              if (kv.value().is<const char*>()) {
                value = kv.value().as<const char*>();
              } else if (kv.value().is<bool>()) {
                value = kv.value().as<bool>() ? "true" : "false";
              } else if (kv.value().is<int>()) {
                value = std::to_string(kv.value().as<int>());
              } else if (kv.value().is<float>()) {
                value = std::to_string(kv.value().as<float>());
              } else {
                // For other types, convert to string representation
                String temp;
                serializeJson(kv.value(), temp);
                value = temp.c_str();
              }
              
              Serial.printf("[controls] Extracted: %s = %s\n", key.c_str(), value.c_str());
              states.emplace_back(key, value);
            }
            
            bool stateChanged = control->update_state(states);
            Serial.printf("[controls] State changed: %s\n", stateChanged ? "YES" : "NO");
            
            if (stateChanged) {
              Serial.printf("[controls] Publishing immediate state update for '%s'\n", control->id().c_str());
              publishControlState(*control);
            }
          }
          return;
        }
      }
    }
    Serial.printf("[controls] No matching control found for topic: %s\n", topic.c_str());
  }

  // Publish state for a specific control as JSON (CLEAN VERSION)
  void publishControlState(const BaseControl& control) {
    if (!mqttManager_ || !mqttManager_->connected()) {
      Serial.println("[controls] Cannot publish: MQTT not connected");
      return;
    }
    
    auto states = control.get_state();
    Serial.printf("[controls] Publishing JSON state for '%s':\n", control.id().c_str());
    
    // Create JSON object with all state variables
    JsonDocument stateDoc;
    JsonObject stateObj = stateDoc.to<JsonObject>();
    
    for (const auto& state : states) {
      const std::string& key = state.first;
      const std::string& value = state.second;
      
      // Try to parse value as different types for better JSON representation
      if (value == "true" || value == "TRUE") {
        stateObj[key] = true;
      } else if (value == "false" || value == "FALSE") {
        stateObj[key] = false;
      } else if (value == "ON") {
        stateObj[key] = true;  // Convert ON/OFF to boolean
      } else if (value == "OFF") {
        stateObj[key] = false; // Convert ON/OFF to boolean
      } else {
        // Try to parse as number
        char* endPtr;
        long longVal = strtol(value.c_str(), &endPtr, 10);
        if (*endPtr == '\0') {
          // Successfully parsed as integer
          stateObj[key] = longVal;
        } else {
          // Try as float
          float floatVal = strtof(value.c_str(), &endPtr);
          if (*endPtr == '\0') {
            stateObj[key] = floatVal;
          } else {
            // Keep as string
            stateObj[key] = value;
          }
        }
      }
    }
    
    // Convert JSON to string
    String jsonString;
    serializeJson(stateDoc, jsonString);
    
    // Publish ONLY to the main state topic (no sub-topics)
    for (const auto& stateTopic : control.state_topics()) {
      Serial.printf("[controls] Publishing to: %s = %s\n", stateTopic.c_str(), jsonString.c_str());
      mqttManager_->publish(stateTopic, jsonString.c_str(), true); // retain=true
    }
  }

  // Publish all control states
  void publishAllStates() {
    for (const auto& control : controls_) {
      publishControlState(*control);
    }
  }

  // Access the controls
  const std::vector<std::unique_ptr<BaseControl>>& controls() const { 
    return controls_.empty() ? empty_ : controls_; 
  }
  std::vector<std::unique_ptr<BaseControl>>& controls() { 
    return controls_; 
  }

private:
  // Generic helper to add controls with consistent error handling and logging
  void addControl(const char* type, const char* id, std::function<std::unique_ptr<BaseControl>()> factory) {
    auto control = factory();
    if (control) {
      Serial.printf("[controls] Added %s: %s\n", type, id);
      controls_.push_back(std::move(control));
    } else {
      Serial.printf("[controls] Failed to build %s: %s\n", type, id);
    }
  }

  // Subscribe to all command topics
  void subscribeToCommandTopics() {
    if (!mqttManager_) return;
    
    for (const auto& control : controls_) {
      for (const auto& cmdTopic : control->cmd_topics()) {
        if (mqttManager_->subscribe(cmdTopic)) {
          Serial.printf("[controls] Subscribed to: %s\n", cmdTopic.c_str());
        }
      }
    }
  }

  // Build a Switch from one JSON object
  std::unique_ptr<BaseControl> buildSwitch_(JsonObjectConst dev) {
    const char* id = dev["id"] | "";
    if (!*id) return nullptr;

    // pins.out (optional, default -1)
    int out_pin = -1;
    if (dev["pins"] && dev["pins"]["out"]) {
      out_pin = dev["pins"]["out"].as<int>();
    }

    // initial_state.on (optional, default false)
    bool initial_on = false;
    if (dev["initial_state"] && dev["initial_state"]["on"]) {
      initial_on = dev["initial_state"]["on"].as<bool>();
    }

    // topics (cmd, state) — optional; if missing, we just keep empty vectors
    std::vector<std::string> cmd_topics;
    std::vector<std::string> state_topics;

    if (dev["topics"]) {
      JsonObjectConst t = dev["topics"];
      if (t["cmd"])   cmd_topics.emplace_back(t["cmd"].as<const char*>());
      if (t["state"]) state_topics.emplace_back(t["state"].as<const char*>());
    }

    return std::unique_ptr<BaseControl>(new Switch(std::string{id}, initial_on, out_pin,
                                                   std::move(cmd_topics), std::move(state_topics)));
  }

  // Build a DummySwitch from one JSON object
  std::unique_ptr<BaseControl> buildDummySwitch_(JsonObjectConst dev) {
    const char* id = dev["id"] | "";
    if (!*id) return nullptr;

    // initial_state.on (optional, default false)
    bool initial_on = false;
    if (dev["initial_state"] && dev["initial_state"]["on"]) {
      initial_on = dev["initial_state"]["on"].as<bool>();
    }

    // topics (cmd, state) — optional; if missing, we just keep empty vectors
    std::vector<std::string> cmd_topics;
    std::vector<std::string> state_topics;

    if (dev["topics"]) {
      JsonObjectConst t = dev["topics"];
      if (t["cmd"])   cmd_topics.emplace_back(t["cmd"].as<const char*>());
      if (t["state"]) state_topics.emplace_back(t["state"].as<const char*>());
    }

    return std::unique_ptr<BaseControl>(new DummySwitch(std::string{id}, initial_on,
                                                        std::move(cmd_topics), std::move(state_topics)));
  }

  std::vector<std::unique_ptr<BaseControl>> controls_;
  MqttManager* mqttManager_;
  static std::vector<std::unique_ptr<BaseControl>> empty_;
  unsigned long lastStatePublish_;  // Track when we last published states
};

// Define the static member outside the class (C++11 compatible)
std::vector<std::unique_ptr<BaseControl>> ControlsManager::empty_{};
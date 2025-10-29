#pragma once
#include <Arduino.h>
#include <Client.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string>
#include <vector>
#include <functional>

struct LwtSettings {
  std::string topic;
  std::string payloadOffline;
  std::string payloadOnline;
  bool retain = true;
};

struct MqttCredentials {
  std::string username;
  std::string password;
};

class MqttManager {
public:
  // Use PubSubClient's callback signature (raw function pointer)
  using MqttCallback = void (*)(char*, uint8_t*, unsigned int);
  using ConnectionCallback = std::function<void(bool connected)>;
  
  MqttManager(Client& net) : client_(net) {}

  void setServer(const std::string& host, uint16_t port) {
    if (host.empty() || port == 0) {
      Serial.println("[mqtt] Error: Invalid server configuration");
      return;
    }
    host_ = host;
    port_ = port;
    client_.setServer(host_.c_str(), port_);
    Serial.printf("[mqtt] Server set to %s:%u\n", host_.c_str(), port_);
  }

  void setClientId(const std::string& id) { 
    clientId_ = id.empty() ? generateDefaultClientId() : id; 
  }

  void setCredentials(const MqttCredentials& creds) { 
    credentials_ = creds; 
    hasCredentials_ = !creds.username.empty();
  }

  void setLwt(const LwtSettings& lwt) { 
    lwt_ = lwt; 
    hasLwt_ = !lwt.topic.empty();
  }

  // Use raw function pointer for PubSubClient compatibility
  void setCallback(MqttCallback cb) {
    client_.setCallback(cb);
  }

  void setConnectionCallback(ConnectionCallback cb) {
    connectionCallback_ = cb;
  }

  bool connected() const { return const_cast<PubSubClient&>(client_).connected(); }

  // Call in loop(); reconnects if needed
  void loop() {
    bool wasConnected = client_.connected();
    
    if (!client_.connected()) {
      reconnect();
    }
    
    client_.loop();
    
    // Notify if connection status changed
    if (connectionCallback_ && wasConnected != client_.connected()) {
      connectionCallback_(client_.connected());
    }
  }

  // String payload overload
  bool publish(const std::string& topic, const std::string& payload, bool retained = false) {
    return client_.publish(topic.c_str(), payload.c_str(), retained);
  }

  // Binary payload overload
  bool publish(const std::string& topic, const uint8_t* payload, size_t length, bool retained = false) {
    return client_.publish(topic.c_str(), payload, length, retained);
  }

  bool subscribe(const std::string& topic) {
    bool success = client_.subscribe(topic.c_str());
    if (success) {
      // Store subscription for re-subscription after reconnect
      auto it = std::find(subscriptions_.begin(), subscriptions_.end(), topic);
      if (it == subscriptions_.end()) {
        subscriptions_.push_back(topic);
      }
    }
    return success;
  }

  bool unsubscribe(const std::string& topic) {
    bool success = client_.unsubscribe(topic.c_str());
    if (success) {
      auto it = std::find(subscriptions_.begin(), subscriptions_.end(), topic);
      if (it != subscriptions_.end()) {
        subscriptions_.erase(it);
      }
    }
    return success;
  }

  // Publish online status (call after successful connection)
  void publishOnlineStatus() {
    if (hasLwt_) {
      publish(lwt_.topic, lwt_.payloadOnline, lwt_.retain);
    }
  }

  // Configure directly from JSON
 // Replace your current configureFromJson with this:
bool configureFromJson(const JsonDocument& config) {
  // Find "mqtt" section (const-safe)
  JsonVariantConst mqttVar = config["mqtt"];
  if (mqttVar.isNull()) {
    Serial.println("[mqtt] No 'mqtt' section in config");
    return false;
  }

  JsonObjectConst mqtt = mqttVar.as<JsonObjectConst>();
  if (mqtt.isNull()) {
    Serial.println("[mqtt] 'mqtt' is not an object");
    return false;
  }

  // Validate required fields
  if (!mqtt.containsKey("server") || !mqtt.containsKey("port")) {
    Serial.println("[mqtt] Missing required server/port in config");
    return false;
  }

  const char* serverC = mqtt["server"] | "";
  uint16_t     port   = mqtt["port"].as<uint16_t>();
  if (!serverC[0] || port == 0) {
    Serial.println("[mqtt] Invalid server/port values in config");
    return false;
  }

  setServer(std::string(serverC), port);

  // Optional client_id (defaults to generated if empty)
  const char* clientIdC = mqtt["client_id"] | "";
  setClientId(std::string(clientIdC));

  // Optional credentials
  if (mqtt.containsKey("username") && mqtt.containsKey("password")) {
    MqttCredentials creds;
    creds.username = std::string(mqtt["username"].as<const char*>());
    creds.password = std::string(mqtt["password"].as<const char*>());
    setCredentials(creds);
  }

  // Optional LWT
  if (mqtt.containsKey("lwt")) {
    JsonObjectConst lwt = mqtt["lwt"].as<JsonObjectConst>();
    if (!lwt.isNull() && lwt.containsKey("topic")) {
      LwtSettings lwtSettings;
      lwtSettings.topic          = std::string(lwt["topic"].as<const char*>());
      lwtSettings.payloadOffline = std::string(lwt["payload_offline"] | "offline");
      lwtSettings.payloadOnline  = std::string(lwt["payload_online"]  | "online");
      lwtSettings.retain         = lwt["retain"] | true;
      setLwt(lwtSettings);
    }
  }

  Serial.printf("[mqtt] Configured: %s:%u (client: %s)\n",
                serverC, port, clientId_.c_str());
  return true;
}


  PubSubClient& raw() { return client_; } // escape hatch

private:
  void reconnect() {
    // Ensure we have valid configuration before attempting connection
    if (host_.empty() || port_ == 0) {
      Serial.println("[mqtt] Error: Server not configured, cannot connect");
      return;
    }
    
    // Use instance members instead of static locals
    if (millis() - lastReconnectAttempt_ < reconnectDelay_) {
      return;
    }
    
    lastReconnectAttempt_ = millis();
    
    if (!client_.connected()) {
      Serial.printf("[mqtt] Connecting to %s:%u ...\n", host_.c_str(), port_);
      
      bool connected = false;
      
      if (hasLwt_ && hasCredentials_) {
        // Connect with both LWT and credentials
        connected = client_.connect(
          clientId_.c_str(),
          credentials_.username.c_str(),
          credentials_.password.c_str(),
          lwt_.topic.c_str(),
          0, // QoS
          lwt_.retain,
          lwt_.payloadOffline.c_str()
        );
      } else if (hasLwt_) {
        // Connect with LWT only
        connected = client_.connect(
          clientId_.c_str(),
          lwt_.topic.c_str(),
          0, // QoS
          lwt_.retain,
          lwt_.payloadOffline.c_str()
        );
      } else if (hasCredentials_) {
        // Connect with credentials only
        connected = client_.connect(
          clientId_.c_str(),
          credentials_.username.c_str(),
          credentials_.password.c_str()
        );
      } else {
        // Simple connection
        connected = client_.connect(clientId_.c_str());
      }
      
      if (connected) {
        Serial.println("[mqtt] Connected");
        
        // Resubscribe to all topics
        resubscribeAll();
        
        // Publish online status
        publishOnlineStatus();
        
        // Reset delay on successful connection
        reconnectDelay_ = 2000;
      } else {
        Serial.printf("[mqtt] Failed, rc=%d. Next retry in %ums\n", 
                     client_.state(), reconnectDelay_);
        
        // Optional: implement exponential backoff
        // reconnectDelay_ = min(reconnectDelay_ * 2, 30000UL); // Max 30s
      }
    }
  }

  void resubscribeAll() {
    for (const auto& topic : subscriptions_) {
      if (client_.subscribe(topic.c_str())) {
        Serial.printf("[mqtt] Resubscribed: %s\n", topic.c_str());
      } else {
        Serial.printf("[mqtt] Failed to resubscribe: %s\n", topic.c_str());
      }
    }
  }

  std::string generateDefaultClientId() {
    // Generate a unique client ID using ESP32's MAC or chip ID
    char buffer[32];
    #ifdef ESP32
      uint64_t chipid = ESP.getEfuseMac();
      snprintf(buffer, sizeof(buffer), "ESP32-%04X%08X", 
               (uint16_t)(chipid >> 32), (uint32_t)chipid);
    #else
      snprintf(buffer, sizeof(buffer), "ESP-Client-%lu", millis());
    #endif
    return std::string(buffer);
  }

  // No default values - these MUST come from configuration
  std::string  host_;        // Empty indicates not configured
  uint16_t     port_{0};     // 0 indicates not configured
  std::string  clientId_;
  
  MqttCredentials credentials_;
  bool hasCredentials_ = false;
  
  LwtSettings lwt_;
  bool hasLwt_ = false;
  
  std::vector<std::string> subscriptions_; // Track subscriptions for reconnect
  ConnectionCallback connectionCallback_;
  
  // Instance-specific reconnect state (not static)
  unsigned long lastReconnectAttempt_ = 0;
  unsigned long reconnectDelay_ = 2000;
  
  PubSubClient client_;
};
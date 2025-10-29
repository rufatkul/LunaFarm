// lib/wifi/Wifi.h
#pragma once
#include <Arduino.h>
#include <Client.h>   // base class for network clients (what PubSubClient expects)
#include <WiFi.h>     // internal use only (WiFi, WiFiClient, WL_CONNECTED, etc.)
#include <string>

/**
 * Credentials for Wi-Fi connection.
 */
struct WifiSettings {
  std::string ssid;
  std::string password;
};

/**
 * Wifi helper that:
 *  - Stores credentials
 *  - Connects / reconnects in loop()
 *  - Owns a WiFiClient and exposes it as a generic Client&
 */
class Wifi {
public:
  static constexpr uint8_t  kDefaultMaxAttempts = 40;
  static constexpr uint32_t kDefaultDelayMs     = 250;

  Wifi() = default;

  // Initialize and attempt connection once.
  void begin(const WifiSettings& s,
             uint8_t  max_attempts = kDefaultMaxAttempts,
             uint32_t delay_ms     = kDefaultDelayMs)
  {
    creds_ = s;
    connect(max_attempts, delay_ms);
  }

  // Call from loop() to keep Wi-Fi alive.
  void loop(uint8_t max_attempts = 20, uint32_t delay_ms = 500) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[wifi] Disconnected; reconnecting...");
      connect(max_attempts, delay_ms);
    }
  }

  // Expose the internal network client as a generic Client&.
  Client& client() { return client_; }

  // Convenience helpers
  bool      connected() const { return WiFi.status() == WL_CONNECTED; }
  IPAddress ip()        const { return WiFi.localIP(); }
  int32_t   rssi()      const { return connected() ? WiFi.RSSI() : 0; }

  void disconnect(bool power_off = false) {
    WiFi.disconnect(true, true);
    if (power_off) WiFi.mode(WIFI_OFF);
    Serial.println("[wifi] Disconnected.");
  }

private:
  WifiSettings creds_;
  WiFiClient   client_;   // concrete type is hidden from users of this header

  void connect(uint8_t max_attempts, uint32_t delay_ms) {
    if (creds_.ssid.empty()) {
      Serial.println("[wifi] Missing SSID, aborting.");
      return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(creds_.ssid.c_str(), creds_.password.c_str());

    for (uint8_t i = 0; i < max_attempts; ++i) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[wifi] Connected. IP: %s\n",
                      WiFi.localIP().toString().c_str());
        return;
      }
      delay(delay_ms);
      Serial.print(".");
    }
    Serial.println("\n[wifi] Connection failed.");
  }
};

#include "config_reader.h"

bool loadConfig(const char* path, JsonDocument& doc) {
  if (!LittleFS.begin(true)) { 
    Serial.println("Failed to mount LittleFS");
    return false;
  }

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.printf("Failed to open %s\n", path);
    return false;
  }

  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.printf("Failed to parse %s: %s\n", path, error.c_str());
    return false;
  }

  Serial.printf("Loaded JSON from %s\n", path);
  return true;
}

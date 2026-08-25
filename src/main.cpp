#include <Arduino.h>
#include <WiFi.h>

#include "api_server.h"
#include "board_pins.h"
#include "motion_controller.h"
#include "wifi_config.h"

namespace {

slider::MotionController controller;
slider::ApiServer api(controller);

void startWifi() {
  constexpr uint32_t kStationTimeoutMs = 10000;
  const bool configured = strlen(SLIDER_WIFI_SSID) > 0 &&
                          strcmp(SLIDER_WIFI_SSID, "your-wifi-name") != 0;

  if (configured) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(SLIDER_WIFI_SSID, SLIDER_WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < kStationTimeoutMs) {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());
      return;
    }
    WiFi.disconnect(true);
  }

  const uint64_t mac = ESP.getEfuseMac();
  char ssid[24];
  snprintf(ssid, sizeof(ssid), "PD-Stepper-%04X", static_cast<uint16_t>(mac));
  const char* password = strlen(SLIDER_FALLBACK_AP_PASSWORD) >= 8
                             ? SLIDER_FALLBACK_AP_PASSWORD
                             : "pd-stepper";
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.printf("Fallback AP '%s': %s\n", ssid, WiFi.softAPIP().toString().c_str());
}
}  // namespace

void setup() {
  pinMode(slider::pins::kTmcEnable, OUTPUT);
  digitalWrite(slider::pins::kTmcEnable, HIGH);
  delay(200);
  Serial.begin(115200);
  Serial.println("PD-Stepper linear slider starting");

  controller.begin();
  startWifi();
  api.begin();
  Serial.println("JSON API ready on port 80");
}

void loop() {
  controller.tick();
  delay(1);
}

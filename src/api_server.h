#pragma once

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

#include "motion_controller.h"

namespace slider {

class ApiServer {
 public:
  explicit ApiServer(MotionController& controller) : controller_(controller), server_(80) {}
  void begin();

 private:
  void handleHelp(AsyncWebServerRequest* request);
  void handleState(AsyncWebServerRequest* request);
  void handleGetConfig(AsyncWebServerRequest* request);
  void handleCommand(AsyncWebServerRequest* request, JsonVariant& json);
  void handlePutConfig(AsyncWebServerRequest* request, JsonVariant& json);
  void bufferRequestBody(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                         size_t index, size_t total);
  void parseBufferedBody(AsyncWebServerRequest* request, bool is_config);
  void sendAccepted(AsyncWebServerRequest* request, const char* action);
  void sendError(AsyncWebServerRequest* request, int status, const char* code,
                 const char* message);

  MotionController& controller_;
  AsyncWebServer server_;
};

}  // namespace slider

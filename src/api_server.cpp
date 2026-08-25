#include "api_server.h"

#include <ArduinoJson.h>
#include <cstdlib>
#include <cstring>

namespace slider {
namespace {

constexpr std::size_t kMaxJsonBodyBytes = 1024;

int controllerErrorStatus(const char* code) {
  if (code == nullptr) return 400;
  if (strcmp(code, "BUSY") == 0 || strcmp(code, "NOT_HOMED") == 0 ||
      strcmp(code, "STATE_CONFLICT") == 0 ||
      strcmp(code, "OUTSIDE_SOFT_LIMITS") == 0 ||
      strcmp(code, "FAULT_RESET_REQUIRED") == 0 ||
      strcmp(code, "POWER_NOT_GOOD") == 0 ||
      strcmp(code, "ENCODER_NOT_READY") == 0 ||
      strcmp(code, "NO_ACTIVE_FAULT") == 0) {
    return 409;
  }
  return 400;
}

}  // namespace

void ApiServer::begin() {
  server_.on("/api/state", HTTP_GET,
             [this](AsyncWebServerRequest* request) { handleState(request); });
  server_.on("/api/config", HTTP_GET,
             [this](AsyncWebServerRequest* request) { handleGetConfig(request); });

  server_.on(
      "/api/command", HTTP_POST,
      [this](AsyncWebServerRequest* request) { parseBufferedBody(request, false); }, nullptr,
      [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index,
             size_t total) { bufferRequestBody(request, data, len, index, total); });

  server_.on(
      "/api/config", HTTP_PUT,
      [this](AsyncWebServerRequest* request) { parseBufferedBody(request, true); }, nullptr,
      [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index,
             size_t total) { bufferRequestBody(request, data, len, index, total); });

  server_.onNotFound([this](AsyncWebServerRequest* request) {
    sendError(request, 404, "NOT_FOUND", "API route not found");
  });
  server_.begin();
}

void ApiServer::handleState(AsyncWebServerRequest* request) {
  const StateSnapshot state = controller_.snapshot();
  JsonDocument document;
  JsonObject root = document.to<JsonObject>();
  root["mode"] = motionModeName(state.mode);
  root["enabled"] = state.enabled;
  root["homed"] = state.homed;

  JsonObject motion = root["motion"].to<JsonObject>();
  motion["position_mm"] = state.position_mm;
  motion["commanded_position_mm"] = state.commanded_position_mm;
  motion["target_position_mm"] = state.target_position_mm;
  motion["velocity_mm_s"] = state.velocity_mm_s;
  motion["executed_microsteps"] = state.executed_microsteps;

  JsonObject limits = root["limits"].to<JsonObject>();
  limits["physical_min_mm"] = state.physical_min_mm;
  limits["physical_max_mm"] = state.physical_max_mm;
  limits["soft_min_mm"] = state.soft_min_mm;
  limits["soft_max_mm"] = state.soft_max_mm;

  JsonObject fault = root["fault"].to<JsonObject>();
  fault["code"] = faultCodeName(state.fault);
  fault["reason"] = faultReasonName(state.fault_reason);
  fault["latched"] = state.fault != FaultCode::kNone;

  JsonObject power = root["power"].to<JsonObject>();
  power["good"] = state.power_good;
  power["vbus_voltage"] = state.vbus_voltage;

  JsonObject encoder = root["encoder"].to<JsonObject>();
  encoder["valid"] = state.encoder_valid;
  encoder["raw_counts"] = state.encoder_raw;
  encoder["unwrapped_counts"] = state.encoder_unwrapped_counts;
  encoder["min_counts"] = state.encoder_min_counts;
  encoder["max_counts"] = state.encoder_max_counts;

  JsonObject driver = root["tmc2209"].to<JsonObject>();
  driver["diag"] = state.diag;
  driver["stallguard_result"] = state.stallguard_result;
  driver["status_raw"] = state.tmc_status_raw;
  driver["overtemperature_warning"] = (state.tmc_status_raw & 0x01U) != 0;
  driver["overtemperature_shutdown"] = (state.tmc_status_raw & 0x02U) != 0;
  driver["short_to_ground_a"] = (state.tmc_status_raw & 0x04U) != 0;
  driver["short_to_ground_b"] = (state.tmc_status_raw & 0x08U) != 0;
  driver["short_to_supply_a"] = (state.tmc_status_raw & 0x10U) != 0;
  driver["short_to_supply_b"] = (state.tmc_status_raw & 0x20U) != 0;
  driver["open_load_a"] = (state.tmc_status_raw & 0x40U) != 0;
  driver["open_load_b"] = (state.tmc_status_raw & 0x80U) != 0;
  JsonObject uart = driver["uart"].to<JsonObject>();
  uart["ok"] = state.tmc_uart_ok;
  // Keep the response shape without retaining detailed UART history.
  uart["ifcnt"] = 0;
  uart["version"] = 0;
  uart["ioin_raw"] = 0;
  uart["consecutive_failures"] = state.tmc_uart_ok ? 0 : 1;
  uart["last_ok_ms"] = 0;
  uart["last_failure_reason"] = state.tmc_uart_ok ? "NONE" : "READ_FAILED";

  AsyncResponseStream* response = request->beginResponseStream("application/json");
  serializeJson(document, *response);
  request->send(response);
}

void ApiServer::handleGetConfig(AsyncWebServerRequest* request) {
  const RuntimeConfig config = controller_.config();
  JsonDocument document;
  JsonObject root = document.to<JsonObject>();
  root["pd_voltage_v"] = config.pd_voltage_v;
  root["run_current_ma"] = config.run_current_ma;
  root["microsteps"] = config.microsteps;
  root["stallguard_threshold"] = config.stallguard_threshold;
  root["standstill_mode"] = standstillModeName(config.standstill_mode);
  root["invert_direction"] = config.invert_direction;
  root["invert_encoder"] = config.invert_encoder;

  JsonObject homing = root["homing"].to<JsonObject>();
  homing["pd_voltage_v"] = 15;
  homing["run_current_ma"] = 800;
  homing["microsteps"] = 4;
  homing["speed_microsteps_s"] = 800;
  homing["acceleration_microsteps_s2"] = 1500;
  homing["stallguard_threshold"] = 20;
  homing["tcoolthrs"] = 400;
  homing["seek_distance_limit_mm"] = 500;
  homing["seek_timeout_ms"] = 15000;
  homing["backoff_mm"] = 5;

  JsonObject safety = root["safety"].to<JsonObject>();
  safety["soft_margin_mm"] = 5;
  safety["encoder_error_counts"] = core::kHardStopErrorCounts;
  safety["encoder_window_ms"] = core::EncoderHardStopMonitor::kPersistenceMs;
  safety["max_normal_speed_mm_s"] = 40;
  safety["max_normal_acceleration_mm_s2"] = 75;

  AsyncResponseStream* response = request->beginResponseStream("application/json");
  serializeJson(document, *response);
  request->send(response);
}

void ApiServer::bufferRequestBody(AsyncWebServerRequest* request, uint8_t* data,
                                  size_t len, size_t index, size_t total) {
  if (total == 0 || total > kMaxJsonBodyBytes) return;
  if (index == 0) request->_tempObject = calloc(total + 1, sizeof(uint8_t));
  if (request->_tempObject == nullptr || index + len > total) return;
  memcpy(static_cast<uint8_t*>(request->_tempObject) + index, data, len);
}

void ApiServer::parseBufferedBody(AsyncWebServerRequest* request, bool is_config) {
  const size_t length = request->contentLength();
  if (length > kMaxJsonBodyBytes) {
    sendError(request, 413, "BODY_TOO_LARGE", "JSON body exceeds 1024 bytes");
    return;
  }
  if (length == 0 || request->_tempObject == nullptr) {
    sendError(request, 400, "INVALID_JSON", "A JSON request body is required");
    return;
  }
  JsonDocument document;
  const DeserializationError parse_error =
      deserializeJson(document, static_cast<const char*>(request->_tempObject), length);
  if (parse_error) {
    sendError(request, 400, "INVALID_JSON", "Could not parse JSON request body");
    return;
  }
  JsonVariant root = document.as<JsonVariant>();
  if (is_config) handlePutConfig(request, root);
  else handleCommand(request, root);
}

void ApiServer::handleCommand(AsyncWebServerRequest* request, JsonVariant& json) {
  if (!json.is<JsonObject>()) {
    sendError(request, 400, "INVALID_JSON", "Expected a JSON object");
    return;
  }
  JsonObject body = json.as<JsonObject>();
  const char* action = body["action"].as<const char*>();
  if (action == nullptr) {
    sendError(request, 400, "MISSING_ACTION", "Field 'action' is required");
    return;
  }

  Command command;
  if (strcmp(action, "home") == 0) command.type = CommandType::kHome;
  else if (strcmp(action, "stop") == 0) command.type = CommandType::kStop;
  else if (strcmp(action, "enable") == 0) command.type = CommandType::kEnable;
  else if (strcmp(action, "disable") == 0) command.type = CommandType::kDisable;
  else if (strcmp(action, "reset_fault") == 0) command.type = CommandType::kResetFault;
  else if (strcmp(action, "move") == 0) {
    if (!body["position_mm"].is<float>() && !body["position_mm"].is<int>()) {
      sendError(request, 400, "MISSING_POSITION", "Move requires numeric position_mm");
      return;
    }
    command.type = CommandType::kMove;
    command.position_mm = body["position_mm"].as<float>();
    command.speed_mm_s = body["speed_mm_s"] | 40.0F;
    command.acceleration_mm_s2 = body["acceleration_mm_s2"] | 75.0F;
  } else if (strcmp(action, "velocity") == 0) {
    if (!body["velocity_mm_s"].is<float>() && !body["velocity_mm_s"].is<int>()) {
      sendError(request, 400, "MISSING_VELOCITY", "Velocity requires numeric velocity_mm_s");
      return;
    }
    command.type = CommandType::kVelocity;
    command.velocity_mm_s = body["velocity_mm_s"].as<float>();
    command.acceleration_mm_s2 = body["acceleration_mm_s2"] | 75.0F;
  } else {
    sendError(request, 400, "UNKNOWN_ACTION", "Unknown command action");
    return;
  }

  const char* error = nullptr;
  if (!controller_.submitCommand(command, error)) {
    sendError(request, controllerErrorStatus(error), error, "Command rejected");
    return;
  }
  sendAccepted(request, action);
}

void ApiServer::handlePutConfig(AsyncWebServerRequest* request, JsonVariant& json) {
  if (!json.is<JsonObject>()) {
    sendError(request, 400, "INVALID_JSON", "Expected a JSON object");
    return;
  }
  const JsonObject body = json.as<JsonObject>();
  RuntimeConfig config = controller_.config();

  const JsonVariantConst pd_voltage = body["pd_voltage_v"];
  if (!pd_voltage.isNull()) {
    const int value = pd_voltage.as<int>();
    if (value < 0 || value > 255) {
      sendError(request, 400, "INVALID_CONFIG", "pd_voltage_v is invalid");
      return;
    }
    config.pd_voltage_v = static_cast<uint8_t>(value);
  }
  const JsonVariantConst run_current = body["run_current_ma"];
  if (!run_current.isNull()) {
    const int value = run_current.as<int>();
    if (value < 0 || value > 65535) {
      sendError(request, 400, "INVALID_CONFIG", "run_current_ma is invalid");
      return;
    }
    config.run_current_ma = static_cast<uint16_t>(value);
  }
  const JsonVariantConst microsteps = body["microsteps"];
  if (!microsteps.isNull()) {
    const int value = microsteps.as<int>();
    if (value < 0 || value > 65535) {
      sendError(request, 400, "INVALID_CONFIG", "microsteps is invalid");
      return;
    }
    config.microsteps = static_cast<uint16_t>(value);
  }
  const JsonVariantConst stallguard = body["stallguard_threshold"];
  if (!stallguard.isNull()) {
    const int value = stallguard.as<int>();
    if (value < 0 || value > 255) {
      sendError(request, 400, "INVALID_CONFIG", "stallguard_threshold is invalid");
      return;
    }
    config.stallguard_threshold = static_cast<uint8_t>(value);
  }
  const JsonVariantConst standstill = body["standstill_mode"];
  if (!standstill.isNull() &&
      !parseStandstillMode(standstill.as<const char*>(), config.standstill_mode)) {
    sendError(request, 400, "INVALID_CONFIG", "standstill_mode is invalid");
    return;
  }
  const JsonVariantConst invert_direction = body["invert_direction"];
  if (!invert_direction.isNull()) {
    if (!invert_direction.is<bool>()) {
      sendError(request, 400, "INVALID_CONFIG", "invert_direction must be boolean");
      return;
    }
    config.invert_direction = invert_direction.as<bool>();
  }
  const JsonVariantConst invert_encoder = body["invert_encoder"];
  if (!invert_encoder.isNull()) {
    if (!invert_encoder.is<bool>()) {
      sendError(request, 400, "INVALID_CONFIG", "invert_encoder must be boolean");
      return;
    }
    config.invert_encoder = invert_encoder.as<bool>();
  }

  const char* error = nullptr;
  if (!controller_.submitConfig(config, error)) {
    sendError(request, controllerErrorStatus(error), error, "Configuration rejected");
    return;
  }
  sendAccepted(request, "configure");
}

void ApiServer::sendAccepted(AsyncWebServerRequest* request, const char* action) {
  JsonDocument document;
  JsonObject root = document.to<JsonObject>();
  root["ok"] = true;
  root["accepted"] = action;
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  response->setCode(202);
  serializeJson(document, *response);
  request->send(response);
}

void ApiServer::sendError(AsyncWebServerRequest* request, int status, const char* code,
                          const char* message) {
  JsonDocument document;
  JsonObject root = document.to<JsonObject>();
  root["ok"] = false;
  JsonObject error = root["error"].to<JsonObject>();
  error["code"] = code == nullptr ? "ERROR" : code;
  error["message"] = message;
  AsyncResponseStream* response = request->beginResponseStream("application/json");
  response->setCode(status);
  serializeJson(document, *response);
  request->send(response);
}

}  // namespace slider

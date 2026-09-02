#include "api_server.h"
#include "wifi_config.h"

#include <ArduinoJson.h>
#ifdef SLIDER_SESSION_OTA_TOKEN
#include <Update.h>
#endif
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace slider {
namespace {

constexpr std::size_t kMaxJsonBodyBytes = 1024;

const char kApiHelpJson[] PROGMEM = R"json({
  "api":"PD-Stepper Linear Slider",
  "version":1,
  "routes":{
    "GET /api/help":"this read-only manifest",
    "GET /api/state":"mode, enabled/homed, motion coordinates, calibrated limits, fault, PG/VBUS, AS5600 and TMC telemetry",
    "GET /api/config":"persistent runtime configuration plus read-only homing and safety data",
    "PUT /api/config":"partial persistent runtime update; accepted only while IDLE or DISABLED and fault-free",
    "POST /api/command":"one JSON action: home, move, velocity, stop, enable, disable or reset_fault",
    "POST /api/session/firmware":"compile-time optional session-token OTA route; controller must be disabled"
  },
  "commands":{
    "home":{"body":{"action":"home"},"requires":"encoder valid, acceptable fault/state and PG at enable","result":"calibrate MIN/MAX from unwrapped encoder counts, move to center, then restore runtime profile"},
    "move":{"body":{"action":"move","position_mm":"required","speed_mm_s":"optional","acceleration_mm_s2":"optional"},"requires":"homed, IDLE, encoder valid, target inside soft limits"},
    "velocity":{"body":{"action":"velocity","velocity_mm_s":"required signed","acceleration_mm_s2":"optional"},"behavior":"finite move toward the soft limit; release/stop is not implicit"},
    "stop":{"body":{"action":"stop"},"behavior":"priority stop; normal motion retains calibration; homing stop invalidates it"},
    "enable":{"body":{"action":"enable"},"requires":"no fault and PG"},
    "disable":{"body":{"action":"disable"},"behavior":"disables ENN and invalidates calibration"},
    "reset_fault":{"body":{"action":"reset_fault"},"behavior":"clears a resettable fault but remains disabled and unhomed"}
  },
  "runtime_config":{
    "persistent":true,
    "fields":{
      "pd_voltage_v":{"allowed":[5,9,12],"default":12},
      "run_current_ma":{"min":100,"max":2000,"default":800},
      "microsteps":{"allowed":[1,2,4,8,16,32,64,128,256],"default":32,"note":"idle-only change preserves encoder calibration and rebases step coordinates; 1 means full-step"},
      "stallguard_threshold":{"min":0,"max":255,"default":20},
      "standstill_mode":{"allowed":["NORMAL","FREEWHEELING","BRAKING","STRONG_BRAKING"],"default":"FREEWHEELING","note":"NORMAL runtime hold is 30% of run current subject to TMC quantization"},
      "default_speed_mm_s":{"gt":0,"max":150,"default":50},
      "default_acceleration_mm_s2":{"gt":0,"max":300,"default":75},
      "invert_direction":{"type":"boolean","default":false,"calibration_effect":"invalidates"},
      "invert_encoder":{"type":"boolean","default":false,"calibration_effect":"invalidates"}
    },
    "command_overrides":"move speed/acceleration and velocity acceleration override persistent defaults for that command only",
    "existing_storage":"firmware defaults do not overwrite already persisted NVS values"
  },
  "homing":{
    "immutable":true,
    "write_error":"INVALID_CONFIG",
    "scope":"fixed profile remains active for both endpoint seeks, backoffs and the automatic center move; runtime settings are restored only after center completes",
    "profile":{
      "pd_voltage_v":12,
      "run_current_ma":800,
      "microsteps":4,
      "driver_mode":"StealthChop with interpolation, PWM autoscale and PWM autograd",
      "standstill_mode":"NORMAL",
      "hold_current_percent":50,
      "hold_basis":"deterministic implementation choice based on current/default code; the historical persisted test setting is unknown",
      "initial_full_current_tuning_ms":150,
      "seek_and_backoff_speed_mm_s":50,
      "seek_and_backoff_acceleration_mm_s2":75,
      "stallguard_threshold":100,
      "tcoolthrs":210,
      "seek_distance_limit_mm":500,
      "seek_timeout_ms":15000,
      "backoff_mm":5,
      "backoff_timeout_ms":2000,
      "diag_arm_delay_ms":25,
      "center_speed_mm_s":40,
      "center_acceleration_mm_s2":75
    },
    "calibration":"travel and coordinates derive from unwrapped AS5600 counts captured at MIN and MAX",
    "success":"runtime profile transition succeeds after center, then homed/IDLE is published",
    "installation_transforms":"invert_direction and invert_encoder define the installed logical orientation; changing either invalidates calibration"
  },
  "motion_limits":{
    "normal_speed_mm_s":{"max":150,"qualification":"software ceiling, not a mechanically validated operating point"},
    "normal_acceleration_mm_s2":{"max":300,"qualification":"software ceiling, not a mechanically validated operating point"},
    "soft_margin_mm":5,
    "encoder_resolution_mm_per_count":0.009765625,
    "hard_stop":{"rolling_window_ms":125,"error_counts":164,"limitation":"steady stalls below about 12.8 mm/s cannot cross this window threshold"}
  },
  "freewheel_and_position":{
    "authority":"while homed and stationary, physical position comes from the continuously powered AS5600 and step coordinates are rebased to it",
    "first_move_after_reenergize":"before every powered move from FREEWHEELING: capture/rebase, temporarily engage NORMAL hold, sample until bounded encoder stability, capture/rebase again, restore FREEWHEELING, then queue motion; no generic ENN delay is assumed",
    "constraints":["continuous controller power","valid encoder","intact motor-to-carriage coupling","manual position inside calibrated range before powered motion"],
    "limitations":["zero physical movement on re-enable is not guaranteed","motor-shaft encoder cannot detect belt or pulley slip"]
  },
  "buttons":{
    "left_gpio35":"hold-to-jog toward MIN after homing; release stops and resynchronizes",
    "right_gpio37":"hold-to-jog toward MAX after homing; release stops and resynchronizes",
    "center_gpio36":"idle hold >=1 s then release starts homing; during motion raw press immediately force-stops, disables and clears calibration",
    "rules":["active-low and debounced","left+right stops and inhibits until released","fresh press required; held buttons cannot start delayed motion","center stop has priority over DIAG and queued work"]
  },
  "operating_constraints":[
    "12 V is the maximum PD request; higher voltages are unsupported",
    "motion requires successful homing and valid encoder feedback",
    "configuration and OTA are rejected while motion is active",
    "GPIO10/GPIO12 firmware LEDs remain off",
    "the center button is a priority software stop, not a hardware e-stop"
  ],
  "errors":{"format":{"ok":false,"error":{"code":"STRING","message":"STRING"}},"common":["INVALID_JSON","INVALID_CONFIG","INVALID_MOTION_PROFILE","BUSY","STATE_CONFLICT","NOT_HOMED","OUTSIDE_SOFT_LIMITS","POWER_NOT_GOOD","ENCODER_NOT_READY","FAULT_RESET_REQUIRED"]}
})json";

#ifdef SLIDER_SESSION_OTA_TOKEN
bool sessionAuthorized(AsyncWebServerRequest* request) {
  const AsyncWebHeader* header = request->getHeader("X-Session-Token");
  return header != nullptr && header->value() == SLIDER_SESSION_OTA_TOKEN;
}
#endif

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
  server_.on("/api/help", HTTP_GET,
             [this](AsyncWebServerRequest* request) { handleHelp(request); });
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

#ifdef SLIDER_SESSION_OTA_TOKEN
  server_.on(
      "/api/session/firmware", HTTP_POST,
      [](AsyncWebServerRequest* request) {
        if (request->getResponse() != nullptr) return;
        if (!sessionAuthorized(request)) {
          request->send(403, "application/json", "{\"ok\":false,\"error\":\"FORBIDDEN\"}");
          return;
        }
        if (!Update.isFinished() || Update.hasError()) {
          request->send(500, "application/json",
                        "{\"ok\":false,\"error\":\"UPDATE_FAILED\"}");
          return;
        }
        request->onDisconnect([]() { ESP.restart(); });
        request->send(200, "application/json", "{\"ok\":true,\"restarting\":true}");
      },
      [this](AsyncWebServerRequest* request, String, size_t index, uint8_t* data,
             size_t len, bool final) {
        if (request->getResponse() != nullptr || !sessionAuthorized(request)) return;
        if (index == 0) {
          const StateSnapshot state = controller_.snapshot();
          if (state.enabled || (state.mode != MotionMode::kDisabled &&
                                state.mode != MotionMode::kFaulted)) {
            request->send(409, "application/json",
                          "{\"ok\":false,\"error\":\"STATE_CONFLICT\"}");
            return;
          }
          if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            request->send(500, "application/json",
                          "{\"ok\":false,\"error\":\"UPDATE_BEGIN_FAILED\"}");
            return;
          }
        }
        if (Update.write(data, len) != len) {
          Update.abort();
          request->send(500, "application/json",
                        "{\"ok\":false,\"error\":\"UPDATE_WRITE_FAILED\"}");
          return;
        }
        if (final && !Update.end(true)) {
          request->send(500, "application/json",
                        "{\"ok\":false,\"error\":\"UPDATE_END_FAILED\"}");
        }
      });
#endif

  server_.onNotFound([this](AsyncWebServerRequest* request) {
    sendError(request, 404, "NOT_FOUND", "API route not found");
  });
  server_.begin();
}

void ApiServer::handleHelp(AsyncWebServerRequest* request) {
  request->send(200, "application/json", kApiHelpJson);
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
  root["default_speed_mm_s"] = config.default_speed_mm_s;
  root["default_acceleration_mm_s2"] = config.default_acceleration_mm_s2;
  root["invert_direction"] = config.invert_direction;
  root["invert_encoder"] = config.invert_encoder;

  JsonObject homing = root["homing"].to<JsonObject>();
  homing["pd_voltage_v"] = 12;
  homing["run_current_ma"] = 800;
  homing["microsteps"] = 4;
  homing["speed_microsteps_s"] = 1000;
  homing["acceleration_microsteps_s2"] = 1500;
  homing["stallguard_threshold"] = 100;
  homing["tcoolthrs"] = 210;
  homing["seek_distance_limit_mm"] = 500;
  homing["seek_timeout_ms"] = 15000;
  homing["backoff_mm"] = 5;
  homing["standstill_mode"] = "NORMAL";
  homing["hold_current_percent"] = 50;
  homing["hold_current_basis"] =
      "deterministic current/default behavior; historical persisted setting unknown";
  homing["center_speed_mm_s"] = 40;
  homing["center_acceleration_mm_s2"] = 75;
  homing["immutable_through_api"] = true;
  homing["runtime_restored_after_center"] = true;

  JsonObject safety = root["safety"].to<JsonObject>();
  safety["soft_margin_mm"] = 5;
  safety["encoder_error_counts"] = core::kHardStopErrorCounts;
  safety["encoder_window_ms"] = core::EncoderHardStopMonitor::kWindowMs;
  safety["max_normal_speed_mm_s"] = runtime::kMaxSpeedMmS;
  safety["max_normal_acceleration_mm_s2"] = runtime::kMaxAccelerationMmS2;

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
  const RuntimeConfig config = controller_.config();
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
    command.speed_mm_s = body["speed_mm_s"] | config.default_speed_mm_s;
    command.acceleration_mm_s2 =
        body["acceleration_mm_s2"] | config.default_acceleration_mm_s2;
  } else if (strcmp(action, "velocity") == 0) {
    if (!body["velocity_mm_s"].is<float>() && !body["velocity_mm_s"].is<int>()) {
      sendError(request, 400, "MISSING_VELOCITY", "Velocity requires numeric velocity_mm_s");
      return;
    }
    command.type = CommandType::kVelocity;
    command.velocity_mm_s = body["velocity_mm_s"].as<float>();
    command.acceleration_mm_s2 =
        body["acceleration_mm_s2"] | config.default_acceleration_mm_s2;
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
  if (!body["homing"].isNull()) {
    sendError(request, 400, "INVALID_CONFIG",
              "Homing settings are fixed and cannot be changed through the API");
    return;
  }
  RuntimeConfig config = controller_.config();

  const JsonVariantConst pd_voltage = body["pd_voltage_v"];
  if (!pd_voltage.isNull()) {
    const int value = pd_voltage.as<int>();
    if (!core::isSupportedPdVoltage(value)) {
      sendError(request, 400, "INVALID_CONFIG", "pd_voltage_v must be 5, 9, or 12");
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
  const JsonVariantConst default_speed = body["default_speed_mm_s"];
  if (!default_speed.isNull()) {
    const float value = default_speed.as<float>();
    if ((!default_speed.is<float>() && !default_speed.is<int>()) ||
        !std::isfinite(value) || value <= 0.0F || value > runtime::kMaxSpeedMmS) {
      sendError(request, 400, "INVALID_CONFIG",
                "default_speed_mm_s must be finite, greater than 0, and at most 150");
      return;
    }
    config.default_speed_mm_s = value;
  }
  const JsonVariantConst default_acceleration = body["default_acceleration_mm_s2"];
  if (!default_acceleration.isNull()) {
    const float value = default_acceleration.as<float>();
    if ((!default_acceleration.is<float>() && !default_acceleration.is<int>()) ||
        !std::isfinite(value) || value <= 0.0F ||
        value > runtime::kMaxAccelerationMmS2) {
      sendError(
          request, 400, "INVALID_CONFIG",
          "default_acceleration_mm_s2 must be finite, greater than 0, and at most 300");
      return;
    }
    config.default_acceleration_mm_s2 = value;
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

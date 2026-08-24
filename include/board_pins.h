#pragma once

#include <stdint.h>

namespace slider::pins {

constexpr uint8_t kTmcEnable = 21;
constexpr uint8_t kStep = 5;
constexpr uint8_t kDirection = 6;
constexpr uint8_t kMs1 = 1;
constexpr uint8_t kMs2 = 2;
constexpr uint8_t kSpread = 7;
constexpr uint8_t kTmcTx = 17;
constexpr uint8_t kTmcRx = 18;
constexpr uint8_t kDiag = 16;
constexpr uint8_t kIndex = 11;

constexpr uint8_t kPowerGood = 15;
constexpr uint8_t kPdCfg1 = 38;
constexpr uint8_t kPdCfg2 = 48;
constexpr uint8_t kPdCfg3 = 47;

constexpr uint8_t kEncoderSda = 8;
constexpr uint8_t kEncoderScl = 9;
constexpr uint8_t kVbusAdc = 4;
constexpr uint8_t kLed1 = 10;
constexpr uint8_t kLed2 = 12;

}  // namespace slider::pins

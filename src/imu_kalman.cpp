/**
 * 一維卡爾曼濾波實作（加速度傾角 + 陀螺儀角速度）
 */

#include "imu_kalman.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace ImuKalman {

void AngleFilter::configure(float qAngle, float qGyro, float rAngle) {
  qAngle_ = qAngle;
  qGyro_ = qGyro;
  rAngle_ = rAngle;
}

void AngleFilter::reset() {
  angleDeg_ = 0.0f;
  gyroBiasDps_ = 0.0f;
  pp_[0][0] = 1.0f;
  pp_[0][1] = 0.0f;
  pp_[1][0] = 0.0f;
  pp_[1][1] = 1.0f;
  initialized_ = false;
}

Result AngleFilter::update(float accelAngleDeg, float gyroDps, float dtSec) {
  if (!initialized_) {
    angleDeg_ = accelAngleDeg;
    initialized_ = true;
  }
  if (dtSec <= 0.0f) {
    dtSec = 0.001f;
  } else if (dtSec > 0.25f) {
    dtSec = 0.25f;
  }

  // (1) 先驗估計：角度 += (角速度 - 零偏) * dt
  angleDeg_ += (gyroDps - gyroBiasDps_) * dtSec;

  // (2) 協方差預測 Pdot 積分
  float pdot0 = qAngle_ - pp_[0][1] - pp_[1][0];
  float pdot1 = -pp_[1][1];
  float pdot2 = -pp_[1][1];
  float pdot3 = qGyro_;

  pp_[0][0] += pdot0 * dtSec;
  pp_[0][1] += pdot1 * dtSec;
  pp_[1][0] += pdot2 * dtSec;
  pp_[1][1] += pdot3 * dtSec;

  // (3) 卡爾曼增益
  const float c0 = 1.0f;
  const float pct0 = c0 * pp_[0][0];
  const float pct1 = c0 * pp_[1][0];
  const float e = rAngle_ + c0 * pct0;
  const float k0 = pct0 / e;
  const float k1 = pct1 / e;

  // (4) 後驗修正
  const float angleErr = accelAngleDeg - angleDeg_;
  const float t0 = pct0;
  const float t1 = c0 * pp_[0][1];

  pp_[0][0] -= k0 * t0;
  pp_[0][1] -= k0 * t1;
  pp_[1][0] -= k1 * t0;
  pp_[1][1] -= k1 * t1;

  angleDeg_ += k0 * angleErr;
  gyroBiasDps_ += k1 * angleErr;

  Result out;
  out.angleDeg = angleDeg_;
  out.gyroBiasDps = gyroBiasDps_;
  out.gyroCorrectedDps = gyroDps - gyroBiasDps_;
  return out;
}

void accelToPitchRoll(float axG, float ayG, float azG, float& pitchDeg, float& rollDeg) {
  rollDeg = atan2f(ayG, azG) * (180.0f / (float)M_PI);
  const float denom = sqrtf(axG * axG + ayG * ayG + azG * azG);
  if (denom < 1e-6f) {
    pitchDeg = 0.0f;
    return;
  }
  pitchDeg = atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * (180.0f / (float)M_PI);
}

}  // namespace ImuKalman

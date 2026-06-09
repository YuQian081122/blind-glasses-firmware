/**
 * 一維卡爾曼濾波：融合加速度計傾角與陀螺儀角速度
 * 演算法參考：https://jasonblog.github.io/note/osvr/qia_er_man_lv_bo_pei_he_cheng_shi_jiang_jie.html
 */

#ifndef IMU_KALMAN_H
#define IMU_KALMAN_H

namespace ImuKalman {

/** 狀態：融合後角度（度）、陀螺儀零偏估計（dps）、去偏後角速度（dps） */
struct Result {
  float angleDeg = 0.0f;
  float gyroBiasDps = 0.0f;
  float gyroCorrectedDps = 0.0f;
};

class AngleFilter {
 public:
  void configure(float qAngle, float qGyro, float rAngle);
  void reset();
  Result update(float accelAngleDeg, float gyroDps, float dtSec);

 private:
  float qAngle_ = 0.01f;
  float qGyro_ = 0.01f;
  float rAngle_ = 0.003f;
  float angleDeg_ = 0.0f;
  float gyroBiasDps_ = 0.0f;
  float pp_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
  bool initialized_ = false;
};

/** 由加速度（g）估算 pitch / roll（度） */
void accelToPitchRoll(float axG, float ayG, float azG, float& pitchDeg, float& rollDeg);

}  // namespace ImuKalman

#endif  // IMU_KALMAN_H

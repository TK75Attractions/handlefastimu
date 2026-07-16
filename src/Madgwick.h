#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdint.h>

class Madgwick {
public:
  Madgwick();

  void reset();

  void changeBeta(float newBeta) {
    beta = newBeta;
  }

  void begin(float confBeta) {
    beta = confBeta;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

  float getQuatW() const {
    return q0;
  }

  float getQuatX() const {
    return q1;
  }

  float getQuatY() const {
    return q2;
  }

  float getQuatZ() const {
    return q3;
  }

private:
  static float invSqrt(float x);

  float delta_t = 0.0f;
  uint32_t now = 0;
  uint32_t last_update = 0;

  float beta = 0.1f;
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
};

struct Vec3 {
  Vec3() = default;
  Vec3(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue) {}

  float x;
  float y;
  float z;
};

Vec3 normalize(const Vec3& v);

float dot(const Vec3& a, const Vec3& b);

#endif

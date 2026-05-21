// orientation.cpp
// Implementation of orientation filter management and quaternion helpers.
// This file holds the Madgwick filter instance and maintains the state
// needed to produce a continuous shaft angle measurement.

#include "orientation.hpp"
#include <math.h>

namespace Orientation {

static Madgwick filter;
static bool hasMag = false;
static float currentBeta = 0.2f;

// Shaft axis expressed in the body frame (set by caller during begin).
static Vec3 shaftAxisBody = {0.0f, 1.0f, 0.0f};

// Reference quaternion locked by lockZeroHere().
struct Quat { float w; float x; float y; float z; };
static Quat qRef = {1.0f, 0.0f, 0.0f, 0.0f};
static bool refLocked = false;
static float prevWrapped = 0.0f;
static float totalAngleRad = 0.0f;

static inline Quat quatConj(const Quat& q) {
  return {q.w, -q.x, -q.y, -q.z};
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  };
}

static inline Quat currentQuat() {
  return { filter.getQuatW(), filter.getQuatX(), filter.getQuatY(), filter.getQuatZ() };
}

// Keep an unwrap helper local so shaftAngleRad stays concise.
static float unwrapAngle(float wrapped, float& prevWrappedLocal, float& totalLocal) {
  float delta = wrapped - prevWrappedLocal;
  if (delta > PI) delta -= 2.0f * PI;
  if (delta < -PI) delta += 2.0f * PI;
  totalLocal += delta;
  prevWrappedLocal = wrapped;
  return totalLocal;
}

void begin(bool hasMagnetometer, const Vec3& shaftAxisBodyIn, float initialBeta) {
  hasMag = hasMagnetometer;
  currentBeta = initialBeta;
  filter.begin(hasMag ? 0.12f : 0.05f);
  filter.changeBeta(currentBeta);
  shaftAxisBody = normalize(shaftAxisBodyIn);
}

void changeBeta(float newBeta) {
  currentBeta = newBeta;
  filter.changeBeta(currentBeta);
}

float getBeta() { return currentBeta; }

void updateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
}

void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  filter.updateIMU(gx, gy, gz, ax, ay, az);
}

float getQuatW() { return filter.getQuatW(); }
float getQuatX() { return filter.getQuatX(); }
float getQuatY() { return filter.getQuatY(); }
float getQuatZ() { return filter.getQuatZ(); }

void lockZeroHere() {
  qRef = currentQuat();
  refLocked = true;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
}

float shaftAngleRad() {
  const Quat qNow = currentQuat();
  const Quat qRel = quatMul(quatConj(qRef), qNow);

  const float wrapped = getAngleAboutAxis(
    qRel.w,
    qRel.x,
    qRel.y,
    qRel.z,
    shaftAxisBody
  );

  return unwrapAngle(wrapped, prevWrapped, totalAngleRad);
}

void quaternionToEulerDeg(float w, float x, float y, float z, float& rollDeg, float& pitchDeg, float& yawDeg) {
  const float sinrCosp = 2.0f * (w * x + y * z);
  const float cosrCosp = 1.0f - 2.0f * (x * x + y * y);
  const float roll = atan2f(sinrCosp, cosrCosp);

  const float sinp = 2.0f * (w * y - z * x);
  const float pitch = asinf(constrain(sinp, -1.0f, 1.0f));

  const float sinyCosp = 2.0f * (w * z + x * y);
  const float cosyCosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw = atan2f(sinyCosp, cosyCosp);

  constexpr float RAD_TO_DEG_F = 57.2957795f;
  rollDeg = roll * RAD_TO_DEG_F;
  pitchDeg = pitch * RAD_TO_DEG_F;
  yawDeg = yaw * RAD_TO_DEG_F;
}

} // namespace Orientation

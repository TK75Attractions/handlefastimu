// orientation.cpp
// Implementation of orientation filter management and shaft angle math.

#include "orientation.hpp"
#include <math.h>

namespace Orientation {

static Madgwick filter;
static float currentBeta = 0.2f;

static Vec3 shaftAxisWorld = {1.0f, 0.0f, 0.0f};
static Vec3 sensorVector = {0.0f, 1.0f, 0.0f};
static Vec3 referenceProjection = {0.0f, 1.0f, 0.0f};

static bool refLocked = false;
static float prevWrapped = 0.0f;
static float totalAngleRad = 0.0f;

static inline Vec3 add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline Vec3 subtract(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline Vec3 scale(const Vec3& v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}

static inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

static inline Quaternion currentQuat() {
  return { filter.getQuatW(), filter.getQuatX(), filter.getQuatY(), filter.getQuatZ() };
}

static Vec3 rotateVector(const Quaternion& q, const Vec3& v) {
  const Vec3 qv = {q.qX, q.qY, q.qZ};
  const Vec3 t = scale(cross(qv, v), 2.0f);
  return add(v, add(scale(t, q.qW), cross(qv, t)));
}

static Vec3 projectOffAxis(const Vec3& v) {
  return subtract(v, scale(shaftAxisWorld, dot(v, shaftAxisWorld)));
}

static Vec3 projectedSensorVector(const Quaternion& q) {
  return projectOffAxis(rotateVector(q, sensorVector));
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

void begin(
  bool hasMagnetometer,
  const Vec3& shaftAxisWorldIn,
  const Vec3& sensorVectorIn,
  float initialBeta
) {
  currentBeta = initialBeta;
  filter.begin(hasMagnetometer ? 0.12f : 0.05f);
  filter.changeBeta(currentBeta);
  shaftAxisWorld = normalize(shaftAxisWorldIn);
  sensorVector = normalize(sensorVectorIn);
  referenceProjection = projectedSensorVector(currentQuat());
  refLocked = false;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
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

void lockZeroHere() {
  referenceProjection = projectedSensorVector(currentQuat());
  refLocked = true;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
}

float shaftAngleRad() {
  if (!refLocked) {
    return 0.0f;
  }

  const Vec3 p = projectedSensorVector(currentQuat());
  const float c = dot(referenceProjection, p);
  const float s = dot(shaftAxisWorld, cross(referenceProjection, p));
  const float wrapped = atan2f(s, c);
  return unwrapAngle(wrapped, prevWrapped, totalAngleRad);
}

} // namespace Orientation

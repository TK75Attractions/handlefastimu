// orientation.cpp
// Implementation of orientation filter management and shaft angle math.

#include "orientation.hpp"
#include <Arduino.h>
#include <math.h>

namespace Orientation {

static Madgwick filter;
static float currentBeta = 0.2f;
static bool useMag = false;

static Vec3 shaftAxisWorld = {0.0f, 0.0f, 1.0f};
static Vec3 shaftAxisSensor = {0.0f, 0.0f, 1.0f};
static Vec3 sensorVector = {0.0f, 1.0f, 0.0f};
static Vec3 referenceProjection = {0.0f, 1.0f, 0.0f};
static Vec3 latestGravityProjection = {0.0f, 0.0f, 0.0f};

static bool refLocked = false;
static bool useGravityAngle = false;
static bool latestGravityProjectionUsable = false;
static uint32_t lastUpdateUs = 0;
static float integratedAngleRad = 0.0f;
static float prevWrapped = 0.0f;
static float totalAngleRad = 0.0f;

constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float MIN_GRAVITY_PROJECTION_SQ = 0.0025f;

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

static Vec3 projectOffSensorAxis(const Vec3& v) {
  return subtract(v, scale(shaftAxisSensor, dot(v, shaftAxisSensor)));
}

static Vec3 projectedSensorVector(const Quaternion& q) {
  return projectOffAxis(rotateVector(q, sensorVector));
}

static bool isUsableProjection(const Vec3& v) {
  return dot(v, v) > MIN_GRAVITY_PROJECTION_SQ;
}

static void updateGravityProjection(float ax, float ay, float az) {
  const Vec3 accel = normalize(Vec3(ax, ay, az));
  latestGravityProjection = projectOffSensorAxis(accel);
  latestGravityProjectionUsable = isUsableProjection(latestGravityProjection);
  if (latestGravityProjectionUsable) {
    latestGravityProjection = normalize(latestGravityProjection);
  }
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

static void updateIntegratedAngle(float gx, float gy, float gz) {
  const uint32_t nowUs = micros();
  if (lastUpdateUs == 0) {
    lastUpdateUs = nowUs;
    return;
  }

  const float dt = (nowUs - lastUpdateUs) * 0.000001f;
  lastUpdateUs = nowUs;

  if (!refLocked || dt <= 0.0f || dt > 0.1f) {
    return;
  }

  const Vec3 gyro = {gx, gy, gz};
  integratedAngleRad += dot(gyro, shaftAxisSensor) * DEG_TO_RAD_F * dt;
}

void begin(
  bool hasMagnetometer,
  const Vec3& shaftAxisWorldIn,
  const Vec3& shaftAxisSensorIn,
  const Vec3& sensorVectorIn,
  float initialBeta
) {
  useMag = hasMagnetometer;
  currentBeta = initialBeta;
  filter.reset();
  filter.begin(hasMagnetometer ? 0.12f : 0.05f);
  filter.changeBeta(currentBeta);
  shaftAxisWorld = normalize(shaftAxisWorldIn);
  shaftAxisSensor = normalize(shaftAxisSensorIn);
  sensorVector = normalize(sensorVectorIn);
  referenceProjection = projectedSensorVector(currentQuat());
  latestGravityProjection = {0.0f, 0.0f, 0.0f};
  refLocked = false;
  useGravityAngle = false;
  latestGravityProjectionUsable = false;
  lastUpdateUs = micros();
  integratedAngleRad = 0.0f;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
}

void changeBeta(float newBeta) {
  currentBeta = newBeta;
  filter.changeBeta(currentBeta);
}

float getBeta() { return currentBeta; }

void resetFilterForResync() {
  filter.reset();
  filter.begin(useMag ? 0.12f : 0.05f);
  filter.changeBeta(currentBeta);
  latestGravityProjectionUsable = false;
  lastUpdateUs = micros();
}

void updateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  updateGravityProjection(ax, ay, az);
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  updateIntegratedAngle(gx, gy, gz);
}

void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  updateGravityProjection(ax, ay, az);
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  updateIntegratedAngle(gx, gy, gz);
}

void lockZeroHere() {
  if (useMag) {
    referenceProjection = projectedSensorVector(currentQuat());
    useGravityAngle = false;
  } else {
    referenceProjection = latestGravityProjection;
    useGravityAngle = latestGravityProjectionUsable;
  }
  refLocked = true;
  lastUpdateUs = micros();
  integratedAngleRad = 0.0f;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
}

bool hasAbsoluteShaftReference() {
  return useMag || (useGravityAngle && latestGravityProjectionUsable);
}

float shaftAngleRad() {
  if (!refLocked) {
    return 0.0f;
  }

  if (!useMag) {
    if (useGravityAngle) {
      if (latestGravityProjectionUsable) {
        const float c = dot(referenceProjection, latestGravityProjection);
        const float s = -dot(shaftAxisSensor, cross(referenceProjection, latestGravityProjection));
        const float wrapped = atan2f(s, c);
        return unwrapAngle(wrapped, prevWrapped, totalAngleRad);
      }
    }
    return integratedAngleRad;
  }

  const Vec3 p = projectedSensorVector(currentQuat());
  const float c = dot(referenceProjection, p);
  const float s = dot(shaftAxisWorld, cross(referenceProjection, p));
  const float wrapped = atan2f(s, c);
  return unwrapAngle(wrapped, prevWrapped, totalAngleRad);
}

} // namespace Orientation

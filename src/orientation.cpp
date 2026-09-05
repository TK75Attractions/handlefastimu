// orientation.cpp
// Implementation of orientation filter management and shaft angle math.

#include "orientation.hpp"
#include <Arduino.h>
#include <math.h>

namespace Orientation {

static Madgwick filter;
static float currentBeta = 0.2f;
static bool useMag = false;

static Vec3 shaftAxisSensor = {0.0f, 0.0f, 1.0f};
static Vec3 zeroGravityProjection = {1.0f, 0.0f, 0.0f};
static Vec3 latestGravityProjection = {0.0f, 0.0f, 0.0f};

static bool refLocked = false;
static bool latestGravityProjectionUsable = false;
static uint32_t lastUpdateUs = 0;
static float integratedAngleRad = 0.0f;
static float prevWrapped = 0.0f;
static float totalAngleRad = 0.0f;

constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float MIN_GRAVITY_PROJECTION_SQ = 0.0025f;

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

static Vec3 projectOffSensorAxis(const Vec3& v) {
  return subtract(v, scale(shaftAxisSensor, dot(v, shaftAxisSensor)));
}

static bool isUsableProjection(const Vec3& v) {
  return dot(v, v) > MIN_GRAVITY_PROJECTION_SQ;
}

static void updateGravityProjectionFromFilter() {
  const float qw = filter.getQuatW();
  const float qx = filter.getQuatX();
  const float qy = filter.getQuatY();
  const float qz = filter.getQuatZ();

  // Gravity direction predicted by the Madgwick quaternion, expressed in
  // sensor coordinates. This is the same gravity model used by the filter's
  // accelerometer correction step.
  const Vec3 gravitySensor = {
    2.0f * (qx * qz - qw * qy),
    2.0f * (qw * qx + qy * qz),
    qw * qw - qx * qx - qy * qy + qz * qz
  };

  latestGravityProjection = projectOffSensorAxis(gravitySensor);
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
  const Vec3& shaftAxisSensorIn,
  const Vec3& zeroGravityDirectionSensorIn,
  float initialBeta
) {
  useMag = hasMagnetometer;
  currentBeta = initialBeta;
  filter.reset();
  filter.begin(hasMagnetometer ? 0.12f : 0.05f);
  filter.changeBeta(currentBeta);
  shaftAxisSensor = normalize(shaftAxisSensorIn);
  zeroGravityProjection = normalize(projectOffSensorAxis(zeroGravityDirectionSensorIn));
  latestGravityProjection = {0.0f, 0.0f, 0.0f};
  refLocked = false;
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
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  updateGravityProjectionFromFilter();
  updateIntegratedAngle(gx, gy, gz);
}

void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  updateGravityProjectionFromFilter();
  updateIntegratedAngle(gx, gy, gz);
}

void restartAngleTracking() {
  refLocked = true;
  lastUpdateUs = micros();
  integratedAngleRad = 0.0f;
  prevWrapped = 0.0f;
  totalAngleRad = 0.0f;
}

bool hasAbsoluteShaftReference() {
  return refLocked && latestGravityProjectionUsable;
}

float shaftAngleRad() {
  if (!refLocked) {
    return 0.0f;
  }

  if (latestGravityProjectionUsable) {
    const float c = dot(zeroGravityProjection, latestGravityProjection);
    const float s = -dot(shaftAxisSensor, cross(zeroGravityProjection, latestGravityProjection));
    const float wrapped = atan2f(s, c);
    const float absoluteAngle = unwrapAngle(wrapped, prevWrapped, totalAngleRad);
    integratedAngleRad = absoluteAngle;
    return absoluteAngle;
  }

  // When the shaft is momentarily too close to gravity, the projected gravity
  // vector cannot define an angle. Continue from the shaft-axis gyro integral
  // until an absolute gravity angle becomes usable again.
  return integratedAngleRad;
}

} // namespace Orientation

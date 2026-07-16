// orientation.hpp
// Encapsulates orientation filtering (Madgwick) and shaft angle math.

#pragma once

#include "IMUBase.hpp"
#include "Madgwick.h"

namespace Orientation {

// Initialize filter parameters and provide shaft geometry.
void begin(
  bool hasMagnetometer,
  const Vec3& shaftAxisWorld,
  const Vec3& shaftAxisSensor,
  const Vec3& sensorVector,
  float initialBeta = 0.2f
);

// Change filter responsiveness parameter at runtime.
void changeBeta(float newBeta);
float getBeta();

// Reset only the live filter state while keeping the locked zero reference.
void resetFilterForResync();

// True when the current setup has an absolute reference for shaft angle.
bool hasAbsoluteShaftReference();

// Update filter using either full mag+imu or imu-only data.
void updateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

// Lock the current orientation as the zero reference for shaft angle.
void lockZeroHere();

// Compute cumulative shaft angle in radians since the last lock.
float shaftAngleRad();

} // namespace Orientation

// orientation.hpp
// Encapsulates orientation filtering (Madgwick) and shaft angle math.

#pragma once

#include "IMUBase.hpp"
#include "Madgwick.h"

namespace Orientation {

// Initialize filter parameters and provide shaft geometry.
// The fixed zero is the pose where zeroGravityDirectionSensor points along
// the gravity estimate; for this project that is sensor -X, which makes
// sensor Y horizontal while sensor Z is the rotation axis.
void begin(
  bool hasMagnetometer,
  const Vec3& shaftAxisSensor,
  const Vec3& zeroGravityDirectionSensor,
  float initialBeta = 0.2f
);

// Change filter responsiveness parameter at runtime.
void changeBeta(float newBeta);
float getBeta();

// Reset only the live filter state while keeping the fixed gravity reference.
void resetFilterForResync();

// True when the current setup has an absolute reference for shaft angle.
bool hasAbsoluteShaftReference();

// Update filter using either full mag+imu or imu-only data.
void updateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

// Start/restart cumulative-angle tracking. The zero reference itself remains
// fixed by gravity and does not depend on the current or startup pose.
void restartAngleTracking();

// Compute cumulative shaft angle in radians relative to the fixed gravity zero.
float shaftAngleRad();

} // namespace Orientation

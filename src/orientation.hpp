// orientation.hpp
// Encapsulates orientation filtering (Madgwick) and quaternion/angle
// utility functions. This keeps all math and filter state inside one
// module so `main.cpp` can remain small and readable.

#pragma once

#include "Madgwick.h"
#include "IMUUtils.hpp"

namespace Orientation {

// Initialize filter parameters and provide the shaft axis in body frame.
void begin(bool hasMagnetometer, const Vec3& shaftAxisBody, float initialBeta = 0.2f);

// Change filter responsiveness parameter at runtime.
void changeBeta(float newBeta);
float getBeta();

// Update filter using either full mag+imu or imu-only data.
void updateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

// Get quaternion components from the internal filter state.
float getQuatW();
float getQuatX();
float getQuatY();
float getQuatZ();

// Lock the current orientation as the zero reference for shaft angle.
void lockZeroHere();

// Compute cumulative shaft angle in radians since the last lock.
float shaftAngleRad();

// Convenience: convert quaternion to Euler angles (degrees).
void quaternionToEulerDeg(float w, float x, float y, float z, float& rollDeg, float& pitchDeg, float& yawDeg);

} // namespace Orientation

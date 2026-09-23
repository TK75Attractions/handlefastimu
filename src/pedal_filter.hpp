#pragma once

#include <math.h>
#include <stdint.h>

// Adaptive low-pass filter for pedal inputs. It suppresses ADC noise while the
// pedal is still, then raises its cutoff frequency as soon as the pedal moves.
class PedalFilter {
public:
  void reset() { initialized_ = false; }
  bool hasValue() const { return initialized_; }

  float update(float sample, uint32_t nowUs) {
    if (!initialized_) {
      initialized_ = true;
      previousUs_ = nowUs;
      previousSample_ = sample;
      filteredValue_ = sample;
      filteredDerivative_ = 0.0f;
      return sample;
    }

    const float dt = float(uint32_t(nowUs - previousUs_)) * 1.0e-6f;
    previousUs_ = nowUs;
    if (dt <= 0.0f || dt > MAX_SAMPLE_INTERVAL_S) {
      previousSample_ = sample;
      filteredValue_ = sample;
      filteredDerivative_ = 0.0f;
      return sample;
    }

    const float derivative = (sample - previousSample_) / dt;
    previousSample_ = sample;
    filteredDerivative_ = lowPass(
      derivative, filteredDerivative_, alpha(DERIVATIVE_CUTOFF_HZ, dt));

    const float cutoff = MIN_CUTOFF_HZ +
      RESPONSE_BETA * fabsf(filteredDerivative_);
    filteredValue_ = lowPass(sample, filteredValue_, alpha(cutoff, dt));
    return filteredValue_;
  }

private:
  // About 2 Hz at rest removes visible jitter. RESPONSE_BETA raises the
  // cutoff aggressively during intentional movement for near-immediate input.
  static constexpr float MIN_CUTOFF_HZ = 2.0f;
  static constexpr float RESPONSE_BETA = 20.0f;
  static constexpr float DERIVATIVE_CUTOFF_HZ = 1.0f;
  static constexpr float MAX_SAMPLE_INTERVAL_S = 0.1f;
  static constexpr float TWO_PI_F = 6.28318530718f;

  static float alpha(float cutoffHz, float dt) {
    return 1.0f - expf(-TWO_PI_F * cutoffHz * dt);
  }

  static float lowPass(float input, float previous, float blend) {
    return previous + blend * (input - previous);
  }

  bool initialized_ = false;
  uint32_t previousUs_ = 0;
  float previousSample_ = 0.0f;
  float filteredValue_ = 0.0f;
  float filteredDerivative_ = 0.0f;
};

#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 2° Order Low-Pass filter
class LowPass2 : public IFilter {
  public:
  LowPass2(const uint32_t period_us = 100 * 1000, const FType tau = 1,
           const FType damping_factor = 1, const FType gain = 1)
      : _period_us(period_us)
      , _T(0)
      , _k(gain)
      , _tau(tau)
      , _z(damping_factor)
      , _a1(0)
      , _a2(0)
      , _b(0)
      , _y0(0)
      , _y1(0)
      , _y2(0)
      , _u1(0)
      , _u2(0) {
    setPeriod(period_us);
    updateCoefficients();
  }

  void setPeriod(const uint32_t period_us) {
    _period_us = period_us;
    _T         = period_us / FType(1000000);
    updateCoefficients();
  }

  uint32_t getPeriod() const { return _period_us; }

  void setGain(const FType gain) {
    _k = gain;
    updateCoefficients();
  }

  FType getGain() const { return _k; }

  void setTimeConstant(const FType tau) {
    _tau = tau;
    updateCoefficients();
  }

  FType getTimeConstant() const { return _tau; }

  void setCutOffFrequency(const FType fc_hz) {
    _tau = 1 / (2 * PI * fc_hz);
    updateCoefficients();
  }

  FType getCutOffFrequency() const { return 1 / (2 * PI * _tau); }

  void setDampingFactor(const FType damping_factor) {
    _z = damping_factor;
    updateCoefficients();
  }

  FType getDampingFactor() const { return _z; }

  void setInitialValue(const FType initial_value) override {
    _y0 = initial_value;
    _y1 = initial_value;
    _y2 = initial_value;
    _u1 = initial_value;
    _u2 = initial_value;
  }

  FType update(const FType input) override {
    // Calculate output
    _y0 = _a1 * _y1 + _a2 * _y2 + _b * (input + 2 * _u1 + _u2);

    // Update state
    _u2 = _u1;
    _u1 = input;
    _y2 = _y1;
    _y1 = _y0;

    return _y0;
  }

  FType getOutput() const override { return _y0; }

  void reset() override {
    _y0 = 0;
    _y1 = 0;
    _y2 = 0;
    _u1 = 0;
    _u2 = 0;
  }

  private:
  void updateCoefficients() {
    FType den = _T * _T + 4 * _z * _T * _tau + 4 * _tau * _tau;
    _a1       = -(2 * _T * _T - 8 * _tau * _tau) / den;
    _a2       = -(_T * _T - 4 * _z * _T * _tau + 4 * _tau * _tau) / den;
    _b        = _T * _T * _k / den;
  }

  uint32_t _period_us;
  FType _T;
  FType _k, _tau, _z;
  FType _a1, _a2, _b;
  FType _y0, _y1, _y2, _u1, _u2;
};

} // namespace Filters::IIR
#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 1° Order High-Pass filter
class HighPass1 : public IFilter {
  public:
  HighPass1(const uint32_t period_us = 100 * 1000, const FType tau = 1, const FType gain = 1)
      : _period_us(period_us)
      , _T(0)
      , _k(gain)
      , _tau(tau)
      , _a(0)
      , _b(0)
      , _y0(0)
      , _y1(0)
      , _u1(0) {
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

  void setInitialValue(const FType initial_value) override {
    _y0 = initial_value;
    _y1 = initial_value;
    _u1 = initial_value;
  }

  FType update(const FType input) override {
    // Calculate output
    _y0 = _a * _y1 + _b * input - _b * _u1;

    // Update state
    _u1 = input;
    _y1 = _y0;

    return _y0;
  }

  FType getOutput() const override { return _y0; }

  void reset() override {
    _y0 = 0;
    _y1 = 0;
    _u1 = 0;
  }

  private:
  void updateCoefficients() {
    _a = -(_T - 2 * _tau) / (_T + 2 * _tau);
    _b = 2 * _k * _tau / (_T + 2 * _tau);
  }

  uint32_t _period_us;
  FType _T;
  FType _k, _tau;
  FType _a, _b;
  FType _y0, _y1, _u1;
};

} // namespace Filters::IIR
#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 2° Order Butterworth High-Pass filter
class ButterworthHighPass2 : public IFilter {
  public:
  ButterworthHighPass2()
      : _period_ms(0)
      , _T(0)
      , _k(0)
      , _tau(0)
      , _a1(0)
      , _a2(0)
      , _b(0)
      , _y0{0}
      , _y1(0)
      , _y2(0)
      , _u1(0)
      , _u2(0) {}

  void setPeriod(const uint32_t period_ms) {
    _period_ms = period_ms;
    _T         = period_ms / FType(1000);
    updateCoefficients();
  }

  uint32_t getPeriod() const { return _period_ms; }

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

  FType update(const FType input) override {
    // Calculate output
    _y0 = _a1 * _y1 + _a2 * _y2 + _b * (input - 2 * _u1 + _u2);

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
    FType den = _T * _T + 4 * _SQRT_2 * _T * _tau + 4 * _tau * _tau;
    _a1       = -(2 * _T * _T - 8 * _tau * _tau) / den;
    _a2       = -(_T * _T - 4 * _SQRT_2 * _T * _tau + 4 * _tau * _tau) / den;
    _b        = 4 * _tau * _tau * _k / den;
  }

  uint32_t _period_ms;
  FType _T;
  FType _k, _tau;
  FType _a1, _a2, _b;
  FType _y0, _y1, _y2, _u1, _u2;
  const static FType _SQRT_2 = 1.4142135623730950488016887242097;
};

} // namespace Filters::IIR
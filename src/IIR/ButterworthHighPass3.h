#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 3° Order Butterworth High-Pass filter
class ButterworthHighPass3 : public IFilter {
  public:
  ButterworthHighPass3()
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
      , _y3(0)
      , _u1(0)
      , _u2(0)
      , _u3(0) {}

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
    _y0 = _a1 * _y1 + _a2 * _y2 + _a3 * _y3 + _b * (input - 3 * _u1 + 3 * _u2 - _u3);

    // Update state
    _u3 = _u2;
    _u2 = _u1;
    _u1 = input;
    _y3 = _y2;
    _y2 = _y1;
    _y1 = _y0;

    return _y0;
  }

  FType getOutput() const override { return _y0; }

  void reset() override {
    _y0 = 0;
    _y1 = 0;
    _y2 = 0;
    _y3 = 0;
    _u1 = 0;
    _u2 = 0;
    _u3 = 0;
  }

  private:
  void updateCoefficients() {
    FType v1  = 8 * _T * _tau * _tau;
    FType v2  = 4 * _T * _T * _tau;
    FType den = _T * _T * _T + v1 + v2 + 8 * _tau * _tau * _tau;

    _a1 = (-3 * _T * _T * _T + v1 - v2 + 24 * _tau * _tau * _tau) / den;
    _a2 = (-3 * _T * _T * _T + v1 + v2 - 24 * _tau * _tau * _tau) / den;
    _a3 = -(_T * _T * _T + v1 - v2 - 8 * _tau * _tau * _tau) / den;
    _b  = 8 * _tau * _tau * _tau * _k / den;
  }

  uint32_t _period_ms;
  FType _T;
  FType _k, _tau;
  FType _a1, _a2, _a3, _b;
  FType _y0, _y1, _y2, _y3, _u1, _u2, _u3;
};

} // namespace Filters::IIR
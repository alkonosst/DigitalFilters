#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 2° Order Pass-Band filter
class PassBand2 : public IFilter {
  public:
  PassBand2()
      : _period_ms(0)
      , _T(0)
      , _k(0)
      , _wn(0)
      , _z(0)
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

  void setCutOffFrequency(const FType fc_hz) {
    _wn = 2 * PI * fc_hz;
    updateCoefficients();
  }

  FType getCutOffFrequency() const { return _wn / (2 * PI); }

  void setBandwidth(const FType bw_hz) {
    _z = bw_hz / (2 * _wn);
    updateCoefficients();
  }

  FType getBandwidth() const { return 2 * _z * _wn; }

  FType update(const FType input) override {
    // Calculate output
    _y0 = _a1 * _y1 + _a2 * _y2 + _b * (input - _u2);

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
    FType den = _T * _T * _wn * _wn + 4 * _z * _T * _wn + 4;
    _a1       = -(2 * _T * _T * _wn * _wn - 8) / den;
    _a2       = -(_T * _T * _wn * _wn - 4 * _z * _T * _wn + 4) / den;
    _b        = 4 * _T * _k * _wn * _z / den;
  }

  uint32_t _period_ms;
  FType _T;
  FType _k, _wn, _z;
  FType _a1, _a2, _b;
  FType _y0, _y1, _y2, _u1, _u2;
};

} // namespace Filters::IIR
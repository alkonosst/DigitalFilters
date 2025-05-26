#pragma once

#include "Filters.h"

namespace Filters::IIR {

// 2° Order Stop-Band filter
class StopBand2 : public IFilter {
  public:
  StopBand2(const uint32_t period_us = 100 * 1000, const FType center_freq_hz = 1,
            const FType bandwidth_hz = 1, const FType gain = 1)
      : _period_us(period_us)
      , _T(0)
      , _k(gain)
      , _w0(0)
      , _bw(bandwidth_hz)
      , _a1(0)
      , _a2(0)
      , _b0_2(0)
      , _b1(0)
      , _y0(0)
      , _y1(0)
      , _y2(0)
      , _u1(0)
      , _u2(0) {
    setPeriod(period_us);
    setCenterFrequency(center_freq_hz);
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

  void setCenterFrequency(const FType center_freq_hz) {
    _w0 = 2 * PI * center_freq_hz;
    updateCoefficients();
  }

  FType getCenterFrequency() const { return _w0 / (2 * PI); }

  void setBandwidth(const FType bandwidth_hz) {
    _bw = bandwidth_hz;
    updateCoefficients();
  }

  FType getBandwidth() const { return _bw; }

  void setInitialValue(const FType initial_value) override {
    _y0 = initial_value;
    _y1 = initial_value;
    _y2 = initial_value;
    _u1 = initial_value;
    _u2 = initial_value;
  }

  FType update(const FType input) override {
    // Calculate output
    _y0 = _a1 * _y1 + _a2 * _y2 + _b0_2 * (input + _u2) + _b1 * _u1;

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
    FType den = _T * _T * _w0 * _w0 + 2 * _bw * _T + 4;
    _a1       = -(2 * _T * _T * _w0 * _w0 - 8) / den;
    _a2       = -(_T * _T * _w0 * _w0 - 2 * _bw * _T + 4) / den;
    _b0_2     = (_k * _T * _T * _w0 * _w0 + 4 * _k) / den;
    _b1       = -(8 * _k - 2 * _T * _T * _w0 * _w0 * _k) / den;
  }

  uint32_t _period_us;
  FType _T;
  FType _k, _w0, _bw;
  FType _a1, _a2, _b0_2, _b1;
  FType _y0, _y1, _y2, _u1, _u2;
};

} // namespace Filters::IIR
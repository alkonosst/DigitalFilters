#pragma once

#include <Arduino.h>

#ifdef FILTERS_USE_DOUBLE
typedef double FType;
#else
typedef float FType;
#endif

namespace Filters {

// Interface class, useful when using pointers
class IFilter {
  public:
  virtual FType update(const FType input) = 0;
  virtual FType getOutput() const         = 0;
  virtual void reset()                    = 0;
};

} // namespace Filters
/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define UNITY_INCLUDE_DOUBLE
#include <unity.h>

#include "IIR/ButterworthHighPass3.h"
using namespace Filters::IIR;

#include "butter_hpf3.h"

constexpr uint32_t period_us = 100 * 1000; // 100 ms
constexpr FType k            = 1;
constexpr FType tau          = 1;
ButterworthHighPass3 filter;

void test_filter_output() {
  // Apply a step input to the filter and compare the output
  for (uint8_t i = 0; i < test_data_size; i++) {
    TEST_ASSERT_EQUAL_FLOAT(test_data[i], filter.update(FType(1)));
  }
}

void setup() {
  delay(2000);

  // Set up the filter
  filter.setPeriod(period_us);
  filter.setGain(k);
  filter.setTimeConstant(tau);

  UNITY_BEGIN();
  RUN_TEST(test_filter_output);
  UNITY_END();
}

void loop() {}
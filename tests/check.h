#pragma once
// Minimal test harness: CHECK / CHECK_NEAR record failures, main() returns test_result().
#include <cmath>
#include <cstdio>

inline int g_failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);   \
      g_failures++;                                                          \
    }                                                                        \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                          \
  do {                                                                                 \
    double a_ = (a), b_ = (b);                                                         \
    if (!(std::fabs(a_ - b_) <= (eps))) {                                              \
      std::printf("%s:%d: %s = %g, expected %g\n", __FILE__, __LINE__, #a, a_, b_);   \
      g_failures++;                                                                    \
    }                                                                                  \
  } while (0)

inline int test_result(const char *name) {
  std::printf("%s: %s\n", name, g_failures ? "FAILED" : "ok");
  return g_failures ? 1 : 0;
}

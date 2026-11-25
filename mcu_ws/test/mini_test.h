// Tiny dependency-free test harness. Each test is tagged with the requirement
// IDs it verifies, e.g. TEST(my_test, "MOT-1,MOT-2") { CHECK(...); }
#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace mt {
struct Case { const char* name; const char* reqs; void (*fn)(); };
inline std::vector<Case>& registry() { static std::vector<Case> r; return r; }
inline int& failures() { static int f = 0; return f; }
struct Reg { Reg(const char* n, const char* r, void (*f)()) { registry().push_back({n, r, f}); } };
}  // namespace mt

#define TEST(name, reqs)                                   \
  static void name();                                      \
  static mt::Reg reg_##name(#name, reqs, name);            \
  static void name()

#define CHECK(cond)                                                              \
  do { if (!(cond)) { std::printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
                      mt::failures()++; } } while (0)

#define CHECK_NEAR(a, b, eps)                                                    \
  do { double _a = (a), _b = (b);                                                \
       if (!(std::fabs(_a - _b) <= (eps))) {                                     \
         std::printf("    FAIL %s:%d: %s ~= %s  (%g vs %g)\n", __FILE__, __LINE__, #a, #b, _a, _b); \
         mt::failures()++; } } while (0)

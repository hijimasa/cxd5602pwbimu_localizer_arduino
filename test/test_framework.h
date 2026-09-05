/**
 * Minimal self-contained test framework (no external dependencies).
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <functional>

struct TestCase
{
  std::string name;
  std::function<void()> fn;
};

inline std::vector<TestCase> &testRegistry()
{
  static std::vector<TestCase> r;
  return r;
}

inline int &testFailures()
{
  static int f = 0;
  return f;
}

struct TestRegistrar
{
  TestRegistrar(const char *name, std::function<void()> fn)
  {
    testRegistry().push_back({name, fn});
  }
};

#define TEST(name)                                          \
  static void test_##name();                                \
  static TestRegistrar registrar_##name(#name, test_##name); \
  static void test_##name()

#define CHECK(cond)                                                              \
  do                                                                             \
  {                                                                              \
    if (!(cond))                                                                 \
    {                                                                            \
      std::printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
      testFailures()++;                                                          \
    }                                                                            \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                  \
  do                                                                                           \
  {                                                                                            \
    const double _a = (a), _b = (b), _t = (tol);                                               \
    if (!(std::fabs(_a - _b) <= _t))                                                           \
    {                                                                                          \
      std::printf("    FAIL %s:%d: %s = %g, expected %s = %g (tol %g)\n", __FILE__, __LINE__, \
                  #a, _a, #b, _b, _t);                                                         \
      testFailures()++;                                                                        \
    }                                                                                          \
  } while (0)

#define CHECK_LT(a, b)                                                                      \
  do                                                                                        \
  {                                                                                         \
    const double _a = (a), _b = (b);                                                        \
    if (!(_a < _b))                                                                         \
    {                                                                                       \
      std::printf("    FAIL %s:%d: %s = %g is not < %s = %g\n", __FILE__, __LINE__, #a, _a, \
                  #b, _b);                                                                  \
      testFailures()++;                                                                     \
    }                                                                                       \
  } while (0)

#define METRIC(label, value) std::printf("    metric %-40s %g\n", label, (double)(value))

#endif // TEST_FRAMEWORK_H

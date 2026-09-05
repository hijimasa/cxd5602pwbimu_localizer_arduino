#include "test_framework.h"
#include <cstring>

int main(int argc, char **argv)
{
  const char *filter = (argc > 1) ? argv[1] : nullptr;
  int run = 0;
  for (const TestCase &tc : testRegistry())
  {
    if (filter && std::strstr(tc.name.c_str(), filter) == nullptr)
    {
      continue;
    }
    const int before = testFailures();
    std::printf("[ RUN  ] %s\n", tc.name.c_str());
    tc.fn();
    run++;
    if (testFailures() == before)
    {
      std::printf("[  OK  ] %s\n", tc.name.c_str());
    }
    else
    {
      std::printf("[ FAIL ] %s\n", tc.name.c_str());
    }
  }
  std::printf("\n%d tests run, %d check(s) failed\n", run, testFailures());
  return testFailures() == 0 ? 0 : 1;
}

// Minimal test macros shared by the host tests.
#pragma once
#include <stdio.h>
#include <string>

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; \
  printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ(a, b) do { checks++; auto _a = (a); auto _b = (b); if (!(_a == _b)) { failures++; \
  printf("FAIL %s:%d: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, \
         (long long)_a, (long long)_b); } } while (0)
#define CHECK_STR(a, b) do { checks++; std::string _a = (a), _b = (b); if (_a != _b) { failures++; \
  printf("FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__, _a.c_str(), _b.c_str()); } } while (0)

static int report(const char *name) {
  printf("%s %s: %d checks, %d failed\n", failures ? "FAIL" : "OK", name, checks, failures);
  return failures ? 1 : 0;
}

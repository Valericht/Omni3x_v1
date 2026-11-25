#pragma once
// Kerangka tes mini (tanpa dependency). TEST(nama) { CHECK(...); }
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

struct TestCase { const char* name; void (*fn)(); };
std::vector<TestCase>& test_registry();
struct TestReg { TestReg(const char* n, void (*f)()) { test_registry().push_back({n, f}); } };
void test_fail(const char* file, int line, const std::string& what);

#define TEST(name) static void name(); static TestReg reg_##name(#name, name); static void name()
#define CHECK(c) do { if (!(c)) test_fail(__FILE__, __LINE__, #c); } while (0)
#define CHECK_MSG(c, msg) do { if (!(c)) test_fail(__FILE__, __LINE__, std::string(#c) + "  <- " + (msg)); } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b), _t = (tol); \
  if (!(std::fabs(_a - _b) <= _t)) { char _m[256]; \
    snprintf(_m, sizeof _m, "%s ~ %s  (%.6g vs %.6g, tol %.3g)", #a, #b, _a, _b, _t); test_fail(__FILE__, __LINE__, _m); } } while (0)

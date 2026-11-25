#include "test_main.h"
#include <cstring>

std::vector<TestCase>& test_registry() { static std::vector<TestCase> r; return r; }
static int g_cur_fail = 0;
void test_fail(const char* file, int line, const std::string& what) {
  g_cur_fail++;
  printf("    GAGAL %s:%d  %s\n", file, line, what.c_str());
}

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int ran = 0, failed = 0;
  for (auto& t : test_registry()) {
    if (filter && !strstr(t.name, filter)) continue;
    g_cur_fail = 0; t.fn(); ran++;
    printf("  %-62s %s\n", t.name, g_cur_fail ? "GAGAL" : "lulus");
    if (g_cur_fail) failed++;
  }
  printf("\nTES HOST FIRMWARE: %d dijalankan, %d lulus, %d gagal, 0 dilewati\n", ran, ran - failed, failed);
  return failed ? 1 : (ran ? 0 : 2);
}

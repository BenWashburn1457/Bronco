#include <string>
#include "mini_test.h"

int main(int argc, char** argv) {
  if (argc > 1 && std::strcmp(argv[1], "--reqs") == 0) {  // list covered IDs
    for (auto& c : mt::registry()) {
      std::string s = c.reqs, tok;
      for (char ch : s + ",") { if (ch == ',') { if (!tok.empty()) std::printf("%s\n", tok.c_str()); tok.clear(); } else if (ch != ' ') tok += ch; }
    }
    return 0;
  }
  int failedTests = 0;
  for (auto& c : mt::registry()) {
    int before = mt::failures();
    c.fn();
    bool ok = mt::failures() == before;
    std::printf("[%s] %s  (%s)\n", ok ? " ok " : "FAIL", c.name, c.reqs);
    if (!ok) failedTests++;
  }
  std::printf("\n%zu tests, %d failed\n", mt::registry().size(), failedTests);
  return failedTests ? 1 : 0;
}

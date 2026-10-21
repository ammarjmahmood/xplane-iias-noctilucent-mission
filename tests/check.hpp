#pragma once
// Test check that stays active in Release builds (unlike assert).
#include <cstdio>
#include <cstdlib>
#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (!(cond)) {                                                                  \
      std::fprintf(stderr, "CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      std::exit(1);                                                                 \
    }                                                                               \
  } while (0)

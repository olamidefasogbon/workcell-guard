// Minimal assertion helper for the host-side tests.
#ifndef CHECK_H
#define CHECK_H
#include <stdio.h>
#include <stdlib.h>

static int g_checks = 0, g_failures = 0;

#define CHECK(cond, ...) do { \
    g_checks++; \
    if (!(cond)) { g_failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

#define TEST(name) printf("  - %s\n", name)

static inline int check_summary(const char *suite) {
    printf("%s: %d checks, %d failed\n", suite, g_checks, g_failures);
    return g_failures ? 1 : 0;
}
#endif

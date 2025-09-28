#ifndef __LOCAL_ASSERT_H__
#define __LOCAL_ASSERT_H__
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void _assert(bool cond, const char * const cond_str, size_t line, bool invert) {
  if(invert != cond) {
    printf(COLOR_RED"ERROR: " COLOR_RESET __FILE__":%lu:0: %s\n", (unsigned long)line, cond_str);
    exit(-1);
  }
}
#define assert_false(CONDITION) _assert(CONDITION, #CONDITION, __LINE__, false)
#define assert_true(CONDITION) _assert(CONDITION, #CONDITION, __LINE__, true)

#ifndef assert
#define assert assert_true
#endif

#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__LOCAL_ASSERT_H__

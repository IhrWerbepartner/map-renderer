#pragma once

#include "base.h"

typedef struct String8 String8;
struct String8 {
  const char *buf;
  U64 len;
};

static U64 cstring8_length(const char *c) {
  U64 length = 0;
  if (c) {
    const char *p = c;
    for (; *p != 0; p += 1)
      ;
    length = (U64)(p - c);
  }
  return length;
}

String8 String8FromCStringLen(const char *string, size_t len) {
  return (String8){
      .buf = string,
      .len = len,
  };
}

String8 String8FromCString(const char *string) {
  U64 len = cstring8_length(string);
  return String8FromCStringLen(string, len);
}

// returns -1 if a is earlier 0 if equal 1 if later
S8 string8_compare(String8 a, String8 b) {
  for (U64 i = 0; i < a.len && b.len; i++) {
    if (a.buf[i] < b.buf[i]) {
      return -1;
    }
    if (a.buf[i] > b.buf[i]) {
      return 1;
    }
  }
  if (a.len < b.len) {
    return -1;
  }
  if (a.len > b.len) {
    return -1;
  }
  return 0;
}

S8 String8Equals(String8 a, String8 b) { return string8_compare(a, b) == 0; }

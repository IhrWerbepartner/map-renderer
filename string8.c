#pragma once

#include "base.h"

typedef struct string8 string8;
struct string8 {
  const char *buf;
  S64 len;
};

U64 cstring8_length(U8 *c)
{
  U64 length = 0;
  if(c)
  {
    U8 *p = c;
    for (;*p != 0; p += 1);
    length = (U64)(p - c);
  }
  return length;
}

string8 string8_from_c_string_len(const char *string, size_t len) {
  return (string8){
      .buf = string,
      .len = len,
  };
}

string8 string8_from_c_string(const char *string) {
  U64 len = cstring8_length(string);
  return string8_from_c_string_len(string, len);
}

// returns -1 if a is earlier 0 if equal 1 if later
S8 string8_compare(string8 a, string8 b) {
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

S8 string8_equals(string8 a, string8 b) {
  return string8_compare(a, b) == 0;
}

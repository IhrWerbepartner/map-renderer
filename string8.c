#include <stddef.h>
#include <string.h>
typedef struct string8 string8;
struct string8 {
  const char *buf;
  size_t len;
};

string8 string8_from_c_string_len(const char *string, size_t len) {
  return (string8){
      .buf = string,
      .len = len,
  };
}

string8 string8_from_c_string(const char *string) {
  size_t len = strlen(string);
  return string8_from_c_string_len(string, len);
}

// returns -1 if a is earlier 0 if equal 1 if later
int string8_compare(string8 a, string8 b) {
  for (size_t i = 0; i < a.len && b.len; i++) {
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

int string8_equals(string8 a, string8 b) {
  return string8_compare(a, b) == 0;
}

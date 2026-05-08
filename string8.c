#include <stddef.h>
typedef struct string8 string8;
struct string8 {
  char *buf;
  size_t len;
};

string8 from_c_string(char *string, size_t len) {
  return (string8){
      .buf = string,
      .len = len,
  };
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

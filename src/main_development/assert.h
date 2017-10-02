

extern void error_catch(void);

#define assert(x)      \
    if (x) {           \
    } else {           \
        error_catch(); \
    }

#define IS_BASE64_ENCODED(c)    \
  ((('A' <= c) && (c <= 'Z')) || (('a' <= c) && (c <= 'z')) || (('0' <= c) && (c <= '9')) || (c == '+') || (c == '/') || (c == '='))

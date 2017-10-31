
// error.h - debugging snippets

extern void error_catch(void);

/* error_catch();                              \ */

#define assert(x)                               \
  if (x) {                                      \
  } else {                                      \
    __asm("BKPT"); \
  } \

#define IS_BASE64_ENCODED(c)                                            \
  ((('A' <= c) && (c <= 'Z')) || (('a' <= c) && (c <= 'z')) || (('0' <= c) && (c <= '9')) || (c == '+') || (c == '/') || (c == '='))

// a 'millisecond' counter for keeping track of time
// for the various state machines
extern volatile uint32_t millis;

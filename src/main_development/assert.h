

extern void error_catch(void);

#define assert(x) if (x) {} else {error_catch();}

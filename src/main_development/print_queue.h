
#include <stdbool.h>
// 2018-08-04 FIXME: change buffer length to reasonable, 'justified' size
#define BUFFER_LEN 200

typedef struct queue_t {
    char char_buf[BUFFER_LEN];
    int head;
    int tail;
    int n_elements;
} queue_t;

void queue_init(queue_t* q);
void queue_push(queue_t* q, char c);
bool queue_is_empty(queue_t* q);
char queue_pop(queue_t* q);

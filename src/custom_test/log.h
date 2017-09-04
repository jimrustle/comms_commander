#include <stdint.h>

typedef enum usart_num {
  USART_1, USART_2
} usart_num;

void pr_ch(usart_num u, char c);
void pr_str(usart_num u, const char * s);
void pr_hex(usart_num u, uint8_t h);
void pr_nl(usart_num u);

#ifndef __UART_H_   /* Include guard */
#define __UART_H_

#include "Common.h"

void uart0_init(void);
BYTE uart0_getchar(void);
BOOLEAN uart0_dataAvailable(void);
void uart0_putchar(char ch);
void uart0_put(char *ptr_str);
void uart2_init(void);
BYTE uart2_getchar(void);
BOOLEAN uart2_dataAvailable(void);
void uart2_putchar(char ch);
void uart2_put(char *ptr_str);

#endif // __UART_H_


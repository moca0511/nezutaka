#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t ugets(UART_HandleTypeDef *uartHandle,uint8_t* data,uint8_t data_len);
void print_str(char *str);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

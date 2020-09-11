#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t ugets(UART_HandleTypeDef *uartHandle,uint8_t* data,uint8_t data_len);


#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

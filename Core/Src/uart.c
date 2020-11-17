/*
 * uart.c
 *
 *  Created on: Jun 8, 2020
 *      Author: junmo
 */

#include "main.h"
#include "uart.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*
 * printf用関数
 * uart1から文字列ptrを1文字ずつlen回出力。タイムアウト1000秒
 */
void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart1, &ch, 1,1);
}

uint8_t ugets(UART_HandleTypeDef *uartHandle, uint8_t *data, uint8_t data_len) {
	uint8_t len = 0;
	uint8_t buff[1] = { 0 };
	for (int i = 0; i < data_len; i++) { // 入力先クリア
		data[i] = '\0';
	}
	do {
		while (HAL_UART_Receive(uartHandle, (uint8_t*) buff, sizeof(buff), 0xF)
				!= 0)
			; //1文字受信
		if (len == 100) { //	バッファオーバー
			len = 0;
			for (int i = 0; i < data_len; i++) {
				data[i] = '\0';
			}
		}
		data[len++] = buff[0]; //	入力先に代入
	} while (buff[0] != '\n'); //	改行されるまで
	return len;
}
void print_str(char *str) {
	if (osMutexWait(UART_MutexHandle, osWaitForever) == osOK) {
		printf("%s", str);
		osMutexRelease(UART_MutexHandle);
	}
}

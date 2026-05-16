#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static void(*uart_tx_complete_callback)(void) = NULL;
static void(*uart_rx_complete_callback)(void) = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool uart1_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0 || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart_tx_complete_callback = callback;
    }
}

void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart_rx_complete_callback = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        if(uart_tx_complete_callback != NULL) {
            uart_tx_complete_callback();
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        if(uart_tx_complete_callback != NULL) {
            uart_tx_complete_callback();
        }
    }
}

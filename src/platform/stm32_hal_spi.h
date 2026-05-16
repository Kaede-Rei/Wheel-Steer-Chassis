#ifndef _stm32_hal_spi_h_
#define _stm32_hal_spi_h_

#include <stdint.h>
#include <stdbool.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define SPI_TIMEOUT 10

extern SPI_HandleTypeDef hspi6;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool spi_write(const uint8_t* data, uint32_t len);
void spi_register_tx_complete_callback(SPI_HandleTypeDef* hspi, void (*callback)(void));

#endif

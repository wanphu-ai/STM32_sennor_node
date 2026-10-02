#pragma once
#include <stdint.h>
#include "main.h"

#define L0_COUNT 0
#define L1_COUNT 2
#define TOTAL_SENSORS (L0_COUNT + L1_COUNT)

// Cấu trúc GPIO đặc thù của STM32
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} SensorGPIO;

// Ánh xạ 3 chân XSHUT của L0 (Đảm bảo bạn đã đặt tên XSHUT1, XSHUT2, XSHUT3 trong CubeMX)
const SensorGPIO L0_XSHUT_PINS[L0_COUNT] = {
    //{XSHUT4_GPIO_Port, XSHUT4_Pin},
    //{XSHUT5_GPIO_Port, XSHUT5_Pin},
    //{XSHUT3_GPIO_Port, XSHUT3_Pin}
};

// Ánh xạ 2 chân XSHUT của L1
const SensorGPIO L1_XSHUT_PINS[L1_COUNT] = {
    {XSHUT0_GPIO_Port, XSHUT0_Pin},
    {XSHUT1_GPIO_Port, XSHUT1_Pin}
};

// Lõi ST yêu cầu nguyên 8-bit
const uint8_t L0_I2C_ADDRS[L0_COUNT] = {
		//0x66,
		//0x68,
		//0x64
};
const uint8_t L1_I2C_ADDRS[L1_COUNT] = {
		0x60,
		0x62
};

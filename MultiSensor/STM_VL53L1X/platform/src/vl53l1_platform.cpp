#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

extern "C" {

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if (HAL_I2C_Mem_Write(&hi2c1, Dev->i2c_slave_address, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100) != HAL_OK) {
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if (HAL_I2C_Mem_Read(&hi2c1, Dev->i2c_slave_address, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100) != HAL_OK) {
        return VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t buffer[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return VL53L1_WriteMulti(Dev, index, buffer, 2);
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    uint8_t buffer[4] = {(uint8_t)(data >> 24), (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF)};
    return VL53L1_WriteMulti(Dev, index, buffer, 4);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    uint8_t buffer[2];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 2);
    if (status == VL53L1_ERROR_NONE) *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    uint8_t buffer[4];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 4);
    if (status == VL53L1_ERROR_NONE) *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
    return status;
}

int8_t VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms) {
    HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us) {
    HAL_Delay((wait_us / 1000) + 1);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(VL53L1_DEV Dev, uint32_t *ptime_ms) {
    *ptime_ms = HAL_GetTick();
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms) {
    uint8_t register_value = 0;
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint32_t start_time_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_time_ms) < timeout_ms) {
        status = VL53L1_RdByte(Dev, index, &register_value);
        if (status != VL53L1_ERROR_NONE) return status;
        if ((register_value & mask) == value) return VL53L1_ERROR_NONE;
        HAL_Delay(poll_delay_ms);
    }
    return VL53L1_ERROR_TIME_OUT;
}

}

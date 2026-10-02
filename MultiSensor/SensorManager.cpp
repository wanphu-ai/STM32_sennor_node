#include <SensorManager.h>
#include "main.h"

SensorManager::SensorManager() {
    for (int i = 0; i < L0_COUNT; i++) {
        p_l0_sensors[i] = &l0_devices[i];
        p_l0_sensors[i]->I2cDevAddr = 0x52;
    }
    for (int i = 0; i < L1_COUNT; i++) {
        p_l1_sensors[i] = &l1_devices[i];
        p_l1_sensors[i]->i2c_slave_address = 0x52;
    }
}

void SensorManager::resetAllSensors() {
    for (int i = 0; i < L0_COUNT; i++) {
        HAL_GPIO_WritePin(L0_XSHUT_PINS[i].port, L0_XSHUT_PINS[i].pin, GPIO_PIN_RESET);
    }
    for (int i = 0; i < L1_COUNT; i++) {
        HAL_GPIO_WritePin(L1_XSHUT_PINS[i].port, L1_XSHUT_PINS[i].pin, GPIO_PIN_RESET);
    }
    HAL_Delay(20);
}

bool SensorManager::init() {
    resetAllSensors();

    if (!initL0Sensors()) return false;
    if (!initL1Sensors()) return false;

    return true;
}

bool SensorManager::initL0Sensors() {
    for (int i = 0; i < L0_COUNT; i++) {
        // 1. Kéo HIGH để bật cảm biến
        HAL_GPIO_WritePin(L0_XSHUT_PINS[i].port, L0_XSHUT_PINS[i].pin, GPIO_PIN_SET);
        HAL_Delay(10);
        
        // 2. Đổi địa chỉ I2C
        VL53L0X_Error status = VL53L0X_SetDeviceAddress(p_l0_sensors[i], L0_I2C_ADDRS[i]);
        if (status != VL53L0X_ERROR_NONE) return false;
        p_l0_sensors[i]->I2cDevAddr = L0_I2C_ADDRS[i];
        
        // 3. Khởi tạo thuật toán
        status = VL53L0X_DataInit(p_l0_sensors[i]);
        if (status != VL53L0X_ERROR_NONE) return false;
        
        status = VL53L0X_StaticInit(p_l0_sensors[i]);
        if (status != VL53L0X_ERROR_NONE) return false;
        
        uint32_t refSpadCount;
        uint8_t isApertureSpads;
        status = VL53L0X_PerformRefSpadManagement(p_l0_sensors[i], &refSpadCount, &isApertureSpads);
        if (status != VL53L0X_ERROR_NONE) return false;
        
        uint8_t VhvSettings;
        uint8_t PhaseCal;
        status = VL53L0X_PerformRefCalibration(p_l0_sensors[i], &VhvSettings, &PhaseCal);
        if (status != VL53L0X_ERROR_NONE) return false;
        
        status = VL53L0X_SetDeviceMode(p_l0_sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        if (status != VL53L0X_ERROR_NONE) return false;
        
        status = VL53L0X_StartMeasurement(p_l0_sensors[i]);
        if (status != VL53L0X_ERROR_NONE) return false;
    }
    return true;
}

bool SensorManager::initL1Sensors() {
    for (int i = 0; i < L1_COUNT; i++) {
        HAL_GPIO_WritePin(L1_XSHUT_PINS[i].port, L1_XSHUT_PINS[i].pin, GPIO_PIN_SET);
        HAL_Delay(20);

        VL53L1_Error status;

        status = VL53L1_WaitDeviceBooted(p_l1_sensors[i]);
        if (status != VL53L1_ERROR_NONE) return false;

        status = VL53L1_SetDeviceAddress(p_l1_sensors[i], L1_I2C_ADDRS[i]);
        if (status != VL53L1_ERROR_NONE) return false;
        p_l1_sensors[i]->i2c_slave_address = L1_I2C_ADDRS[i];

        status = VL53L1_DataInit(p_l1_sensors[i]);
        if (status != VL53L1_ERROR_NONE) return false;

        status = VL53L1_StaticInit(p_l1_sensors[i]);
        if (status != VL53L1_ERROR_NONE) return false;

        status = VL53L1_PerformRefSpadManagement(p_l1_sensors[i]);
        if (status != VL53L1_ERROR_NONE) return false;

        VL53L1_SetDistanceMode(p_l1_sensors[i], VL53L1_DISTANCEMODE_LONG);
        VL53L1_SetMeasurementTimingBudgetMicroSeconds(p_l1_sensors[i], 50000);
        VL53L1_SetInterMeasurementPeriodMilliSeconds(p_l1_sensors[i], 100);

        status = VL53L1_StartMeasurement(p_l1_sensors[i]);
        if (status != VL53L1_ERROR_NONE) return false;
    }
    return true;
}

uint16_t SensorManager::getDistance(uint8_t index) {
    if (index >= TOTAL_SENSORS) return 0xFFFF;

    if (index < L0_COUNT) {
        VL53L0X_RangingMeasurementData_t rangingData;
        VL53L0X_Error status = VL53L0X_GetRangingMeasurementData(p_l0_sensors[index], &rangingData);

        if (status == VL53L0X_ERROR_NONE && rangingData.RangeStatus == 0) {
            return rangingData.RangeMilliMeter;
        }
    }
    else {
        uint8_t l1_index = index - L0_COUNT;
        VL53L1_RangingMeasurementData_t rangingData;
        VL53L1_Error status = VL53L1_GetRangingMeasurementData(p_l1_sensors[l1_index], &rangingData);

        if (status == VL53L1_ERROR_NONE && rangingData.RangeStatus == 0) {
            return rangingData.RangeMilliMeter;
        }
    }
    return 0xFFFF;
}
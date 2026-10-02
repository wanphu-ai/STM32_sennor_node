#pragma once
#include "config.h"

extern "C" {
    #include "vl53l0x_api.h"
	#undef _LOG_FUNCTION_START
    #undef _LOG_FUNCTION_END
    #undef _LOG_FUNCTION_END_FMT
    #undef PALDevDataSet
    #undef PALDevDataGet
	#include "vl53l1_api.h"
}

class SensorManager {
private:
	VL53L0X_Dev_t l0_devices[L0_COUNT];
	VL53L0X_DEV   p_l0_sensors[L0_COUNT];

	VL53L1_Dev_t  l1_devices[L1_COUNT];
	VL53L1_DEV    p_l1_sensors[L1_COUNT];

	void resetAllSensors();
	bool initL0Sensors();
	bool initL1Sensors();

public:
	SensorManager();
	bool init();
    uint16_t getDistance(uint8_t sensorIndex);
};

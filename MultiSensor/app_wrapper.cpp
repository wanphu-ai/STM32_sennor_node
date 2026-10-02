#include <SensorManager.h>
#include "app_wrapper.h"

SensorManager mySensors;

extern "C" bool sensor_system_init(void) {
    return mySensors.init();
}

extern "C" uint16_t sensor_system_get_distance(uint8_t index) {
    return mySensors.getDistance(index);
}

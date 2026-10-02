#ifndef APP_WRAPPER_H
#define APP_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool sensor_system_init(void);
uint16_t sensor_system_get_distance(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif

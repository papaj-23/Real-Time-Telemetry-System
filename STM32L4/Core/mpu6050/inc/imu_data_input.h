#ifndef IMU_DATA_INPUT
#define IMU_DATA_INPUT

#include <stdint.h>

/**
 * @brief Init RTOS resources for MPU6050.
 *
 * @retval 0 OK
 * @retval 1 Error
 */
uint8_t imu_rtos_init();

#endif
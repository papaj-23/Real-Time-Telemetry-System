#ifndef MPU_6050_H
#define MPU_6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* measurement mode */
typedef enum {
  MPU_SINGLE_MODE = 0U,
  MPU_BURST_MODE = 1U,
  MPU_LOWPOWER_CYCLE_MODE = 2U,
} MPU_6050_mode_t;

/* i2c bus access */
typedef enum {
  MPU_BUS_UNLOCKED = 0U,
  MPU_BUS_LOCKED = 1U
} MPU_6050_bus_access_t;

/* data struct for raw format */
typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MPU_6050_rawdata_t;

/* sensor handle struct */
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t *rx_buffer; 
  MPU_6050_rawdata_t *inter_buffer;
  void (*delay_ms_wrapper)(uint32_t);
  MPU_6050_mode_t meas_mode;
  MPU_6050_bus_access_t bus_status;
  uint16_t burst_count;
  volatile uint16_t fifo_count;
  volatile uint8_t fifo_count_raw[2];
  volatile uint8_t int_status;
  uint8_t payload_bytes;
  uint8_t gyro_scale;
  uint8_t accel_scale;
  volatile uint8_t fifo_oflow_flag;
  struct {
    uint8_t acc_x :1;
    uint8_t acc_y :1;
    uint8_t acc_z :1;
    uint8_t temp :1;
    uint8_t gyro_x :1;
    uint8_t gyro_y :1;
    uint8_t gyro_z :1;
    uint8_t reserved :1;
  } active_sources;
} MPU_6050_t;

/* selftest results struct */
typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} MPU_6050_selftest_t;

/* data struct for readable float format */
typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
  float temp;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} MPU_6050_data_t;

/* library eneble/disable */
typedef enum {
  MPU_DISABLE = 0U,
  MPU_ENABLE = 1U
} MPU_6050_state_t;

/* gyroscope measurement range (deg/s) */
typedef enum {
  DPS_250 = 0U,
  DPS_500 = 1U,
  DPS_1000 = 2U,
  DPS_2000 = 3U
} MPU_6050_gyro_range_t;

/* accelerometer measurement range (g) */
typedef enum {
  G_2 = 0U,
  G_4 = 1U,
  G_8 = 2U,
  G_16 = 3U
} MPU_6050_accel_range_t;

/* fifo content enable/disable */
typedef enum {
  FIFO_ACCEL = 8U,
  FIFO_GYRO_Z = 16U,
  FIFO_GYRO_Y = 32U,
  FIFO_GYRO_X = 64U,
  FIFO_TEMP = 128U
} MPU_6050_fifo_content_t;

/* low power cycle frequency (Hz) */
typedef enum {
  F_1_25HZ = 0U,
  F_5HZ = 1U,
  F_20HZ = 2U,
  F_40HZ = 3U
} MPU_6050_lp_freq_t;

/* enable/disable a specific sensor */
typedef enum {
  ACCEL_X_CH = 0U,
  ACCEL_Y_CH = 1U,
  ACCEL_Z_CH = 2U,
  TEMP_CH = 3U,
  GYRO_X_CH = 4U,
  GYRO_Y_CH = 5U,
  GYRO_Z_CH = 6U
} MPU_6050_meas_channel_t;


#define I2C_ADDRESS_7B      (uint16_t) 0x68U 

/* Amount of data bytes in FIFO buffer that triggers a burst read.
  Each active sensor channel delivers 2 bytes per sample. Accelerometer axes can not be selected as 
  FIFO members separately - any active axis means 6 bytes per sample. 
  BURST COUNT max value is 1024 - size of hardware buffer.
  BURST COUNT value should be divisible by the amount of bytes written in a single sample */
#define BURST_COUNT 560U

/* Sample Rate = GyroRate / (1 + SMPLRT_DIV)
 * GyroRate = 1kHz (DLPF on) or 8kHz (DLPF off) */
#define SMPLTR_DIV_VAL     0x63U

/* For details check out README section DPLF Setting or vendor datasheet directly */
#define CONFIG_VAL         0x04U

/* setting this to 1 enables a simple mechanism to read all registers significant for this library */
#define MPU_USE_DEBUG_REGISTERS 0


#if (MPU_USE_DEBUG_REGISTERS == 1)
/**
  * @brief  Read all register values.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  values List to store register values
  * 
  * @details Read all values of registers significant for this library.
  * 
  * @note This function is for debug purpose, it has to be enabled by setting MPU_USE_DEBUG_REGISTERS to 1
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_check_registers(MPU_6050_t *handles);
#endif

/**
  * @brief  Initialize MPU6050 sensor with default configuration.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This function provides startup sequence for MPU6050. The sensor comes up in single read mode.
  * Writes a set registers (power management, sample rate divider,
  * low-pass filter config, FIFO enable, interrupt enable) included in init_registers[].
  * Brings all onboard sensors to active state
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_init(MPU_6050_t *handles);

/**
  * @brief  Configure MPU6050 power/streaming mode (single sample, FIFO burst, or low-power cycle).
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  mode    Selected mode: MPU_SINGLE_MODE, MPU_BURST_MODE, or MPU_LOWPOWER_CYCLE_MODE.
  *
  * @details
  * The function configures interrupt source, FIFO usage and power-management bits
  * to match the selected operating mode:
  *
  * MPU_SINGLE_MODE:
  *  - Enables only Data Ready interrupt (INT_ENABLE: DATA_RDY_EN).
  *  - Disables FIFO streaming and all fifo contents.
  *  - Forces normal operation: CYCLE = 0, SLEEP = 0.
  *  - Initially enables all onboard sensors.
  *
  * MPU_BURST_MODE:
  *  - Enables only FIFO Overflow interrupt (INT_ENABLE: FIFO_OFLOW_EN).
  *  - Enables FIFO streaming, initially enables all fifo contents.
  *  - Forces normal operation: CYCLE = 0, SLEEP = 0.
  *  - Initially enables all onboard sensors.
  *
  * MPU_LOWPOWER_CYCLE_MODE:
  *  - Enables only Data Ready interrupt (INT_ENABLE: DATA_RDY_EN).
  *  - Disables FIFO streaming and all fifo contents.
  *  - Configures accel-only low power cycling:
  *      * TEMP disabled (PWR_MGMT_1: TEMP_DIS = 1)
  *      * Gyro placed in standby (PWR_MGMT_2: gyro standby bits = 1)
  *      * Accel enabled (PWR_MGMT_2: accel standby bits = 0)
  *      * CYCLE enabled, SLEEP disabled (PWR_MGMT_1: CYCLE = 1, SLEEP = 0)
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_mode(MPU_6050_t *handles, MPU_6050_mode_t mode);

/**
  * @brief  Turn on/off Sleep Mode
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  state MPU_ENABLE to enable, MPU_DISABLE to disable.
  * 
  * @details In sleep mode all onboard sensor and internal clock source are turned off.
  * This mode can be used in low power applications. Low powers cycle mode automatically
  * switches betweern sleep mode and taking a sample. 
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_sleep(MPU_6050_t *handles, MPU_6050_state_t state);

/**
  * @brief  Configure wake-up frequency for accelerometer low-power cycle mode.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  freq    Low-power wake-up frequency selection (MPU_6050_lp_freq_t).
  *
  * @details
  * Available wake-up frequencies:
  *  - F_1_25HZ → 1.25 Hz
  *  - F_5HZ    → 5 Hz
  *  - F_20HZ   → 20 Hz
  *  - F_40HZ   → 40 Hz
  *
  * @note
  * This setting has effect only when CYCLE mode is enabled
  * (PWR_MGMT_1: CYCLE = 1). In normal continuous measurement
  * modes, this field does not affect sensor output rate.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_lp_wakeup_freq(MPU_6050_t *handles, MPU_6050_lp_freq_t freq);

/**
  * @brief  Enable or disable selected measurement channel.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  ch      Measurement channel to configure (accelerometer axes,
  *                 gyroscope axes or temperature).
  * @param  state   MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @details
  * FIFO content is updated automatically for the affected channel:
  * - Gyroscope axes and temperature sensor directly enable/disable their
  *   corresponding FIFO sources.
  * - Accelerometer FIFO entry is shared between X/Y/Z axes, therefore it is
  *   enabled when at least one accelerometer axis is active and disabled only
  *   when all accelerometer axes are turned off.
  *
  * FIFO configuration may also be controlled independently using
  * MPU_6050_Set_FIFO_Content() if manual configuration is required.
  *
  * @attention This function has an effect only in burst read mode and single read mode
  *            In low power cycle mode channels are fixed
  * @note Internal logic is inverted relative to the register bits:
  *            passing MPU_ENABLE enables the selected measurement channel,
  *            while MPU_DISABLE disables it.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_source(MPU_6050_t *handles, MPU_6050_meas_channel_t ch, MPU_6050_state_t state);

/**
  * @brief  Enable or disable selected sensor data in FIFO buffer.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  content FIFO content mask (combination of MPU_6050_fifo_content_t values).
  * @param  state   MPU_ENABLE to set bits, MPU_DISABLE to clear bits.
  *
  * @details
  * This function allows user to manualy define fifo contents in burst read mode. 
  * Usually mechanism included in MPU_6050_set_source() is sufficient, but for some
  * aplications it might be required to keep for instance the temperature sensor up
  * while not including its measurements in FIFO buffer.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_fifo_content(MPU_6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state);

/**
  * @brief  Reset MPU6050 FIFO buffer and re-enable FIFO operation.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Clears FIFO enable bit, triggers FIFO reset bit, then re-enables FIFO.
  * If handles->delay_ms_wrapper is provided, it may be used to wait a short time
  * between reset and re-enable.
  * 
  * @note most frequent use case of this function is when FIFO_OVERFLOW interrupt occurs
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_fifo_reset(MPU_6050_t *handles);

/**
  * @brief  Perform built-in self-test procedure of MPU6050.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  result  Pointer to structure storing self-test results (in %).
  *
  * @details
  * The device should remain stationary during the test. This function
  * performs a sequence of operations that allows the user to determine 
  * whether MPU6050 accuracy is within tolerance level which is +-14%
  * according to the vendor datasheet for both accelerometer and gyroscope.
  * For more details about the sequence, check the implementation in .c file
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_self_test(MPU_6050_t *handles, MPU_6050_selftest_t *result);

/**
  * @brief  Set gyroscope full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Gyroscope range selection (DPS_250, DPS_500, DPS_1000, DPS_2000).
  *
  * @details
  * Updates FS_SEL bits in GYRO_CONFIG register. Afterwards, the gyro signal path
  * reset is triggered to ensure internal filtering state is refreshed.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_gyro_range(MPU_6050_t *handles, MPU_6050_gyro_range_t range);

/**
  * @brief  Set accelerometer full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Accelerometer range selection (G_2, G_4, G_8, G_16).
  *
  * @details
  * Updates AFS_SEL bits in ACCEL_CONFIG register. Afterwards, the accel signal path
  * reset is triggered to ensure internal filtering state is refreshed.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_accel_range(MPU_6050_t *handles, MPU_6050_accel_range_t range);

/**
  * @brief  Start single DMA read of sensor measurement payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This function is to be used in single read mode. It theoreticaly can be used in burst read mode,
  * but currently there is no mechanism allowing to set a separate payload for single read and burst read
  * 
  * @attention This function can (and should) be called from isr context 
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_single_read(MPU_6050_t *handles);

/**
  * @brief  Start single DMA read of accelerometer data in low power cycle mode.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details 
  * This function is dedicated to use in low power cycle mode
  * 
  * @attention This function can (and should) be called from isr context 
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_low_power_read(MPU_6050_t *handles);

/**
  * @brief  Extracts current FIFO buffer count.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Reads FIFO_COUNT_H and FIFO_COUNT_L registers (16-bit) to determine the number of
  * bytes currently stored in the FIFO buffer.
  * 
  * This function is to be used only in burst read mode. Typically it should be triggered by a timer
  * and called frequent enough to always catch fifo count reaching limit set in BURST_COUNT before 
  * FIFO overflows
  * 
  * @attention This function can (and should) be called from isr context 
  *
  * @note According to the datasheet, reading FIFO_COUNT_H latches the value of both
  * FIFO count registers. Therefore the count must be read in high-then-low order
  * to obtain a coherent snapshot.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_read_fifo_cnt(MPU_6050_t *handles);

/**
  * @brief  Converts raw fifo_count to uint16_t fifo_count
  * @param  handles Pointer to MPU6050 handle structure.
  * 
  * @retval None
  */
void MPU_6050_process_burst_cnt(MPU_6050_t *handles);

/**
  * @brief  Start burst DMA read from FIFO.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This function triggers a burst read of MPU6050 internal FIFO register
  * It reads exactly as many bytes as specified in BURST_COUNT
  * 
  * @attention This function can (and should) be called from isr context 
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_burst_read(MPU_6050_t *handles);

/**
  * @brief  Converts raw uint8_t payload into int16_t payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Parses the content of handles->rx_buffer according to the currently selected
  * measurement mode and stores converted signed 16-bit values in the buffer
  * pointed to by handles->inter_buffer.
  *
  * In MPU_SINGLE_MODE, the function parses one full measurement frame:
  * accelerometer XYZ, temperature and gyroscope XYZ.
  *
  * In MPU_LOWPOWER_CYCLE_MODE, the function parses one reduced measurement frame
  * containing only accelerometer XYZ values.
  *
  * In MPU_BURST_MODE, the function parses the entire FIFO payload currently stored
  * in handles->rx_buffer. The layout of each frame depends on the currently active
  * measurement sources and handles->payload_bytes value.
  *
  * @note
  * In MPU_SINGLE_MODE and MPU_LOWPOWER_CYCLE_MODE, the buffer pointed to by
  * handles->inter_buffer should provide space for exactly one MPU_6050_rawdata_t
  * sample.
  *
  * In MPU_BURST_MODE, the whole received payload is parsed at once, therefore
  * handles->inter_buffer should provide enough space for
  * (handles->burst_count / handles->payload_bytes) samples.
  *
  * @retval None.
  */
void MPU_6050_parse_payload(MPU_6050_t *handles);

/**
  * @brief  Convert parsed raw data to scaled physical units.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  payload Pointer to converted raw measurement array (7 x int16_t).
  *
  * @details
  * Uses currently selected ranges to scale:
  *  - accelerometer to g,
  *  - gyroscope to deg/s,
  * and converts temperature to degrees Celsius.
  *
  * Note: scaling depends on internal range state used by this driver.
  *
  * @retval MPU6050_data_t structure with scaled values.
  */
MPU_6050_data_t MPU_6050_payload_to_readable(MPU_6050_t *handles);

/**
  * @brief  Template for handling INT pin ISR.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This is a reference to an example isr implementation with __attribute__(weak).
  * User can use it, overwrite it or ignore it and provide own implementation
  * 
  * @retval HAL status
  */
HAL_StatusTypeDef MPU_6050_int_isr(MPU_6050_t *handles);

/**
  * @brief  Template for handling I2C rxcplt ISR.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This is a reference to an example isr implementation with __attribute__(weak).
  * User can use it, overwrite it or ignore it and provide own implementation
  * 
  * @retval HAL status
  */
HAL_StatusTypeDef MPU_6050_i2c_rxcplt_isr(MPU_6050_t *handles);

#ifdef __cplusplus
}
#endif

#endif
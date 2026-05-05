# MPU6050 STM32 Library (HAL / DMA)

Lightweight C driver for the **InvenSense MPU6050** written for **STM32** microcontrollers using **STM32 HAL**.

Author: **papaj-23**

---

## Overview

This library provides a light and practical MPU6050 driver designed for embedded workflows:

- HAL-based register access
- DMA-based data acquisition
- Flexible integration (bare-metal / RTOS)

---

## Quick Start

```c
MPU_6050_t imu = {
    .hi2c = &hi2c1,
    .rx_buffer = dma_rx_buf,
    .inter_buffer = raw_data_buf,
    .delay_ms_wrapper = delay_ms
};

MPU_6050_init(&imu);
MPU_6050_set_mode(&imu, MPU_SINGLE_MODE);
MPU_6050_set_accel_range(&imu, G_2);
MPU_6050_set_gyro_range(&imu, DPS_250);

/* inside ISR */
MPU_6050_single_read(&imu);

/* inside processing task */
MPU_6050_parse_payload(&imu);
MPU_6050_data_t data = MPU_6050_payload_to_readable(&imu);
```

---

## Features

- HAL I2C support (`HAL_I2C_Mem_Read/Write`)
- Init sequence
- Built-in self-test procedure
- DMA reads:
  - Data registers
  - FIFO count
  - INT status
- Operating modes:
  - `MPU_SINGLE_MODE`
  - `MPU_BURST_MODE`
  - `MPU_LOWPOWER_CYCLE_MODE`
- Per-channel enable/disable
- FIFO content selection
- FIFO reset helper
- Gyro/Accel range configuration
- Wakeup frequency configuration
- Parsing pipeline:
  - raw → int16 → physical units
- Optional ISR helpers

---

## Requirements

- Project that uses STM32 HAL
- Configured I2C peripheral
- Configured DMA for I2C
- Configured external GPIO for INT pin
- Delay function, preferably non blocking one

---

## Files

- `mpu-6050.h` - public API
- `mpu-6050.c` - implementation

---

## MPU_6050 Handle Description

- `I2C_HandleTypeDef *hi2c` - pointer to i2c instance configured for MPU6050
- `uint8_t *rx_buffer` - pointer to dma rx buffer, used to store measurements only
- `MPU_6050_rawdata_t *inter_buffer` - pointer to intermediate buffer used for data parsing
- `void (*delay_ms_wrapper)(uint32_t)` - hook to delay function, that takes milliseconds as argument.  
**Providing a delay function is necessary for some functions to work properly !**
- `MPU_6050_mode_t meas_mode` - current measurement mode. Not to be written manually. Can be read if needed.
- `MPU_6050_bus_access_t bus_status` - i2c bus lock. Can be modified manually if needed. It blocks data operations while locked though.
- `uint16_t burst_count` - burst read threshold. Can be set once through BURST_COUNT macro during initialization and modified later if needed.
- `volatile uint16_t fifo_count` - amount of bytes in fifo buffer. Can be used manually in user's ISR implementation.
- `volatile uint8_t fifo_count_raw[2]` - destination buffer for dma read of FIFO_COUNT H/L registers. Not to be used manually.
- `volatile uint8_t int_status` - raw value of INT_STATUS register. Can be used manually in user's ISR implementation.
- `uint8_t payload_bytes` - amount of bytes that current payload consists of. Managed internally by library.
- `uint8_t gyro_scale` - current gyroscope scale. Not to be written manually. Can be read if needed.
- `uint8_t accel_scale` - current accelerometer scale. Not to be written manually. Can be read if needed.
- `volatile uint8_t fifo_oflow_flag` - FIFO overflow flag used only by template ISR due to polling nature of fifo reset function.
- `active_sources` - struct that stores current states (enabled/disabled) of all sensor channels.

Minimal required initialization:
```c
MPU_6050_t imu = {
    .hi2c = &hi2cx,
    .rx_buffer = &dma_rx_buf,
    .inter_buffer = &raw_data_buf,
    .delay_ms_wrapper = delay_ms_function
};
```

---

## Interface Macros

- `I2C_ADDRESS_7B` - I2C address of MPU6050. Can be either 0x68 or 0x69 depending on AD0 external pullup or pulldown.
- `BURST_COUNT` - burst read threshold. Has to be set between 0U and 1024U. Should be divisible by the amount of bytes written to FIFO within single sample to avoid data frame fracturing
- `SMPLTR_DIV_VAL` - SMPLTR_DIV register value that determines sample rate according to equation below:  
`Sample Rate = GyroRate / (1 + SMPLRT_DIV)`   
    - GyroRate = 1 kHz (DLPF enabled)
    - GyroRate = 8 kHz (DLPF disabled)
- `CONFIG_VAL` - value of 3 LSB in CONFIG register that determine Digital Low-Pass Filter parameters
- `MPU_USE_DEBUG_REGISTERS` - enable/disable register value check mechanism

---

## Public API Functions

### Initialization & Configuration
- `MPU_6050_init()` - Initialize MPU6050 driver and hardware interface
- `MPU_6050_set_mode()` - Configure operating mode (single, burst, low-power cycle)
- `MPU_6050_set_sleep()` - Enable/disable sleep mode
- `MPU_6050_set_lp_wakeup_freq()` - Configure low-power wake-up frequency
- `MPU_6050_set_gyro_range()` - Set gyroscope full-scale range
- `MPU_6050_set_accel_range()` - Set accelerometer full-scale range

### Measurement Control
- `MPU_6050_set_source()` - Enable/disable measurement channels
- `MPU_6050_set_fifo_content()` - Configure FIFO content manually

### Data Acquisition
- `MPU_6050_single_read()` - Start single DMA read (single mode)
- `MPU_6050_low_power_read()` - Start DMA read (low-power cycle mode)
- `MPU_6050_read_fifo_cnt()` - Read current FIFO byte count
- `MPU_6050_burst_read()` - Start burst DMA read from FIFO

### Data Processing
- `MPU_6050_parse_payload()` - Convert raw byte payload to int16_t values
- `MPU_6050_payload_to_readable()` - Convert raw data to physical units (g, deg/s, °C)
- `MPU_6050_process_burst_cnt()` - Convert raw fifo_count to uint16_t

### Maintenance & Diagnostics
- `MPU_6050_fifo_reset()` - Reset FIFO buffer and resume operation
- `MPU_6050_self_test()` - Perform built-in self-test procedure

### Interrupt Handling (Templates)
- `MPU_6050_int_isr()` - INT pin interrupt handler template
- `MPU_6050_i2c_rxcplt_isr()` - I2C RX complete interrupt handler template

### **For more detailed description go to comments in header file of the library**

---

## Mode Workflows

### Single Read Mode

Typical measurement flow:

1. Wait for interrupt (INT pin)
2. Read INT status register
3. Verify `DATA_READY` interrupt source
4. Call `MPU_6050_single_read()`
5. Wait for DMA transfer completion
6. Parse and process received data
7. Store or forward results
8. Repeat for next interrupt

---

### Burst Read Mode (FIFO)

Typical measurement flow:

1. Periodically read FIFO count using `MPU_6050_read_fifo_cnt()`
2. Check if FIFO count ≥ configured `burst_count`
3. If condition is met:
   - Call `MPU_6050_burst_read()`
   - Wait for DMA transfer completion
   - Parse entire FIFO payload
4. Process and store extracted samples
5. Repeat periodically

**Notes:**
- If FIFO overflow interrupt occurs, it is recommended to call `MPU_6050_fifo_reset()`
- DMA buffer and intermediate buffer must be large enough to hold full burst payload
- Sampling frequency and polling rate should be aligned to avoid overflow

---

### Low Power Cycle Mode

Typical measurement flow:

1. Wait for interrupt (INT pin)
2. Read INT status register
3. Verify `DATA_READY` interrupt source
4. Call `MPU_6050_low_power_read()`
5. Wait for DMA transfer completion
6. Parse and process received data (accelerometer only)
7. Store or forward results
8. Repeat for next interrupt

---

## Build Integration (CMake)

The driver is provided as a static library:

```cmake
add_library(mpu6050 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu-6050.c
)

target_include_directories(mpu6050
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mpu6050
    PUBLIC
        hal
    PRIVATE
        m
)
```

It depends on math standard library (m) and STM32 HAL ('hal' or any other name provided by user)
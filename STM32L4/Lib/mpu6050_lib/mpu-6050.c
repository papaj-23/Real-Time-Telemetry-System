#include "mpu-6050.h"
#include <math.h>

#define I2C_ADDRESS_HAL     (I2C_ADDRESS_7B << 1)
#define MPU6050_REG_SIZE    I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT         10U     /*  ms  */
#define FULL_PAYLOAD_SIZE   14U
#define LP_PAYLOAD_SIZE     6U
#define SELFTEST_PAYLOAD    12U
#define SELFTEST_SAMPLE_AMOUNT 10
/*  Sensor register addresses  */

/*  Registers  */
#define SELF_TEST_X         0x0DU
#define SELF_TEST_Y         0x0EU
#define SELF_TEST_Z         0x0FU
#define SELF_TEST_A         0x10U

#define SMPLRT_DIV_REG      0x19U
#define CONFIG_REG          0x1AU
#define GYRO_CONFIG_REG     0x1BU
#define ACCEL_CONFIG_REG    0x1CU
#define FIFO_EN_REG         0x23U
#define INT_PIN_CFG_REG     0x37U
#define INT_ENABLE_REG      0x38U
#define INT_STATUS_REG      0x3AU
#define SIGNAL_PATH_RESET   0x68U
#define USER_CTRL_REG       0x6AU

#define ACCEL_XOUT_H        0x3BU
#define ACCEL_XOUT_L        0x3CU
#define ACCEL_YOUT_H        0x3DU
#define ACCEL_YOUT_L        0x3EU
#define ACCEL_ZOUT_H        0x3FU
#define ACCEL_ZOUT_L        0x40U

#define TEMP_OUT_H          0x41U
#define TEMP_OUT_L          0x42U

#define GYRO_XOUT_H         0x43U
#define GYRO_XOUT_L         0x44U
#define GYRO_YOUT_H         0x45U
#define GYRO_YOUT_L         0x46U
#define GYRO_ZOUT_H         0x47U
#define GYRO_ZOUT_L         0x48U

#define PWR_MGMT_1          0x6BU
#define PWR_MGMT_2          0x6CU
#define FIFO_COUNT_H        0X72U
#define FIFO_COUNT_L        0X73U
#define FIFO_R_W            0x74U
#define WHO_AM_I            0x75U

/* default register values  */

#define FIFO_EN_VAL_DEFAULT         0x00U   /* 0000 0000 */
#define INT_PIN_CFG_VAL_DEFAULT     0x00U   /* 0010 0000 */
#define INT_ENABLE_VAL_DEFAULT      0x01U   /* 0000 0001 */
#define PWR_MGMT_1_VAL_DEFAULT      0x00U   /* 0000 0000 */     

/* register masks */

#define GYRO_RANGE_POS      3U
#define GYRO_RANGE          ((uint8_t)(3U << GYRO_RANGE_POS))

#define ACCEL_RANGE_POS     3U
#define ACCEL_RANGE         ((uint8_t)(3U << ACCEL_RANGE_POS))   

#define DATA_READY_INT_POS  0U
#define DATA_READY_INT      ((uint8_t)(1U << DATA_READY_INT_POS))

#define FIFO_OFLOW_INT_POS  4U
#define FIFO_OFLOW_INT      ((uint8_t)(1U << FIFO_OFLOW_INT_POS))

#define FIFO_ENABLE_POS     6U
#define FIFO_ENABLE         ((uint8_t)(1U << FIFO_ENABLE_POS))

#define FIFO_RESET_POS      2U
#define FIFO_RESET          ((uint8_t)(1U << FIFO_RESET_POS))

#define LP_WAKE_CTRL_POS    6U
#define LP_WAKE_CTRL        ((uint8_t)(3U << LP_WAKE_CTRL_POS))

#define ACCEL_X_STANDBY_POS 5U
#define ACCEL_X_STANDBY     ((uint8_t)(1U << ACCEL_X_STANDBY_POS))

#define ACCEL_Y_STANDBY_POS 4U
#define ACCEL_Y_STANDBY     ((uint8_t)(1U << ACCEL_Y_STANDBY_POS))

#define ACCEL_Z_STANDBY_POS 3U
#define ACCEL_Z_STANDBY     ((uint8_t)(1U << ACCEL_Z_STANDBY_POS))

#define GYRO_X_STANDBY_POS  2U
#define GYRO_X_STANDBY      ((uint8_t)(1U << GYRO_X_STANDBY_POS))

#define GYRO_Y_STANDBY_POS  1U
#define GYRO_Y_STANDBY      ((uint8_t)(1U << GYRO_Y_STANDBY_POS))

#define GYRO_Z_STANDBY_POS  0U
#define GYRO_Z_STANDBY      ((uint8_t)(1U << GYRO_Z_STANDBY_POS))

#define TEMP_DIS_POS        3U
#define TEMP_DIS            ((uint8_t)(1U << TEMP_DIS_POS))

#define CYCLE_MODE_POS      5U
#define CYCLE_MODE          ((uint8_t)(1U << CYCLE_MODE_POS))

#define SLEEP_MODE_POS      6U
#define SLEEP_MODE          ((uint8_t)(1U << SLEEP_MODE_POS))

#define DEVICE_RESET_POS    7U
#define DEVICE_RESET        ((uint8_t)(1U << DEVICE_RESET_POS))


#define STATUS_CHECK(status)  do{                                   \
                            if((status) != HAL_OK)                  \
                            {                                       \
                                goto exit;                          \
                            }                                       \
                        }while (0)    
            
#define MEM_CHECK(memory)  do{                                      \
                            if((memory) == NULL)                    \
                            {                                       \
                                return HAL_ERROR;                   \
                            }                                       \
                        }while (0)

/* local strucutres definitions */

typedef struct {
    uint8_t addr;
    uint8_t val;
} reg_t;

/* static functions declarations */

static void set_all_src_active(MPU_6050_t *handles);
static uint8_t get_source_state(MPU_6050_t *handles, MPU_6050_meas_channel_t ch);
static HAL_StatusTypeDef set_fifo_content(MPU_6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state);
static void mpu_delay(MPU_6050_t *handles, uint32_t ms);
static MPU_6050_selftest_t calculate_ft(const uint8_t gyro[3], const uint8_t accel[3]);
static void parse_payload_selftest(const uint8_t raw[12], int16_t *inter);
static inline float selftest_ratio(int16_t diff, float ft);
static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb);
static HAL_StatusTypeDef gyro_path_reset(MPU_6050_t *handles);
static HAL_StatusTypeDef accel_path_reset(MPU_6050_t *handles);
static void fifo_parser(MPU_6050_t *handles);
static void lock_bus(MPU_6050_t *handles);
static void unlock_bus(MPU_6050_t *handles);
static HAL_StatusTypeDef bitset_helper(MPU_6050_t *handles, uint8_t reg_address, uint8_t mask, MPU_6050_state_t state);

/* const lookup tables */

static const reg_t init_registers[] = {
    { PWR_MGMT_1,        PWR_MGMT_1_VAL_DEFAULT},
    { SMPLRT_DIV_REG,    SMPLTR_DIV_VAL },
    { CONFIG_REG,        CONFIG_VAL },
    { FIFO_EN_REG,       FIFO_EN_VAL_DEFAULT },
    { INT_ENABLE_REG,    INT_ENABLE_VAL_DEFAULT },
    { INT_PIN_CFG_REG,   INT_PIN_CFG_VAL_DEFAULT}
};


#if (MPU_USE_DEBUG_REGISTERS == 1)
    typedef struct {
        const char *name;
        uint8_t addr;
    } mpu6050_reg_check_t;

    static const mpu6050_reg_check_t mpu6050_registers[] = {
        {"SELF_TEST_X",     SELF_TEST_X},
        {"SELF_TEST_Y",     SELF_TEST_Y},
        {"SELF_TEST_Z",     SELF_TEST_Z},
        {"SELF_TEST_A",     SELF_TEST_A},

        {"SMPLRT_DIV",      SMPLRT_DIV_REG},
        {"CONFIG",          CONFIG_REG},
        {"GYRO_CONFIG",     GYRO_CONFIG_REG},
        {"ACCEL_CONFIG",    ACCEL_CONFIG_REG},

        {"FIFO_EN",         FIFO_EN_REG},

        {"INT_PIN_CFG",     INT_PIN_CFG_REG},
        {"INT_ENABLE",      INT_ENABLE_REG},
        {"INT_STATUS",      INT_STATUS_REG},

        {"ACCEL_XOUT_H",    ACCEL_XOUT_H},
        {"ACCEL_XOUT_L",    ACCEL_XOUT_L},
        {"ACCEL_YOUT_H",    ACCEL_YOUT_H},
        {"ACCEL_YOUT_L",    ACCEL_YOUT_L},
        {"ACCEL_ZOUT_H",    ACCEL_ZOUT_H},
        {"ACCEL_ZOUT_L",    ACCEL_ZOUT_L},

        {"TEMP_OUT_H",      TEMP_OUT_H},
        {"TEMP_OUT_L",      TEMP_OUT_L},

        {"GYRO_XOUT_H",     GYRO_XOUT_H},
        {"GYRO_XOUT_L",     GYRO_XOUT_L},
        {"GYRO_YOUT_H",     GYRO_YOUT_H},
        {"GYRO_YOUT_L",     GYRO_YOUT_L},
        {"GYRO_ZOUT_H",     GYRO_ZOUT_H},
        {"GYRO_ZOUT_L",     GYRO_ZOUT_L},

        {"SIGNAL_PATH_RESET", SIGNAL_PATH_RESET},
        {"USER_CTRL",       USER_CTRL_REG},

        {"PWR_MGMT_1",      PWR_MGMT_1},
        {"PWR_MGMT_2",      PWR_MGMT_2},

        {"FIFO_COUNT_H",    FIFO_COUNT_H},
        {"FIFO_COUNT_L",    FIFO_COUNT_L},

        {"WHO_AM_I",        WHO_AM_I}
    };

    #define CHECK_REGISTER_CNT (sizeof(mpu6050_registers)/sizeof(mpu6050_registers[0]))
    mpu6050_reg_check_t rx_reg_values[CHECK_REGISTER_CNT];


    /**
     * @brief  Read all current register values.
     * @param  handles Pointer to MPU6050 handle structure.
     * @param  values List to store register values
     * 
     * @retval HAL status.
     */
    HAL_StatusTypeDef MPU_6050_check_registers(MPU_6050_t *handles) {
        MEM_CHECK(handles);
        HAL_StatusTypeDef status = HAL_OK;
        for(unsigned int i = 0; i < CHECK_REGISTER_CNT; i++) {
            uint8_t value;
            status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                                mpu6050_registers[i].addr, MPU6050_REG_SIZE,
                                &value, 1, I2C_TIMEOUT);
            STATUS_CHECK(status);
            rx_reg_values[i].name = mpu6050_registers[i].name;
            rx_reg_values[i].addr = value;
        }

        exit:
        return status;
    }
#endif


/**
  * @brief  Initialize MPU6050 sensor with default configuration (detailed description in header).
  * @param  handles Pointer to MPU6050 handle structure.
  * 
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_init(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    lock_bus(handles);

    /* perform reset */
    status = bitset_helper(handles, PWR_MGMT_1, DEVICE_RESET, MPU_ENABLE);
    STATUS_CHECK(status);
    mpu_delay(handles, 100);

    for(size_t i = 0; i < sizeof(init_registers)/sizeof(init_registers[0]); i++) {
        uint8_t v = init_registers[i].val;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                  init_registers[i].addr, MPU6050_REG_SIZE,
                                  &v, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);
    }
    
    handles->meas_mode = MPU_SINGLE_MODE;
    handles->burst_count = BURST_COUNT;
    handles->fifo_count = 0U;
    handles->int_status = 0U;
    handles->gyro_scale = DPS_250;
    handles->accel_scale = G_2;
    handles->fifo_oflow_flag = 0U;
    set_all_src_active(handles);

    mpu_delay(handles, 10);

    exit:
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Configure MPU6050 power/streaming mode (single sample, FIFO burst, or low-power cycle).
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  mode    Selected mode: MPU_SINGLE_MODE, MPU_BURST_MODE, or MPU_LOWPOWER_CYCLE_MODE.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_mode(MPU_6050_t *handles, MPU_6050_mode_t mode) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;
    lock_bus(handles);

    switch (mode) {
    case MPU_SINGLE_MODE:
        /* INT: Data Ready */
        reg = DATA_READY_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    INT_ENABLE_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO OFF */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* ALL FIFO SOURCES OFF */
        reg = 0U;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    FIFO_EN_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* Reset FIFO to prevent FIFO_OFLOW_INT bit setting every sample if FIFO is full */
        status = MPU_6050_fifo_reset(handles);
        STATUS_CHECK(status);

        /* Enable temp + gyro axes */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                                MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable accel axes */
        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                                MPU_DISABLE);
        STATUS_CHECK(status);

        /* Normal power (no cycle, no sleep) */
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);

        set_all_src_active(handles);
        handles->meas_mode = MPU_SINGLE_MODE;
    break;

    case MPU_BURST_MODE:
        /* INT: FIFO overflow */
        reg = FIFO_OFLOW_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    INT_ENABLE_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO ON */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_ENABLE);
        STATUS_CHECK(status);

        /* ALL FIFO SOURCES ON */
        reg = 248U;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    FIFO_EN_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* Enable temp + gyro axes */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                                MPU_DISABLE);
        STATUS_CHECK(status);

        /* Enable accel axes */
        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                                MPU_DISABLE);
        STATUS_CHECK(status);

        /* Normal power (no cycle, no sleep) */
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);

        set_all_src_active(handles);
        handles->meas_mode = MPU_BURST_MODE;
    break;

    case MPU_LOWPOWER_CYCLE_MODE:
        /* INT: Data Ready */
        reg = DATA_READY_INT;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    INT_ENABLE_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* FIFO OFF (cycle mode is accel-only low power; FIFO streaming doesn't make sense here) */
        status = bitset_helper(handles, USER_CTRL_REG, FIFO_ENABLE, MPU_DISABLE);
        STATUS_CHECK(status);

        /* ALL FIFO SOURCES OFF */
        reg = 0U;
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                    FIFO_EN_REG, MPU6050_REG_SIZE,
                                    &reg, 1, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* Reset FIFO to prevent FIFO_OFLOW_INT bit setting every sample if FIFO is full */
        status = MPU_6050_fifo_reset(handles);
        STATUS_CHECK(status);

        /* Required for accel-only low power cycle:
            - TEMP disabled
            - Gyro in standby
            - CYCLE enabled
            - SLEEP disabled */
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, MPU_ENABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                                MPU_ENABLE);
        STATUS_CHECK(status);

        /* Ensure accel enabled (standby bits cleared) */
        status = bitset_helper(handles, PWR_MGMT_2,
                                (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY),
                                MPU_DISABLE);
        STATUS_CHECK(status);

        status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, MPU_DISABLE);
        STATUS_CHECK(status);
        status = bitset_helper(handles, PWR_MGMT_1, CYCLE_MODE, MPU_ENABLE);
        STATUS_CHECK(status);

        handles->active_sources.acc_x = MPU_ENABLE;
        handles->active_sources.acc_y = MPU_ENABLE;
        handles->active_sources.acc_z = MPU_ENABLE;
        handles->active_sources.temp = MPU_DISABLE;
        handles->active_sources.gyro_x = MPU_DISABLE;
        handles->active_sources.gyro_y = MPU_DISABLE;
        handles->active_sources.gyro_z = MPU_DISABLE;
        handles->payload_bytes = LP_PAYLOAD_SIZE;
        handles->meas_mode = MPU_LOWPOWER_CYCLE_MODE;
    break;

    default:
        status = HAL_ERROR;
    }

    exit:
    unlock_bus(handles);

    return status;
}


static void set_all_src_active(MPU_6050_t *handles) {
    handles->active_sources.acc_x = MPU_ENABLE;
    handles->active_sources.acc_y = MPU_ENABLE;
    handles->active_sources.acc_z = MPU_ENABLE;
    handles->active_sources.temp = MPU_ENABLE;
    handles->active_sources.gyro_x = MPU_ENABLE;
    handles->active_sources.gyro_y = MPU_ENABLE;
    handles->active_sources.gyro_z = MPU_ENABLE;
    handles->payload_bytes = FULL_PAYLOAD_SIZE;
}


/**
  * @brief  Turn on/off Sleep Mode
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  state MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_sleep(MPU_6050_t *handles, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    lock_bus(handles);

    status = bitset_helper(handles, PWR_MGMT_1, SLEEP_MODE, state);
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Configure wake-up frequency for accelerometer low-power cycle mode.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  freq    Low-power wake-up frequency selection (MPU_6050_lp_freq_t).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_lp_wakeup_freq(MPU_6050_t *handles, MPU_6050_lp_freq_t freq) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;
    lock_bus(handles);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_2, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    reg &= ~(LP_WAKE_CTRL);
    reg |= (uint8_t)(freq << LP_WAKE_CTRL_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_2, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);

    exit:
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Enable or disable selected measurement channel gyro xyz|accel xyz|temp.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  ch      Measurement channel to configure (accel, gyro axis or temperature).
  * @param  state   MPU_ENABLE to enable, MPU_DISABLE to disable.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_source(MPU_6050_t *handles, MPU_6050_meas_channel_t ch, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    /* Cycle mode has fixed settings regarding measurement sources */
    if(handles->meas_mode == MPU_LOWPOWER_CYCLE_MODE) {
        return HAL_ERROR;
    }

    /* return if new state equals current state */
    if(get_source_state(handles, ch) == state) {
        return HAL_OK;
    }

    MPU_6050_state_t inv_state = (state == MPU_ENABLE) ? MPU_DISABLE : MPU_ENABLE;
    HAL_StatusTypeDef status = HAL_OK;
    lock_bus(handles);

    switch (ch) {
    case ACCEL_X_CH:
        status = bitset_helper(handles, PWR_MGMT_2, ACCEL_X_STANDBY, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.acc_x = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            if(!(handles->active_sources.acc_y || handles->active_sources.acc_z)) {
                handles->payload_bytes += state ? 6 : -6;
                status = set_fifo_content(handles, FIFO_ACCEL, state);
            }
        }
    break;

    case ACCEL_Y_CH:
        status = bitset_helper(handles, PWR_MGMT_2, ACCEL_Y_STANDBY, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.acc_y = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            if(!(handles->active_sources.acc_x || handles->active_sources.acc_z)) {
                handles->payload_bytes += state ? 6 : -6;
                status = set_fifo_content(handles, FIFO_ACCEL, state);
            }
        }
    break;

    case ACCEL_Z_CH:
    status = bitset_helper(handles, PWR_MGMT_2, ACCEL_Z_STANDBY, inv_state);
    STATUS_CHECK(status);
    handles->active_sources.acc_z = state;
    if(handles->meas_mode == MPU_BURST_MODE) {
        if(!(handles->active_sources.acc_y || handles->active_sources.acc_x)) {
            handles->payload_bytes += state ? 6 : -6;
            status = set_fifo_content(handles, FIFO_ACCEL, state);
        }
    }
    break;

    case GYRO_X_CH:
        status = bitset_helper(handles, PWR_MGMT_2, GYRO_X_STANDBY, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.gyro_x = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            handles->payload_bytes += state ? 2 : -2;
            status = set_fifo_content(handles, FIFO_GYRO_X, state);
        }
    break;

    case GYRO_Y_CH:
        status = bitset_helper(handles, PWR_MGMT_2, GYRO_Y_STANDBY, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.gyro_y = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            handles->payload_bytes += state ? 2 : -2;
            status = set_fifo_content(handles, FIFO_GYRO_Y, state);
        }
    break;

    case GYRO_Z_CH:
        status = bitset_helper(handles, PWR_MGMT_2, GYRO_Z_STANDBY, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.gyro_z = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            handles->payload_bytes += state ? 2 : -2;
            status = set_fifo_content(handles, FIFO_GYRO_Z, state);
        }
    break;

    case TEMP_CH:
        status = bitset_helper(handles, PWR_MGMT_1, TEMP_DIS, inv_state);
        STATUS_CHECK(status);
        handles->active_sources.temp = state;
        if(handles->meas_mode == MPU_BURST_MODE) {
            handles->payload_bytes += state ? 2 : -2;
            status = set_fifo_content(handles, FIFO_TEMP, state);
        }
    break;

    default:
        status = HAL_ERROR;
    }

    exit:
    unlock_bus(handles);

    return status;
}


static uint8_t get_source_state(MPU_6050_t *handles, MPU_6050_meas_channel_t ch) {
    switch(ch) {
        case ACCEL_X_CH: return handles->active_sources.acc_x;
        case ACCEL_Y_CH: return handles->active_sources.acc_y;
        case ACCEL_Z_CH: return handles->active_sources.acc_z;
        case TEMP_CH:    return handles->active_sources.temp;
        case GYRO_X_CH:  return handles->active_sources.gyro_x;
        case GYRO_Y_CH:  return handles->active_sources.gyro_y;
        case GYRO_Z_CH:  return handles->active_sources.gyro_z;
        default:         return 0xFF;;
    }
}


static HAL_StatusTypeDef set_fifo_content(MPU_6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;

    status = bitset_helper(handles, FIFO_EN_REG, (uint8_t)content, state);

    return status;
}


/**
  * @brief  Enable or disable selected sensor data in FIFO buffer.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  content FIFO content mask (combination of MPU_6050_fifo_content_t values).
  * @param  state   MPU_ENABLE to set bits, MPU_DISABLE to clear bits.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_fifo_content(MPU_6050_t *handles, MPU_6050_fifo_content_t content, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    lock_bus(handles);

    status = bitset_helper(handles, FIFO_EN_REG, (uint8_t)content, state);
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Reset MPU6050 FIFO buffer and re-enable FIFO operation.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_fifo_reset(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;
    lock_bus(handles);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              USER_CTRL_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(FIFO_ENABLE);
    reg |= FIFO_RESET;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               USER_CTRL_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    mpu_delay(handles, 2);

    reg |= FIFO_ENABLE;
    reg &= ~(FIFO_RESET);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               USER_CTRL_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);

    exit:
    handles->fifo_oflow_flag = 0U;
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Perform built-in self-test procedure of MPU6050.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  result  Pointer to structure storing self-test results (in %).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_self_test(MPU_6050_t *handles, MPU_6050_selftest_t *result) {
    MEM_CHECK(handles);
    MEM_CHECK(result);
    HAL_StatusTypeDef status = HAL_OK;
    lock_bus(handles);

    uint8_t power_mgmt1_original;
    uint8_t power_mgmt2_original;

    /* turn off cycle/sleep mode*/
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_1, MPU6050_REG_SIZE,
                              &power_mgmt1_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);                       
    status = bitset_helper(handles, PWR_MGMT_1,
                           (uint8_t)(SLEEP_MODE | CYCLE_MODE), MPU_DISABLE);
    STATUS_CHECK(status);

    /* turn on all accel and gyro axis if any were disabled before */
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              PWR_MGMT_2, MPU6050_REG_SIZE,
                              &power_mgmt2_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);                       
    status = bitset_helper(handles, PWR_MGMT_2,
                           (uint8_t)(ACCEL_X_STANDBY | ACCEL_Y_STANDBY | ACCEL_Z_STANDBY | GYRO_X_STANDBY | GYRO_Y_STANDBY | GYRO_Z_STANDBY),
                           MPU_DISABLE);
    STATUS_CHECK(status);

    /* store original measurement range */
    enum {GYRO = 0, ACCEL = 1};
    uint8_t original_config[2];
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                              original_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    /* set gyroscope range to +-250dps and accelerometer range to +-8g for selftest */
    uint8_t test_config[2] = {0U, (uint8_t)(1U << 4)};

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               test_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);
    mpu_delay(handles, 50);

    uint8_t test_raw[4];
    uint8_t gyro_test[3];
    uint8_t accel_test[3];
    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                            SELF_TEST_X, MPU6050_REG_SIZE,
                            test_raw, 4, I2C_TIMEOUT);
    STATUS_CHECK(status);

    enum{x = 0, y = 1, z = 2, a = 3};
    accel_test[x] = ((test_raw[x] & 0xE0U) >> 3) | ((test_raw[a] & 0x30U) >> 4);
    accel_test[y] = ((test_raw[y] & 0xE0U) >> 3) | ((test_raw[a] & 0x0CU) >> 2);
    accel_test[z] = ((test_raw[z] & 0xE0U) >> 3) | (test_raw[a] & 0x03U);
    gyro_test[x] = test_raw[x] & 0x1FU;
    gyro_test[y] = test_raw[y] & 0x1FU;
    gyro_test[z] = test_raw[z] & 0x1FU;

    MPU_6050_selftest_t ft = calculate_ft(gyro_test, accel_test);
    
    struct {
        int32_t accel_x;
        int32_t accel_y;
        int32_t accel_z;
        int32_t gyro_x;
        int32_t gyro_y;
        int32_t gyro_z;
    } diff = {0};

    for(unsigned int i = 0; i < SELFTEST_SAMPLE_AMOUNT; i++) {
        uint8_t test_dis_data_raw[SELFTEST_PAYLOAD];
        uint8_t test_en_data_raw[SELFTEST_PAYLOAD];
        int16_t test_dis_data_numerical[SELFTEST_PAYLOAD/2];
        int16_t test_en_data_numerical[SELFTEST_PAYLOAD/2];

        /* read measurements with selftest disabled */
        status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                                ACCEL_XOUT_H, MPU6050_REG_SIZE,
                                test_dis_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
        STATUS_CHECK(status);
        status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                                GYRO_XOUT_H, MPU6050_REG_SIZE,
                                test_dis_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
        STATUS_CHECK(status);

        test_config[GYRO] |= (uint8_t)(7U << 5);
        test_config[ACCEL] |= (uint8_t)(7U << 5);

        /* enable selftest */
        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                                test_config, 2, I2C_TIMEOUT);
        STATUS_CHECK(status);
        mpu_delay(handles, 100);

        /* read measurements with selftest enabled */
        status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                                ACCEL_XOUT_H, MPU6050_REG_SIZE,
                                test_en_data_raw, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
        STATUS_CHECK(status);
        status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                                GYRO_XOUT_H, MPU6050_REG_SIZE,
                                test_en_data_raw + SELFTEST_PAYLOAD/2, SELFTEST_PAYLOAD/2, I2C_TIMEOUT);
        STATUS_CHECK(status);

        /* disable selftest */
        test_config[GYRO] &= ~((uint8_t)(7U << 5));
        test_config[ACCEL] &= ~((uint8_t)(7U << 5));

        status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                                GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                                test_config, 2, I2C_TIMEOUT);
        STATUS_CHECK(status);
        mpu_delay(handles, 100);

        parse_payload_selftest(test_dis_data_raw, test_dis_data_numerical);
        parse_payload_selftest(test_en_data_raw, test_en_data_numerical);

        diff.accel_x += test_en_data_numerical[0] - test_dis_data_numerical[0];
        diff.accel_y += test_en_data_numerical[1] - test_dis_data_numerical[1];
        diff.accel_z += test_en_data_numerical[2] - test_dis_data_numerical[2];
        diff.gyro_x  += test_en_data_numerical[3] - test_dis_data_numerical[3];
        diff.gyro_y  += test_en_data_numerical[4] - test_dis_data_numerical[4];
        diff.gyro_z  += test_en_data_numerical[5] - test_dis_data_numerical[5];
    }

    diff.accel_x /= SELFTEST_SAMPLE_AMOUNT;
    diff.accel_y /= SELFTEST_SAMPLE_AMOUNT;
    diff.accel_z /= SELFTEST_SAMPLE_AMOUNT;
    diff.gyro_x  /= SELFTEST_SAMPLE_AMOUNT;
    diff.gyro_y  /= SELFTEST_SAMPLE_AMOUNT;
    diff.gyro_z  /= SELFTEST_SAMPLE_AMOUNT;

    result->accel_x = selftest_ratio(diff.accel_x, ft.accel_x);
    result->accel_y = selftest_ratio(diff.accel_y, ft.accel_y);
    result->accel_z = selftest_ratio(diff.accel_z, ft.accel_z);
    result->gyro_x  = selftest_ratio(diff.gyro_x,  ft.gyro_x);
    result->gyro_y  = selftest_ratio(diff.gyro_y,  ft.gyro_y);
    result->gyro_z  = selftest_ratio(diff.gyro_z,  ft.gyro_z);

    /* restore original measuement ranges */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               original_config, 2, I2C_TIMEOUT);
    STATUS_CHECK(status);

    /* restore previous PWR_MGMT registers */
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_1, MPU6050_REG_SIZE,
                               &power_mgmt1_original, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               PWR_MGMT_2, MPU6050_REG_SIZE,
                               &power_mgmt2_original, 1, I2C_TIMEOUT);

    exit:
    unlock_bus(handles);

    return status;
}


/**
  * @details
  * If a user-defined delay callback is provided in the handle structure,
  * it is executed with the requested delay time. If the callback pointer
  * is NULL, the function returns immediately without performing any delay.
  */
static void mpu_delay(MPU_6050_t *handles, uint32_t ms) {
    if(handles->delay_ms_wrapper == NULL) {
        return;
    }
    handles->delay_ms_wrapper(ms);
}


static MPU_6050_selftest_t calculate_ft(const uint8_t gyro[3], const uint8_t accel[3]) {
    float ft_raw[6];
    for(int i = 0; i < 3; i++) {
        int8_t c = (i%2) ? -1 : 1;
        if(accel[i] == 0) {
            ft_raw[i] = 0;
        }
        else {
            ft_raw[i] = 4096.0f*0.34f*powf(0.92f/0.34f, (accel[i]-1.0f)/30.0f);
        }

        if(gyro[i] == 0) {
            ft_raw[i+3] = 0;
        }
        else {
            ft_raw[i+3] = c*25.0f*131.0f*powf(1.046f, gyro[i]-1.0f);
        }
    }
    MPU_6050_selftest_t ft = {
        .accel_x = ft_raw[0],
        .accel_y = ft_raw[1],
        .accel_z = ft_raw[2],
        .gyro_x  = ft_raw[3],
        .gyro_y  = ft_raw[4],
        .gyro_z  = ft_raw[5]
    };
    return ft;
}


static void parse_payload_selftest(const uint8_t raw[12], int16_t *inter) {
    for(unsigned int i = 0; i < 6; i++){
        *inter++ = conv_to_i16(raw[2*i], raw[2*i+1]);
    }
}


static inline float selftest_ratio(int16_t diff, float ft) {
    return (ft == 0.0f) ? 0.0f : (((float)diff - ft) / ft)*100.0f /* result in % */;
}


/**
  * @brief  Set gyroscope full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Gyroscope range selection (DPS_250, DPS_500, DPS_1000, DPS_2000).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_gyro_range(MPU_6050_t *handles, MPU_6050_gyro_range_t range) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;
    lock_bus(handles);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(GYRO_RANGE);
    reg |= (uint8_t)(range << GYRO_RANGE_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               GYRO_CONFIG_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    handles->gyro_scale = range;
    status = gyro_path_reset(handles);

    exit:
    unlock_bus(handles);

    return status;
}


/**
  * @brief  Set accelerometer full-scale range.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  range   Accelerometer range selection (G_2, G_4, G_8, G_16).
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_set_accel_range(MPU_6050_t *handles, MPU_6050_accel_range_t range) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 0;
    lock_bus(handles);

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              ACCEL_CONFIG_REG, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    reg &= ~(ACCEL_RANGE);
    reg |= (uint8_t)(range << ACCEL_RANGE_POS);
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               ACCEL_CONFIG_REG, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    handles->accel_scale = range;
    status = accel_path_reset(handles);

    exit:
    unlock_bus(handles);

    return status;
}


static HAL_StatusTypeDef gyro_path_reset(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 4U;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               SIGNAL_PATH_RESET, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    mpu_delay(handles, 100);

    exit:
    return status;
}


static HAL_StatusTypeDef accel_path_reset(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg = 2U;
    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               SIGNAL_PATH_RESET, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    mpu_delay(handles, 100);

    exit:
    return status;
}


/**
  * @brief  Start single DMA read of sensor measurement payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_single_read(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }
    
    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  ACCEL_XOUT_H, MPU6050_REG_SIZE,
                                  handles->rx_buffer, FULL_PAYLOAD_SIZE);

    return status;
}


/**
  * @brief  Start single DMA read of sensor measurement payload.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_low_power_read(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }
    
    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  ACCEL_XOUT_H, MPU6050_REG_SIZE,
                                  handles->rx_buffer, LP_PAYLOAD_SIZE);

    return status;
}


/**
  * @brief  Extracts current FIFO buffer count.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_read_fifo_cnt(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }

    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  FIFO_COUNT_H, MPU6050_REG_SIZE,
                                  handles->fifo_count_raw, 2);

    return status;
}


/**
  * @brief  Converts raw fifo_count to uint16_t fifo_count
  * @param  handles Pointer to MPU6050 handle structure.
  * 
  * @retval None
  */
void MPU_6050_process_burst_cnt(MPU_6050_t *handles) {
    if(handles == NULL) {
        return;
    }
    handles->fifo_count = ((uint16_t)handles->fifo_count_raw[0] << 8) | (uint16_t)handles->fifo_count_raw[1];
}


/**
  * @brief  Start burst DMA read from FIFO.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status.
  */
HAL_StatusTypeDef MPU_6050_burst_read(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }    

    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  FIFO_R_W, MPU6050_REG_SIZE,
                                  handles->rx_buffer, handles->burst_count);

    return status;
}


static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb) {
    uint16_t u = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u;
}


/**
  * @brief  Convert raw 14-byte payload into 16-bit signed values.
  * @param  raw   Pointer to raw sensor payload (14 bytes).
  *
  * @retval None.
  */
void MPU_6050_parse_payload(MPU_6050_t *handles) {
    if(handles == NULL || handles->inter_buffer == NULL) {
        return;
    }

    switch(handles->meas_mode) {
    case MPU_SINGLE_MODE:
        handles->inter_buffer->accel_x = conv_to_i16(handles->rx_buffer[0],  handles->rx_buffer[1]);
        handles->inter_buffer->accel_y = conv_to_i16(handles->rx_buffer[2],  handles->rx_buffer[3]);
        handles->inter_buffer->accel_z = conv_to_i16(handles->rx_buffer[4],  handles->rx_buffer[5]);
        handles->inter_buffer->temp    = conv_to_i16(handles->rx_buffer[6],  handles->rx_buffer[7]);
        handles->inter_buffer->gyro_x  = conv_to_i16(handles->rx_buffer[8],  handles->rx_buffer[9]);
        handles->inter_buffer->gyro_y  = conv_to_i16(handles->rx_buffer[10], handles->rx_buffer[11]);
        handles->inter_buffer->gyro_z  = conv_to_i16(handles->rx_buffer[12], handles->rx_buffer[13]);
    break;

    case MPU_BURST_MODE:
        fifo_parser(handles);
    break;

    case MPU_LOWPOWER_CYCLE_MODE:
        handles->inter_buffer->accel_x = conv_to_i16(handles->rx_buffer[0],  handles->rx_buffer[1]);
        handles->inter_buffer->accel_y = conv_to_i16(handles->rx_buffer[2],  handles->rx_buffer[3]);
        handles->inter_buffer->accel_z = conv_to_i16(handles->rx_buffer[4],  handles->rx_buffer[5]);
    break;
    }
}

static void fifo_parser(MPU_6050_t *handles) {
    uint32_t offset = 0U;
    MPU_6050_rawdata_t *rawdata = handles->inter_buffer;

    for(unsigned int i = 1; i <= handles->burst_count / handles->payload_bytes; i++) {
        if(handles->active_sources.acc_x || handles->active_sources.acc_y || handles->active_sources.acc_z) {
            if(handles->active_sources.acc_x == MPU_ENABLE) {
                rawdata->accel_x = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            }
            offset += 2U;

            if(handles->active_sources.acc_y == MPU_ENABLE) {
                rawdata->accel_y = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            }
            offset += 2U;

            if(handles->active_sources.acc_z == MPU_ENABLE) {
                rawdata->accel_z = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            }
            offset += 2U;
        }

        if(handles->active_sources.temp == MPU_ENABLE) {
            rawdata->temp = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            offset += 2U;
        }

        if(handles->active_sources.gyro_x == MPU_ENABLE) {
            rawdata->gyro_x = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            offset += 2U;
        }

        if(handles->active_sources.gyro_y == MPU_ENABLE) {
            rawdata->gyro_y = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            offset += 2U;
        }

        if(handles->active_sources.gyro_z == MPU_ENABLE) {
            rawdata->gyro_z = conv_to_i16(handles->rx_buffer[offset], handles->rx_buffer[offset + 1]);
            offset += 2U;
        }

        if(i != handles->burst_count / handles->payload_bytes) {
            rawdata++;
        }
    }
}


/**
  * @brief  Convert parsed raw data to scaled physical units.
  * @param  handles Pointer to MPU6050 handle structure.
  * @param  payload Pointer to converted raw measurement array (7 x int16_t).
  *
  * @retval MPU6050_data_t structure with scaled values.
  */
MPU_6050_data_t MPU_6050_payload_to_readable(MPU_6050_t *handles) {
    MPU_6050_data_t readable = {0};
    if(handles == NULL || handles->inter_buffer == NULL) {
        return readable;
    }
    float accel_div = 16384.0f / (1U << handles->accel_scale);
    float gyro_div = 131.072f / (1U << handles->gyro_scale); 
    readable.accel_x = handles->inter_buffer->accel_x/accel_div;
    readable.accel_y = handles->inter_buffer->accel_y/accel_div;
    readable.accel_z = handles->inter_buffer->accel_z/accel_div;
    readable.temp = (handles->inter_buffer->temp/340.0f) + 35.0f;
    readable.gyro_x = handles->inter_buffer->gyro_x/gyro_div;
    readable.gyro_y = handles->inter_buffer->gyro_y/gyro_div;
    readable.gyro_z = handles->inter_buffer->gyro_z/gyro_div;

    return readable;
}


/**
  * @brief  Template for handling INT pin ISR.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status
  */
__attribute__((weak)) HAL_StatusTypeDef MPU_6050_int_isr(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }

    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  INT_STATUS_REG, MPU6050_REG_SIZE,
                                  &handles->int_status, 1);

    return status;
}


/**
  * @brief  Template for handling I2C rxcplt ISR.
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @retval HAL status
  */
__attribute__((weak)) HAL_StatusTypeDef MPU_6050_i2c_rxcplt_isr(MPU_6050_t *handles) {
    HAL_StatusTypeDef status = HAL_OK;

    switch(handles->meas_mode) {
    case MPU_SINGLE_MODE:
        if(handles->int_status & DATA_READY_INT) {
            handles->int_status &= ~DATA_READY_INT;
            status = MPU_6050_single_read(handles);
        }
    break;

    case MPU_BURST_MODE:
        MPU_6050_process_burst_cnt(handles);
        if(handles->int_status & FIFO_OFLOW_INT) {
            handles->int_status &= ~FIFO_OFLOW_INT;
            handles->fifo_oflow_flag = 1U;
        }
        else if(handles->fifo_count >= BURST_COUNT) {
            status = MPU_6050_burst_read(handles);
        }
    break;

    case MPU_LOWPOWER_CYCLE_MODE:
        if(handles->int_status & DATA_READY_INT) {
            handles->int_status &= ~DATA_READY_INT;
            status = MPU_6050_low_power_read(handles);
        }
    break;
    
    default:
        status = HAL_ERROR;
    break;
    }

    return status;
}


static void lock_bus(MPU_6050_t *handles) {
    handles->bus_status = MPU_BUS_LOCKED;
    uint32_t start_tick = HAL_GetTick();
    uint32_t current_tick;
    while(HAL_I2C_GetState(handles->hi2c) != HAL_I2C_STATE_READY) {
        current_tick = HAL_GetTick();
        if(current_tick - start_tick > I2C_TIMEOUT) {
            break;
        }
    }
}


static void unlock_bus(MPU_6050_t *handles) {
    handles->bus_status = MPU_BUS_UNLOCKED;
}


static HAL_StatusTypeDef bitset_helper(MPU_6050_t *handles, uint8_t reg_address, uint8_t mask, MPU_6050_state_t state) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reg;

    status = HAL_I2C_Mem_Read(handles->hi2c, I2C_ADDRESS_HAL,
                              (uint16_t)reg_address, MPU6050_REG_SIZE,
                              &reg, 1, I2C_TIMEOUT);
    STATUS_CHECK(status);
    if(state == MPU_ENABLE) {
        reg |= mask;
    }
    else {
        reg &= ~(mask);
    }

    status = HAL_I2C_Mem_Write(handles->hi2c, I2C_ADDRESS_HAL,
                               (uint16_t)reg_address, MPU6050_REG_SIZE,
                               &reg, 1, I2C_TIMEOUT);

    exit:
    return status;
}
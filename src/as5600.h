/*!
 *******************************************************************************
 * @file as5600.h
 *
 * @brief 
 *
 * @author Raúl Gotor (raulgotor@gmail.com)
 * @date 23.05.21
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2021 Raúl Gotor
 * All rights reserved.
 *******************************************************************************
 */

#ifndef AS5600_H
#define AS5600_H

/*
 *******************************************************************************
 * Public Macros                                                               *
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Public Data Types                                                           *
 *******************************************************************************
 */

typedef enum as5600_error_e {
        AS5600_ERROR_SUCCESS = 0,
        AS5600_ERROR_BAD_PARAMETER,
        AS5600_ERROR_RUNTIME_ERROR,
        AS5600_ERROR_I2C_ERROR,
        AS5600_ERROR_NOT_INITIALIZED,
        AS5600_ERROR_MAGNET_NOT_DETECTED,
        AS5600_ERROR_MAX_WRITE_CYCLES_REACHED,
        AS5600_ERROR_MIN_ANGLE_TOO_SMALL,
        AS5600_ERROR_GENERAL_ERROR,
        AS5600_ERROR_COUNT
} as5600_error_t;

typedef uint32_t (*pf_i2c_xfer_as5600_t)(
        uint8_t const slave_addr,
        uint8_t const * const p_tx_buffer,
        size_t const tx_buffer_size,
        uint8_t * const p_rx_buffer,
        size_t const rx_buffer_size);

/*!
 * @brief BQ25611 Registers
 *
 * | Address | Access| Acronym       | Register Name                            |
 * | ------: | :---: | :------------ | :--------------------------------------- |
 * |    0x00 |  R     | ZMCO          |  |
 * |    0x01 |  R/W/P | ZPOS (HI)     |  |
 * |    0x02 |  R/W/P | ZPOS (LO)     |  |
 * |    0x03 |  R/W/P | MPOS (HI)     |  |
 * |    0x04 |  R/W/P | MPOS (LO)     |  |
 * |    0x05 |  R/W/P | MANG (HI)     |  |
 * |    0x06 |  R/W/P | MANG (LO)     |  |
 * |    0x07 |  R/W/P | CONF (HI)     |  |
 * |    0x08 |  R/W/P | CONF (LO)     |  |
 * |    -    |  -     | -             |  |
 * |    0x0B |  R     | STATUS        |  |
 * |    0x0C |  R     | RAWANGLE (HI) |  |
 * |    0x0D |  R     | RAWANGLE (LO) |  |
 * |    0x0E |  R     | ANGLE (HI)    |  |
 * |    0x0F |  R     | ANGLE (LO)    |  |
 * |    -    |  -     | -             |  |
 * |    0x1A |  R     | AGC           |  |
 * |    0x1B |  R     | MAGNITUDE (HI)|  |
 * |    0x1C |  R     | MAGNITUDE (LO)|  |
 * |    -    |  -     | -             |  |
 * |    0xFF |  W     | BURN          |  |
 */

typedef enum {
        AS5600_REGISTER_ZMCO = 0x00,
        AS5600_REGISTER_ZPOS_H,
        AS5600_REGISTER_ZPOS_L,
        AS5600_REGISTER_MPOS_H,
        AS5600_REGISTER_MPOS_L,
        AS5600_REGISTER_MANG_H,
        AS5600_REGISTER_MANG_L,
        AS5600_REGISTER_CONF_H,
        AS5600_REGISTER_CONF_L,
        AS5600_REGISTER_STATUS = 0x0B,
        AS5600_REGISTER_RAWANGLE_H,
        AS5600_REGISTER_RAWANGLE_L,
        AS5600_REGISTER_ANGLE_H,
        AS5600_REGISTER_ANGLE_L,
        AS5600_REGISTER_AGC = 0x1A,
        AS5600_REGISTER_MAGNITUDE_H,
        AS5600_REGISTER_MAGNITUDE_L,
        AS5600_REGISTER_BURN = 0xFF,
} as5600_register_t;

typedef enum {
        AS5600_POWER_MODE_NOM = 0,
        AS5600_POWER_MODE_LPM1,
        AS5600_POWER_MODE_LPM2,
        AS5600_POWER_MODE_LPM3,
        AS5600_POWER_MODE_COUNT
} as5600_power_mode_t;

typedef enum {
        AS5600_HYSTERESIS_OFF = 0,
        AS5600_HYSTERESIS_1LSB,
        AS5600_HYSTERESIS_2LSB,
        AS5600_HYSTERESIS_3LSB,
        AS5600_HYSTERESIS_COUNT
} as5600_hysteresis_t;

typedef enum {
        AS5600_OUTPUT_STAGE_ANALOG_FR = 0,
        AS5600_OUTPUT_STAGE_ANALOG_RR,
        AS5600_OUTPUT_STAGE_DIGITAL_PWM,
        AS5600_OUTPUT_STAGE_COUNT
} as5600_output_stage_t;

typedef enum {
        AS5600_PWM_FREQUENCY_115HZ = 0,
        AS5600_PWM_FREQUENCY_230HZ,
        AS5600_PWM_FREQUENCY_460HZ,
        AS5600_PWM_FREQUENCY_920HZ,
        AS5600_PWM_FREQUENCY_COUNT
} as5600_pwm_frequency_t;

typedef enum {
        AS5600_SLOW_FILTER_16X = 0,
        AS5600_SLOW_FILTER_8X,
        AS5600_SLOW_FILTER_4X,
        AS5600_SLOW_FILTER_2X,
        AS5600_SLOW_FILTER_COUNT
} as5600_slow_filter_t;

typedef enum {
        AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY = 0,
        AS5600_FF_THRESHOLD_6LSB,
        AS5600_FF_THRESHOLD_7LSB,
        AS5600_FF_THRESHOLD_9LSB,
        AS5600_FF_THRESHOLD_18LSB,
        AS5600_FF_THRESHOLD_21LSB,
        AS5600_FF_THRESHOLD_24LSB,
        AS5600_FF_THRESHOLD_10LSB,
        AS5600_FF_THRESHOLD_COUNT
} as5600_ff_threshold_t;

typedef struct as5600_configuration_s {
        as5600_power_mode_t power_mode;
        as5600_hysteresis_t hysteresis;
        as5600_output_stage_t output_stage;
        as5600_pwm_frequency_t pwm_frequency;
        as5600_slow_filter_t slow_filter;
        as5600_ff_threshold_t ff_threshold;
        bool watchdog;
} as5600_configuration_t;

typedef enum
{
        // @brief No magnet was detected
        AS5600_STATUS_NO_MANGET = 0x0U,
        // @brief AGC minimum gain overflow, magnet too strong
        AS5600_STATUS_MH = 0x08U,
        // @brief AGC maximum gain overflow, magnet too weak
        AS5600_STATUS_ML = 0x10U,
        // @brief Magnet was detected
        AS5600_STATUS_MD = 0x04U,
} as5600_status_t;

typedef enum
{
        AS5600_BURN_MODE_BURN_SETTING = 0x40U,
        AS5600_BURN_MODE_BURN_ANGLE = 0x80U
} as5600_burn_mode_t;

/*
 *******************************************************************************
 * Public Constants                                                            *
 *******************************************************************************
 */


/*
 *******************************************************************************
 * Public Function Prototypes                                                  *
 *******************************************************************************
 */

//! @brief Initialize the module
as5600_error_t as5600_init(pf_i2c_xfer_as5600_t const pf_transfer_func);

//! @brief Deinitialize the module
as5600_error_t as5600_deinit(void);

//! @brief Get the counter of already write operations to the OTP
as5600_error_t as5600_get_otp_write_counter(uint8_t * const p_write_counter);

//! @brief Set start position angle (ZPOS)
as5600_error_t as5600_set_start_position(uint16_t const start_position);

//! @brief Get start position angle (ZPOS)
as5600_error_t as5600_get_start_position(uint16_t * const p_start_position);

//! @brief Set stop position angle (MPOS)
as5600_error_t as5600_set_stop_position(uint16_t const stop_position);

//! @brief Get stop position angle (MPOS)
as5600_error_t as5600_get_stop_position(uint16_t * const p_stop_position);

//! @brief Set maximum angle (MANG)
as5600_error_t as5600_set_maximum_angle(uint16_t const maximum_angle);

//! @brief Get maximum angle (MANG)
as5600_error_t as5600_get_maximum_angle(uint16_t * const p_maximum_angle);

//! @brief Set configuration of the AS5600
as5600_error_t as5600_set_configuration(
                                 as5600_configuration_t const * const p_config);

//! @brief Get configuration of the AS5600
as5600_error_t as5600_get_configuration(as5600_configuration_t * const p_config);

//! @brief Set slow filter
as5600_error_t as5600_set_slow_filter(as5600_slow_filter_t const slow_filter,
                                      as5600_configuration_t * const p_config);

//! @brief Get slow filter
as5600_error_t as5600_get_slow_filter(
                                 as5600_slow_filter_t * const p_slow_filter,
                                 as5600_configuration_t const * const p_config);

//! @brief Set fast filter threshold
as5600_error_t as5600_set_ff_threshold(as5600_ff_threshold_t const ff_threshold,
                                       as5600_configuration_t * const p_config);

//! @brief Get fast filter threshold
as5600_error_t as5600_get_ff_threshold(
                                 as5600_ff_threshold_t * const p_ff_threshold,
                                 as5600_configuration_t const * const p_config);

//! @brief Set watchdog status
as5600_error_t as5600_set_watchdog_enabled(
                                       bool const enabled,
                                       as5600_configuration_t * const p_config);

//! @brief Get status of the watchdog
as5600_error_t as5600_is_watchdog_enabled(
                                 bool * const p_enabled,
                                 as5600_configuration_t const * const p_config);

//! @brief Set power mode
as5600_error_t as5600_set_power_mode(as5600_power_mode_t const power_mode,
                                     as5600_configuration_t * const p_config);

//! @brief Get power mode
as5600_error_t as5600_get_power_mode(
                                 as5600_power_mode_t * const p_power_mode,
                                 as5600_configuration_t const * const p_config);

//! @brief Set hysteresis sensitivity
as5600_error_t as5600_set_hysteresis(as5600_hysteresis_t const hysteresis,
                                     as5600_configuration_t * const p_config);

//! @brief Get hysteresis sensitivity
as5600_error_t as5600_get_hysteresis(
                                 as5600_hysteresis_t * const p_hysteresis,
                                 as5600_configuration_t const * const p_config);

//! @brief Set output stage
as5600_error_t as5600_set_output_stage(as5600_output_stage_t const output_stage,
                                       as5600_configuration_t * const p_config);

//! @brief Get output stage
as5600_error_t as5600_get_output_stage(
                                 as5600_output_stage_t * const p_output_stage,
                                 as5600_configuration_t const * const p_config);

//! @brief Set PWM frequency
as5600_error_t as5600_set_pwm_frequency(
                                     as5600_pwm_frequency_t const pwm_frequency,
                                     as5600_configuration_t * const p_config);

//! @brief Get PWM frequency
as5600_error_t as5600_get_pwm_frequency(
                                 as5600_pwm_frequency_t * const p_pwm_frequency,
                                 as5600_configuration_t const * const p_config);

//! @brief Get RAW angle
as5600_error_t as5600_get_raw_angle(uint16_t * const p_raw_angle);

//! @brief Get angle
as5600_error_t as5600_get_angle(uint16_t * const p_angle);

//! @brief Get status of the device
as5600_error_t as5600_get_status(as5600_status_t * const p_status);

//! @brief Get automatic gain control value
as5600_error_t as5600_get_automatic_gain_control(uint8_t * const p_agc);

//! @brief Get cordic magnitude
as5600_error_t as5600_get_cordic_magnitude(uint16_t * const p_magnitude);

//! @brief Send burn command to the device
as5600_error_t as5600_burn_command(as5600_burn_mode_t const mode);

#endif //AS5600_H
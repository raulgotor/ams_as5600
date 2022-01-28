/*!
 *******************************************************************************
 * @file as5600.h
 *
 * @brief 
 *
 * @author Raúl Gotor
 * @date 23.05.21
 *
 * @par
 * (c) Copyright 2021 Raúl Gotor.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************
 */

#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C"
{
#endif // defined (__cplusplus)

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

//! @brief Return error codes for the module
typedef enum as5600_error_e
{
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

//! @brief I2C transfer function pointer
typedef uint32_t (* pf_i2c_xfer_as5600_t)(
                uint8_t const slave_addr,
                uint8_t const * const p_tx_buffer,
                size_t const tx_buffer_size,
                uint8_t * const p_rx_buffer,
                size_t const rx_buffer_size);

/*!
 * @brief AS5600 Register map
 *
 * @warning Do not change the order of this enumeration
 *
 * | Addr | Access| Acronym       |
 * |------|-------|---------------|
 * | 0x00 | R     | ZMCO          |
 * | 0x01 | R/W/P | ZPOS (HI)     |
 * | 0x02 | R/W/P | ZPOS (LO)     |
 * | 0x03 | R/W/P | MPOS (HI)     |
 * | 0x04 | R/W/P | MPOS (LO)     |
 * | 0x05 | R/W/P | MANG (HI)     |
 * | 0x06 | R/W/P | MANG (LO)     |
 * | 0x07 | R/W/P | CONF (HI)     |
 * | 0x08 | R/W/P | CONF (LO)     |
 * | -    | -     | -             |
 * | 0x0B | R     | STATUS        |
 * | 0x0C | R     | RAWANGLE (HI) |
 * | 0x0D | R     | RAWANGLE (LO) |
 * | 0x0E | R     | ANGLE (HI)    |
 * | 0x0F | R     | ANGLE (LO)    |
 * | -    | -     | -             |
 * | 0x1A | R     | AGC           |
 * | 0x1B | R     | MAGNITUDE (HI)|
 * | 0x1C | R     | MAGNITUDE (LO)|
 * | -    | -     | -             |
 * | 0xFF | W     | BURN          |
 */
typedef enum
{
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

//! @brief Power modes for PM bitfield at the CONF register
typedef enum
{
        AS5600_POWER_MODE_NOM = 0,
        AS5600_POWER_MODE_LPM1,
        AS5600_POWER_MODE_LPM2,
        AS5600_POWER_MODE_LPM3,
        AS5600_POWER_MODE_COUNT
} as5600_power_mode_t;

//! @brief Hysteresis lsb's for HYST bitfield at the CONF register
typedef enum
{
        //! @brief No hysteresis
        AS5600_HYSTERESIS_OFF = 0,

        //! @brief Least significant bit hysteresis
        AS5600_HYSTERESIS_1LSB,

        //! @brief Two least significant bits hysteresis
        AS5600_HYSTERESIS_2LSB,

        //! @brief Three least significant bits hysteresis
        AS5600_HYSTERESIS_3LSB,

        //! @brief Fence member
        AS5600_HYSTERESIS_COUNT
} as5600_hysteresis_t;

//! @brief Output stage types for OUTS bitfield at the CONF register
typedef enum
{
        //! @brief Output stage analog full range 0 - 100%
        AS5600_OUTPUT_STAGE_ANALOG_FR = 0,

        //! @brief Output stage analog digital range 10 - 90%
        AS5600_OUTPUT_STAGE_ANALOG_RR,

        //! @brief Output stage digital PWM
        AS5600_OUTPUT_STAGE_DIGITAL_PWM,

        //! @brief Fence member
        AS5600_OUTPUT_STAGE_COUNT
} as5600_output_stage_t;

//! @brief Allowed PWM output frequencies at the PWMF bitfield at the CONF register
typedef enum
{
        //! @brief PWM Frequency of 115 Hz
        AS5600_PWM_FREQUENCY_115HZ = 0,

        //! @brief PWM Frequency of 230 Hz
        AS5600_PWM_FREQUENCY_230HZ,

        //! @brief PWM Frequency of 460 Hz
        AS5600_PWM_FREQUENCY_460HZ,

        //! @brief PWM Frequency of 920 Hz
        AS5600_PWM_FREQUENCY_920HZ,

        //! @brief Fence member
        AS5600_PWM_FREQUENCY_COUNT
} as5600_pwm_frequency_t;

//! @brief Slow filter step response delays for SF bitfield at CONF register
typedef enum
{
        //! @brief Slow filter with 16x step response delay
        AS5600_SLOW_FILTER_16X = 0,

        //! @brief Slow filter with 8x step response delay
        AS5600_SLOW_FILTER_8X,

        //! @brief Slow filter with 4x step response delay
        AS5600_SLOW_FILTER_4X,

        //! @brief Slow filter with 2x step response delay
        AS5600_SLOW_FILTER_2X,

        //! @brief Fence member
        AS5600_SLOW_FILTER_COUNT
} as5600_slow_filter_t;

//! @brief Fast filter threshold options for FF bitfield at CONF register
typedef enum
{
        //! @brief Use slow filter only
        AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY = 0,

        //! @brief Fast filter threshold of 6 LSB
        AS5600_FF_THRESHOLD_6LSB,

        //! @brief Fast filter threshold of 7 LSB
        AS5600_FF_THRESHOLD_7LSB,

        //! @brief Fast filter threshold of 9 LSB
        AS5600_FF_THRESHOLD_9LSB,

        //! @brief Fast filter threshold of 18 LSB
        AS5600_FF_THRESHOLD_18LSB,

        //! @brief Fast filter threshold of 21 LSB
        AS5600_FF_THRESHOLD_21LSB,

        //! @brief Fast filter threshold of 24 LSB
        AS5600_FF_THRESHOLD_24LSB,

        //! @brief Fast filter threshold of 10 LSB
        AS5600_FF_THRESHOLD_10LSB,

        //! @brief Fence member
        AS5600_FF_THRESHOLD_COUNT
} as5600_ff_threshold_t;

//! @brief Configuration register (CONF) structure
typedef struct as5600_configuration_s
{
        //! @brief Power Mode (PM) bitfield
        as5600_power_mode_t power_mode;

        //! @brief Hysteresis (HYST) bitfield
        as5600_hysteresis_t hysteresis;

        //! @brief Output stage (OUTS) bitfield
        as5600_output_stage_t output_stage;

        //! @brief PWM Frequency (PWMF) bitfield
        as5600_pwm_frequency_t pwm_frequency;

        //! @brief Slow filter (SF) bitfield
        as5600_slow_filter_t slow_filter;

        //! @brief Fast filter threshold (FFT) bitfield
        as5600_ff_threshold_t ff_threshold;

        //! @brief Watchdog (WD) bitfield
        bool watchdog;

} as5600_configuration_t;

//! @brief Status register (STATUS) elements
typedef enum
{
        // @brief No magnet was detected
        AS5600_STATUS_NO_MANGET = 0x0U,
        // @brief AGC minimum gain overflow, magnet too strong
        AS5600_STATUS_MH = 0x08U,
        // @brief AGC maximum gain overflow, magnet too weak
        AS5600_STATUS_ML = 0x10U,
        // @brief Magnet was detected
        AS5600_STATUS_MD = 0x20U,
        // @brief Magnet detected with AGC minimum gain overflow, magnet too strong
        AS5600_STATUS_MH_MD = 0x28U,
        // @brief Magnet detected with AGC maximum gain overflow, magnet too weak
        AS5600_STATUS_ML_MD = 0x30U,

} as5600_status_t;

//! @brief BURN register commands
typedef enum
{
        //! @brief Command for burning a setting configuration
        AS5600_BURN_MODE_BURN_SETTING = 0x40U,

        //! @brief Command for burning start and end angles
        AS5600_BURN_MODE_BURN_ANGLE = 0x80U,

        //! @brief Fence member
        AS5600_BURN_MODE_COUNT,

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

#ifdef __cplusplus
}
#endif // defined (__cplusplus)

#endif //AS5600_H
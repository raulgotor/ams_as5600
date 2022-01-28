/*!
 *******************************************************************************
 * @file as5600.c
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

/*
 *******************************************************************************
 * #include Statements                                                         *
 *******************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "as5600.h"

/*
 *******************************************************************************
 * Private Macros                                                              *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Data types                                                                  *
 *******************************************************************************
 */

//! @brief Data structure for bitfield specification
typedef struct {
        as5600_register_t reg;
        uint8_t lsbit_pos;
        uint8_t width;
} as5600_bit_field_specs_t;

//! @brief Different bitfields in AS5600 ordered by address and lsb
typedef enum {
        AS5600_BIT_FIELD_ZMCO = 0,
        AS5600_BIT_FIELD_ZPOS_HI_BYTE,
        AS5600_BIT_FIELD_ZPOS_LO_BYTE,
        AS5600_BIT_FIELD_MPOS_HI_BYTE,
        AS5600_BIT_FIELD_MPOS_LO_BYTE,
        AS5600_BIT_FIELD_MANG_HI_BYTE,
        AS5600_BIT_FIELD_MANG_LO_BYTE,
        AS5600_BIT_FIELD_SF,
        AS5600_BIT_FIELD_FTH,
        AS5600_BIT_FIELD_WD,
        AS5600_BIT_FIELD_PM,
        AS5600_BIT_FIELD_HYST,
        AS5600_BIT_FIELD_OUTS,
        AS5600_BIT_FIELD_PWMF,
        AS5600_BIT_FIELD_RAWANGLE_HI_BYTE,
        AS5600_BIT_FIELD_RAWANGLE_LO_BYTE,
        AS5600_BIT_FIELD_ANGLE_HI_BYTE,
        AS5600_BIT_FIELD_ANGLE_LO_BYTE,
        AS5600_BIT_FIELD_STATUS,
        AS5600_BIT_FIELD_AGC,
        AS5600_BIT_FIELD_MAGNITUDE_HI_BYTE,
        AS5600_BIT_FIELD_MAGNITUDE_LO_BYTE,
        AS5600_BIT_FIELD_BURN,
        AS5600_BIT_FIELD_COUNT
} as5600_bit_field_t;

/*
 *******************************************************************************
 * Constants                                                                   *
 *******************************************************************************
 */

//! @brief Pointer to a I2C transfer function
static pf_i2c_xfer_as5600_t m_as5600_xfer_func = NULL;

//! @brief I2C address of the AS5600
static uint8_t const m_as5600_i2c_addr = 0x36U;

//! @brief Control whether the driver is initialized or not
static bool m_is_initialized = false;

/*!
 * @brief Minimum angular range in steps that can be configured at MANG register
 *
 *      m_min_angle_deg   = 18;
 *
 *      m_min_angle_steps = ceil(4095 * m_min_angle_deg / 359);
 */
static uint32_t const m_min_angle_steps = 206;


/*!
 * @brief List of the different specification for each bitfield, ordered by
 *        address and lsb
 */
as5600_bit_field_specs_t m_bitfields[] = {
        {
                //! @brief Configuration of ZMCO bitfield
                .reg = AS5600_REGISTER_ZMCO,
                .lsbit_pos = 0,
                .width = 2
        },
        {
                //! @brief Configuration of high byte of ZPOS bitfield
                .reg = AS5600_REGISTER_ZPOS_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of ZPOS bitfield
                .reg = AS5600_REGISTER_ZPOS_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of high byte of MPOS bitfield
                .reg = AS5600_REGISTER_MPOS_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of MPOS bitfield
                .reg = AS5600_REGISTER_MPOS_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of high byte of MANG bitfield
                .reg = AS5600_REGISTER_MANG_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of MANG bitfield
                .reg = AS5600_REGISTER_MANG_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of SF bitfield
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 0,
                .width = 2,
        },
        {
                //! @brief Configuration of FTH bitfield
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 2,
                .width = 3
        },
        {
                //! @brief Configuration of WD bitfield
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 5,
                .width = 1
        },
        {
                //! @brief Configuration of PM bitfield
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 0,
                .width = 2
        },
        {
                //! @brief Configuration of HYST bitfield
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 2,
                .width = 2
        },
        {
                //! @brief Configuration of OUTS bitfield
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 4,
                .width = 2
        },
        {
                //! @brief Configuration of PWMF bitfield
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 6,
                .width = 2
        },
        {
                //! @brief Configuration of high byte of RAWANGLE bitfield
                .reg = AS5600_REGISTER_RAWANGLE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of RAWANGLE bitfield
                .reg = AS5600_REGISTER_RAWANGLE_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of high byte of ANGLE bitfield
                .reg = AS5600_REGISTER_ANGLE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of ANGLE bitfield
                .reg = AS5600_REGISTER_ANGLE_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of STATUS bitfield
                .reg = AS5600_REGISTER_STATUS,
                .lsbit_pos = 3,
                .width = 3
        },
        {
                //! @brief Configuration of AGC bitfield
                .reg = AS5600_REGISTER_AGC,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of high byte of MAGNITUDE bitfield
                .reg = AS5600_REGISTER_MAGNITUDE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                //! @brief Configuration of low byte of MAGNITUDE bitfield
                .reg = AS5600_REGISTER_MAGNITUDE_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                //! @brief Configuration of BURN bitfield
                .reg = AS5600_REGISTER_BURN,
                .lsbit_pos = 0,
                .width = 8
        }
};

//! @brief Available status readouts for STATUS register
static as5600_status_t const m_available_status[] =
                {
                        AS5600_STATUS_NO_MANGET,
                        AS5600_STATUS_MH,
                        AS5600_STATUS_ML,
                        AS5600_STATUS_MD,
                        AS5600_STATUS_MH_MD,
                        AS5600_STATUS_ML_MD,
                };

//! @brief Number of available status readouts for STATUS register
static uint8_t const m_available_status_len = sizeof(m_available_status) /
                                              sizeof(m_available_status[0]);
/*
 *******************************************************************************
 * Private Function Prototypes                                                 *
 *******************************************************************************
 */

//! @brief Read n consecutive bytes from a register
static as5600_error_t as5600_read_n_consecutive_bytes(
                                                    as5600_register_t const reg,
                                                    uint8_t * const p_rx_buffer,
                                                    size_t const bytes_count);
//! @brief Write n consecutive bytes to a register
static as5600_error_t as5600_write_n_consecutive_bytes(
                                              as5600_register_t const reg,
                                              uint8_t const * const p_tx_buffer,
                                              size_t const bytes_count);
//! @brief Read 16-bit register
static as5600_error_t as5600_read_16register(as5600_register_t const reg,
                                             uint16_t * const p_rx_buffer);
//! @brief Write 16-bit register
static as5600_error_t as5600_write_16register(as5600_register_t const reg,
                                              uint16_t const tx_buffer);

//! @brief Write 8-bit register
static as5600_error_t as5600_write_8register(as5600_register_t const reg,
                                             uint8_t const tx_buffer);


//! @brief Read 8-bit register
static as5600_error_t as5600_read_8register(as5600_register_t const reg,
                                            uint8_t * const p_rx_buffer);

//! @brief Set bit field value from a given register
static as5600_error_t as5600_reg_set_bit_field_value(
                                             uint8_t const value,
                                             as5600_bit_field_t const bit_field,
                                             uint8_t * const p_reg_value);

//! @brief Get value from bit field
static as5600_error_t as5600_reg_get_bit_field_value(
                                             uint8_t * const p_value,
                                             as5600_bit_field_t const bit_field,
                                             uint8_t const reg_value);

//! @brief Configuration structure to 16-bit register value
static as5600_error_t as5600_cfg_to_reg16(
                                  as5600_configuration_t const * const p_config,
                                  uint16_t * const reg);

//! @brief 16-bit register value to configuration structure
static as5600_error_t as5600_reg16_to_cfg(
                                       uint16_t const * const p_reg,
                                       as5600_configuration_t * const p_config);

//! @brief Check if configuration is valid
static bool as5600_is_valid_configuration(
                                 as5600_configuration_t const * const p_config);

//! @brief Check if register is valid
static bool as5600_is_register_valid(as5600_register_t const reg);

/*
 *******************************************************************************
 * Public Data Declarations                                                    *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Static Data Declarations                                                    *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Public Function Bodies                                                      *
 *******************************************************************************
 */

/*!
 * @brief Initialize the module
 *
 * This function initializes the module for operation. The module can only
 * be initialized once.
 *
 * @param[in]           pf_transfer_func    Object containing the pointer
 *                                          to a data transfer function
 *
 * @return              as5600_error_t          Result of the operation
 * @retval              AS5600_ERROR_SUCCESS    If everything went well
 * @retval              AS5600_ERROR_RUNTIME_ERROR Module already initialized
 * @retval              AS5600_ERROR_BAD_PARAMETER Invalid transfer function pointer
 */
as5600_error_t as5600_init(pf_i2c_xfer_as5600_t const pf_transfer_func)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;

        if (m_is_initialized) {
                result = AS5600_ERROR_RUNTIME_ERROR;
        } else if (NULL == pf_transfer_func) {
                result = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == result) {
                m_as5600_xfer_func = pf_transfer_func;
                m_is_initialized = true;
        }

        return result;
}

as5600_error_t as5600_deinit(void)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;

        if (!m_is_initialized) {
                result = AS5600_ERROR_RUNTIME_ERROR;
        } else {
                m_is_initialized = false;
                m_as5600_xfer_func = NULL;
        }

        return result;
}

/*!
 * @brief Get the counter of already write operations to the OTP
 *
 * Gets the counter of performed writing operations to the one time programable
 * (OTP) register ZMCO
 *
 * The host microcontroller can perform a permanent programming of ZPOS and
 * MPOS with a BURN_ANGLE command. ZMCO shows how many times ZPOS and MPOS have
 * been permanently written.
 *
 * @param[out]          p_write_counter     Pointer were to return the number of
 *                                          write operations to the OTP register
 *
 * @return              as5600_error_t          Result of the operation
 * @retval              AS5600_ERROR_SUCCESS    If everything went well
 * @retval              AS5600_ERROR_BAD_PARAMETER Invalid pointer
 */
as5600_error_t as5600_get_otp_write_counter(uint8_t * const p_write_counter)
{
        as5600_bit_field_t const field = AS5600_BIT_FIELD_ZMCO;
        as5600_register_t const reg = m_bitfields[field].reg;

        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t bit_field_value;
        uint8_t reg_value;

        if (NULL == p_write_counter) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_8register(reg, &reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&bit_field_value,
                                                         field, reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_write_counter = bit_field_value;
        }

        return success;
}

/*!
 * @brief Set start position angle (ZPOS)
 *
 * These registers are used to configure the start position (ZPOS) and a stop
 * position (MPOS) or maximum angle (MANG) for a narrower angular range.
 * The angular range must be greater than 18 degrees. In case of narrowed
 * angular range, the resolution is not scaled to narrowed range
 * (e.g. 0° to 360°(full-turn) → 4096 dec; 0° to 180° → 2048 dec).
 *
 * @param[in]       start_position          start position value [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Parameter OOR
 */
as5600_error_t as5600_set_start_position(uint16_t const start_position)
{
        as5600_register_t const reg = AS5600_REGISTER_ZPOS_H;
        uint16_t const reg_mask = 0x0FFFU;
        uint16_t const reg_value = start_position;

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if (reg_mask < reg_value) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_16register(reg, reg_value);
        }

        return success;
}

/*!
 * @brief Get start position angle (ZPOS)
 *
 * @see `as5600_set_start_position`
 *
 * @param[out]      p_start_position        Pointer to memory where to write
 *                                          the start position to [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_get_start_position(uint16_t * const p_start_position)
{
        as5600_register_t const reg = AS5600_REGISTER_ZPOS_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t buffer;

        if (NULL == p_start_position) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &buffer);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_start_position = buffer;
        }

        return success;
}


/*!
 * @brief Set stop position angle (MPOS)
 *
 * These registers are used to configure the start position (ZPOS) and a stop
 * position (MPOS) or maximum angle (MANG) for a narrower angular range.
 * The angular range must be greater than 18 degrees. In case of narrowed
 * angular range, the resolution is not scaled to narrowed range
 * (e.g. 0° to 360°(full-turn) → 4096 dec; 0° to 180° → 2048 dec).
 *
 * @param[in]       stop_position           Stop position value [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Parameter OOR
 */
as5600_error_t as5600_set_stop_position(uint16_t const stop_position)
{
        as5600_register_t const reg = AS5600_REGISTER_MPOS_H;
        uint16_t const reg_mask = 0x0FFFU;
        uint16_t const reg_value = stop_position;

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if (reg_mask < reg_value) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_16register(reg, reg_value);
        }

        return success;

}

/*!
 * @brief Get stop position angle (MPOS)
 *
 * @see `as5600_set_stop_position`
 *
 * @param[out]      p_stop_position         Pointer to memory where to write
 *                                          the stop position to [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_get_stop_position(uint16_t * const p_stop_position)
{
        as5600_register_t const reg = AS5600_REGISTER_MPOS_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t buffer;

        if (NULL == p_stop_position) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &buffer);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_stop_position = buffer;
        }

        return success;
}

/*!
 * @brief Set maximum angle (MANG)
 *
 * These registers are used to configure the start position (ZPOS) and a stop
 * position (MPOS) or maximum angle (MANG) for a narrower angular range.
 * The angular range must be greater than 18 degrees. In case of narrowed
 * angular range, the resolution is not scaled to narrowed range
 * (e.g. 0° to 360°(full-turn) → 4096 dec; 0° to 180° → 2048 dec).
 *
 * @param[in]       max_angle               Maximum allowed angle [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Parameter OOR
 */
as5600_error_t as5600_set_maximum_angle(uint16_t const max_angle)
{
        as5600_register_t const reg = AS5600_REGISTER_MANG_H;
        uint16_t const reg_mask = 0x0FFFU;
        uint16_t const reg_value = max_angle;

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((reg_mask < reg_value) || (m_min_angle_steps > max_angle)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_16register(reg, reg_value);
        }

        return success;

}

/*!
 * @brief Get maximum angle (MANG)
 *
 * @see `as5600_set_maximum_angle`
 *
 * @param[out]      p_max_angle             Pointer to memory where to write
 *                                          the maximum angle to [0 - 4095]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_get_maximum_angle(uint16_t * const p_max_angle)
{
        as5600_register_t const reg = AS5600_REGISTER_MANG_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t buffer;

        if (NULL == p_max_angle) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &buffer);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_max_angle = buffer;
        }

        return success;
}

/*!
 * @brief Set configuration of the AS5600
 *
 * This function will configure the AS5600 CONF register according to the
 * configuration structure passed to it. The function checks for the validity of
 * the passed configuration, converts the configuration to AS5600 register value
 * and performs a write operation. The elements of the structure can be manually
 * configured, or with the according safe functions functions
 *
 * @see `as5600_set_slow_filter`, `as5600_set_ff_threshold`,
 *      `as5600_set_watchdog_enabled`, `as5600_set_power_mode`,
 *      `as5600_set_hysteresis`, `as5600_set_output_stage`,
 *      `as5600_set_pwm_frequency`
 *
 * @param[in]       p_config                Pointer to a configuration structure
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or configuration invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_set_configuration(
                                  as5600_configuration_t const * const p_config)
{
        as5600_register_t const reg = AS5600_REGISTER_CONF_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        uint16_t reg_value;

        if (NULL == p_config) {
                success = AS5600_ERROR_BAD_PARAMETER;
        } else if (!as5600_is_valid_configuration(p_config)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_cfg_to_reg16(p_config, &reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_16register(reg, reg_value);
        }

        return success;
}

/*!
 * @brief Get configuration of the AS5600
 *
 * This function will get the current configuration of the AS5600 CONF register
 * and will convert it to a `as5600_configuration_t` object in a single read
 * operation. The values can be accessed directly from the structure or via
 * the other getter methods.
 *
 * @see `as5600_set_configuration`
 *
 * @see `as5600_get_slow_filter`, `as5600_get_ff_threshold`,
 *      `as5600_is_watchdog_enabled`, `as5600_get_power_mode`,
 *      `as5600_get_hysteresis`, `as5600_get_output_state`,
 *      `as5600_get_pwm_frequency`
 *
 * @param[out]      p_config                Pointer to a configuration structure
 *                                          of type `as5600_configuration_t`
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_configuration(as5600_configuration_t * const p_config)
{
        as5600_register_t const reg = AS5600_REGISTER_CONF_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        as5600_configuration_t temp_config;
        uint16_t reg_value;

        if (NULL == p_config) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg16_to_cfg(&reg_value, &temp_config);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_config = temp_config;
        }

        return success;
}

/*!
 * @brief Set slow filter
 *
 * Sets the slow filter field of a `as5600_configuration_t` object.
 *
 * The AS5600 has a digital post-processing programmable filter which can be set
 * in fast or slow modes. The fast filter mode can be enabled by setting a fast
 * filter threshold in the FTH bits of the CONF register. If the fast filter is
 * OFF, the step output response is controlled by the slow linear filter. The
 * step response of the slow filter is programmable with the SF bits in the CONF
 * register.
 *
 * @note `AS5600_SLOW_FILTER_16X` is forced in low power mode (LPM)
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       slow_filter             Step response delay to set,
 *                                          multiples of 0.143 ms
 *                                          `as5600_slow_filter_t` type
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer/parameter value invalid
 */
as5600_error_t as5600_set_slow_filter(as5600_slow_filter_t const slow_filter,
                                      as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_slow_filter_t const fence = AS5600_SLOW_FILTER_COUNT;

        if ((NULL == p_config) || (fence <= slow_filter)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->slow_filter = slow_filter;
        }

        return success;

}

/*!
 * @brief Get slow filter
 *
 * Gets the slow filter field of a `as5600_configuration_t` object.
 *
 * @see  `as5600_set_slow_filter`
 *
 * @note this function only reads the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_slow_filter           Current step response delay
 *                                          `as5600_slow_filter_t` type
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_get_slow_filter(
                                  as5600_slow_filter_t * const p_slow_filter,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_slow_filter)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_slow_filter = p_config->slow_filter;
        }

        return success;

}

/*!
 * @brief Set fast filter threshold
 *
 * Sets the fast filter threshold field of a `as5600_configuration_t` object.
 *
 * For a fast step response and low noise after settling, the fast filter can be
 * enabled. The fast filter works only if the input variation is greater than
 * the fast filter threshold, otherwise the output response is determined only
 * by the slow filter. The fast filter threshold is programmed with the FTH bits
 * in the CONF Register
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       ff_threshold            Slow to fast filter threshold (LSB),
 *                                          to be set, `as5600_ff_threshold_t`
 *                                          type
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer/parameter value invalid
 */
as5600_error_t as5600_set_ff_threshold(as5600_ff_threshold_t const ff_threshold,
                                       as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_ff_threshold_t const fence = AS5600_FF_THRESHOLD_COUNT;

        if ((NULL == p_config) || (fence <= ff_threshold)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->ff_threshold = ff_threshold;
        }

        return success;

}

/*!
 * @brief Get fast filter threshold
 *
 * Gets the fast filter threshold field of a `as5600_configuration_t` object.
 *
 * @see  `as5600_set_ff_threshold`
 *
 * @note this function only reads the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_ff_threshold          Current slow to fast filter
 *                                          threshold
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_get_ff_threshold(
                                  as5600_ff_threshold_t * const p_ff_threshold,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_ff_threshold)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_ff_threshold = p_config->ff_threshold;
        }

        return success;

}

/*!
 * @brief Set watchdog status
 *
 * Set the status of the watchdog in a `as5600_configuration_t` object.
 *
 * The watchdog timer allows saving power by switching into LMP3 if the angle
 * stays within the watchdog threshold of 4 LSB for at least one minute. The
 * watchdog function can be enabled with the WD bit in the CONF register.
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       enabled                 True: enable watchdog
 *                                          False: disable watchdog
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_set_watchdog_enabled(
                                        bool const enabled,
                                        as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if (NULL == p_config) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->watchdog = true;
        }

        return success;

}

/*!
 * @brief Get status of the watchdog
 *
 * Gets the status of the watchdog in a `as5600_configuration_t` object.
 *
 * @see  `as5600_set_watchdog_enabled`
 *
 * @note this function only reads the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_enabled               Pointer to save the status of the
 *                                          watchdog at.
 *                                          True: watchdog is enabled
 *                                          False: watchdog is disabled
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Invalid pointer
 */
as5600_error_t as5600_is_watchdog_enabled(
                                  bool * const p_enabled,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_enabled)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_enabled = p_config->watchdog;
        }

        return success;

}

/*!
 * @brief Set power mode
 *
 * Set power mode of the AS5600 configured in a `as5600_configuration_t` object
 *
 * A digital state machine automatically manages the low power modes to reduce
 * the average current consumption. Three low power modes are available and can
 * be enabled with the PM bits in the CONF register. Current consumption and
 * polling times are shown below
 *
 * Power Mode | Polling time (ms) | Supply current (mA)
 * -----------|-------------------|--------------------
 * NOM        | Always on         | 6.5
 * LPM1       | 5                 | 3.4
 * LPM2       | 20                | 1.8
 * LPM3       | 100               | 1.5
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       power_mode              Value of type `as5600_power_mode_t`
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_set_power_mode(as5600_power_mode_t const power_mode,
                                     as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_power_mode_t const fence = AS5600_POWER_MODE_COUNT;

        if ((NULL == p_config) || (fence <= power_mode)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->power_mode = power_mode;
        }

        return success;

}

/*!
 * @brief Get power mode
 *
 * Get power mode of the AS5600 configured in a `as5600_configuration_t` object
 *
 * @see  `as5600_set_power_mode`
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_power_mode            Pointer to object to save the power
 *                                          mode value at
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_get_power_mode(
                                  as5600_power_mode_t * const p_power_mode,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_power_mode)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_power_mode = p_config->power_mode;
        }

        return success;

}

/*!
 * @brief Set hysteresis sensitivity
 *
 * Set hysteresis sensitivity configured in a `as5600_configuration_t` object
 *
 * To avoid any toggling of the output when the magnet is not moving, a 1 to 3
 * LSB hysteresis of the 12-bit resolution can be enabled with the HYST bits
 * in the CONF register.
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       hysteresis              Value of type `as5600_hysteresis_t`
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_set_hysteresis(as5600_hysteresis_t const hysteresis,
                                     as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_hysteresis_t const fence = AS5600_HYSTERESIS_COUNT;

        if ((NULL == p_config) || (fence <= hysteresis)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->hysteresis = hysteresis;
        }

        return success;

}

/*!
 * @brief Get hysteresis sensitivity
 *
 * Get hysteresis sensitivity configured in a `as5600_configuration_t` object
 *
 * @see  `as5600_set_hysteresis`
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_hysteresis            Pointer to object to save the
 *                                          hysteresis value at
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_get_hysteresis(
                                  as5600_hysteresis_t * const p_hysteresis,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_hysteresis)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_hysteresis = p_config->hysteresis;
        }

        return success;

}

/*!
 * @brief Set output stage
 *
 * Set output stage in a `as5600_configuration_t` object
 *
 * The OUTS bits in the CONF register are used to choose between an analog
 * ratiometric output (default) and a digital PWM output. If PWM is selected,
 * the DAC is powered down. Without regard to which output is enabled, an
 * external unit can read the angle from the ANGLE register through I2C
 * interface at any time.
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       output_stage            Value of type `as5600_output_stage_t`
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or value invalid
 */
as5600_error_t as5600_set_output_stage(as5600_output_stage_t const output_stage,
                                       as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_output_stage_t const fence = AS5600_OUTPUT_STAGE_COUNT;

        if ((NULL == p_config) || (fence <= output_stage)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->output_stage = output_stage;
        }

        return success;

}

/*!
 * @brief Get output stage
 *
 * Get output stage in a `as5600_configuration_t` object
 *
 * @see `as5600_set_output_stage`
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_output_stage          Pointer to object to save the
 *                                          output stage value at
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_get_output_stage(
                                  as5600_output_stage_t * const p_output_stage,
                                  as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_output_stage)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_output_stage = p_config->output_stage;
        }

        return success;

}

/*!
 * @brief Set PWM frequency
 *
 * Set PWM frequency in a `as5600_configuration_t` object
 *
 * The AS5600 output stage can be programmed in the OUTS bits of the CONF
 * register for a PWM-encoded digital output (OUTS = 10). In this mode, the OUT
 * pin provides a digital PWM signal. The duty cycle of each pulse is
 * proportional to the absolute angle of the rotating magnet.
 * The PWM signal consists of a frame of 4351 PWM clock periods.
 * This PWM frame is composed of the following sections:
 * • 128 PWM clock periods high
 * • 4095 PWM clock periods data
 * • 128 PWM clock periods low
 * The angle is represented in the data part of the frame, and one PWM clock
 * period represents one 4096th of the full angular range. The PWM frequency is
 * programmed with the PWMF bits in the CONF register.
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C write operation
 *
 * @param[in]       pwm_frequency           Value of type `as5600_pwm_frequency_t`
 *
 * @param[in/out]   p_config                Pointer to `as5600_configuration_t`
 *                                          object to be modified
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or value invalid
 */
as5600_error_t as5600_set_pwm_frequency(
                                     as5600_pwm_frequency_t const pwm_frequency,
                                     as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_pwm_frequency_t const fence = AS5600_PWM_FREQUENCY_COUNT;

        if ((NULL == p_config) || (fence <= pwm_frequency)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->pwm_frequency = pwm_frequency;
        }

        return success;

}

/*!
 * @brief Get PWM frequency
 *
 * Get PWM frequency in a `as5600_configuration_t` object
 *
 * @see `as5600_set_pwm_frequency`
 *
 * @note this function only modifies the `as5600_configuration_t` object and
 *       performs no I2C read operation
 *
 * @param[out]      p_output_stage          Pointer to object to save the pwm
 *                                          frequency value at
 *
 * @param[in]       p_config                Pointer to `as5600_configuration_t`
 *                                          object to be read
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
as5600_error_t as5600_get_pwm_frequency(
                                 as5600_pwm_frequency_t * const p_pwm_frequency,
                                 as5600_configuration_t const * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_pwm_frequency)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_pwm_frequency = p_config->pwm_frequency;
        }

        return success;

}

/*!
 * @brief Get RAW angle
 *
 * Get RAW angle by directly reading the corresponding device register
 *
 * The RAW ANGLE register contains the unscaled and unmodified angle. The scaled
 * output value is available in the ANGLE register.
 *
 * @note The ANGLE register has a 10-LSB hysteresis at the limit of the 360
 *       degree range to avoid discontinuity points or toggling of the output
 *       within one rotation.
 *
 * @param[out]      p_raw_angle             Pointer to object to save the raw
 *                                          angle value at
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_raw_angle(uint16_t * const p_raw_angle)
{
        as5600_register_t const reg = AS5600_REGISTER_RAWANGLE_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t buffer;

        if (NULL == p_raw_angle) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &buffer);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_raw_angle = buffer;
        }

        return success;
}

/*!
 * @brief Get angle
 *
 * Get scaled / modified output angle by directly reading the corresponding
 * device register
 *
 * @note The ANGLE register has a 10-LSB hysteresis at the limit of the 360
 *       degree range to avoid discontinuity points or toggling of the output
 *       within one rotation.
 *
 * @param[out]      p_angle                 Pointer to object to save the
 *                                          angle value at
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_angle(uint16_t * const p_angle)
{
        as5600_register_t const reg = AS5600_REGISTER_ANGLE_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t buffer;

        if (NULL == p_angle) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &buffer);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_angle = buffer;
        }

        return success;
}

/*!
 * @brief Get status of the device
 *
 * The STATUS register provides bits that indicate the current state of the
 * AS5600. The possible states are:
 * - MH, AGC minimum gain overflow, magnet too strong
 * - ML, AGC maximum gain overflow, magnet too weak
 * - MD, Magnet was detected
 *
 * @param[out]       p_status                Pointer to object to save the
 *                                           status value at
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_status(as5600_status_t * const p_status)
{
        as5600_register_t const reg = AS5600_REGISTER_STATUS;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        bool found = false;
        uint8_t reg_value;
        uint8_t i;

        if (NULL == p_status) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_8register(reg, &reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                for (i = 0; (m_available_status_len > i) && (!found); ++i) {
                        if (m_available_status[i] == reg_value) {
                                found = true;
                        }
                }

                if (!found) {
                        success = AS5600_ERROR_RUNTIME_ERROR;
                }
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_status = (as5600_status_t)reg_value;
        }

        return success;
}

/*!
 * @brief Get automatic gain control value
 *
 * The AS5600 uses Automatic Gain Control in a closed loop to compensate for
 * variations of the magnetic field strength due to changes of temperature,
 * airgap between IC and magnet, and magnet degradation. The AGC register
 * indicates the gain. For the most robust performance, the gain value should
 * be in the center of its range. The airgap of the physical system can be
 * adjusted to achieve this value.
 * In 5V operation, the AGC range is 0-255 counts. The AGC range is reduced
 * to 0-128 counts in 3.3V mode.
 *
 * @param[out]      p_agc                   Pointer to object to save the
 *                                          agc value at [0 - 255]
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_automatic_gain_control(uint8_t * const p_agc)
{
        as5600_register_t const reg = AS5600_REGISTER_AGC;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t reg_value;

        if (NULL == p_agc) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_8register(reg, &reg_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_agc = reg_value;
        }

        return success;
}

/*!
 * @brief Get cordic magnitude
 *
 * The MAGNITUDE register indicates the magnitude value of the internal CORDIC.
 *
 * The signals coming from the Hall sensors are first amplified and filtered
 * before being converted by the analog-to-digital converter (ADC). The output
 * of the ADC is processed by the hardwired CORDIC block (Coordinate Rotation
 * Digital Computer) to compute the angle and magnitude of the magnetic field
 * vector. The intensity of the magnetic field is used by the automatic gain
 * control (AGC) to adjust the amplification level to compensate for temperature
 * and magnetic field variations.
 * The angle value provided by the CORDIC algorithm is used by the output stage
 *
 * @param[out]      p_magnitude                 Pointer to object to save the
 *                                              cordic value at
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_get_cordic_magnitude(uint16_t * const p_magnitude)
{
        as5600_register_t const reg = AS5600_REGISTER_MAGNITUDE_H;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint16_t reg_val;

        if (NULL == p_magnitude) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_read_16register(reg, &reg_val);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_magnitude = reg_val;
        }

        return success;
}

/*!
 * @brief Send burn command to the device
 *
 * Angle Programming
 *
 * For applications which do not use the full 0 to 360 degree angular range, the
 * output resolution can be enhanced by programming the range which is actually
 * used. In this case, the full resolution of the output is automatically scaled
 * to the programmed angular range. The angular range must be greater than 18
 * degrees. The range is specified by programming a start position (ZPOS) and
 * either a stop position (MPOS) or the size of the angular range (MANG).
 *
 * 1- Power up the AS5600.
 * 2- Turn the magnet to the start position.
 * 3- Read the RAW ANGLE register. Write the RAW ANGLE value into the ZPOS
 *    register. Wait at least 1 ms.
 * 4- Rotate the magnet in the direction defined by the level on the DIR pin
 *    (GND for clockwise, VDD for counterclockwise) to the stop position.
 *    The amount of rotation must be greater than18 degrees.
 * 5- Read the RAW ANGLE register.
 *    Write the RAW ANGLE value into the MPOS register. Wait at least 1 ms.
 * 6- Perform a BURN_ANGLE command to permanently program the device.
 *    Wait at least 1 ms.
 *
 * Programming a Maximum Angular Range and default configuration settings
 *
 * 1- Power up the AS5600.
 * 2- Use the I2C interface to write the maximum angular range into the MANG
 *    register. For example, if the maximum angular range is 90 degrees, write
 *    the MANG register with 0x400. Configure additional configuration settings
 *    by writing the CONFIG register.Wait at least 1 ms.
 * 3- Perform a BURN_SETTINGS command to permanently program the device. Wait at
 *    least 1 ms.
 *
 * @warning This function performs irreversible changes to the IC and is NOT
 *          tested. Proceed at your own risk
 *
 * @warning This function may only be executed if the presence of the magnet is
 *          detected (MD = 1).
 *
 * @warning The BURN_ANGLE command can be executed up to 3 times.
 *
 * @warning MANG can be written only if ZPOS and MPOS have never been permanently
 *          written (ZMCO = 00).
 *
 * @warning The BURN_ SETTING command can be performed only one time.
 *
 * @warning Page 22 of the datasheet mentions writting 0x01, 0x11 and 0x10
 *          sequentially to the BURN register, in order to load the OTP values
 *          back, but this seems not to be properly documented. Proceed at your
 *          own risk
 *
 * @note ZMCO shows how many times ZPOS and MPOS have been permanently written.
 *
 * @see `as5600_get_otp_write_counter`
 *
 * @param[in]       mode                    Burn command to execute
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
as5600_error_t as5600_burn_command(as5600_burn_mode_t const mode)
{
        as5600_register_t const reg = AS5600_REGISTER_BURN;
        uint8_t const burn_setting_max_writes = 1;
        uint8_t const burn_angle_max_writes = 3;
        uint8_t load_sequence[] = {0x01U, 0x11U, 0x10U};

        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_status_t status;
        uint8_t counter;
        uint16_t start_pos;
        uint16_t stop_pos;
        uint16_t max_angle;
        bool min_angle_ok;

        if (AS5600_BURN_MODE_COUNT <= mode) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_get_otp_write_counter(&counter);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_get_status(&status);
        }

        if (AS5600_ERROR_SUCCESS != success) {
                // Code style exception for readability;
                return success;
        }

        switch (mode) {
        case AS5600_BURN_MODE_BURN_ANGLE:

                // Abort if maximum write cycles reached or magnet not detected
                if (burn_angle_max_writes <= counter) {
                        success = AS5600_ERROR_MAX_WRITE_CYCLES_REACHED;
                } else if (AS5600_STATUS_MD != status) {
                        success = AS5600_ERROR_MAGNET_NOT_DETECTED;
                }

                // Check for resulting angle to be over minimum angle
                if (AS5600_ERROR_SUCCESS == success) {
                        success = as5600_get_start_position(&start_pos);
                }

                if (AS5600_ERROR_SUCCESS == success) {
                        success = as5600_get_stop_position(&stop_pos);
                }

                if (AS5600_ERROR_SUCCESS == success) {

                        // XOR Swap for avoiding negative overflow
                        if (start_pos > stop_pos)
                        {
                                stop_pos  ^= start_pos;
                                start_pos ^= stop_pos;
                                stop_pos  ^= start_pos;
                        }

                        min_angle_ok = (m_min_angle_steps <=
                                        (stop_pos - start_pos));
                }

                break;

        case AS5600_BURN_MODE_BURN_SETTING:

                // Abort if maximum write cycles reached
                if (burn_setting_max_writes <= counter) {
                        success = AS5600_ERROR_MAX_WRITE_CYCLES_REACHED;
                }

                // Check for resulting angle to be over minimum angle
                if (AS5600_ERROR_SUCCESS == success) {
                        success = as5600_get_maximum_angle(&max_angle);
                }

                if (AS5600_ERROR_SUCCESS == success) {
                        min_angle_ok = (m_min_angle_steps <= max_angle);
                }

                break;

        default:
                success = AS5600_ERROR_BAD_PARAMETER;
                break;

        }

        // Abort if minimum angle not being fullfiled
        if ((AS5600_ERROR_SUCCESS == success) && (!min_angle_ok)) {
                success = AS5600_ERROR_MIN_ANGLE_TOO_SMALL;
        }

        // Burn the settings in OTP
        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_8register(reg, mode);
        }

        // Load OTP content
        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_8register(reg, load_sequence[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_8register(reg, load_sequence[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_8register(reg, load_sequence[2]);
        }

        return success;

}

/*
 *******************************************************************************
 * Private Function Bodies                                                     *
 *******************************************************************************
 */

/*!
 * @brief Read 8-bit register
 *
 * Read 8-bit register of given address in a given buffer
 *
 * @param[in]       reg                     Address of the register to read from
 * @param[out]      p_rx_buffer             Pointer to buffer to save the
 *                                          register value at
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
static as5600_error_t as5600_read_8register(as5600_register_t const reg,
                                            uint8_t * const p_rx_buffer)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        size_t const count = sizeof(uint8_t);

        if (NULL == p_rx_buffer) {
                result = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == result) {
                result = as5600_read_n_consecutive_bytes(reg,
                                                         p_rx_buffer, count);
        }

        return result;

}

/*!
 * @brief Write 8-bit register
 *
 * Write a given buffer to a 8-bit register of a given address
 *
 * @param[in]       reg                     Address of the register to write to
 * @param[in]       tx_buffer               Buffer with data to write to the
 *                                          register
 *
 * @return          as5600_error_t          Result of the operation
 * @retval          AS5600_ERROR_SUCCESS    If everything went well
 * @retval          *                       Any other errors returned by the
 *                                          sub-callees
 */
static as5600_error_t as5600_write_8register(as5600_register_t const reg,
                                             uint8_t const tx_buffer)
{
        as5600_error_t success;
        uint8_t const buffer = tx_buffer;
        size_t const count = sizeof(uint8_t);

        success = as5600_write_n_consecutive_bytes(reg, &buffer, count);

        return success;
}

/*!
 * @brief Read 16-bit register
 *
 * Read 16-bit register of given address in a given buffer
 *
 * @param[in]       reg                     Lowest byte address of the register
 *                                          to read from
 * @param[out]      p_rx_buffer             Pointer to buffer to save the
 *                                          register value at.
 *
 * @note User should make sure that the buffer is of the right size
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
static as5600_error_t as5600_read_16register(as5600_register_t const reg,
                                             uint16_t * const p_rx_buffer)
{
        size_t const count = sizeof(uint16_t);
        uint8_t buffer[count];
        as5600_error_t result = AS5600_ERROR_SUCCESS;

        if (NULL == p_rx_buffer) {
                result = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == result) {
                result = as5600_read_n_consecutive_bytes(reg,
                                                         buffer, count);
        }

        if (AS5600_ERROR_SUCCESS == result) {
                *p_rx_buffer = ((uint16_t)buffer[0] << 8) | ((uint16_t)buffer[1]);
        }

        return result;

}


/*!
 * @brief Write 16-bit register
 *
 * Write a given buffer to a 16-bit register of a given address
 *
 * @param[in]       reg                     Address of the register to write to
 * @param[in]       tx_buffer               Buffer with data to write to the
 *                                          register
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
static as5600_error_t as5600_write_16register(as5600_register_t const reg,
                                              uint16_t const tx_buffer)
{
        uint16_t const first_byte_mask = 0x00FF;
        size_t const count = sizeof(uint16_t);
        as5600_error_t success;
        uint8_t buffer[2];

        buffer[0] = (uint8_t)((tx_buffer >> 8) & first_byte_mask);
        buffer[1] = (uint8_t)(tx_buffer & first_byte_mask);

        success = as5600_write_n_consecutive_bytes(reg, buffer, count);

        return success;
}

/*!
 * @brief Read n consecutive bytes from a register
 *
 * Read n consecutive bytes from a register of a given address
 *
 * @param[in]       reg                     Lowest byte address of the register
 *                                          to read from
 * @param[out]      p_rx_buffer             Pointer to buffer to save the
 *                                          register value at.
 * @param[in]       bytes_count             Number of bytes to read from the
 *                                          register.
 *
 * @note User should make sure that the buffer is of the right size
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or register invalid
 * @retval          AS5600_ERROR_GENERAL_ERROR  If transfer function pointer is null
 * @retval          AS5600_ERROR_NOT_INITIALIZED If module wasn't initialized
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
static as5600_error_t as5600_read_n_consecutive_bytes(
                                                    as5600_register_t const reg,
                                                    uint8_t * const p_rx_buffer,
                                                    size_t const bytes_count)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        uint8_t const reg_addr = (uint8_t)reg;
        uint8_t xfer_func_result;

        if ((NULL == p_rx_buffer) || (!as5600_is_register_valid(reg))) {
                result = AS5600_ERROR_BAD_PARAMETER;
        } else if (!m_is_initialized) {
                result = AS5600_ERROR_NOT_INITIALIZED;
        } else if (NULL == m_as5600_xfer_func) {
                result = AS5600_ERROR_GENERAL_ERROR;
        }

        if (AS5600_ERROR_SUCCESS == result) {


                xfer_func_result = m_as5600_xfer_func(m_as5600_i2c_addr,
                                                      &reg_addr,
                                                      sizeof(reg_addr),
                                                      p_rx_buffer, bytes_count);

                if (0 != xfer_func_result) {
                        result = AS5600_ERROR_I2C_ERROR;
                }
        }

        return result;
}

/*!
 * @brief Write n consecutive bytes to a register
 *
 * Write n consecutive bytes to a register of a given address
 *
 * @param[in]       reg                     Lowest byte address of the register
 *                                          to write to
 * @param[out]      p_tx_buffer             Pointer to buffer to read the data
 *                                          from
 * @param[in]       bytes_count             Number of bytes to write from the
 *                                          register.
 *
 * @note User should make sure that the buffer is of the right size
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or register invalid
 * @retval          AS5600_ERROR_GENERAL_ERROR  If transfer function pointer is null
 * @retval          AS5600_ERROR_NOT_INITIALIZED If module wasn't initialized
 * @retval          *                           Any other errors returned by the
 *                                              sub-callees
 */
static as5600_error_t as5600_write_n_consecutive_bytes(
                                              as5600_register_t const reg,
                                              uint8_t const * const p_tx_buffer,
                                              size_t const bytes_count)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        uint8_t buffer[bytes_count + 1];
        uint8_t xfer_func_result;
        uint8_t const reg_addr = (uint8_t)reg;
        uint8_t i;

        if ((NULL == p_tx_buffer) || (!as5600_is_register_valid(reg))) {
                result = AS5600_ERROR_BAD_PARAMETER;
        } else if (!m_is_initialized) {
                result = AS5600_ERROR_NOT_INITIALIZED;
        } else if (NULL == m_as5600_xfer_func) {
                result = AS5600_ERROR_GENERAL_ERROR;
        }

        if (AS5600_ERROR_SUCCESS == result) {

                buffer[0] = reg_addr;
                for (i = 0; bytes_count > i; ++i) {
                        buffer[i + 1] = p_tx_buffer[i];
                }

                xfer_func_result = m_as5600_xfer_func(m_as5600_i2c_addr,
                                                      buffer,
                                                      sizeof(buffer),
                                                      NULL, 0);

                if (0 != xfer_func_result) {
                        result = AS5600_ERROR_I2C_ERROR;
                }
        }

        return result;
}

/*!
 * @brief Set bit field value from a given register
 *
 * Set a value to the given bit field of a given register
 *
 * @param[in]       value                   Value to set
 *
 * @param[in]       bit_field               Bit field to modify
 *
 * @param[in/out]   p_reg_value             Pointer to the register value to
 *                                          modify.
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
static as5600_error_t as5600_reg_set_bit_field_value(
                                             uint8_t const value,
                                             as5600_bit_field_t const bit_field,
                                             uint8_t * const p_reg_value)
{
        as5600_bit_field_specs_t const specs = m_bitfields[bit_field];
        uint8_t const bit_field_lsb = specs.lsbit_pos;
        uint8_t const bit_field_width = specs.width;
        uint8_t const max_value = ((1 << bit_field_width) - 1);

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        uint8_t temp;
        uint8_t mask = 0;

        if ((NULL == p_reg_value) || (max_value < value) ||
            (AS5600_BIT_FIELD_COUNT <= bit_field)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {

                mask |= ((1 << bit_field_width) - 1);
                mask <<= bit_field_lsb;

                temp = *p_reg_value;
                temp &= mask;
                temp |= (value << bit_field_lsb);

                *p_reg_value |= temp;
        }

        return success;
}

/*!
 * @brief Get value from bit field
 *
 * Get the value from a given register and bit field
 *
 * @param[out]      p_value                 Pointer to object where to store the
 *                                          value to
 *
 * @param[in]       bit_field               Bit field to read
 *
 * @param[in]       p_reg_value             Register value to read from
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
 static as5600_error_t as5600_reg_get_bit_field_value(
                                             uint8_t * const p_value,
                                             as5600_bit_field_t const bit_field,
                                             uint8_t const reg_value)
{
        as5600_bit_field_specs_t const specs = m_bitfields[bit_field];
        uint8_t const bit_field_width = specs.width;
        uint8_t const bit_field_lsb = specs.lsbit_pos;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t temp = reg_value;
        uint8_t mask;

        if ((NULL == p_value) || (AS5600_BIT_FIELD_COUNT <= bit_field)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                mask = ((1 << bit_field_width) - 1);
                temp >>= bit_field_lsb;
                temp &= mask;
                *p_value = temp;
        }

        return success;

}

/*!
 * @brief Configuration structure to 16-bit register value
 *
 * This function takes a `as5600_configuration_t` structure object and converts
 * it to the corresponding 16-bit register value
 *
 * @param[in]       p_config                Pointer to the configuration object
 *                                          to read from for the conversion
 *
 * @param[out]      value                   Pointer to object where to save the
 *                                          the resulting register value
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer or configuration invalid
 */
static as5600_error_t as5600_cfg_to_reg16(
                                  as5600_configuration_t const * const p_config,
                                  uint16_t * const reg)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        uint8_t buffer[2] = {0};
        uint8_t field_val;

        if ((NULL == p_config) || (NULL == reg) ||
            (!as5600_is_valid_configuration(p_config))) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->power_mode);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_PM,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->hysteresis);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_HYST,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->output_stage);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_OUTS,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->pwm_frequency);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_PWMF,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->slow_filter);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_SF,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (uint8_t)(p_config->ff_threshold);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_FTH,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {

                field_val = (p_config->watchdog ? 0x01 : 0x00);
                success = as5600_reg_set_bit_field_value(field_val,
                                                         AS5600_BIT_FIELD_WD,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *reg = 0U;
                *reg |= (buffer[0] << 8);
                *reg |= buffer[1];
        }

        return success;
}

/*!
 * @brief 16-bit register value to configuration structure
 *
 * This function takes a 16-bit register value and converts it to the
 * corresponding `as5600_configuration_t` structure object
 *
 * @param[in]       p_reg                   Pointer to the register value to
 *                                          convert from
 *
 * @param[out]      p_config                Pointer to a configuration object
 *                                          where to store the resulting
 *                                          structure
 *
 * @return          as5600_error_t              Result of the operation
 * @retval          AS5600_ERROR_SUCCESS        If everything went well
 * @retval          AS5600_ERROR_BAD_PARAMETER  Pointer invalid
 */
static as5600_error_t as5600_reg16_to_cfg(
                                        uint16_t const * const p_reg,
                                        as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t reg_buffer[2];

        uint8_t power_mode;
        uint8_t hysteresis;
        uint8_t output_stage;
        uint8_t pwm_frequency;
        uint8_t slow_filter;
        uint8_t ff_threshold;
        uint8_t watchdog;

        if ((NULL == p_reg) || (NULL == p_config)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                reg_buffer[0] = (uint8_t)(*p_reg >> 8);
                reg_buffer[1] = (uint8_t)(*p_reg & 0x00FF);

                success = as5600_reg_get_bit_field_value(&power_mode,
                                                         AS5600_BIT_FIELD_PM,
                                                         reg_buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&hysteresis,
                                                         AS5600_BIT_FIELD_HYST,
                                                         reg_buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&output_stage,
                                                         AS5600_BIT_FIELD_OUTS,
                                                         reg_buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&pwm_frequency,
                                                         AS5600_BIT_FIELD_PWMF,
                                                         reg_buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&slow_filter,
                                                         AS5600_BIT_FIELD_SF,
                                                         reg_buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&ff_threshold,
                                                         AS5600_BIT_FIELD_FTH,
                                                         reg_buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_get_bit_field_value(&watchdog,
                                                         AS5600_BIT_FIELD_WD,
                                                         reg_buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->power_mode = (as5600_power_mode_t) power_mode;
                p_config->hysteresis = (as5600_hysteresis_t) hysteresis;
                p_config->output_stage = (as5600_output_stage_t) output_stage;
                p_config->pwm_frequency = (as5600_pwm_frequency_t) pwm_frequency;
                p_config->slow_filter = (as5600_slow_filter_t) slow_filter;
                p_config->ff_threshold = (as5600_ff_threshold_t) ff_threshold;
                p_config->watchdog = (bool) watchdog;
        }

        return success;
}

/*!
 * @brief Check if configuration is valid
 *
 * This function takes a pointer to a `as5600_configuration_t` object and checks
 * if the values of its members make sense
 *
 * @param[in]           p_config            Pointer to the configuration object
 *                                          to be checked
 *
 * @return              bool                Result of the operation
 * @retval              True                Valid configuration
 * @retval              False               Invalid configuration
 */
static bool as5600_is_valid_configuration(
                                  as5600_configuration_t const * const p_config)
{
        return ((NULL != p_config) &&
                (AS5600_POWER_MODE_COUNT > p_config->power_mode) &&
                (AS5600_HYSTERESIS_COUNT > p_config->hysteresis) &&
                (AS5600_OUTPUT_STAGE_COUNT > p_config->output_stage) &&
                (AS5600_PWM_FREQUENCY_COUNT > p_config->pwm_frequency) &&
                (AS5600_SLOW_FILTER_COUNT > p_config->slow_filter) &&
                (AS5600_FF_THRESHOLD_COUNT > p_config->ff_threshold));
}

/*!
 * @brief Check if register is valid
 *
 * This function takes a `as5600_register_t` object and checks
 * if its value exists.
 *
 * @param[in]           reg                 Register object to be checked
 *
 * @return              bool                Result of the operation
 * @retval              True                Valid register
 * @retval              False               Invalid register
 */
static bool as5600_is_register_valid(as5600_register_t const reg)
{
        bool is_valid;
 
        switch (reg) {
        case AS5600_REGISTER_ZMCO:
        case AS5600_REGISTER_ZPOS_H:
        case AS5600_REGISTER_ZPOS_L:
        case AS5600_REGISTER_MPOS_H:
        case AS5600_REGISTER_MPOS_L:
        case AS5600_REGISTER_MANG_H:
        case AS5600_REGISTER_MANG_L:
        case AS5600_REGISTER_CONF_H:
        case AS5600_REGISTER_CONF_L:
        case AS5600_REGISTER_STATUS:
        case AS5600_REGISTER_RAWANGLE_H:
        case AS5600_REGISTER_RAWANGLE_L:
        case AS5600_REGISTER_ANGLE_H:
        case AS5600_REGISTER_ANGLE_L:
        case AS5600_REGISTER_AGC:
        case AS5600_REGISTER_MAGNITUDE_H:
        case AS5600_REGISTER_MAGNITUDE_L:
        case AS5600_REGISTER_BURN:
                is_valid = true;
                break;
        default:
                is_valid = false;
                break;
        }
        return is_valid;
}

/*
 *******************************************************************************
 * Interrupt Service Routines / Tasks / Thread Main Functions                  *
 *******************************************************************************
 */

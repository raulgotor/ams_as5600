/*!
 *******************************************************************************
 * @file as5600.c
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

#define AS5600_REG_ADDR_ZMCO                    0x00U

#define AS5600_REG_ADDR_ZPOS_HI_BYTE            0x01U

#define AS5600_REG_ADDR_ZPOS_LO_BYTE            0x02U

#define AS5600_REG_ADDR_MPOS_HI_BYTE            0x03U

#define AS5600_REG_ADDR_MPOS_LO_BYTE            0x04U

#define AS5600_REG_ADDR_MANG_HI_BYTE            0x05U

#define AS5600_REG_ADDR_MANG_LO_BYTE            0x06U

#define AS5600_REG_ADDR_CONF_HI_BYTE            0x07U

#define AS5600_REG_ADDR_CONF_LO_BYTE            0x08U

#define AS5600_REG_ADDR_RAWANGLE_HI_BYTE        0x0CU

#define AS5600_REG_ADDR_RAWANGLE_LO_BYTE        0x0DU

#define AS5600_REG_ADDR_ANGLE_HI_BYTE           0x0EU

#define AS5600_REG_ADDR_ANGLE_LO_BYTE           0x0FU

#define AS5600_REG_ADDR_STATUS                  0x0BU

#define AS5600_REG_ADDR_AGC                     0x1AU

#define AS5600_REG_ADDR_MAGNITUDE_HI_BYTE       0x1BU

#define AS5600_REG_ADDR_MAGNITUDE_LO_BYTE       0x1CU

#define AS5600_REG_ADDR_BURN                    0xFFU

/*
 *******************************************************************************
 * Data types                                                                  *
 *******************************************************************************
 */

typedef struct {
        as5600_register_t reg;
        uint8_t lsbit_pos;
        uint8_t width;
} as5600_bit_field_specs_t;

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
        AS5600_BIT_FIELD_STATUS,
        AS5600_BIT_FIELD_RAWANGLE_HI_BYTE,
        AS5600_BIT_FIELD_RAWANGLE_LO_BYTE,
        AS5600_BIT_FIELD_ANGLE_HI_BYTE,
        AS5600_BIT_FIELD_ANGLE_LO_BYTE,
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

static const pf_i2c_xfer_as5600_t m_as5600_xfer_func = NULL;

static bool m_is_initialized = false;

static const uint8_t m_instance_count = 1;
static const uint8_t m_instance_addr[m_instance_count] = { 0x36 };

as5600_bit_field_specs_t fields[] = {
        {
                .reg = AS5600_REGISTER_ZMCO,
                .lsbit_pos = 0,
                .width = 2
        },
        {
                .reg = AS5600_REGISTER_ZPOS_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_ZPOS_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_MPOS_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_MPOS_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_MANG_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_MANG_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {       //SF
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 0,
                .width = 2,
        },
        {       //FTH
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 2,
                .width = 3
        },
        {       //WD
                .reg = AS5600_REGISTER_CONF_H,
                .lsbit_pos = 5,
                .width = 1
        },
        {       //PM
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 0,
                .width = 2
        },
        {       //HYST
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 2,
                .width = 2
        },
        {       //OUT
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 4,
                .width = 2
        },
        {       //PWMF
                .reg = AS5600_REGISTER_CONF_L,
                .lsbit_pos = 6,
                .width = 2
        },
        {
                .reg = AS5600_REGISTER_RAWANGLE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_RAWANGLE_H,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_ANGLE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_ANGLE_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_STATUS,
                .lsbit_pos = 3,
                .width = 3
        },
        {
                .reg = AS5600_REGISTER_AGC,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_MAGNITUDE_H,
                .lsbit_pos = 0,
                .width = 4
        },
        {
                .reg = AS5600_REGISTER_MAGNITUDE_L,
                .lsbit_pos = 0,
                .width = 8
        },
        {
                .reg = AS5600_REGISTER_BURN,
                .lsbit_pos = 0,
                .width = 8
        },

};

/*
 *******************************************************************************
 * Private Function Prototypes                                                 *
 *******************************************************************************
 */

static as5600_error_t as5600_get_field_value(as5600_bit_field_t const field,
                                             uint8_t * const p_field_value);

static as5600_error_t as5600_set_field_value(as5600_bit_field_t const field,
                                             uint8_t const field_value);

static as5600_error_t as5600_read_n_consecutive_bytes(uint8_t const reg, uint8_t * const p_rx_buffer, size_t const bytes_count);

static as5600_error_t as5600_write_n_consecutive_bytes(uint8_t const reg, uint8_t const * const p_tx_buffer, size_t const bytes_count);

static as5600_error_t as5600_read_16register(uint8_t const reg, uint16_t * const p_rx_buffer);

static as5600_error_t as5600_write_16register(uint8_t const reg, uint16_t const tx_buffer);

static as5600_error_t as5600_write_8register(uint8_t const instance,
                                             uint8_t const reg,
                                             uint8_t const * const p_tx_buffer);


static as5600_error_t as5600_read_8register(uint8_t const instance,
                                            uint8_t const reg,
                                            uint8_t * const p_tx_buffer);

static as5600_error_t as5600_set_conf_bit_field(uint8_t const start_bit, uint8_t const width, uint8_t const value);

static as5600_error_t as5600_get_conf_bitfield(uint8_t const instance,
                                               uint8_t const start_bit,
                                               uint8_t const width,
                                               uint8_t * const value);

static as5600_error_t as5600_reg_set_bit_field_value(uint8_t const value,
                                                     as5600_bit_field_t const bit_field,
                                                     uint8_t * const p_reg_value);

static as5600_error_t as5600_reg_get_bit_field_value(uint8_t * const value,
                                                     as5600_bit_field_t const bit_field,
                                                     uint8_t const reg_value);

static as5600_error_t as5600_cfg_to_reg16(
        as5600_configuration_t const * const p_config,
        uint16_t * const reg);

static as5600_error_t as5600_reg16_to_cfg(uint16_t const * const reg,
                                          as5600_configuration_t * const p_config);

static bool as5600_is_valid_configuration(
                                 as5600_configuration_t const * const p_config);
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

as5600_error_t as5600_init(pf_i2c_xfer_as5600_t const pf_xfer_function)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;

        if (m_is_initialized) {
                result = AS5600_ERROR_RUNTIME_ERROR;
        } else if (NULL == pf_xfer_function) {
                result = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == result) {
                m_is_initialized = true;
        }

        return result;
}

as5600_error_t as5600_get_start_and_stop_position_write_counter(
                                                      uint8_t * p_write_counter)
{
        as5600_bit_field_t const field = AS5600_BIT_FIELD_ZMCO;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t field_value;

        if (NULL == p_write_counter) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_get_field_value(field, &field_value);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_write_counter = field_value;
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
 * (e.g. 0° to 360°(full-turn) → 4096dec; 0° to180°→2048dec).
 *
 * @param       start_position              start position value [0 - 4095]
 *
 * @return      as5600_error_t              Result of the operation
 * @retval      AS5600_ERROR_SUCCESS        If everything went well
 * @retval      AS5600_ERROR_BAD_PARAMETER  Parameter OOR
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

as5600_error_t as5600_set_maximum_angle(uint16_t const max_angle)
{
        as5600_register_t const reg = AS5600_REGISTER_MANG_H;
        uint16_t const reg_mask = 0x0FFFU;
        uint16_t const reg_value = max_angle;

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if (reg_mask < reg_value) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_16register(reg, reg_value);
        }

        return success;

}

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

as5600_error_t as5600_set_configuration(as5600_configuration_t const * const p_config)
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

as5600_error_t as5600_get_power_mode(as5600_power_mode_t * const p_power_mode,
                                     as5600_configuration_t * const p_config)
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

as5600_error_t as5600_get_hysteresis(as5600_hysteresis_t * const p_hysteresis,
                                     as5600_configuration_t * const p_config)
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

as5600_error_t as5600_set_output_state(as5600_output_stage_t const output_stage,
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

as5600_error_t as5600_get_output_stage(as5600_output_stage_t * const p_output_stage,
                                       as5600_configuration_t * const p_config)
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

as5600_error_t as5600_set_pwm_frequency(as5600_pwm_frequency_t const pwm_frequency,
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

as5600_error_t as5600_get_pwm_frequency(as5600_pwm_frequency_t * const p_pwm_frequency,
                                        as5600_configuration_t * const p_config)
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

as5600_error_t as5600_get_slow_filter(as5600_slow_filter_t * const p_slow_filter,
                                      as5600_configuration_t * const p_config)
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

as5600_error_t as5600_get_ff_threshold(as5600_ff_threshold_t * const p_ff_threshold,
                                       as5600_configuration_t * const p_config)
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

as5600_error_t as5600_set_watchdog_enabled(bool const enabled,
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

as5600_error_t as5600_is_watchdog_enabled(bool * const p_enabled,
                                          as5600_configuration_t * const p_config)
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

/*
as5600_error_t as5600_set_x(as5600_x_t const x,
                                     as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        as5600_x_t const fence = AS5600_x_COUNT;

        if ((NULL == p_config) || (fence <= x)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                p_config->x = x;
        }

        return success;

}

as5600_error_t as5600_get_x(as5600_x_t * const p_x,
                                     as5600_configuration_t * const p_config)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        if ((NULL == p_config) || (NULL == p_x)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *p_x = p_config->x;
        }

        return success;

}
 */


/*
 *******************************************************************************
 * Private Function Bodies                                                     *
 *******************************************************************************
 */


static as5600_error_t as5600_set_conf_bit_field(uint8_t const start_bit,
                                                uint8_t const width,
                                                uint8_t const value)
{
        uint8_t const start_register = AS5600_REGISTER_CONF_H;
        uint16_t const reserved_mask = 0x3FFFU;

        uint16_t cfg;
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        uint8_t bit_field_mask = (1 << width);
        bit_field_mask = bit_field_mask << start_bit;

        if (value != (bit_field_mask | value)) {
                result = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == result) {
                result = as5600_read_16register(start_register, &cfg);
        }

        if (AS5600_ERROR_SUCCESS == result) {
                cfg &= reserved_mask;
                cfg &= bit_field_mask;
                cfg |= ((((uint16_t)value) << start_bit) & bit_field_mask);
                result = as5600_write_16register(start_register, cfg);
        }

        return result;

}

static as5600_error_t as5600_read_8register(uint8_t const instance,
                                            uint8_t const reg,
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

static as5600_error_t as5600_write_8register(uint8_t const instance,
                                             uint8_t const reg,
                                             uint8_t const * const p_tx_buffer)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        size_t const count = sizeof(uint8_t);

        if (NULL == p_tx_buffer) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_write_n_consecutive_bytes(reg,
                                                           p_tx_buffer, count);
        }

        return success;

}

static as5600_error_t as5600_read_16register(uint8_t const reg, uint16_t * const p_rx_buffer)
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

static as5600_error_t as5600_write_16register(uint8_t const reg, uint16_t const tx_buffer)
{ return 0;}


static as5600_error_t as5600_write_n_consecutive_bytes(uint8_t const reg, uint8_t const * const p_tx_buffer, size_t const bytes_count)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        uint8_t addr;
        uint8_t xfer_func_result = 0;
        uint8_t reg_addr;
        uint8_t buffer[bytes_count + 1];
        uint8_t i;

        if ((NULL == p_tx_buffer) ||
            (AS5600_REGISTER_COUNT < reg)) {

                result = AS5600_ERROR_BAD_PARAMETER;
        } else if (!m_is_initialized) {
                result = AS5600_ERROR_NOT_INITIALIZED;
        } else if (NULL == m_as5600_xfer_func) {
                result = AS5600_ERROR_GENERAL_ERROR;
        }

        if (AS5600_ERROR_SUCCESS == result) {

                addr = m_instance_addr[instance];
                reg_addr = m_as5600_reg_addr[reg];
                buffer[0] = reg_addr;
                for (i = 0; bytes_count > i; ++i) {
                        buffer[i + 1] = p_tx_buffer[i];
                }

                xfer_func_result = m_as5600_xfer_func(addr,
                                                      buffer,
                                                      sizeof(buffer),
                                                      NULL, 0);

                if (0 != xfer_func_result) {
                        result = AS5600_ERROR_I2C_ERROR;
                }
        }



        return result;
}

static as5600_error_t as5600_read_n_consecutive_bytes(uint8_t const reg, uint8_t * const p_rx_buffer, size_t const bytes_count)
{
        as5600_error_t result = AS5600_ERROR_SUCCESS;
        uint8_t addr;
        uint8_t xfer_func_result = 0;
        uint8_t reg_addr;

        if ((NULL == p_rx_buffer) ||
            (AS5600_REGISTER_COUNT < reg)) {

                result = AS5600_ERROR_BAD_PARAMETER;
        } else if (!m_is_initialized) {
                result = AS5600_ERROR_NOT_INITIALIZED;
        } else if (NULL == m_as5600_xfer_func) {
                result = AS5600_ERROR_GENERAL_ERROR;
        }

        if (AS5600_ERROR_SUCCESS == result) {

                addr = m_instance_addr[instance];
                reg_addr = m_as5600_reg_addr[reg];

                xfer_func_result = m_as5600_xfer_func(addr,
                                                      &reg_addr,
                                                      sizeof(reg_addr),
                                                      p_rx_buffer, bytes_count);

                if (0 != xfer_func_result) {
                        result = AS5600_ERROR_I2C_ERROR;
                }
        }



        return result;
}

static as5600_error_t as5600_reg_set_bit_field_value(uint8_t const value,
                                                     as5600_bit_field_t const bit_field,
                                                     uint8_t * const p_reg_value)
{
        as5600_bit_field_specs_t const specs = fields[bit_field];
        uint8_t const bit_field_lsb = specs.lsbit_pos;
        uint8_t const bit_field_width = specs.width;
        uint8_t const max_value = ((1 << bit_field_width) - 1);

        as5600_error_t success = AS5600_ERROR_SUCCESS;

        uint8_t temp;
        uint8_t mask;

        if ((NULL == p_reg_value) || (max_value < *p_reg_value) ||
            (AS5600_BIT_FIELD_COUNT <= bit_field)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {

                mask &= ((1 << bit_field_width) - 1);
                mask <<= bit_field_lsb;
                mask = ~mask;

                temp = *p_reg_value;
                temp &= mask;
                temp &= (value << bit_field_lsb);

                *p_reg_value = temp;
        }

        return success;
}

static as5600_error_t as5600_reg_get_bit_field_value(uint8_t * const value,
                                                     as5600_bit_field_t const bit_field,
                                                     uint8_t const reg_value)
{
        as5600_bit_field_specs_t const specs = fields[bit_field];
        uint8_t const bit_field_width = specs.width;
        uint8_t const bit_field_lsb = specs.lsbit_pos;
        as5600_error_t success = AS5600_ERROR_SUCCESS;
        uint8_t temp = reg_value;
        uint8_t mask;

        if ((NULL == value) || (AS5600_BIT_FIELD_COUNT <= bit_field)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                mask = ((1 << bit_field_width) - 1);
                temp >>= bit_field_lsb;
                temp &= mask;
                *value = temp;
        }

        return success;

}

static as5600_error_t as5600_cfg_to_reg16(
                                  as5600_configuration_t const * const p_config,
                                  uint16_t * const reg)
{
        as5600_error_t success = AS5600_ERROR_SUCCESS;

        uint8_t buffer[2] = {0};

        if ((NULL == p_config) || (NULL == reg) ||
            (!as5600_is_valid_configuration(p_config))) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->power_mode,
                                                         AS5600_BIT_FIELD_PM,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->hysteresis,
                                                         AS5600_BIT_FIELD_HYST,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->output_stage,
                                                         AS5600_BIT_FIELD_OUTS,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->pwm_frequency,
                                                         AS5600_BIT_FIELD_PWMF,
                                                         &buffer[1]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->slow_filter,
                                                         AS5600_BIT_FIELD_SF,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->ff_threshold,
                                                         AS5600_BIT_FIELD_FTH,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                success = as5600_reg_set_bit_field_value(p_config->watchdog,
                                                         AS5600_BIT_FIELD_WD,
                                                         &buffer[0]);
        }

        if (AS5600_ERROR_SUCCESS == success) {
                *reg &= (buffer[0] << 8);
                *reg &= buffer[1];
        }

        return success;
}

static as5600_error_t as5600_reg16_to_cfg(
                                        uint16_t const * const reg,
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

        if ((NULL == reg) || (NULL == p_config)) {
                success = AS5600_ERROR_BAD_PARAMETER;
        }

        if (AS5600_ERROR_SUCCESS == success) {
                reg_buffer[0] = (uint8_t)(*reg >> 8);
                reg_buffer[1] = (uint8_t)(*reg & 0x00FF);
        }

        if (AS5600_ERROR_SUCCESS == success) {
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
                p_config->power_mode = (uint8_t) power_mode;
                p_config->hysteresis = (uint8_t) hysteresis;
                p_config->output_stage = (uint8_t) output_stage;
                p_config->pwm_frequency = (uint8_t) pwm_frequency;
                p_config->slow_filter = (uint8_t) slow_filter;
                p_config->ff_threshold = (uint8_t) ff_threshold;
                p_config->watchdog = (bool) watchdog;
        }

        return success;
}

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


/*
 *******************************************************************************
 * Interrupt Service Routines / Tasks / Thread Main Functions                  *
 *******************************************************************************
 */

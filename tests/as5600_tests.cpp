/*!
 *******************************************************************************
 * @file as5600_tests.cpp
 *
 * @brief 
 *
 * @author Raúl Gotor (raulgotor@gmail.com)
 * @date 09.01.22
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

#include "CppUTest/TestHarness.h"
#include "as5600.c"
#include "as5600_mocks.cpp"

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

/*
 *******************************************************************************
 * Constants                                                                   *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Private Function Prototypes                                                 *
 *******************************************************************************
 */

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

//! @brief A valid value for ZPOS and MPOS registers
static uint16_t const m_valid_start_stop_position = 1100;

//! @brief A valid value for MANG register
static uint16_t const m_valid_maximum_angle = 1000;

//! @brief First valid value for MANG register
static uint16_t const m_lowest_valid_maximum_angle = 206;

//! @brief A valid value for RAW_ANGLE register
static uint16_t const m_valid_raw_angle = 900;

//! @brief A valid value for AGC register
static uint8_t m_valid_agc_value = 128;

//! @brief A valid value for MAGNITUDE register
static uint16_t m_valid_magnitude = 128;

//! @brief First out of bounds value for ZPOS and MPOS registers
static uint16_t const m_oor_start_stop_position = 4096;

//! @brief First out of bounds value for MANG on the low end
static uint16_t const m_oor_l_maximum_angle = 205;

//! @brief First out of bounds value for MANG register on the high end
static uint16_t const m_oor_h_maximum_angle = 4096;

//! @brief A valid configuration for CONF register
static as5600_configuration_t const m_valid_configuration =
                {
                        .power_mode = AS5600_POWER_MODE_LPM2,
                        .hysteresis = AS5600_HYSTERESIS_2LSB,
                        .output_stage = AS5600_OUTPUT_STAGE_ANALOG_FR,
                        .pwm_frequency = AS5600_PWM_FREQUENCY_115HZ,
                        .slow_filter = AS5600_SLOW_FILTER_8X,
                        .ff_threshold = AS5600_FF_THRESHOLD_10LSB,
                        .watchdog = true
                };

/*
 *******************************************************************************
 * Public Function Bodies                                                      *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Private Function Bodies                                                     *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Interrupt Service Routines / Tasks / Thread Main Functions                  *
 *******************************************************************************
 */

TEST_GROUP(as5600_no_init)
{
        void setup()
        {
        }

        void teardown()
        {
                as5600_deinit();
        }
};

TEST(as5600_no_init, initialization)
{
        as5600_error_t result = as5600_init(i2c_io_stub);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600_no_init, deinitialization_was_not_initialized)
{
        as5600_error_t result = as5600_deinit();

        ENUMS_EQUAL_INT(AS5600_ERROR_RUNTIME_ERROR, result);
}

TEST(as5600_no_init, initialization_wrong_transfer_function)
{
        as5600_error_t result = as5600_init(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600_no_init, set_start_position_not_initialized_fails){

        as5600_error_t result = as5600_set_start_position(m_valid_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_start_position_not_initialized_fails){

        uint16_t start_position;
        as5600_error_t result = as5600_get_start_position(&start_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, start_stop_position_not_initialized_fails){

        as5600_error_t result = as5600_set_stop_position(m_valid_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_stop_position_not_initialized_fails){

        uint16_t stop_position;
        as5600_error_t result = as5600_get_stop_position(&stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}


TEST(as5600_no_init, set_maximum_angle_not_initialized_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_valid_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_maximum_angle_not_initialized_fails){

        uint16_t max_angle;
        as5600_error_t result = as5600_get_maximum_angle(&max_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_raw_angle_not_initialized_fails)
{
        uint16_t raw_angle;
        as5600_error_t result = as5600_get_raw_angle(&raw_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_angle_not_initialized_fails)
{
        uint16_t angle;
        as5600_error_t result = as5600_get_raw_angle(&angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_status_not_initialized_fails)
{
        as5600_status_t status;
        as5600_error_t result = as5600_get_status(&status);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_agc_not_initialized_fails)
{
        uint8_t agc;
        as5600_error_t result = as5600_get_automatic_gain_control(&agc);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST(as5600_no_init, get_get_cordic_magnitude_not_initialized_fails)
{
        uint16_t magnitude;
        as5600_error_t result = as5600_get_cordic_magnitude(&magnitude);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

TEST_GROUP(as5600)
{

        /*!
         * @brief Check a 12 bit value with the contents of a given register
         *
         * The function fail the test if the provided value and the value
         * at the given register don't match
         *
         * @param           reg_h           Register address holding the 4 MSB
         *                                  of the value
         *
         * @param           value           Value to compare with the register
         *
         * @return          -               -
         */
        void check_register_12(as5600_register_t const reg_h,
                               uint16_t const expected_value)
        {
                uint8_t const actual_value_reg_h =
                                *(m_memory.get_registers(reg_h));

                uint8_t const actual_value_reg_l =
                                *(m_memory.get_registers(reg_h + 1));

                BITS_EQUAL(expected_value >> 8, actual_value_reg_h, 0xFF);
                BITS_EQUAL(expected_value, actual_value_reg_l, 0x0F);
        }

        /*!
         * @brief Set a 12 bit value at the given registers
         *
         * @param           reg_h           Register address holding the 4 MSB
         *                                  of the value
         *
         * @param           value           Value to set at the register
         *
         * @return          -               -
         */
        void set_register_12(as5600_register_t const reg_h,
                             uint16_t const value)
        {
                uint8_t buffer[2] =
                                {
                                                (value >> 8) & 0x0F,
                                        (value & 0xFF)
                                };

                m_memory.set_memory(buffer, reg_h, 2);

        }

        /*!
         * @brief Check a 8 bit value with the contents of a given register
         *
         * The function fail the test if the provided value and the value
         * at the given register don't match
         *
         * @param           reg             Register address
         *
         * @param           value           Value to compare with the register
         *
         * @return          -               -
         */
        void check_register_8(as5600_register_t const reg,
                              uint8_t const expected_value)
        {
                uint8_t const actual_value_reg = *(m_memory.get_registers(reg));

                BITS_EQUAL(expected_value, actual_value_reg, 0xFF);
        }

        /*!
         * @brief Set a 8 bit value at the given registers
         *
         * @param           reg             Register address
         *
         * @param           value           Value to set at the register
         *
         * @return          -               -
         */
        void set_register_8(as5600_register_t const reg, uint8_t const value)
        {
                m_memory.set_memory(&value, reg, 1);
        }


        void setup()
        {
                (void)as5600_init(i2c_io_stub);
                m_memory.clear_registers();
        }

        void teardown()
        {
                (void)as5600_deinit();
        }
};

TEST(as5600, double_initialization_fails)
{
        as5600_error_t result = as5600_init(i2c_io_stub);

        ENUMS_EQUAL_INT(AS5600_ERROR_RUNTIME_ERROR, result);
}

TEST(as5600, get_otp_write_counter_null_pointer)
{
        as5600_error_t result = as5600_get_otp_write_counter(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_otp_write_counter_correct)
{
        uint8_t const otp_set_value = 2;
        uint8_t otp;
        as5600_error_t result;

        m_memory.set_memory(&otp_set_value, AS5600_REGISTER_ZMCO, 1);
        result = as5600_get_otp_write_counter(&otp);

        LONGS_EQUAL(otp_set_value, otp);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, set_start_position_oor_high_fails){

        as5600_error_t result = as5600_set_start_position(m_oor_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_start_position_correct){

        as5600_error_t result;

        m_memory.clear_registers();
        result = as5600_set_start_position(m_valid_start_stop_position);

        check_register_12(AS5600_REGISTER_ZPOS_H, m_valid_start_stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_start_position_null_pointer){

        as5600_error_t result = as5600_get_start_position(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_start_position_correct){

        uint16_t start_position;

        (void)as5600_set_start_position(m_valid_start_stop_position);
        as5600_error_t result = as5600_get_start_position(&start_position);

        LONGS_EQUAL(m_valid_start_stop_position, start_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, set_stop_position_oor_high_fails){

        as5600_error_t result = as5600_set_stop_position(m_oor_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_stop_position_correct){

        as5600_error_t result = as5600_set_stop_position(m_valid_start_stop_position);

        check_register_12(AS5600_REGISTER_MPOS_H, m_valid_start_stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_stop_position_null_pointer){

        as5600_error_t result = as5600_get_stop_position(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_stop_position_correct){

        uint16_t stop_position;

        (void)as5600_set_stop_position(m_valid_start_stop_position);
        as5600_error_t result = as5600_get_stop_position(&stop_position);

        LONGS_EQUAL(m_valid_start_stop_position, stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, set_maximum_angle_oor_low_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_oor_l_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_maximum_angle_oor_high_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_oor_h_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_maximum_angle_correct)
{
        as5600_error_t result = as5600_set_maximum_angle(m_valid_maximum_angle);

        check_register_12(AS5600_REGISTER_MANG_H, m_valid_maximum_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_maximum_angle_null_pointer){

        as5600_error_t result = as5600_set_maximum_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_maximum_angle_correct){

        uint16_t maximum_angle;

        (void)as5600_set_maximum_angle(m_valid_maximum_angle);
        as5600_error_t result = as5600_get_maximum_angle(&maximum_angle);

        LONGS_EQUAL(m_valid_maximum_angle, maximum_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, set_configuration_null_pointer)
{
        as5600_error_t result = as5600_set_configuration(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);

}

TEST(as5600, set_configuration_valid)
{
        as5600_error_t result = as5600_set_configuration(&m_valid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, set_configuration_power_mode_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.power_mode = AS5600_POWER_MODE_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_configuration_hysteresis_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.hysteresis = AS5600_HYSTERESIS_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_configuration_output_stage_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.output_stage = AS5600_OUTPUT_STAGE_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_configuration_pwm_frequency_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.pwm_frequency = AS5600_PWM_FREQUENCY_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_configuration_slow_filter_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.slow_filter = AS5600_SLOW_FILTER_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_configuration_ff_threshold_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.ff_threshold = AS5600_FF_THRESHOLD_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_configuration_valid)
{
        as5600_error_t result;
        as5600_configuration_t read_configuration;

        (void)as5600_set_configuration(&m_valid_configuration);
        result = as5600_get_configuration(&read_configuration);

        ENUMS_EQUAL_INT(m_valid_configuration.power_mode,       read_configuration.power_mode);
        ENUMS_EQUAL_INT(m_valid_configuration.hysteresis,       read_configuration.hysteresis);
        ENUMS_EQUAL_INT(m_valid_configuration.output_stage,     read_configuration.output_stage);
        ENUMS_EQUAL_INT(m_valid_configuration.pwm_frequency,    read_configuration.pwm_frequency);
        ENUMS_EQUAL_INT(m_valid_configuration.slow_filter,      read_configuration.slow_filter);
        ENUMS_EQUAL_INT(m_valid_configuration.ff_threshold,     read_configuration.ff_threshold);
        ENUMS_EQUAL_INT(m_valid_configuration.watchdog,         read_configuration.watchdog);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_configuration_null_pointer)
{
        as5600_error_t result = as5600_get_configuration(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_raw_angle_null_pointer)
{
        as5600_error_t result = as5600_get_raw_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_raw_angle_valid)
{
        uint16_t raw_angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_RAWANGLE_H, m_valid_raw_angle);
        result = as5600_get_raw_angle(&raw_angle);

        check_register_12(AS5600_REGISTER_RAWANGLE_H, m_valid_raw_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_angle_null_pointer)
{
        as5600_error_t result = as5600_get_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_angle_valid)
{
        uint16_t angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_ANGLE_H, m_valid_raw_angle);
        result = as5600_get_angle(&angle);

        check_register_12(AS5600_REGISTER_ANGLE_H, m_valid_raw_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_status_null_pointer)
{
        as5600_error_t result = as5600_get_status(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_status_valid)
{
        as5600_status_t status;
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        result = as5600_get_status(&status);

        check_register_8(AS5600_REGISTER_STATUS, status);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, get_agc_null_pointer)
{
        as5600_error_t result = as5600_get_automatic_gain_control(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, get_agc_valid)
{
        uint8_t agc;
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_AGC, m_valid_agc_value);

        result = as5600_get_automatic_gain_control(&agc);

        check_register_8(AS5600_REGISTER_AGC, m_valid_agc_value);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

TEST(as5600, as5600_get_cordic_magnitude_null_pointer)
{
        as5600_error_t result = as5600_get_cordic_magnitude(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, as5600_get_cordic_magnitude_valid)
{
        uint16_t raw_angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_MAGNITUDE_H, m_valid_magnitude);
        result = as5600_get_cordic_magnitude(&raw_angle);

        check_register_12(AS5600_REGISTER_MAGNITUDE_H, m_valid_magnitude);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}


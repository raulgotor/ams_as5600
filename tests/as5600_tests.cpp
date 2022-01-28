/*!
 *******************************************************************************
 * @file as5600_tests.cpp
 *
 * @brief 
 *
 * @author Raúl Gotor
 * @date 09.01.22
 *
 * @par
 * (c) Copyright 2022 Raúl Gotor.
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

#include "CppUTest/TestHarness.h"
#include "as5600.h"

#include "mocks/registers.cpp"
#include "as5600_mocks.h"

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

//! @brief Check a 12 bit value with the contents of a given register
static void check_register_12(
                as5600_register_t const reg_h, uint16_t const expected_value);

//! @brief Set a 12 bit value at the given registers
static void set_register_12(
                as5600_register_t const reg_h, uint16_t const value);

//! @brief Check a 8 bit value with the contents of a given register
static void check_register_8(
                as5600_register_t const reg, uint8_t const expected_value);

//! @brief Set a 8 bit value at the given registers
static void set_register_8(as5600_register_t const reg, uint8_t const value);

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

//! @brief Object where to perform the fake memory access operations
static registers m_registers;

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

//! @brief Maximum amount of write cycles for BURN_SETTINGS command
static uint8_t const m_max_burn_settings_count = 1;

//! @brief Maximum amount of write cycles for BURN_ANGLE command
static uint8_t const m_max_burn_angle_count = 3;

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
static void check_register_12(
                as5600_register_t const reg_h, uint16_t const expected_value)
{
        uint8_t const actual_value_reg_h =
                        *(m_registers.get_registers(reg_h));

        uint8_t const actual_value_reg_l =
                        *(m_registers.get_registers(reg_h + 1));

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
static void set_register_12(
                as5600_register_t const reg_h, uint16_t const value)
{
        uint8_t buffer[2] =
                        {
                                        (value >> 8) & 0x0F,
                                        (value & 0xFF)
                        };

        m_registers.set_memory(buffer, reg_h, 2);

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
static void check_register_8(
                as5600_register_t const reg, uint8_t const expected_value)
{
        uint8_t const actual_value_reg = *(m_registers.get_registers(reg));

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
static void set_register_8(as5600_register_t const reg, uint8_t const value)
{
        m_registers.set_memory(&value, reg, 1);
}

/*
 *******************************************************************************
 * Interrupt Service Routines / Tasks / Thread Main Functions                  *
 *******************************************************************************
 */

/*!
 * @brief Test group for tests requiring no initialization of the module
 *
 * This test group runs the tests without initializing the module. After each
 * tests, the module is deinitialized.
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

/*!
 * @brief Test that `as5600_init` with a valid transfer function succeeds
 */
TEST(as5600_no_init, initialization)
{
        as5600_error_t result = as5600_init(i2c_io_stub);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_deinit` without having initialized first fails
 */
TEST(as5600_no_init, deinitialization_was_not_initialized)
{
        as5600_error_t result = as5600_deinit();

        ENUMS_EQUAL_INT(AS5600_ERROR_RUNTIME_ERROR, result);
}

/*!
 * @brief Test that `as5600_init` with an invalid transfer function fails
 */
TEST(as5600_no_init, initialization_wrong_transfer_function)
{
        as5600_error_t result = as5600_init(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_start_position` without an initialized module
 *        fails
 */
TEST(as5600_no_init, set_start_position_not_initialized_fails){

        as5600_error_t result = as5600_set_start_position(
                        m_valid_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_start_position` without an initialized module
 *        fails
 */
TEST(as5600_no_init, get_start_position_not_initialized_fails){

        uint16_t start_position;
        as5600_error_t result = as5600_get_start_position(&start_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_set_stop_position` without an initialized module
 *        fails
 */
TEST(as5600_no_init, start_stop_position_not_initialized_fails){

        as5600_error_t result = as5600_set_stop_position(
                        m_valid_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_stop_position` without an initialized module
 *        fails
 */
TEST(as5600_no_init, get_stop_position_not_initialized_fails){

        uint16_t stop_position;
        as5600_error_t result = as5600_get_stop_position(&stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}


/*!
 * @brief Test that `as5600_set_maximum_angle` without an initialized module
 *        fails
 */
TEST(as5600_no_init, set_maximum_angle_not_initialized_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_valid_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_maximum_angle` without an initialized module
 *        fails
 */
TEST(as5600_no_init, get_maximum_angle_not_initialized_fails){

        uint16_t max_angle;
        as5600_error_t result = as5600_get_maximum_angle(&max_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_raw_angle` without an initialized module fails
 */
TEST(as5600_no_init, get_raw_angle_not_initialized_fails)
{
        uint16_t raw_angle;
        as5600_error_t result = as5600_get_raw_angle(&raw_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_raw_angle` without an initialized module fails
 */
TEST(as5600_no_init, get_angle_not_initialized_fails)
{
        uint16_t angle;
        as5600_error_t result = as5600_get_raw_angle(&angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_status` without an initialized module fails
 */
TEST(as5600_no_init, get_status_not_initialized_fails)
{
        as5600_status_t status;
        as5600_error_t result = as5600_get_status(&status);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_automatic_gain_control` without an initialized
 *        module fails
 */
TEST(as5600_no_init, get_agc_not_initialized_fails)
{
        uint8_t agc;
        as5600_error_t result = as5600_get_automatic_gain_control(&agc);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test that `as5600_get_cordic_magnitude` without an initialized module
 *        fails
 */
TEST(as5600_no_init, get_get_cordic_magnitude_not_initialized_fails)
{
        uint16_t magnitude;
        as5600_error_t result = as5600_get_cordic_magnitude(&magnitude);

        ENUMS_EQUAL_INT(AS5600_ERROR_NOT_INITIALIZED, result);
}

/*!
 * @brief Test group for tests requiring previous initialization of the module
 *
 * This test group initializes the module with a mocked i2c transfer function
 * that writes to a fake memory. The fake memory is cleared every time, and the
 * module deinitialized after the test
 */
TEST_GROUP(as5600)
{

        void setup()
        {
                (void)as5600_mocks_init(&m_registers);
                (void)as5600_init(i2c_io_stub);
                m_registers.clear_registers();
        }

        void teardown()
        {
                (void)as5600_deinit();
        }
};

/*!
 * @brief Test that `as5600_init` fails when called twice
 */
TEST(as5600, double_initialization_fails)
{
        as5600_error_t result = as5600_init(i2c_io_stub);

        ENUMS_EQUAL_INT(AS5600_ERROR_RUNTIME_ERROR, result);
}

/*!
 * @brief Test that `as5600_get_otp_write_counter` with invalid parameters fails
 */
TEST(as5600, get_otp_write_counter_null_pointer)
{
        as5600_error_t result = as5600_get_otp_write_counter(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_otp_write_counter` passes back the value written
 *        in the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_otp_write_counter_correct)
{
        uint8_t const otp_set_value = 2;
        uint8_t otp;
        as5600_error_t result;

        m_registers.set_memory(&otp_set_value, AS5600_REGISTER_ZMCO, 1);
        result = as5600_get_otp_write_counter(&otp);

        LONGS_EQUAL(otp_set_value, otp);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_start_position` fails when trying to set an out
 *        of range at the high end value
 */
TEST(as5600, set_start_position_oor_high_fails){

        as5600_error_t result = as5600_set_start_position(
                        m_oor_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_start_position` passes back the value written in
 *        the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
TEST(as5600, set_start_position_correct){

        as5600_error_t result;

        m_registers.clear_registers();
        result = as5600_set_start_position(m_valid_start_stop_position);

        check_register_12(AS5600_REGISTER_ZPOS_H, m_valid_start_stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_start_position` with invalid parameters fails
 */
TEST(as5600, get_start_position_null_pointer){

        as5600_error_t result = as5600_get_start_position(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_start_position` passes back the value written in
 *        the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_start_position_correct){

        uint16_t start_position;

        (void)as5600_set_start_position(m_valid_start_stop_position);
        as5600_error_t result = as5600_get_start_position(&start_position);

        LONGS_EQUAL(m_valid_start_stop_position, start_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_stop_position` fails when trying to set an out
 *        of range (at the high end) value
 */
 TEST(as5600, set_stop_position_oor_high_fails){

        as5600_error_t result = as5600_set_stop_position(
                        m_oor_start_stop_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_stop_position` is able to set a valid value and
 *        returns a success error code
 *
 * The test checks that the passed valid value is written at the expected
 * address at the fake memory
 */
TEST(as5600, set_stop_position_correct){

        as5600_error_t result = as5600_set_stop_position(
                        m_valid_start_stop_position);

        check_register_12(AS5600_REGISTER_MPOS_H, m_valid_start_stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_stop_position` with invalid parameters fails
 */
TEST(as5600, get_stop_position_null_pointer){

        as5600_error_t result = as5600_get_stop_position(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_stop_position` passes back the value written in
 *        the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_stop_position_correct){

        uint16_t stop_position;
        as5600_error_t result;

        (void)as5600_set_stop_position(m_valid_start_stop_position);
        result = as5600_get_stop_position(&stop_position);

        LONGS_EQUAL(m_valid_start_stop_position, stop_position);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_maximum_angle` fails when trying to set an out
 *        of range (at the low end) value
 */
 TEST(as5600, set_maximum_angle_oor_low_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_oor_l_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_maximum_angle` fails when trying to set an out
 *        of range (at the high end) value
 */
 TEST(as5600, set_maximum_angle_oor_high_fails){

        as5600_error_t result = as5600_set_maximum_angle(m_oor_h_maximum_angle);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_maximum_angle` is able to set a valid value and
 *        returns a success error code
 *
 * The test checks that the passed valid value is written at the expected
 * address at the fake memory
 */
 TEST(as5600, set_maximum_angle_correct)
{
        as5600_error_t result = as5600_set_maximum_angle(m_valid_maximum_angle);

        check_register_12(AS5600_REGISTER_MANG_H, m_valid_maximum_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_maximum_angle` with invalid parameters fails
 */
TEST(as5600, get_maximum_angle_null_pointer){

        as5600_error_t result = as5600_set_maximum_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_maximum_angle` passes back the value written in
 *        the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_maximum_angle_correct){

        uint16_t maximum_angle;

        (void)as5600_set_maximum_angle(m_valid_maximum_angle);
        as5600_error_t result = as5600_get_maximum_angle(&maximum_angle);

        LONGS_EQUAL(m_valid_maximum_angle, maximum_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_configuration` with invalid parameters fails
 */
TEST(as5600, set_configuration_null_pointer)
{
        as5600_error_t result = as5600_set_configuration(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);

}

/*!
 * @brief Test that `as5600_set_configuration` returns a success error code
 *        when setting a valid configuration
 */
TEST(as5600, set_configuration_valid)
{
        as5600_error_t result = as5600_set_configuration(
                        &m_valid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `power_mode` value
 */
TEST(as5600, set_configuration_power_mode_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.power_mode = AS5600_POWER_MODE_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `hysteresis` value
 */
TEST(as5600, set_configuration_hysteresis_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.hysteresis = AS5600_HYSTERESIS_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `output_stage` value
 */
TEST(as5600, set_configuration_output_stage_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.output_stage = AS5600_OUTPUT_STAGE_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `pwm_frequency` value
 */
TEST(as5600, set_configuration_pwm_frequency_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.pwm_frequency = AS5600_PWM_FREQUENCY_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `slow_filter` value
 */
TEST(as5600, set_configuration_slow_filter_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.slow_filter = AS5600_SLOW_FILTER_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_set_configuration` returns a failure error code
 *        when setting a configuration with an invalid `ff_threshold` value
 */
TEST(as5600, set_configuration_ff_threshold_invalid)
{
        as5600_error_t result;
        as5600_configuration_t invalid_configuration = m_valid_configuration;

        invalid_configuration.ff_threshold = AS5600_FF_THRESHOLD_COUNT;

        result = as5600_set_configuration(&invalid_configuration);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_configuration` is able to retrieve a
 *        `as5600_configuration_t` from the corresponding registers with a
 *        success error code
 *
 * The test previously writes a valid configuration to the memory, and then
 * retrieves it back. Then each retrieved element of the configuration structure
 * is compared with the original one.
 */
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

/*!
 * @brief Test that `as5600_get_configuration` with invalid parameters fails
 */
TEST(as5600, get_configuration_null_pointer)
{
        as5600_error_t result = as5600_get_configuration(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_raw_angle` with invalid parameters fails
 */
TEST(as5600, get_raw_angle_null_pointer)
{
        as5600_error_t result = as5600_get_raw_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_raw_angle` passes back the value written in the
 *        corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_raw_angle_valid)
{
        uint16_t raw_angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_RAWANGLE_H, m_valid_raw_angle);
        result = as5600_get_raw_angle(&raw_angle);

        check_register_12(AS5600_REGISTER_RAWANGLE_H, m_valid_raw_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_angle` with invalid parameters fails
 */
TEST(as5600, get_angle_null_pointer)
{
        as5600_error_t result = as5600_get_angle(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_angle` passes back the value written in the
 *        corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_angle_valid)
{
        uint16_t angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_ANGLE_H, m_valid_raw_angle);
        result = as5600_get_angle(&angle);

        check_register_12(AS5600_REGISTER_ANGLE_H, m_valid_raw_angle);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_status` with invalid parameters fails
 */
TEST(as5600, get_status_null_pointer)
{
        as5600_error_t result = as5600_get_status(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_status` passes back the value written in the
 *        corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_status_valid)
{
        as5600_status_t status;
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);
        result = as5600_get_status(&status);

        check_register_8(AS5600_REGISTER_STATUS, status);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_automatic_gain_control` with invalid parameters
 *        fails
 */
TEST(as5600, get_agc_null_pointer)
{
        as5600_error_t result = as5600_get_automatic_gain_control(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_automatic_gain_control` passes back the value
 *        written in the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, get_agc_valid)
{
        uint8_t agc;
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_AGC, m_valid_agc_value);
        result = as5600_get_automatic_gain_control(&agc);

        check_register_8(AS5600_REGISTER_AGC, m_valid_agc_value);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_get_cordic_magnitude` with invalid parameters fails
 */
TEST(as5600, as5600_get_cordic_magnitude_null_pointer)
{
        as5600_error_t result = as5600_get_cordic_magnitude(NULL);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_get_cordic_magnitude` passes back the value written
 *        in the corresponding register with a success error code
 *
 * The test previously writes the value to be read back at the corresponding
 * register address
 */
 TEST(as5600, as5600_get_cordic_magnitude_valid)
{
        uint16_t raw_angle;
        as5600_error_t result;

        set_register_12(AS5600_REGISTER_MAGNITUDE_H, m_valid_magnitude);
        result = as5600_get_cordic_magnitude(&raw_angle);

        check_register_12(AS5600_REGISTER_MAGNITUDE_H, m_valid_magnitude);
        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_burn_command` with invalid parameters fails
 */
TEST(as5600, as5600_burn_command_invalid)
{
        as5600_error_t result = as5600_burn_command(AS5600_BURN_MODE_COUNT);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

/*!
 * @brief Test that `as5600_burn_command` without a magnet being detected fails
 *
 * The test ensures an `AS5600_STATUS_NO_MANGET` value at the specific register
 * and then tries to execute `as5600_burn_command`
 */
 TEST(as5600, as5600_burn_command_no_magnet_detected)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_NO_MANGET);
        result = as5600_burn_command(AS5600_BURN_MODE_BURN_ANGLE);

        ENUMS_EQUAL_INT(AS5600_ERROR_MAGNET_NOT_DETECTED, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns successful error code when
 *        start and stop angles difference is equal or bigger than the minimum
 *        allowed angle
 *
 * The test sets the status register to have a magnet detected, and the start
 * and stop (ZPOS and MPOS) registers to have values whose difference is equal
 * or bigger than the minimum allowed angle. Then executes the burn command with
 * `AS5600_BURN_MODE_BURN_ANGLE` as a parameter and reads the return value of
 * the function.
 */
TEST(as5600, as5600_burn_command_valid_angle_in_start_stop_registers)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        /* set the start and stop angle registers (ZPOS and MPOS respectively)
         * to hold values whose difference is >= than the minimum valid ange
         */
        set_register_12(AS5600_REGISTER_ZPOS_H, m_valid_start_stop_position);
        set_register_12(AS5600_REGISTER_MPOS_H,
                        m_valid_start_stop_position +
                        m_lowest_valid_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_ANGLE);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns failure error code when
 *        start and stop angles difference is lower than the minimum allowed
 *        angle
 *
 * The test sets the status register to have a magnet detected, and the start
 * and stop (ZPOS and MPOS) registers to have values whose difference is lower
 * than the minimum allowed angle. Then executes the burn command with
 * `AS5600_BURN_MODE_BURN_ANGLE` as a parameter and reads the return value of
 * the function.
 */
 TEST(as5600, as5600_burn_command_invalid_angle_in_start_stop_registers)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        /* set the start and stop angle registers (ZPOS and MPOS respectively)
         * to hold values whose difference is lower than the minimum valid ange
         */
        set_register_12(AS5600_REGISTER_ZPOS_H, m_valid_start_stop_position);
        set_register_12(AS5600_REGISTER_MPOS_H,
                        m_valid_start_stop_position + m_oor_l_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_ANGLE);

        ENUMS_EQUAL_INT(AS5600_ERROR_MIN_ANGLE_TOO_SMALL, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns failure error code when
 *        maximum allowed writing cycles has ben reached for BURN_ANGLE cmd
 *
 * The test sets the status register to have a magnet detected, the ZMCO
 * register to maximum number of writes allowed, and the start and stop
 * (ZPOS and MPOS) registers to have values whose difference is lower
 * than the minimum allowed angle. Then executes the burn command with
 * `AS5600_BURN_MODE_BURN_ANGLE` as a parameter and reads the return value of
 * the function.
 */
TEST(as5600, as5600_burn_command_valid_angle_in_start_stop_registers_invalid_zmco)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);
        set_register_8(AS5600_REGISTER_ZMCO, m_max_burn_angle_count);

        /* set the start and stop angle registers (ZPOS and MPOS respectively)
         * to hold values whose difference is >= than the minimum valid ange
         */
        set_register_12(AS5600_REGISTER_ZPOS_H, m_valid_start_stop_position);
        set_register_12(AS5600_REGISTER_MPOS_H,
                        m_valid_start_stop_position +
                        m_lowest_valid_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_ANGLE);

        ENUMS_EQUAL_INT(AS5600_ERROR_MAX_WRITE_CYCLES_REACHED, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns success error code when
 *        with `BURN_SETTINGS` command and valid maximum angle value
 *
 * The test sets the status register to have a magnet detected, and maximum angle
 * register to have value bigger or equal to the minimum allowed angle.
 * Then executes the burn command with `AS5600_BURN_MODE_BURN_SETTINGS` as a
 * parameter and reads the return value of the function.
 */
 TEST(as5600, as5600_burn_command_valid_angle_in_max_angle_register)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);
        check_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        // set the max angle registers (MANG) to hold a minimum valid range angle
        set_register_12(AS5600_REGISTER_MANG_H, m_lowest_valid_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_SETTING);

        ENUMS_EQUAL_INT(AS5600_ERROR_SUCCESS, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns failure error code when
 *        maximum allowed writing cycles has ben reached for `BURN_SETTINGS` cmd
 *
 * The test sets the status register to have a magnet detected, the ZMCO
 * register to maximum number of writes allowed, and maximum angle register
 * to have value bigger or equal than the minimum allowed angle.
 * Then executes the burn command with `AS5600_BURN_MODE_BURN_SETTINGS` as a
 * parameter and reads the return value of the function.
 */
TEST(as5600, as5600_burn_command_valid_angle_in_max_angle_register_invalid_counter_zmco)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);
        set_register_8(AS5600_REGISTER_ZMCO, m_max_burn_settings_count);
        check_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        // set the max angle regs. (MANG) to hold a minimum valid range angle
        set_register_12(AS5600_REGISTER_MANG_H, m_lowest_valid_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_SETTING);

        ENUMS_EQUAL_INT(AS5600_ERROR_MAX_WRITE_CYCLES_REACHED, result);
}

/*!
 * @brief Test that `as5600_burn_command` returns failure error code when
 *        maximum angle at MANG register is lower than allowed
 *
 * The test sets the status register to have a magnet detected, and maximum
 * angle register to have value lower than the minimum allowed angle.
 * Then executes the burn command with `AS5600_BURN_MODE_BURN_SETTINGS` as a
 * parameter and reads the return value of the function.
 */
TEST(as5600, as5600_burn_command_invalid_angle_in_max_angle_register)
{
        as5600_error_t result;

        set_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);
        check_register_8(AS5600_REGISTER_STATUS, AS5600_STATUS_MD);

        // set the max angle regs. (MANG) to hold an invalid minimum range angle
        set_register_12(AS5600_REGISTER_MANG_H, m_oor_l_maximum_angle);

        result = as5600_burn_command(AS5600_BURN_MODE_BURN_SETTING);

        ENUMS_EQUAL_INT(AS5600_ERROR_MIN_ANGLE_TOO_SMALL, result);
}

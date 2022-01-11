/*!
 *******************************************************************************
 * @file as5600_tests.cpp
 *
 * @brief 
 *
 * @author Raúl Gotor (raul.gotor@midge-medical.com)
 * @date 09.01.22
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2021 Midge Medical GmbH
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

static uint16_t const m_valid_start_stop_position = 0xABF;

static uint16_t const m_valid_maximum_angle = 0xABF;

static uint16_t const m_oor_l_maximum_angle = 205;

static uint16_t const m_oor_h_maximum_angle = 4096;

static uint16_t const m_oor_start_position = 4096;

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

TEST_GROUP(as5600)
{

        void check_register_12(as5600_register_t const reg_h,
                               uint16_t const expected_value)
        {
                uint8_t const actual_value_reg_h =
                                *(m_memory.get_memory(reg_h));

                uint8_t const actual_value_reg_l =
                                *(m_memory.get_memory(reg_h + 1));

                BITS_EQUAL(expected_value >> 8, actual_value_reg_h, 0xFF);
                BITS_EQUAL(expected_value, actual_value_reg_l, 0x0F);
        }

        void setup()
        {
                (void)as5600_init(i2c_io_stub);
                m_memory.clear_memory();
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

        as5600_error_t result = as5600_set_start_position(m_oor_start_position);

        ENUMS_EQUAL_INT(AS5600_ERROR_BAD_PARAMETER, result);
}

TEST(as5600, set_start_position_correct){

        as5600_error_t result;

        m_memory.clear_memory();
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

        as5600_error_t result = as5600_set_stop_position(m_oor_start_position);

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
        as5600_error_t result = as5600_set_maximum_angle(m_valid_start_stop_position);

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

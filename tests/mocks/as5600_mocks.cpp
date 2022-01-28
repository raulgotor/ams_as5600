/*!
 *******************************************************************************
 * @file as5600_mocks.cpp
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

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include "registers.cpp"
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

typedef enum {
        ACCESS_READ = 0,
        ACCESS_WRITE,
        ACCESS_READ_WRITE_PROGRAM,
        ACCES_NO_ACCESS,
        ACESS_COUNT
} access_t;

/*
 *******************************************************************************
 * Constants                                                                   *
 *******************************************************************************
 */

//! @brief Pointer to the fake registers where to perform the access operations
static registers * m_registers;

//! @brief Whether module was initialized or not
static bool m_is_initialized;

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

//! @brief Access type of the different registers
static access_t m_registers_access[] =
                {
                        [0x00] = ACCESS_READ,
                        [0x01 ... 0x08] = ACCESS_READ_WRITE_PROGRAM,
                        [0x09 ... 0x0A] = ACCES_NO_ACCESS,
                        [0x0B ... 0x0F] = ACCESS_READ,
                        [0x10 ... 0x19] = ACCES_NO_ACCESS,
                        [0x1A ... 0x1C] = ACCESS_READ,
                        [0x1D ... 0xFE] = ACCES_NO_ACCESS,
                        [0xFF] = ACCESS_WRITE,

                };

/*
 *******************************************************************************
 * Public Function Bodies                                                      *
 *******************************************************************************
 */

/*!
 * @brief Initialize module
 *
 * @param               p_register          registers object pointer where to
 *                                          perform the access operations
 *
 * @return              bool                Result of the operation
 */
bool as5600_mocks_init(registers * p_register)
{
        bool success = (p_register != nullptr);

        if (success) {
                m_registers = p_register;
                m_is_initialized = true;
        }

        return success;
}

/*!
 * @brief I2C read stub function
 *
 * This function will read from a fake memory registers to a read buffer.
 *
 * @note Data on session registers will be skipped
 *
 * @param               i2c_slave_address   address of the device to be read
 *                                          (not used)
 *
 * @param               p_rx_buffer         Pointer to buffer where to store
 *                                          the read memory
 * @param               size                Size of the memory to read
 *
 * @return              uint32_t            Result of the operation
 * @retval              0                   If everything was ok
 */
uint32_t i2c_io_stub(uint8_t const i2c_slave_address,
                     uint8_t const * const p_tx_buffer,
                     size_t const tx_buffer_size,
                     uint8_t * const p_rx_buffer,
                     size_t const rx_buffer_size)
{
        size_t const tx_data_offset = 1;
        uint32_t result = 0;
        size_t i;
        uint8_t register_addr;
        access_t access_type;
        bool is_rx_operation = true;

        (void)i2c_slave_address;

        if ((!m_is_initialized) || (NULL == p_tx_buffer) || (0 == tx_buffer_size)) {
                result = 1;
        }

        if (0 == result) {
                if ((NULL == p_rx_buffer) || (0 == rx_buffer_size)) {
                        is_rx_operation = false;
                }

                register_addr = p_tx_buffer[0];
                access_type = m_registers_access[register_addr];

                if ((!is_rx_operation) &&
                    ((ACCESS_WRITE == access_type) ||
                     (ACCESS_READ_WRITE_PROGRAM == access_type))) {

                        for (i = 0; (i < tx_buffer_size); ++i) {
                                m_registers->registers[register_addr + i] = p_tx_buffer[i + tx_data_offset];
                        }

                } else if ((is_rx_operation) &&
                           ((ACCESS_READ == access_type) ||
                            (ACCESS_READ_WRITE_PROGRAM == access_type))) {

                        for (i = 0; (i < rx_buffer_size); ++i) {
                                p_rx_buffer[i] = m_registers->registers[register_addr + i];
                        }

                } else {
                        result = 1;
                }
        }

        return result;
}

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

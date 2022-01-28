/*!
 *******************************************************************************
 * @file registers.cpp
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
#include <cstring>

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

/*!
 * @brief Class simulating and performing some operations on a register memory
 */
class registers
{

public:
        //! @brief Size in bytes of the registers memory map
        static size_t const size = 0xFF;
        uint8_t registers[size];
        uint8_t expected_registers[size];

        /*!
         * @brief Set all registers to 0x00
         */
        void clear_registers() {
                memset(registers, 0, size);
                memset(expected_registers, 0, size);
        }

        /*!
         * @brief Get registers
         *
         * @param           address         Address of the registers to get
         *
         * @return          uint8_t *       Pointer to the specified registers address
         */
        uint8_t * get_registers(uint16_t address = 0) {
                return &registers[address];
        }

        /*!
         * @brief Set registers to the data in a specified buffer
         *
         * @param           p_buffer        Pointer to buffer to be set
         * @param           address         Registers address where to set the data
         * @param           size            Number of bytes to set
         *
         * @return          -               -
         */
        void set_memory(uint8_t const * const p_buffer,
                        uint16_t const address,
                        size_t const size) {

                memcpy(&registers[address], p_buffer, size);
        }

        /*!
         * @brief Get expected registers value
         *
         * @param           -               -
         *
         * @return          uint8_t *       Pointer to the specified register data
         */
        uint8_t * get_expected_registers(void) {
                return expected_registers;
        }

        /*!
         * @brief Set expected registers value to the data in a specified buffer
         *
         * @param           p_buffer        Pointer to buffer to be set
         * @param           address         Address where to set the data
         * @param           size            Number of bytes to set
         *
         * @return          -               -
         */
        void set_expected_registers_value(uint8_t const * const p_buffer,
                                          uint16_t const address,
                                          size_t const size) {

                memcpy(&expected_registers[address], p_buffer, size);
        }
};

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

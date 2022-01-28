/*!
 *******************************************************************************
 * @file as5600_mocks.h
 *
 * @brief 
 *
 * @author Raúl Gotor
 * @date 10.01.22
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

#ifndef AS5600_MOCKS_H
#define AS5600_MOCKS_H

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

//! @brief Initialize module
bool as5600_mocks_init(registers * p_register);

//! @brief I2C read stub function
uint32_t i2c_io_stub(uint8_t const i2c_slave_address,
                     uint8_t const * const p_tx_buffer,
                     size_t const tx_buffer_size,
                     uint8_t * const p_rx_buffer,
                     size_t const rx_buffer_size);


#endif //AS5600_MOCKS_H
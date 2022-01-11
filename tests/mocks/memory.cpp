/*!
 *******************************************************************************
 * @file memory.cpp
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

#include "cstdint"
#include "cstring"

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
class memory
{

public:
        static size_t const size = 0xFF;
        uint8_t memory[size];
        uint8_t expected_memory[size];

        uint8_t io_block_addr;

        /*!
         * @brief Set all memory to 0x00
         */
        void clear_memory() {
                memset(memory, 0, size);
                memset(expected_memory, 0, size);
                io_block_addr = 0;
        }

        /*!
         * @brief Get memory
         *
         * @param           address         Address of the memory to get
         *
         * @return          uint8_t *       Pointer to the specified memory
         */
        uint8_t * get_memory(uint16_t address = 0) {
                return &memory[address];
        }

        /*!
         * @brief Set memory to the data in a specified buffer
         *
         * @param           p_buffer        Pointer to buffer to be set
         * @param           address         Address where to set the memory
         * @param           size            Number of bytes to set
         *
         * @return          -               -
         */
        void set_memory(uint8_t const * const p_buffer, uint16_t const address, size_t const size) {
                memcpy(&memory[address], p_buffer, size);
        }

        /*!
         * @brief Get expected memory
         *
         * @param           -               -
         *
         * @return          uint8_t *       Pointer to the specified memory
         */
        uint8_t * get_expected_memory(void) {
                return expected_memory;
        }

        /*!
         * @brief Set expected memory to the data in a specified buffer
         *
         * @param           p_buffer        Pointer to buffer to be set
         * @param           address         Address where to set the memory
         * @param           size            Number of bytes to set
         *
         * @return          -               -
         */
        void set_expected_memory(uint8_t const * const p_buffer, uint16_t const address, size_t const size) {
                memcpy(&expected_memory[address], p_buffer, size);
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

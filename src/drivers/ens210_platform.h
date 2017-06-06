/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

#ifndef __ENS210_PLATFORM_H_
#define __ENS210_PLATFORM_H_

#include <stdint.h>

/** @defgroup Platform   Platform Specific API
 * This file describes the platform function prototypes needed by ENS210 module.
 * @{
 */

/** I2C error codes. */
#define I2C_RESULT_OK          0

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/
/**
 * @brief   Write to I2C Interface.
 * @param   slave       : Slave address of the I2C device
 * @param   writeBuf    : Pointer to buffer to be written
 * @param   writeSize   : Number of bytes to be written
 * @return  I2C_RESULT_OK - Success, Otherwise - I2C failure.
 */
int I2C_Write(int slave, void *writeBuf, int writeSize);

/**
 * @brief   Read from I2C Interface.
 * @param   slave       : Slave address of the I2C device
 * @param   writeBuf    : Pointer to buffer to be written
 * @param   writeSize   : Number of bytes to be written
 * @param   readBuf     : Pointer to buffer to held the data to be read
 * @param   readSize    : Number of bytes to be read.
 * @return  I2C_RESULT_OK - Success, Otherwise - I2C failure.
 */
int I2C_Read(int slave, void *writeBuf, int writeSize, void *readBuf, int readSize);

/**
 * @brief   Wait for specified amount of time
 * @param   millisec   :    Waiting time in milliseconds.
 * @note    The wait time is guaranteed to be at least 'millisec'; it might be longer.
 * @note    Due to implementation constraints, there is typically a minimal wait time and a timing granularity.
 *          This functions "rounds up".
 */
void WaitMsec(unsigned int  millisec);

/**
 * @}
 */

#endif /* __ENS210_PLATFORM_H_ */


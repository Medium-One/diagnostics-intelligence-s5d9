/*
 *  Copyright (c) 2017 Medium One, Inc
 *  www.mediumone.com
 *
 *  Portions of this work may be based on third party contributions.
 *  Medium One, Inc reserves copyrights to this work whose
 *  license terms are defined under a separate Software License
 *  Agreement (SLA).  Re-distribution of any or all of this work,
 *  in source or binary form, is prohibited unless authorized by
 *  Medium One, Inc under SLA.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *
 *  Platform specific wrappers for drivers (excluding the wrappers specified by ens210_platform.h)
 *
 */

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_


#include <stddef.h>
#include <stdint.h>

#include "led.h"
#include "ens210_platform.h"


enum status_code {
    STATUS_OK,
    STATUS_ERR_OVERFLOW,
};


struct i2c_master_packet {
    uint8_t address;
    size_t data_length;
    uint8_t * data;
};


/*
 * I2C Read function used by BMC driver
 */
signed char bmc150_read(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes);

/*
 * I2C Write function used by BMC driver
 */
signed char bmc150_write(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes);

/*
 * I2C Write function used by MS driver
 */
enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *);

/*
 * I2C Read function used by MS driver
 */
enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *);

#define delay_ms WaitMsec


#endif /* DRIVERS_I2C_H_ */

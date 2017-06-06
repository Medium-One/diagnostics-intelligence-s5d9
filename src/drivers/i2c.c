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
 #include <math.h>

#include "net_thread.h"
#include "adc_thread.h"
#include "ens210_platform.h"
#include "led.h"
#include "i2c.h"
#include "app.h"
#ifdef USE_M1DIAG
#include <m1diagnostics_agent.h>
#endif


#define FLAKY_I2C 0x01
#define GOOD_I2C 0x02
#define USE_I2C_CALLBACK

/*
 * synergy i2c driver callback
 */
#ifdef USE_I2C_CALLBACK
void flakey_i2c(i2c_callback_args_t * p_args) {
    switch (p_args->event) {
        case I2C_EVENT_ABORTED:
            tx_event_flags_set(&g_new_event_flags0, FLAKY_I2C, TX_OR);
            break;
        case I2C_EVENT_RX_COMPLETE:
        case I2C_EVENT_TX_COMPLETE:
            tx_event_flags_set(&g_new_event_flags0, GOOD_I2C, TX_OR);
            break;
        default:
#ifdef USE_M1DIAG
            M1_LOG(error, "Unknown I2C callback event", p_args->event);
#endif
            break;
    }
}
#endif

signed char bmc150_write(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes) {
    ssp_err_t err;
    ULONG actual_flags;
    uint8_t temp_buf[50];

    tx_mutex_get(&g_i2c_bus_mutex, TX_WAIT_FOREVER);
    err = g_i2c0.p_api->slaveAddressSet(g_i2c0.p_ctrl, i2c_address, I2C_ADDR_MODE_7BIT);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Error setting slave address", err);
#endif
        return -1;
    }
    temp_buf[0] = register_address;
    memcpy(&temp_buf[1], buf, bytes);
#ifdef USE_I2C_CALLBACK
    do {
#endif
        err = g_i2c0.p_api->write(g_i2c0.p_ctrl, temp_buf, (uint32_t)bytes + 1, false);
        if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error writing to I2C bus", err);
#endif
            return -2;
        }
#ifdef USE_I2C_CALLBACK
        tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
    } while (actual_flags != GOOD_I2C);
#endif
    tx_mutex_put(&g_i2c_bus_mutex);
    return 0;
}

signed char bmc150_read(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes) {
    ssp_err_t err;
    ULONG actual_flags;

    tx_mutex_get(&g_i2c_bus_mutex, TX_WAIT_FOREVER);
    err = g_i2c0.p_api->slaveAddressSet(g_i2c0.p_ctrl, i2c_address, I2C_ADDR_MODE_7BIT);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Error setting slave address", err);
#endif
        return -1;
    }
#ifdef USE_I2C_CALLBACK
    do {
#endif
        err = g_i2c0.p_api->write(g_i2c0.p_ctrl, &register_address, 1, true);
        if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error writing to I2C bus", err);
#endif
            return -2;
        }
#ifdef USE_I2C_CALLBACK
        tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (actual_flags & GOOD_I2C) {
#endif
            err = g_i2c0.p_api->read(g_i2c0.p_ctrl, buf, bytes, false);
            if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
                M1_LOG(error, "Error reading from I2C bus", err);
#endif
                return -3;
            }
#ifdef USE_I2C_CALLBACK
            tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        }
    } while (actual_flags != GOOD_I2C);
#endif
    tx_mutex_put(&g_i2c_bus_mutex);
    return 0;
}

/*
 * Delay function used by BMC and ENS drivers
 */
void WaitMsec(unsigned int ms) {
    tx_thread_sleep((ULONG)ceil(ms/10.0));
}

/*
 * I2C Write function used by ENS driver
 */
int I2C_Write(int slave, void *writeBuf, int writeSize) {
    ssp_err_t err;
    ULONG actual_flags;

    tx_mutex_get(&g_i2c_bus_mutex, TX_WAIT_FOREVER);
    err = g_i2c0.p_api->slaveAddressSet(g_i2c0.p_ctrl, (uint16_t)slave, I2C_ADDR_MODE_7BIT);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Error setting slave address", err);
#endif
        return -1;
    }
#ifdef USE_I2C_CALLBACK
    do {
#endif
        err = g_i2c0.p_api->write(g_i2c0.p_ctrl, writeBuf, (uint32_t)writeSize, false);
        if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error writing to I2C bus", err);
#endif
            return -2;
        }
#ifdef USE_I2C_CALLBACK
        tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
    } while (actual_flags != GOOD_I2C);
#endif
    tx_mutex_put(&g_i2c_bus_mutex);
    return 0;
}

/*
 * I2C Read function used by ENS driver
 */
int I2C_Read(int slave, void *writeBuf, int writeSize, void *readBuf, int readSize) {
    ssp_err_t err;
    ULONG actual_flags;

    tx_mutex_get(&g_i2c_bus_mutex, TX_WAIT_FOREVER);
    err = g_i2c0.p_api->slaveAddressSet(g_i2c0.p_ctrl, (uint16_t)slave, I2C_ADDR_MODE_7BIT);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Error setting slave address", err);
#endif
        return -1;
    }
#ifdef USE_I2C_CALLBACK
    do {
#endif
        if (writeSize) {
            err = g_i2c0.p_api->write(g_i2c0.p_ctrl, writeBuf, (uint32_t)writeSize, true);
            if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
                M1_LOG(error, "Error writing to I2C bus", err);
#endif
                return -2;
            }
#ifdef USE_I2C_CALLBACK
            tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
            if (actual_flags & GOOD_I2C) {
#endif
                err = g_i2c0.p_api->read(g_i2c0.p_ctrl, readBuf, (uint32_t)readSize, false);
                if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
                    M1_LOG(error, "Error reading from I2C bus", err);
#endif
                    return -3;
                }
#ifdef USE_I2C_CALLBACK
                tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
            }
#endif
        } else {
            err = g_i2c0.p_api->read(g_i2c0.p_ctrl, readBuf, (uint32_t)readSize, false);
#ifdef USE_M1DIAG
            if (err != SSP_SUCCESS) {
                M1_LOG(error, "Error reading from I2C bus", err);
                return -3;
            }
#endif
#ifdef USE_I2C_CALLBACK
            tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
#endif
        }
#ifdef USE_I2C_CALLBACK
    } while (actual_flags != GOOD_I2C);
#endif
    tx_mutex_put(&g_i2c_bus_mutex);
    return 0;
}


enum status_code i2c_master_write_packet_wait(struct i2c_master_packet * transfer) {
    if (transfer->data_length)
        I2C_Write(transfer->address, transfer->data, (int)transfer->data_length);
    return STATUS_OK;
}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet * transfer) {
    if (transfer->data_length)
        I2C_Read(transfer->address, NULL, 0, transfer->data, (int)transfer->data_length);
    return STATUS_OK;
}

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
#include "bma2x2.h"
#include "led.h"
#include "i2c.h"
#include "app.h"
#ifdef USE_M1DIAG
#include <m1diagnostics_agent.h>
#endif


#define I2C_RETRIES 3
#define I2C_TX_BUFFER_SIZE 50

//#define USE_I2C_CALLBACK


enum I2C_STATUS {
    I2C_OK = 0,
    I2C_ERR_DRIVER,
    I2C_ERR_READ,
    I2C_ERR_WRITE,
    I2C_ERR_TRANSFER_SIZE,
    I2C_ERR_NULL_PTR,
};


#ifdef USE_I2C_CALLBACK
#define FLAKY_I2C 0x01
#define GOOD_I2C 0x02


/*
 * synergy i2c driver callback
 */
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

/*
 * i2c read/write/read&write low-level
 *
 * if tx_data is not NULL, this function will perform a start (S) followed by
 * the write address, even if tx_bytes is 0 (allows for device probing). read
 * will be performed if rx_buf is not NULL. if reading and restart is true, a
 * repeated start (Sr) will follow the write, rather than a stop (Sp) and
 * start (S),
 */
static enum I2C_STATUS i2c_transfer(uint8_t i2c_address,
                               uint8_t * tx_data,
                               uint32_t tx_bytes,
                               uint8_t * rx_buf,
                               uint32_t rx_bytes,
                               bool restart) {
    ssp_err_t err;
    enum I2C_STATUS ret = I2C_OK;
#ifdef USE_I2C_CALLBACK
    ULONG actual_flags;
#endif

    tx_mutex_get(&g_i2c_bus_mutex, TX_WAIT_FOREVER);

    err = g_i2c0.p_api->slaveAddressSet(g_i2c0.p_ctrl, i2c_address, I2C_ADDR_MODE_7BIT);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Error setting slave address", err);
#endif
        ret = I2C_ERR_DRIVER;
        goto end;
    }
    if (tx_data) {
        err = g_i2c0.p_api->write(g_i2c0.p_ctrl, tx_data, (uint32_t)tx_bytes, (rx_buf && restart) ? true : false);
        if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error writing to I2C bus", err);
#endif
            ret = I2C_ERR_WRITE;
            goto end;
        }
#ifdef USE_I2C_CALLBACK
        tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (actual_flags != GOOD_I2C) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error writing to I2C bus", 0);
#endif
            ret = I2C_ERR_WRITE;
            goto end;
        }
#endif
    }
    if (rx_buf) {
        err = g_i2c0.p_api->read(g_i2c0.p_ctrl, rx_buf, rx_bytes, false);
        if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error reading from I2C bus", err);
#endif
            ret = I2C_ERR_READ;
            goto end;
        }
#ifdef USE_I2C_CALLBACK
        tx_event_flags_get(&g_new_event_flags0, GOOD_I2C | FLAKY_I2C, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (actual_flags != GOOD_I2C) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Error reading from I2C bus", 0);
#endif
            ret = I2C_ERR_READ;
            goto end;
        }
#endif
    }
end:
    tx_mutex_put(&g_i2c_bus_mutex);
    return ret;
}

static enum I2C_STATUS i2c_transfer_with_retry(uint8_t i2c_address,
                                          uint8_t * tx_data,
                                          uint32_t tx_bytes,
                                          uint8_t * rx_buf,
                                          uint32_t rx_bytes,
                                          bool restart) {
    int retries = 0;
    enum I2C_STATUS ret;

    while (retries++ < I2C_RETRIES) {
        ret = i2c_transfer(i2c_address, tx_data, tx_bytes, rx_buf, rx_bytes, restart);
        if (ret == I2C_OK)
            break;
    }
    return ret;
}

static enum I2C_STATUS i2c_register_transfer(uint8_t i2c_address,
                                        uint8_t register_address,
                                        uint8_t * tx_data,
                                        uint32_t tx_bytes,
                                        uint8_t * rx_buf,
                                        uint32_t rx_bytes,
                                        bool restart,
                                        bool retry) {
    uint8_t tx_buf[I2C_TX_BUFFER_SIZE];

    if (tx_bytes >= I2C_TX_BUFFER_SIZE) {
#ifdef USE_M1DIAG
        M1_LOG(error, "I2C transfer size too large", (int)tx_bytes);
#endif
        return I2C_ERR_TRANSFER_SIZE;
    }

    if (!tx_data && tx_bytes) {
#ifdef USE_M1DIAG
        M1_LOG(error, "I2C write specified with NULL pointer", I2C_ERR_NULL_PTR);
#endif
        return I2C_ERR_NULL_PTR;
    }

    tx_buf[0] = register_address;

    if (tx_data && tx_bytes)
        memcpy(&tx_buf[1], tx_data, tx_bytes);

    if (retry)
        return i2c_transfer_with_retry(i2c_address, tx_buf, tx_bytes + 1, rx_buf, rx_bytes, restart);
    return i2c_transfer(i2c_address, tx_buf, tx_bytes + 1, rx_buf, rx_bytes, restart);
}

signed char bmc150_write(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes) {
    if (i2c_register_transfer(i2c_address, register_address, buf, bytes, NULL, 0, false, true) != I2C_OK)
        return ERROR;
    return SUCCESS;
}

signed char bmc150_read(uint8_t i2c_address, uint8_t register_address, uint8_t * buf, uint8_t bytes) {
    if (i2c_register_transfer(i2c_address, register_address, NULL, 0, buf, bytes, true, true) != I2C_OK)
        return ERROR;
    return SUCCESS;
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
    if (writeSize < 0)
        return -2;
    if (i2c_transfer_with_retry((uint8_t)slave, writeBuf, (uint32_t)writeSize, NULL, 0, false) == I2C_OK)
        return I2C_RESULT_OK;
    else
        return -1;
}

/*
 * I2C Read function used by ENS driver
 */
int I2C_Read(int slave, void *writeBuf, int writeSize, void *readBuf, int readSize) {
    if (writeSize < 0)
        return -2;
    if (readSize < 0)
        return -3;
    if (i2c_transfer_with_retry((uint8_t)slave, writeBuf, (uint32_t)writeSize, readBuf, (uint32_t)readSize, true) == I2C_OK)
        return I2C_RESULT_OK;
    else
        return -1;
}

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet * transfer) {
    if (transfer->data_length) {
        if (I2C_Write(transfer->address, transfer->data, (int)transfer->data_length) != I2C_RESULT_OK)
            return STATUS_ERR_OVERFLOW;
    }
    return STATUS_OK;
}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet * transfer) {
    if (transfer->data_length) {
        if (I2C_Read(transfer->address, NULL, 0, transfer->data, (int)transfer->data_length) != I2C_RESULT_OK)
            return STATUS_ERR_OVERFLOW;
    }
    return STATUS_OK;
}

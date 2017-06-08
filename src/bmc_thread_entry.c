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
#include "bmc_thread.h"
#include "net_thread.h"
#include "i2c.h"
#include "bmm050.h"
#include "bma2x2.h"
#include "agg.h"
#include "app.h"


#define ACCEL_ADDR      0x11
#define MAG_ADDR        0x13


#ifdef USE_M1DIAG
volatile agg_t x = { .name = "x_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t y = { .name = "y_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t z = { .name = "z_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t x_mag = { .name = "x_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t y_mag = { .name = "y_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t z_mag = { .name = "z_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t temp1 = { .name = "temp1", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif

/*
 * returns non-zero if the avg was crossed in either direction
 */
static int zero_crossing(float val, float prev, float avg) {
    return (((val < avg) && (prev >= avg)) || ((val > avg) && (prev <= avg)));
}

/******************************************************************************
* Function Name: bmc_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                BMC150 I2C sensor. Continuously samples the accelerometer,
*                magnetometer, and temperature sensor and adds to the running
*                aggregates. Aggregates are sent to the cloud when
*                BMC_TRANSFER_REQUEST is received. Aggregates contain:
*                    - min
*                    - max
*                    - average
*                    - last sample value
******************************************************************************/
void bmc_thread_entry(void)
{
    ULONG status;
    ULONG actual_flags;
    struct bma2x2_accel_data_temp sample_xyzt;
    struct bmm050_mag_data_float_t data_float;
    struct bma2x2_t bma2x2 = {0};
    struct bmm050_t bmm050 = {0};
#ifndef USE_M1DIAG
    agg_t x = { .name = "x_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t y = { .name = "y_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t z = { .name = "z_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t x_mag = { .name = "x_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t y_mag = { .name = "y_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t z_mag = { .name = "z_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t temp1 = { .name = "temp1", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif
    float scale;
    uint8_t range;
    uint32_t x_zero_crossings = 0, y_zero_crossings = 0, z_zero_crossings = 0;
    float x_prev_avg = 0, y_prev_avg = 0, z_prev_avg = 0;
    float x_last = 0;
    float y_last = 0;
    float z_last = 0;
    float t_last = 0;
    float xmag_last = 0;
    float ymag_last = 0;
    float zmag_last = 0;

    bma2x2.bus_read = bmc150_read;
    bma2x2.bus_write = bmc150_write;
    bma2x2.delay_msec = WaitMsec;
    bma2x2.dev_addr = ACCEL_ADDR;
    bma2x2_init(&bma2x2);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    bma2x2_set_i2c_wdt(0, 1);  // enable WDT
    bma2x2_set_i2c_wdt(1, 0);  // set WDT period to 1ms
    bma2x2_set_range(BMA2x2_RANGE_2G);
    bma2x2_get_range(&range);
    switch (range) {
        case BMA2x2_RANGE_2G:
            scale = 0.00098f;
            break;
        case BMA2x2_RANGE_4G:
            scale = 0.00195f;
            break;
        case BMA2x2_RANGE_8G:
            scale = 0.00391f;
            break;
        case BMA2x2_RANGE_16G:
            scale = 0.00781f;
            break;
        default:
            scale = 0;
            break;
    }

    bmm050.bus_read = bmc150_read;
    bmm050.bus_write = bmc150_write;
    bmm050.delay_msec = WaitMsec;
    bmm050.dev_addr = MAG_ADDR;
    bmm050_init(&bmm050);
    bmm050_set_functional_state(BMM050_NORMAL_MODE);

    while (1) {
        status = tx_event_flags_get(&g_sensor_event_flags, BMC_TRANSFER_REQUEST | BMC_THRESHOLD_UPDATE | BMC_SAMPLE_REQUEST, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (status == TX_SUCCESS) {
            if (actual_flags & BMC_SAMPLE_REQUEST) {
                bma2x2_read_accel_xyzt(&sample_xyzt);
                x_last = sample_xyzt.x * scale;
                y_last = sample_xyzt.y * scale;
                z_last = sample_xyzt.z * scale;
                t_last = 23 + 0.5f * sample_xyzt.temp;
                if (zero_crossing(x_last, x.value, x_prev_avg))
                    x_zero_crossings++;
                update_agg(&x, x_last);
                if (zero_crossing(y_last, y.value, y_prev_avg))
                    y_zero_crossings++;
                update_agg(&y, y_last);
                if (zero_crossing(z_last, z.value, z_prev_avg))
                    z_zero_crossings++;
                update_agg(&z, z_last);
                update_agg(&temp1, t_last);

                bmm050_read_mag_data_XYZ_float(&data_float);
                xmag_last = data_float.datax;
                ymag_last = data_float.datay;
                zmag_last = data_float.dataz;
                update_agg(&x_mag, xmag_last);
                update_agg(&y_mag, ymag_last);
                update_agg(&z_mag, zmag_last);
            }
            if (actual_flags & BMC_THRESHOLD_UPDATE) {
                update_threshold((agg_t *)&g_x, &x);
                update_threshold((agg_t *)&g_y, &y);
                update_threshold((agg_t *)&g_z, &z);
                update_threshold((agg_t *)&g_temp1, &temp1);
                update_threshold((agg_t *)&g_x_mag, &x_mag);
                update_threshold((agg_t *)&g_y_mag, &y_mag);
                update_threshold((agg_t *)&g_z_mag, &z_mag);
            }
            if (actual_flags & BMC_TRANSFER_REQUEST) {
                transfer_agg(&x, &g_x);
                transfer_agg(&y, &g_y);
                transfer_agg(&z, &g_z);
                transfer_agg(&temp1, &g_temp1);
                transfer_agg(&x_mag, &g_x_mag);
                transfer_agg(&y_mag, &g_y_mag);
                transfer_agg(&z_mag, &g_z_mag);
                g_x_zero_crossings = x_zero_crossings;
                g_y_zero_crossings = y_zero_crossings;
                g_z_zero_crossings = z_zero_crossings;
                tx_event_flags_set(&g_sensor_event_flags, BMC_TRANSFER_COMPLETE, TX_OR);
                x_prev_avg = x.total / (float)x.count;
                y_prev_avg = y.total / (float)y.count;
                z_prev_avg = z.total / (float)z.count;
                reset_agg(&x);
                reset_agg(&y);
                reset_agg(&z);
                reset_agg(&temp1);
                reset_agg(&x_mag);
                reset_agg(&y_mag);
                reset_agg(&z_mag);
                x_zero_crossings = y_zero_crossings = z_zero_crossings = 0;
            }
        }
    }
}

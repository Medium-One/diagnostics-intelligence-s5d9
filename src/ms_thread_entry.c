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
#include "ms_thread.h"
#include "net_thread.h"
#include "ms5637.h"
#include "agg.h"
#include "app.h"


extern TX_THREAD ms_thread;


#ifdef USE_M1DIAG
volatile agg_t temp3 = { .name = "temp3", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t pressure = { .name = "pressure", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif


/******************************************************************************
* Function Name: ms_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                MS5637 I2C sensor. Continuously samples the temperature and
*                pressure sensors and adds to the running
*                aggregates. Aggregates are sent to the cloud when
*                MS_TRANSFER_REQUEST is received. Aggregates contain:
*                    - min
*                    - max
*                    - average
*                    - last sample value
******************************************************************************/
void ms_thread_entry(void) {
#ifndef USE_M1DIAG
    agg_t temp3 = { .name = "temp3", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t pressure = { .name = "pressure", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif
    float temp2_last = 0;
    float pressure_last = 0;
    UINT status;
    ULONG actual_flags;

    ms5637_init();
    if (!ms5637_is_connected())
        tx_thread_suspend(&ms_thread);
    if (ms5637_reset() != ms5637_status_ok)
        tx_thread_suspend(&ms_thread);

    while (1) {
        status = tx_event_flags_get(&g_sensor_event_flags, MS_TRANSFER_REQUEST | MS_THRESHOLD_UPDATE | MS_SAMPLE_REQUEST, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (status == TX_SUCCESS) {
            if (actual_flags & MS_SAMPLE_REQUEST) {
                if (ms5637_read_temperature_and_pressure(&temp2_last, &pressure_last) == ms5637_status_ok) {
                    update_agg(&temp3, temp2_last);
                    update_agg(&pressure, pressure_last);
                }
            }
            if (actual_flags & MS_THRESHOLD_UPDATE) {
                update_threshold((agg_t *)&g_temp3, &temp3);
                update_threshold((agg_t *)&g_pressure, &pressure);
            }
            if (actual_flags & MS_TRANSFER_REQUEST) {
                transfer_agg(&temp3, &g_temp3);
                transfer_agg(&pressure, &g_pressure);
                tx_event_flags_set(&g_sensor_event_flags, MS_TRANSFER_COMPLETE, TX_OR);
                reset_agg(&temp3);
                reset_agg(&pressure);
            }
        }
    }
}

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
#include "ens_thread.h"
#include "net_thread.h"
#include "ens210_platform.h"
#include "ens210.h"
#include "agg.h"
#include "app.h"


#define AMS_ENS210_ADDR 0x43


#ifdef USE_M1DIAG
volatile agg_t temp2 = { .name = "temp2", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t humidity = { .name = "humidity", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif


/******************************************************************************
* Function Name: ens_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                ENS210 I2C sensor. Continuously samples the temperature and
*                humidity sensors and adds to the running
*                aggregates. Aggregates are sent to the cloud when
*                ENS_TRANSFER_REQUEST is received. Aggregates contain:
*                    - min
*                    - max
*                    - average
*                    - last sample value
******************************************************************************/
void ens_thread_entry(void)
{
    float temp_last = 0;
    float humidity_last = 0;
#ifndef USE_M1DIAG
    agg_t temp2 = { .name = "temp2", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
    agg_t humidity = { .name = "humidity", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif
    uint32_t T_Raw = 0, H_Raw = 0;
    int status;
    ULONG actual_flags;

    while (1) {
        status = (int)tx_event_flags_get(&g_sensor_event_flags, ENS_TRANSFER_REQUEST | ENS_THRESHOLD_UPDATE | ENS_SAMPLE_REQUEST, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (status == TX_SUCCESS) {
            if (actual_flags & ENS_SAMPLE_REQUEST) {
                //Start the measurement
                status = ENS210_SensStart_Set(ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START);
                if (status == I2C_RESULT_OK) {
                    //Wait for sensor measurement
                    WaitMsec(ENS210_T_H_CONVERSION_TIME_MS);

                    //Get the temperature and humidity raw value
                    status = ENS210_THVal_Get(&T_Raw, &H_Raw);

                    if (status == I2C_RESULT_OK) {

                        //Verify the temperature raw value
                        if (ENS210_IsCrcOk(T_Raw) && ENS210_IsDataValid(T_Raw)) {
                            temp_last = (float)ENS210_ConvertRawToCelsius(T_Raw, 1000) / 1000.0f;
                            update_agg(&temp2, temp_last);
                        }

                        //Verify the relative humidity raw value
                        if (ENS210_IsCrcOk(H_Raw) && ENS210_IsDataValid(H_Raw)) {
                            humidity_last = (float)ENS210_ConvertRawToPercentageH(H_Raw, 1000) / 1000.0f;
                            update_agg(&humidity, humidity_last);
                        }
                    }
                }
            }
            if (actual_flags & ENS_THRESHOLD_UPDATE) {
                update_threshold((agg_t *)&g_temp2, &temp2);
                update_threshold((agg_t *)&g_humidity, &humidity);
            }
            if (actual_flags & ENS_TRANSFER_REQUEST) {
                transfer_agg(&temp2, &g_temp2);
                transfer_agg(&humidity, &g_humidity);
                tx_event_flags_set(&g_sensor_event_flags, ENS_TRANSFER_COMPLETE, TX_OR);
                reset_agg(&temp2);
                reset_agg(&humidity);
            }
        }
    }
}

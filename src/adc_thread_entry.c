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
#include <stdio.h>

#include "adc_thread.h"
#include "net_thread.h"
#include "nx_dns.h"
#include "fx_stdio.h"
#include <i2c.h>
#include "led.h"
#include "app.h"
#include "agg.h"
#include "m1_agent.h"
#ifdef USE_M1DIAG
#include "m1diagnostics_agent.h"
#endif


#define ADC_BASE_PTR  R_S12ADC0_Type *
#define AVG_CNT 2000        // Sampling at 4Khz, average over .5 sec


#ifdef USE_M1DIAG
volatile agg_t mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif

static volatile uint16_t sound_level = 0;


/******************************************************************************
* Function Name: adc_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                ADC (connected to mic). After 150000 ADC samples, takes the
*                peak-peak (max - min) and adds to the running aggregate.
*                Aggregate is sent to the cloud when ADC_TRANSFER_REQUEST is
*                received. Aggregate contains:
*                    - min
*                    - max
*                    - average
*                    - last sample value
******************************************************************************/
void adc_thread_entry(void)
{
    ssp_err_t err;
    ULONG actual_flags;
    ADC_BASE_PTR p_adc;
    adc_instance_ctrl_t * p_ctrl = (adc_instance_ctrl_t *) g_adc0.p_ctrl;
#ifndef USE_M1DIAG
    agg_t mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
#endif

    p_adc = (ADC_BASE_PTR) p_ctrl->p_reg;
    /** Disable differential inputs */
    p_adc->ADPGADCR0 = 0;
    /** Bypass PGA amplifier */
    p_adc->ADPGACR = 0x0111;

    err = g_sf_adc_periodic0.p_api->start(g_sf_adc_periodic0.p_ctrl);
    if (err != SSP_SUCCESS) {
#ifdef USE_M1DIAG
        M1_LOG(error, "Unable to start ADC periodic framework", err);
#endif
        return;
    }

    while (1) {
        err = tx_event_flags_get(&g_sensor_event_flags, ADC_TRANSFER_REQUEST | ADC_THRESHOLD_UPDATE | ADC_BATCH_COMPLETE, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (err == TX_SUCCESS) {
            if (actual_flags & ADC_BATCH_COMPLETE)
                update_agg(&mic, sound_level);
            if (actual_flags & ADC_THRESHOLD_UPDATE)
                update_threshold((agg_t *)&g_mic, &mic);
            if (actual_flags & ADC_TRANSFER_REQUEST) {
                transfer_agg(&mic, &g_mic);
                tx_event_flags_set(&g_sensor_event_flags, ADC_TRANSFER_COMPLETE, TX_OR);
                reset_agg(&mic);
            }
        }
    }
}

void g_adc_framework_user_callback(sf_adc_periodic_callback_args_t * p_args) {
    static uint16_t sample_count=0;
    static uint16_t max = 0U;
    static uint16_t min = 4095U;
    uint16_t data = g_adc_raw_data[p_args->buffer_index];


    if (data > max)
        max = data;
    if (data < min)
        min = data;

    if (sample_count++ > AVG_CNT) {
        sample_count = 0;
        sound_level = (uint16_t) (max-min);
        max = 0U;
        min = 4095U;
        tx_event_flags_set(&g_sensor_event_flags, ADC_BATCH_COMPLETE, TX_OR);
    }

}

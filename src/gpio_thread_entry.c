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
#include "gpio_thread.h"
#include <m1_agent.h>

#define IRQ2_FLAG   0x00000001
#define IRQ5_FLAG   0x00000002


/*
 * sets pin to the output direction and sets the output to level
 * WARNING: this API does not:
 *      - verify that pin mode is GPIO
 *      - verify that pin is not being used for other purposes
 */
void set_pin(ioport_port_pin_t pin, ioport_level_t level);


/*
 * callback for IRQ2 (GPIO input P100)
 */
void irq2_callback(external_irq_callback_args_t * p_args) {
    SSP_PARAMETER_NOT_USED(p_args);
    tx_event_flags_set(&g_irq_event_flags, IRQ2_FLAG, TX_OR);
}

/*
 * callback for IRQ5 (GPIO input P410)
 */
void irq5_callback(external_irq_callback_args_t * p_args) {
    SSP_PARAMETER_NOT_USED(p_args);
    tx_event_flags_set(&g_irq_event_flags, IRQ5_FLAG, TX_OR);
}

void set_pin(ioport_port_pin_t pin, ioport_level_t level) {
    g_ioport.p_api->pinDirectionSet(pin, IOPORT_DIRECTION_OUTPUT);
    g_ioport.p_api->pinWrite(pin, level);
}

/******************************************************************************
* Function Name: gpio_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                external IRQ drivers for IRQ2 and IRQ5. Waits for an IRQ to
*                be raised. Sends an event indicating which irq(s) was
*                triggered.
******************************************************************************/
void gpio_thread_entry(void) {
    UINT ret;
    ULONG actual_flags;

    g_external_irq0.p_api->open(g_external_irq0.p_ctrl, g_external_irq0.p_cfg);
    g_external_irq1.p_api->open(g_external_irq1.p_ctrl, g_external_irq1.p_cfg);
    while (1) {
        ret = tx_event_flags_get(&g_irq_event_flags, IRQ2_FLAG | IRQ5_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (ret == TX_SUCCESS) {
            if ((actual_flags & (IRQ2_FLAG | IRQ5_FLAG)) == (IRQ2_FLAG | IRQ5_FLAG))
                m1_publish_event("{\"irq2_triggered\":true,\"irq5_triggered\":true}", NULL);
            else if (actual_flags & IRQ2_FLAG)
                m1_publish_event("{\"irq2_triggered\":true}", NULL);
            else
                m1_publish_event("{\"irq5_triggered\":true}", NULL);
        }
    }
}

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

#include <net_thread.h>
#include "led.h"


#define LED1 IOPORT_PORT_01_PIN_02
#define LED2 IOPORT_PORT_01_PIN_03
#define LED3 IOPORT_PORT_01_PIN_13


static uint8_t led_state = 0;

void set_leds(uint8_t r, uint8_t y, uint8_t g) {
    g_ioport.p_api->pinWrite(LED3, r ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
    g_ioport.p_api->pinWrite(LED2, y ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
    g_ioport.p_api->pinWrite(LED1, g ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
    // TODO: synchronize updates to led_state
    led_state = (uint8_t)(((r & 0x01) << 2) | ((y & 0x01) << 1) | (g & 0x01));
}

void toggle_leds(uint8_t r, uint8_t y, uint8_t g) {
    uint8_t r_state = (led_state >> 2) & 0x01;
    uint8_t y_state = (led_state >> 1) & 0x01;
    uint8_t g_state = led_state & 0x01;
    set_leds(r ? !r_state : r_state, y ? !y_state : y_state, g ? !g_state : g_state);
}

void set_led(uint8_t leds, uint8_t val) {
    // TODO: synchronize updates to led_state
    if ((leds >> 2) & 0x01) {
        g_ioport.p_api->pinWrite(LED3, val ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
        led_state = (uint8_t)((led_state & ~(1 << 2)) | (val ? (1 << 2) : 0));
    }
    if ((leds >> 1) & 0x01) {
        g_ioport.p_api->pinWrite(LED2, val ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
        led_state = (uint8_t)((led_state & ~(1 << 1)) | (val ? (1 << 1) : 0));
    }
    if (leds & 0x01) {
        g_ioport.p_api->pinWrite(LED1, val ? IOPORT_LEVEL_HIGH : IOPORT_LEVEL_LOW);
        led_state = (uint8_t)((led_state & ~1) | (val ? 1 : 0));
    }
}

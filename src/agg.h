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

#ifndef AGG_H_
#define AGG_H_


#include "app.h"


// aggregate structure
typedef struct {
    float min;
    float max;
    float total;
    float last_sent;  // last value uploaded
    ULONG last_sent_tick;  // tick of last upload
    float value;  // current value
    int count;  // samples aggregated
    char * name;  // tag name
    // TODO: use message framework to update instead of volatile?
    volatile float threshold;  // relative or absolute threshold to trigger upload
    volatile uint8_t absolute_threshold;  // absolute threshold?
} agg_t;


// global sensor aggregates
extern volatile agg_t g_x;
extern volatile agg_t g_y;
extern volatile agg_t g_z;
extern volatile agg_t g_x_mag;
extern volatile agg_t g_y_mag;
extern volatile agg_t g_z_mag;
extern volatile agg_t g_temp1;
extern volatile agg_t g_temp2;
extern volatile agg_t g_temp3;
extern volatile agg_t g_humidity;
extern volatile agg_t g_pressure;
extern volatile agg_t g_mic;
extern volatile uint32_t g_x_zero_crossings, g_y_zero_crossings, g_z_zero_crossings;

/*
 * Resets aggregate
 *      - Sets total & count to 0
 *      - Sets last_sent to value
 *      - Sets last_sent_tick to current tick
 */
#ifdef USE_M1DIAG
void reset_agg(volatile agg_t * agg);
#else
void reset_agg(agg_t * agg);
#endif

/*
 * Returns non-zero if value exceeds last_sent by threshold
 */
#ifdef USE_M1DIAG
uint8_t over_threshold(volatile agg_t * agg);
#else
uint8_t over_threshold(agg_t * agg);
#endif


/*
 * Uploads value to the cloud ({name: value}) and updates last_sent, last_sent_tick if value exceeds last_sent by threshold
 */
#ifdef USE_M1DIAG
void send_if_over_treshold(volatile agg_t * agg);
#else
void send_if_over_treshold(agg_t * agg);
#endif

/*
 * Updates aggregate fields for new sample value
 *      - Updates min & max as required
 *      - Add value to total
 *      - Sets agg->value to value
 *      - Increments count
 *      - Uploads value if change exceeds threshold
 */
#ifdef USE_M1DIAG
void update_agg(volatile agg_t * agg, float value);
#else
void update_agg(agg_t * agg, float value);
#endif

/*
 * Copies all values from from to to, except for threshold & absolute_threshold
 */
#ifdef USE_M1DIAG
void transfer_agg(volatile agg_t * from, volatile agg_t * to);
#else
void transfer_agg(agg_t * from, volatile agg_t * to);
#endif

/*
 * Copies threshold & absolute_threshold values from from to to
 */
#ifdef USE_M1DIAG
void update_threshold(volatile agg_t * from, volatile agg_t * to);
#else
void update_threshold(agg_t * from, agg_t * to);
#endif


#endif /* AGG_H_ */

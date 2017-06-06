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
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "tx_api.h"
#include "agg.h"
#include "app.h"
#include "m1_agent.h"


#ifdef USE_M1DIAG
void reset_agg(volatile agg_t * agg) {
#else
void reset_agg(agg_t * agg) {
#endif
    agg->total = 0.0f;
    agg->count = 0;
    agg->last_sent = agg->value;
    agg->last_sent_tick = tx_time_get();
}

#ifdef USE_M1DIAG
uint8_t over_threshold(volatile agg_t * agg) {
#else
uint8_t over_threshold(agg_t * agg) {
#endif
    if (!(fabs(agg->threshold) > 0.0f))
        return 0;
    if (!agg->absolute_threshold && ((fabs((agg->last_sent - agg->value) / agg->last_sent) * 100) > agg->threshold))
        return 1;
    if (agg->absolute_threshold && (fabs(agg->last_sent - agg->value) > agg->threshold))
        return 1;
    return 0;
}

#ifdef USE_M1DIAG
void send_if_over_treshold(volatile agg_t * agg) {
#else
void send_if_over_treshold(agg_t * agg) {
#endif
    char event[100];
    if (over_threshold(agg)) {
        ULONG curr_tick = tx_time_get();
        if ((curr_tick - agg->last_sent_tick) > RATE_LIMIT) {
            sprintf(event, "{\"%s\":{\"value\":%f}}", agg->name, agg->value);
            m1_publish_event(event, NULL);
            agg->last_sent = agg->value;
            agg->last_sent_tick = curr_tick;
        }
    }
}

#ifdef USE_M1DIAG
void update_agg(volatile agg_t * agg, float value) {
#else
void update_agg(agg_t * agg, float value) {
#endif
    if (!agg->count)
        agg->min = agg->max = value;
    else {
        if (value < agg->min)
            agg->min = value;
        if (value > agg->max)
            agg->max = value;
    }
    agg->total += value;
    agg->count++;
    agg->value = value;
    send_if_over_treshold(agg);
}

#ifdef USE_M1DIAG
void transfer_agg(volatile agg_t * from, volatile agg_t * to) {
#else
void transfer_agg(agg_t * from, volatile agg_t * to) {
#endif
    to->total = from->total;
    to->count = from->count;
    to->max = from->max;
    to->min = from->min;
    to->value = from->value;
}

#ifdef USE_M1DIAG
void update_threshold(volatile agg_t * from, volatile agg_t * to) {
#else
void update_threshold(agg_t * from, agg_t * to) {
#endif
    to->absolute_threshold = from->absolute_threshold;
    to->threshold = from->threshold;
}

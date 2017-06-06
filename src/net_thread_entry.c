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
#include "net_thread.h"
#include "system_thread.h"
#include "nx_dns.h"
#include "nx_dhcp.h"
#include "fx_stdio.h"
#include "led.h"
#include "agg.h"
#include "app.h"

#include <m1_agent.h>
#ifdef USE_M1DIAG
#include <m1diagnostics_agent.h>
#endif


#define DATA_FLASH_BLOCK_SIZE 64
#define PROVISIONED_FLAG 1


/*
 * global sensor aggregates
 *
 * sensor threads sample their sensors and aggregate in local aggregate
 * structures.
 * when a transfer is requested, the sensor thread copies the local aggregate
 * data into the corresponding global structure below.
 */
volatile agg_t g_x = { .name = "x_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y = { .name = "y_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z = { .name = "z_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_x_mag = { .name = "x_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y_mag = { .name = "y_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z_mag = { .name = "z_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp1 = { .name = "temp1", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp2 = { .name = "temp2", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp3 = { .name = "temp3", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_humidity = { .name = "humidity", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_pressure = { .name = "pressure", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile uint32_t g_x_zero_crossings = 0, g_y_zero_crossings = 0, g_z_zero_crossings = 0;

#ifdef USE_M1DIAG
/*
 * the diagnostics agent asynchronously samples peripherals.
 * to ensure it sees real-time updated data, we expose the "local" aggregate
 * structures used by the sensor threads
 */
extern volatile agg_t x;
extern volatile agg_t y;
extern volatile agg_t z;
extern volatile agg_t temp1;
extern volatile agg_t x_mag;
extern volatile agg_t y_mag;
extern volatile agg_t z_mag;
extern volatile agg_t temp2;
extern volatile agg_t humidity;
extern volatile agg_t temp3;
extern volatile agg_t pressure;
extern volatile agg_t mic;

static periphery_access_t x_accel_access = {(char)1, NULL, {.address=&x.value}};
static periphery_access_t y_accel_access = {(char)1, NULL, {.address=&y.value}};
static periphery_access_t z_accel_access = {(char)1, NULL, {.address=&z.value}};
static periphery_access_t temp_access = {(char)1, NULL, {.address=&temp1.value}};
static periphery_access_t xmag_access = {(char)1, NULL, {.address=&x_mag.value}};
static periphery_access_t ymag_access = {(char)1, NULL, {.address=&y_mag.value}};
static periphery_access_t zmag_access = {(char)1, NULL, {.address=&z_mag.value}};
static periphery_access_t temp2_access = {(char)1, NULL, {.address=&temp2.value}};
static periphery_access_t humidity_access = {(char)1, NULL, {.address=&humidity.value}};
static periphery_access_t temp3_access = {(char)1, NULL, {.address=&temp3.value}};
static periphery_access_t pressure_access = {(char)1, NULL, {.address=&pressure.value}};
static periphery_access_t mic_access = {(char)1, NULL, {.address=&mic.value}};

/*
 * if the diagnostics agent is used, the SSL library needs additional pool mem
 * for the second connection
 */
#define SSL_MEM_SIZE (60 * 1024)
static char ssl_mem[SSL_MEM_SIZE];
#else
#define SSL_MEM_SIZE (35 * 1024)
static char ssl_mem[SSL_MEM_SIZE];
#endif


// TODO: use message framework to update instead of volatile?
volatile ULONG sample_period = 15 * 60 * 100;

#define SENSORS 12
static volatile agg_t * const sensors[SENSORS] = {&g_x,
                                                  &g_y,
                                                  &g_z,
                                                  &g_x_mag,
                                                  &g_y_mag,
                                                  &g_z_mag,
                                                  &g_temp1,
                                                  &g_temp2,
                                                  &g_temp3,
                                                  &g_humidity,
                                                  &g_pressure,
                                                  &g_mic};

// holds the bitmask of leds that should be blinked in the RTC callback
static volatile uint8_t leds_to_toggle = 0;
// holds the number of blinks that should currently be performed in the RTC callback
static volatile int n_blinks = 0;


extern TX_THREAD net_thread;
extern TX_THREAD adc_thread;
extern TX_THREAD bmc_thread;
extern TX_THREAD ens_thread;
extern TX_THREAD ms_thread;

// connection details for diagnostics agent
static const char * rna_broker_url = "mqtt-renesas-na-sandbox.mediumone.com";
static const uint16_t rna_port = 61620;


// extern GPIO pin setting function
void set_pin(ioport_port_pin_t pin, ioport_level_t level);

// prototype for m1_callback below
void m1_callback(int type, char * topic, char * msg, int length);


/*
 * copies a line from open file f into the buffer pointed to by out, up to
 * out_size bytes. if eof_ok is non-zero, we don't treat encountering EOF as an
 * error.
 */
static int get_line(FILE * f, char * out, size_t out_size, uint8_t eof_ok) {
    char buf[200];

    do {
        if (fgets(buf, sizeof(buf), f) == NULL) {
            // EOF encountered, or error
            if (!eof_ok)
                return -1;
            if (!feof(f))  // not EOF, so an error
                return -2;
            buf[strcspn(buf, "\r\n")] = 0;
            break;
        }
        buf[strcspn(buf, "\r\n")] = 0;
    } while (!strnlen(buf, sizeof(buf)));
    strncpy(out, buf, out_size);
    return 0;
}

/*
 * extracts credentials from the file "m1config.txt".
 * the expected structure is:
 *      <api key>
 *      <mqtt project id>
 *      <mqtt registration user id> <- optional
 *      <registration user password> <- required if <mqtt registration user id> present
 * Returns:
 *      - -1 if "m1config.txt" does not exist
 *      - -2 if the first line cannot be read or encounters EOF
 *      - -3 if the second line cannot be read or encounters EOF
 *      - -4 if the third line cannot be read or encounters EOF
 *      - -5 if the last line cannot be read
 */
static int extract_credentials_from_config(project_credentials_t * project, user_credentials_t * user) {
    int ret = 0;
    FILE * config_file = fopen("m1config.txt", "r");
    if (!config_file)
        return -1;
    if (get_line(config_file, project->apikey, sizeof(project->apikey), 0)) {
        ret = -2;
        goto end;
    }
    if (get_line(config_file, project->proj_id, sizeof(project->proj_id), 0)) {
        ret = -3;
        goto end;
    }
    if (get_line(config_file, user->user_id, sizeof(user->user_id), 0)) {
        ret = -4;
        goto end;
    }
    if (get_line(config_file, user->password, sizeof(user->password), 1)) {
        ret = -5;
        goto end;
    }
end:
    fclose(config_file);
    return ret;
}

/*
 * handles messages published to one of the topics subscribed to by the m1 agent.
 * we use the convention of the first character of the msg indicating a command.
 * multiple commands can be sent in one message, separated by ';'. ';' must not
 * be used within a command. Supported commands are:
 *      - S: set new sampling period
 *      - T: update sensor threshold
 *      - L: set, clear, or toggle LED(s)
 *      - B: blink LED(s)
 *      - G: set GPIO pin output level
 */
void m1_callback(int type, char * topic, char * msg, int length) {
    SSP_PARAMETER_NOT_USED(type);
    SSP_PARAMETER_NOT_USED(topic);
    SSP_PARAMETER_NOT_USED(length);
    char * p = msg, * temp_str;
    int temp;
    int msglen;

    while (p != NULL) {
        temp_str = strchr(p, ';');
        if (temp_str != NULL)
            msglen = temp_str - p;
        else
            msglen = length - (p - msg);
        if (type == 1) {
            switch (p[0]) {
                case 'S': {
                    ULONG new_sample_period;
                    sscanf(&p[1], "%lu", &new_sample_period);
                    sample_period = new_sample_period;
                    tx_thread_wait_abort(&net_thread);
                    break;
                }
                case 'T': {
                    char name[30];
                    float threshold;
                    int threshold_type;
                    if (sscanf(&p[1], "%29[^:]:%d:%f", name, &threshold_type, &threshold) == 3) {
                        for (int i = 0; i < SENSORS; i++) {
                            if (!strcmp(sensors[i]->name, name)) {
                                sensors[i]->absolute_threshold = threshold_type ? 1 : 0;
                                sensors[i]->threshold = threshold;
                                break;
                            }
                        }
                        tx_event_flags_set(&g_sensor_event_flags, BMC_THRESHOLD_UPDATE | ADC_THRESHOLD_UPDATE | ENS_THRESHOLD_UPDATE | MS_THRESHOLD_UPDATE, TX_OR);
                    }
                    break;
                }
                case 'L': {
                    int val;
                    sscanf(&p[1], "%d", &temp);
                    temp_str = strchr(&p[1], ':');
                    // no strnchr...
                    if ((temp_str != NULL) && ((temp_str - p) < msglen)) {
                        if (sscanf(temp_str + 1, "%d", &val) == 1)
                            set_led((uint8_t)temp, (uint8_t)val);
                    } else
                        toggle_leds((temp >> 2) & 0x01, (temp >> 1) & 0x01, temp & 0x01);
                    break;
                }
                case 'B':
                    n_blinks = 0;
                    if (sscanf(&p[1], "%d:%d", &temp, &n_blinks) < 1)
                        break;
                    if (!temp)
                        break;
                    if (g_rtc0.p_api->open(g_rtc0.p_ctrl, g_rtc0.p_cfg) != SSP_SUCCESS)
                        break;
                    leds_to_toggle = (uint8_t)temp;
                    g_rtc0.p_api->periodicIrqRateSet(g_rtc0.p_ctrl, RTC_PERIODIC_IRQ_SELECT_1_SECOND);
                    g_rtc0.p_api->calendarCounterStart(g_rtc0.p_ctrl);
                    g_rtc0.p_api->irqEnable(g_rtc0.p_ctrl, RTC_EVENT_PERIODIC_IRQ);
                    break;
                case 'G': {
                    int pin;
                    int level;
                    if (sscanf(&p[1], "%d:%d:%d", &temp, &pin, &level) == 3)
                        set_pin(pin + (temp << 8), level == 0 ? 0 : 1);
                    break;
                }
                default:
                    break;
            }
        }
        p = strchr(p, ';');
        if (p != NULL)
            p++;
    }
}

/*
 * rtc callback. blinks LEDs if required
 */
void rtc_callback(rtc_callback_args_t * p_args) {
    static int count = 0;
    if (p_args->event == RTC_EVENT_PERIODIC_IRQ) {
        if (count++ < (n_blinks * 2)) {
            toggle_leds((leds_to_toggle >> 2) & 0x01, (leds_to_toggle >> 1) & 0x01, leds_to_toggle & 0x01);
        } else {
            count = 0;
            g_rtc0.p_api->close(g_rtc0.p_ctrl);
        }
    }
}


/*
 * helper function to determine how many dataflash blocks are spanned
 */
static uint32_t bytes_to_df_blocks(uint32_t bytes) {
    if (!bytes)
        return 0;
    return bytes / DATA_FLASH_BLOCK_SIZE + 1;
}

/*
 * extracts credentials from m1config.txt, and optionally m1user.txt and
 * m1broker.txt.
 *
 * if m1 diagnostics agent is enabled, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * if m1user.txt provided user credentials, attempts connection
 * with those credentials. if connection fails, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user credentials are not
 * available, board goes into error state. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * after successful connections, sends "on-connect" event and resumes sensor
 * threads. if diagnostics agent is enabled, configures sensor peripherals.
 * finally, enters infinite loop requesting sensor aggregates every sample_period.
 * if sensor aggregates are not received within 1 second of request, we assume
 * a sensor thread is dead-locked and reset the board. after receiving sensor
 * aggregates, publishes events for all sensor aggregates to the cloud.
 */
void net_thread_entry(void)
{
    ssp_err_t status = SSP_SUCCESS;
    int ret;
    char buf[200];
    int provisioned = 0;
    fmi_product_info_t * effmi;
    project_credentials_t project;
    user_credentials_t device, original_device, registration;
    original_device.user_id[0] = '\0';
    original_device.password[0] = '\0';
#ifdef USE_M1DIAG
    const project_credentials_t diag_project = {
        .proj_id="Ohafs8Q31jU",
        .apikey="VRWM4JXNCKZEHIDP4T7TQRZQGU4TSOJQGI4WMZBQGQ3DAMBQ"
    };
    user_credentials_t diag_device, original_diag_device;
    original_diag_device.user_id[0] = '\0';
    original_diag_device.password[0] = '\0';
    diag_device.user_id[0] = '\0';
    diag_device.password[0] = '\0';
    user_credentials_t diag_registration = {
        .user_id="f_acx06uqDA",
        .password="Medium-1!"
    };
#endif
    FILE * f_manual_creds;
    char mqtt_broker_host[64] = {0};
    const char * p_mqtt_broker_host = rna_broker_url;
    uint16_t mqtt_broker_port = rna_port;
    ULONG start, actual_flags;
    char event[1536];

    ret = extract_credentials_from_config(&project, &registration);
    if ((ret < 0) && (ret > -3))
        goto err;

    if ((f_manual_creds = fopen("m1user.txt", "r"))) {
        if ((!get_line(f_manual_creds, device.user_id, sizeof(device.user_id), 0))
            && (!get_line(f_manual_creds, device.password, sizeof(device.password), 1))) {
                provisioned = 1;
                memcpy(&original_device, &device, sizeof(original_device));
        }
        fclose(f_manual_creds);
    }

    if ((f_manual_creds = fopen("m1broker.txt", "r"))) {
        if (!get_line(f_manual_creds, buf, sizeof(buf), 0)) {
            if ((sscanf(buf, "%hu", &mqtt_broker_port) == 1) &&
                    !get_line(f_manual_creds, mqtt_broker_host, sizeof(mqtt_broker_host), 1))
                p_mqtt_broker_host = mqtt_broker_host;
            else
                mqtt_broker_port = rna_port;
        }
        fclose(f_manual_creds);
    }

    g_fmi.p_api->productInfoGet(&effmi);

    status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    if (status)
        goto err;
    status = g_flash0.p_api->read(g_flash0.p_ctrl, (UCHAR *)buf, 0x40100000, sizeof(buf) - 1);
    if (status)
        goto err;
    status = g_flash0.p_api->close(g_flash0.p_ctrl);
    if (status)
        goto err;
    buf[sizeof(buf) - 1] = '\0';
    if (buf[0] == 'P') {
        // board may have been auto-provisioned; try provisioned credentials
#ifdef USE_M1DIAG
        if (provisioned)
            ret = sscanf(&buf[1], "%*d;%*11[^;];%*63[^;];%11[^;];%63[^;]", diag_device.user_id, diag_device.password);
        else
            ret = sscanf(&buf[1], "%*d;%11[^;];%63[^;];%11[^;];%63[^;]", device.user_id, device.password, diag_device.user_id, diag_device.password);
        if ((ret >= 2) && !provisioned) {
            memcpy(&original_diag_device, &diag_device, sizeof(original_diag_device));
#else
        ret = sscanf(&buf[1], "%*d;%11s;%63s", device.user_id, device.password);
        if ((ret == 2) && !provisioned) {
#endif
            provisioned = 1;
            memcpy(&original_device, &device, sizeof(original_device));
        }
    }

    set_leds(1, 0, 0);

    set_leds(0, 1, 0);

    set_leds(0, 1, 1);

    int idlen = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        idlen += sprintf(&event[idlen], "%02X", effmi->unique_id[k]);

#if 0  // enable, with UART on SCI driver instance, for MQTT debug logging
    logger_init((void *) &g_uart0, 0);
#endif

#ifdef USE_M1DIAG
    m1_diag_initdata_t initdata;
    strcpy(initdata.email_address, "your@email.add");
    int sn_len = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        sn_len += sprintf(&initdata.serial_number[sn_len], "%02X", effmi->unique_id[k]);
    strcpy(initdata.model_number, "s5d9-revB");
    snprintf(initdata.fw_version, sizeof(initdata.fw_version), "%c%c%c%c%c%c%c%c-%d.%d",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR);
    ret = m1_diag_start("mqtt.mediumone.com",
                            61618,
                            &diag_project,
                            &diag_registration,
                            &diag_device,
                            event,
                            5,
                            5,
                            60,
                            1,
                            ssl_mem,
                            SSL_MEM_SIZE,
                            &g_http_packet_pool,
                            &g_http_ip,
                            &g_dns_client,
                            900,
                            &initdata,
                            debug,
                            NULL);
    if ((ret == M1_SUCCESS) && strncmp(original_diag_device.password, diag_device.password, sizeof(original_diag_device.password))) {
        sprintf(buf,
                "P%d;%.11s;%.63s;%.11s;%.63s",
                strlen(device.user_id) + strlen(device.password) + strlen(diag_device.user_id) + strlen(diag_device.password) + 3,
                device.user_id,
                device.password,
                diag_device.user_id,
                diag_device.password);
        uint32_t blocks = bytes_to_df_blocks(strlen(buf));
        status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
        if (status)
            goto err;
        status = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, blocks);
        if (status)
            goto err;
        status = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)buf, 0x40100000, blocks * DATA_FLASH_BLOCK_SIZE);
        if (status)
            goto err;
        status = g_flash0.p_api->close(g_flash0.p_ctrl);
        if (status)
            goto err;
    }
#endif

    ret = m1_register_subscription_callback(m1_callback);
    ret = m1_auto_enroll_connect(p_mqtt_broker_host,
                        mqtt_broker_port,
                        &project,
                        &registration,
                        &device,
                        "s5d9",
                        5,
                        5,
                        60,
                        1,
                        ssl_mem,
                        SSL_MEM_SIZE,
                        &g_http_packet_pool,
                        &g_http_ip,
                        &g_dns_client);

    if (ret != M1_SUCCESS)
        goto err;

    set_leds(0, 0, 1);

    if (strncmp(original_device.password, device.password, sizeof(original_device.password))) {
#ifdef USE_M1DIAG
        if (!status)
            sprintf(buf,
                    "P%d;%s;%s;%s;%s",
                    strlen(device.user_id) + strlen(device.password) + strlen(diag_device.user_id) + strlen(diag_device.password) + 3,
                    device.user_id,
                    device.password,
                    diag_device.user_id,
                    diag_device.password);
        else
            sprintf(buf,
                    "P%d;%s;%s;;",
                    strlen(device.user_id) + strlen(device.password) + 3,
                    device.user_id,
                    device.password);
#else
        sprintf(buf,
                "P%d;%s;%s",
                strlen(device.user_id) + strlen(device.password) + 1,
                device.user_id,
                device.password);
#endif
        uint32_t blocks = bytes_to_df_blocks(strlen(buf));
        status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
        if (status)
            goto err;
        status = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, blocks);
        if (status)
            goto err;
        status = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)buf, 0x40100000, blocks * DATA_FLASH_BLOCK_SIZE);
        if (status)
            goto err;
        status = g_flash0.p_api->close(g_flash0.p_ctrl);
        if (status)
            goto err;
    }

    unsigned long mac_address[2];
    nx_ip_interface_info_get(&g_http_ip, 0, NULL, NULL, NULL, NULL, &mac_address[0], &mac_address[1]);
    sprintf(event,
            "{\"firmware_version\":\"%c%c%c%c%c%c%c%c-%d.%d\",\"mac_address\":\"%02lX:%02lX:%02lX:%02lX:%02lX:%02lX\"}",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR,
            (mac_address[0] >> 8) & 0xff,
            mac_address[0] & 0xff,
            (mac_address[1] >> 24) & 0xff,
            (mac_address[1] >> 16) & 0xff,
            (mac_address[1] >> 8) & 0xff,
            mac_address[1] & 0xff);
    m1_publish_event(event, NULL);
#ifdef USE_M1DIAG
    M1_LOG(info, "M1 VSA connected", 0);
    sprintf((char *)buf, "{\"mqtt_user_id\":\"%s\",\"mqtt_project_id\":\"%s\",\"m1_api_key\":\"%s\"}",
            device.user_id,
            project.proj_id,
            project.apikey);
    m1_diag_update_configuration((char *)buf);
#endif

    status = g_i2c0.p_api->open(g_i2c0.p_ctrl, g_i2c0.p_cfg);
    if (status)
        goto err;

    tx_thread_resume(&adc_thread);
    tx_thread_resume(&bmc_thread);
    tx_thread_resume(&ens_thread);
    tx_thread_resume(&ms_thread);

#ifdef USE_M1DIAG
    m1_diag_register_acceleration(&x_accel_access, &y_accel_access, &z_accel_access);
    m1_diag_register_humidity(&humidity_access);
    m1_diag_register_periphery("temp_ambient_c_1", M1_DIAG_FLOAT, &temp_access);
    m1_diag_register_periphery("x_magnetic_t", M1_DIAG_FLOAT, &xmag_access);
    m1_diag_register_periphery("y_magnetic_t", M1_DIAG_FLOAT, &ymag_access);
    m1_diag_register_periphery("z_magnetic_t", M1_DIAG_FLOAT, &zmag_access);
    m1_diag_register_periphery("temp_ambient_c_2", M1_DIAG_FLOAT, &temp2_access);
    m1_diag_register_periphery("temp_ambient_c_3", M1_DIAG_FLOAT, &temp3_access);
    m1_diag_register_periphery("pressure_mbar", M1_DIAG_FLOAT, &pressure_access);
    m1_diag_register_periphery("mic", M1_DIAG_FLOAT, &mic_access);
#endif

    start = tx_time_get();

    while (1) {
        actual_flags = tx_time_get() - start;
        tx_thread_sleep(actual_flags <= sample_period  ? sample_period - actual_flags : 0);
        start = tx_time_get();
        tx_event_flags_set(&g_sensor_event_flags, 0x0000000F, TX_OR);
        status = tx_event_flags_get(&g_sensor_event_flags, 0x000000F0, TX_AND_CLEAR, &actual_flags, 100);
        if (status == TX_NO_EVENTS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Sensor threads did not respond in time", status);
            tx_thread_sleep(500);
#endif
            // most likely means deadlock (i2c callback not called)
            SCB->AIRCR = 0x05FA0004;
        }
        int index = 1;
        event[0] = '{';
        event[1] = '\0';
        int i;
        for (i = 0; i < SENSORS; i++) {
            if (sensors[i]->count) {
                index += sprintf(&event[index],
                                "\"%s\":{\"value\":%f,\"avg\":%f,\"min\":%f,\"max\":%f,\"samples\":%d},",
                                sensors[i]->name, sensors[i]->value, sensors[i]->total / (float)sensors[i]->count,
                                sensors[i]->min, sensors[i]->max, sensors[i]->count);
            }
        }
        sprintf(&event[index],
                "\"x_zero_crossings\":%lu,\"y_zero_crossings\":%lu,\"z_zero_crossings\":%lu}",
                g_x_zero_crossings, g_y_zero_crossings, g_z_zero_crossings);
        m1_publish_event(event, NULL);
    }

err:
#ifdef USE_M1DIAG
    M1_LOG(error, "Critical error (flash, i2c, or initial connection)", ret + status);
#endif
    while (1) {
        set_leds(1, 1, 1);
        tx_thread_sleep(100);
        set_leds(0, 0, 0);
        tx_thread_sleep(100);
    }
}

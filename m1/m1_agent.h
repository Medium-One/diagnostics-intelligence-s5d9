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
#ifndef INCLUDE_M1_AGENT_H_
#define INCLUDE_M1_AGENT_H_


#include <nx_api.h>
#include <nx_dns.h>
#include <m1.h>


/*
 * Description:
 * 		Establishes a MQTT connection to Medium One using the provided credentials. If device::user_id
 * 		has a non-zero length, device credentials are used when attempting to connect to the Medium One
 * 		MQTT broker. If the credentials are rejected, and registration credentials are provided, the
 * 		connection will be re-attempted with registration credentials. If the connection is successful
 * 		with registration credentials, enrollment will be initiated to obtain credentials for this device.
 * 		Enrollment is dependent on the device having a unique_id provided by FMI. If successful, the new
 * 		credentials will be returned in device.
 *
 * 		A dedicated M1 agent thread is created, if it does not already exist. During m1_auto_enroll_connect(),
 * 		if connection is successful, the following data is transmitted to
 * 		the Medium One "raw" datastream:
 * 			“connected”: true
 * 			"ip_address": <WAN_IP>
 * 			"unique_id": <Unique ID> (from FMI)
 * 			"product_name": <Product Name> (from FMI)
 * 			"product_market": <Product Marking> (from FMI)
 * 			"mask_revision": <Mask Revision> (from FMI)
 * 			"quality_code": <Quality Code> (from FMI)
 * 			"ssp_version": <SSP Version> (ssp_version_t) (from FMI)
 * 			"mac_address": <mac address> (if available) (from NetX)
 * 			"lan_address": <lan_address> (if available) (from NetX)
 * 		Finally a subscription is made to the Medium One subscription topics for the specified user.
 * 		The dedicated agent thread maintains and monitors the MQTT connection. If the connection drops,
 * 		the agent will continuously try to reconnect, with a delay of 5 seconds between attempts.
 * 		This is a blocking call.
 * Parameters:
 * 		mqtt_url: 		MQTT broker
 * 		mqtt_port: 		MQTT port
 * 		project:		MQTT project ID (provided by docs) and project API key
 * 		registration: 	optional MQTT user ID and password for API registration user
 * 		device: 		MQTT user ID and password for API basic user
 * 		device_id:		device identifier, maximum 60 characters
 * 		retry_limit: 	The number of times to retry (recommend 5 times)
 * 		retry_delay: 	The time delay (seconds) between retry (recommend 5 seconds)
 * 		mqtt_heart_beat:period (seconds) between MQTT ping (recommended 300 seconds)
 * 		tls_enabled: 	1 to use TLS
 *		ssl_mem:		optional pointer to start of contiguous memory block to be used by SSL.
 *						should be ~30k for successful TLS connections. if not specified, memory is allocated from heap
 *		ssl_mem_size:	optional size of contiguous memory block pointed to by ssl_mem
 *		p_ppool:		pointer to NX_PACKET_POOL instance to use
 *		p_ip:			pointer to NX_IP instance to use
 *		p_dns:			pointer to NX_DNS instance to use
 * Errors:
 * 		M1_ERROR_INVALID_URL -			must end with "mediumone.com"
 * 		M1_ERROR_UNABLE_TO_CONNECT - 	returned if a fatal error occurs. fatal errors include:
 *									 		- Network unavailable
 *                                          - Incorrect MQTT broker URL or port
 *                                          - Using TLS with non-TLS port, or vice-versa
 *		M1_ERROR_ALREADY_CONNECTED -	returned if already connected
 *		M1_ERROR_BAD_CREDENTIALS - 		returned if
 *		                                    - device is NULL
 *                                          - device credentials are incorrect and registration is NULL
 *                                          - device credentials are incorrect and registration credentials are incorrect
 *
 * Success:
 * 		return 0
 */
int m1_auto_enroll_connect(const char * mqtt_url,
			 			   	int mqtt_port,
			                const project_credentials_t * project,
			                const user_credentials_t * registration,
			                user_credentials_t * device,
							const char * device_id,
							int retry_limit,
							int retry_delay,
							int mqtt_heart_beat,
							int tls_enabled,
							void * ssl_mem,
							int ssl_mem_size,
			                NX_PACKET_POOL * p_ppool,
			                NX_IP * p_ip,
			                NX_DNS * p_dns);

/*
 *
 * ***DEPRECATED***: This API will be removed in a future version.
 * Use m1_auto_enroll_connect instead.
 * This API dynamically allocates ~30k from the heap. Please make sure your heap
 * is sized large enough to accomodate this.
 *
 * Description:
 *      Establishes a MQTT connection to Medium One using the provided credentials.
 * 		A dedicated M1 agent thread is created, if it does not already exist. During m1_connect(),
 * 		if connection is successful, the following data is transmitted to
 * 		the Medium One "raw" datastream:
 * 			“connected”: true
 * 			"ip_address": <WAN_IP>
 * 			"unique_id": <Unique ID> (from FMI)
 * 			"product_name": <Product Name> (from FMI)
 * 			"product_market": <Product Marking> (from FMI)
 * 			"mask_revision": <Mask Revision> (from FMI)
 * 			"quality_code": <Quality Code> (from FMI)
 * 			"ssp_version": <SSP Version> (ssp_version_t) (from FMI)
 * 			"mac_address": <mac address> (if available) (from NetX)
 * 			"lan_address": <lan_address> (if available) (from NetX)
 * 		Finally a subscription is made to the Medium One subscription topics for the specified user.
 * 		The dedicated agent thread maintains and monitors the MQTT connection. If the connection drops,
 * 		the agent will continuously try to reconnect, with a delay of 5 seconds between attempts.
 * 		This is a blocking call.
 * Parameters:
 * 		mqtt_url: 		MQTT broker
 * 		mqtt_port: 		MQTT port
 *		mqtt_user_id: 	MQTT user ID (provided by docs)
 *		password: 		API user password
 *		mqtt_project_id:MQTT project ID (provided by docs)
 *		api_key:		API key
 * 		device: 		MQTT user ID and password for API basic user
 * 		device_id:		device identifier, maximum 60 characters
 * 		retry_limit: 	The number of times to retry (recommend 5 times)
 * 		retry_delay: 	The time delay (seconds) between retry (recommend 5 seconds)
 * 		mqtt_heart_beat:period (seconds) between MQTT ping (recommended 300 seconds)
 * 		tls_enabled: 	1 to use TLS
 *		p_ppool:		pointer to NX_PACKET_POOL instance to use
 *		p_ip:			pointer to NX_IP instance to use
 *		p_dns:			pointer to NX_DNS instance to use
 * Errors:
 * 		M1_ERROR_INVALID_URL -			must end with "mediumone.com"
 * 		M1_ERROR_UNABLE_TO_CONNECT - 	returned if a fatal error occurs. fatal errors include:
 *									 		- Network unavailable
 *									 		- Incorrect MQTT broker URL or port
 *									 		- Using TLS with non-TLS port, or vice-versa
 *		M1_ERROR_ALREADY_CONNECTED -	returned if already connected
 *		M1_ERROR_BAD_CREDENTIALS - 		returned if any of mqtt_user_id, password, mqtt_project_id, or api_key are incorrect
 *
 * Success:
 * 		return 0
 */
__attribute__((__deprecated__))
int m1_connect(const char * mqtt_url,
				int mqtt_port,
				char * mqtt_user_id,
				char * password,
				char * mqtt_project_id,
				char * api_key,
				const char * device_id,
				int retry_limit,
				int retry_delay,
				int mqtt_heart_beat,
				int tls_enabled);

/*
 * Description:
 * 		Disconnects from Medium One MQTT broker. This is a blocking call.
 */
void m1_disconnect();

/*
 * Description:
 * 		This sends a json event to Medium One. This is a non-blocking call.
 * Parameters:
 * 		json_payload: serialized JSON object to send to Medium One
 * 			eg. "{\"my_key\": 12345}"
 * 		observed_at: ISO8601-format datetime string. If NULL, not included in event
 * 		(observed_at generated by M1 broker on reception)
 * 			eg. "2016-08-29T16:03:12.794925-07:00"
 * Errors:
 * 		M1_ERROR_NOT_CONNECTED
 * 		M1_ERROR_NULL_PAYLOAD
 * 		M1_ERROR_UNABLE_TO_PUBLISH
 * Success:
 * 		return 0
 */
int m1_publish_event(char * json_payload, char * observed_at);

/*
 * 	Description:
 * 		This function registers a callback function which is called when a message is received from Medium One.
 * 	Parameters:
 * 		subscription_callback: function pointer to callback
 * 			Callback function parameters:
 * 				type: type of message received (1 - cloud to device; 2 - cloud to group)
 * 				topic: full topic identifier for received message (null-terminated string)
 * 				msg: message content
 * 				length: message length
 * 	Errors:
 * 		M1_ERROR_NULL_CALLBACK
 * 	Success:
 * 		return 0
 */
int m1_register_subscription_callback(void (* subscription_callback)(int type, char * topic, char * msg, int length));


#endif /* INCLUDE_M1_AGENT_H_ */

/***********************************************************************************************************************
 * Copyright [2015] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
 * display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
 * purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
 * SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
 * INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
 * be subject to different terms.
 **********************************************************************************************************************/
#include "system_thread.h"
#include "fx_stdio.h"
#include "qspi_filex_io_driver.h"
#include "r_qspi_subsector_erase.h"


extern TX_THREAD net_thread;


void SetMacAddress(nx_mac_address_t *p_mac_config);
void network_setup(FX_MEDIA * pMedia);
void R_BSP_WarmStart (bsp_warm_start_event_t event);
ssp_err_t R_IOPORT_PinCfg (ioport_port_pin_t pin, uint32_t cfg);

static UCHAR    g_qspi_mem[QSPI_SUB_SECTOR_SIZE] BSP_ALIGN_VARIABLE(8);
FX_MEDIA g_qspi_media;

void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_PRE_C == event)
    {
        /* C runtime environment has not been setup so you cannot use globals. System clocks and pins are not setup. */

    }
    else if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment, system clocks, and pins are all setup. */
        /*
         * Don't let USB enumerate until the drive is initialized with all information available
         *
         */
        ssp_feature_t       ssp_feature = {{(ssp_ip_t) 0U}};
        ssp_feature.unit = (uint32_t) SSP_IP_UNIT_USBFS;
        R_BSP_ModuleStop(&ssp_feature);
        R_IOPORT_PinCfg(IOPORT_PORT_04_PIN_07,IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_NMOS_ENABLE);
    }
    else
    {
        /* Do nothing */
    }
}

void SetMacAddress(nx_mac_address_t *p_mac_config)
{
    //  REA's Vendor MAC range: 00:30:55:xx:xx:xx
    fmi_unique_id_t id;
    g_fmi.p_api->uniqueIdGet(&id);

    p_mac_config->nx_mac_address_h = 0x0030;
    p_mac_config->nx_mac_address_l = ((0x55000000) | (id.unique_id[0] &(0x00FFFFFF)));

}

void network_setup(FX_MEDIA * pMedia)
{
    FX_FILE file;
    CHAR * interface_name;

    ULONG  status, actual_status_bits;
    ULONG dns_ip[3];
    ULONG dns_ip_size;
    ULONG ip_address;
    ULONG ip_mask;
    ULONG mtu_size;
    ULONG mac_high;
    ULONG mac_low;

    char msg[80];

    fx_file_delete(pMedia, "ipconfig.txt");
    status = fx_file_create(pMedia, "ipconfig.txt");
    if (status) __BKPT();

    status = fx_file_open(pMedia, &file, "ipconfig.txt", FX_OPEN_FOR_WRITE);
    if (status) __BKPT();

    /** Wait for init to finish. */
    status = nx_ip_interface_status_check(&g_http_ip, 0, NX_IP_LINK_ENABLED, &actual_status_bits, NX_WAIT_FOREVER);
    if (status) __BKPT();

    status = nx_dhcp_start (&g_dhcp);
    if (status) __BKPT();

    status = nx_ip_status_check(&g_http_ip, NX_IP_ADDRESS_RESOLVED, &actual_status_bits, TX_WAIT_FOREVER);
    if (status) __BKPT();

    status = nx_ip_address_get(&g_http_ip, &ip_address, &ip_mask );
    if (status) __BKPT();

    sprintf(msg, "IP Address: %d.%d.%d.%d\r\n", (int)(ip_address>>24), (int)(ip_address>>16)&0xFF, (int)(ip_address>>8)&0xFF, (int)(ip_address)&0xFF );
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    sprintf(msg, "IP Mask: %d.%d.%d.%d\r\n", (int)(ip_mask>>24), (int)(ip_mask>>16)&0xFF, (int)(ip_mask>>8)&0xFF, (int)(ip_mask)&0xFF );
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    dns_ip_size = sizeof(dns_ip);
    status = nx_dhcp_user_option_retrieve(&g_dhcp, NX_DHCP_OPTION_DNS_SVR, (UCHAR *)dns_ip, (UINT *)&dns_ip_size);
    if (status == NX_DHCP_DEST_TO_SMALL) {
        dns_ip[0] = 0x08080808UL;
        status = nx_dns_server_add(&g_dns_client, dns_ip[0]);
        if (status) __BKPT();

        sprintf((char *)msg, "DNS Address 1: %d.%d.%d.%d\r\n", (int)(dns_ip[0]>>24), (int)(dns_ip[0]>>16)&0xFF, (int)(dns_ip[0]>>8)&0xFF, (int)(dns_ip[0])&0xFF );

        status = fx_file_write(&file, msg, strlen(msg));
        if (status) __BKPT();
    } else {
        if (status) __BKPT();

        status = nx_dns_server_add(&g_dns_client, dns_ip[0]);
        if (status) __BKPT();

        sprintf((char *)msg, "DNS Address 1: %d.%d.%d.%d\r\n", (int)(dns_ip[0]>>24), (int)(dns_ip[0]>>16)&0xFF, (int)(dns_ip[0]>>8)&0xFF, (int)(dns_ip[0])&0xFF );

        status = fx_file_write(&file, msg, strlen(msg));
        if (status) __BKPT();

        if (dns_ip_size > 4)
        {
            status = nx_dns_server_add(&g_dns_client, dns_ip[1]);
            if (status) __BKPT();

            sprintf((char *)msg, "DNS Address 2: %d.%d.%d.%d\r\n", (int)(dns_ip[1]>>24), (int)(dns_ip[1]>>16)&0xFF, (int)(dns_ip[1]>>8)&0xFF, (int)(dns_ip[1])&0xFF );
            status = fx_file_write(&file, msg, strlen(msg));
            if (status) __BKPT();
        }
    }

    status = nx_ip_interface_info_get(&g_http_ip, 0, &interface_name, &ip_address, &ip_mask, &mtu_size, &mac_high, &mac_low);
    if (status) __BKPT();

    sprintf(msg, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",(int) (mac_high>>8), (int) (mac_high&0xFF), (int) (mac_low>>24), (int) (mac_low>>16&0xFF),(int) (mac_low>>8&0xFF),(int) (mac_low&0xFF));
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    status = fx_file_close(&file);
    if (status)
           __BKPT();
    status = fx_media_flush(pMedia);
    if (status)
           __BKPT();
}
void  usb_init(void);
/* System Thread entry function */
void system_thread_entry(void)
{
    UINT status;
    status = fx_media_open(&g_qspi_media, (CHAR *) "QSPI Media", _fx_qspi_driver, (void *) &g_qspi, g_qspi_mem, QSPI_SUB_SECTOR_SIZE);
    R_IOPORT_PinCfg(IOPORT_PORT_04_PIN_07,(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS));
    if (FX_SUCCESS == status)
    {
        _fx_stdio_initialize();
        _fx_stdio_drive_register('A', &g_qspi_media);
    }
    network_setup(&g_qspi_media);
    tx_thread_resume(&net_thread);

    /*
     * Make the drive available to the host by enabling enumeration on VBUS
     */
    R_IOPORT_PinCfg(IOPORT_PORT_04_PIN_07,(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS));

    while (1)
    {
        tx_semaphore_get(&g_usb_qspi_active, 200);
        fx_media_flush(&g_qspi_media);
    }
}

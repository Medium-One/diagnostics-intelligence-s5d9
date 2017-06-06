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
#include "r_qspi.h"
#include "../synergy/ssp/src/driver/r_qspi/r_qspi_private.h"
#include "../synergy/ssp/src/driver/r_qspi/r_qspi_private_api.h"
#include <r_qspi_subsector_erase.h>

ssp_err_t R_QSPI_SubSectorErase (void * p_api_ctrl, uint8_t * p_device_address)
{
    uint32_t chip_address = (uint32_t) p_device_address - BSP_PRV_QSPI_DEVICE_PHYSICAL_ADDRESS;
    qspi_instance_ctrl_t * p_ctrl = (qspi_instance_ctrl_t *) p_api_ctrl;

#if QSPI_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(p_ctrl);
    SSP_ASSERT(p_device_address);
#endif

    /* Send command to enable writing */
    HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, QSPI_COMMAND_WRITE_ENABLE);

    /* Close the SPI bus cycle */
    HW_QSPI_DIRECT_COMMUNICATION_ENTER(p_ctrl->p_reg);

    if (4 == p_ctrl->num_address_bytes)
    {
        /* Send command to erase */
        HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, QSPI_COMMAND_4BYTE_SECTOR_ERASE);

        /* Send command to write data */
        HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, (uint8_t)(chip_address >> 24));
    }
    else
    {
        /* Send command to erase */
        HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, QSPI_COMMAND_4BYTE_SUBSECTOR_ERASE);
    }

    HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, (uint8_t)(chip_address >> 16));
    HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, (uint8_t)(chip_address >> 8));
    HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, (uint8_t)(chip_address));

    /* Close the SPI bus cycle */
    HW_QSPI_DIRECT_COMMUNICATION_ENTER(p_ctrl->p_reg);

    /* Send command to disable writing */
    HW_QSPI_BYTE_WRITE(p_ctrl->p_reg, QSPI_COMMAND_WRITE_DISABLE);

    /* Close the SPI bus cycle */
    HW_QSPI_DIRECT_COMMUNICATION_ENTER(p_ctrl->p_reg);

    return SSP_SUCCESS;
}

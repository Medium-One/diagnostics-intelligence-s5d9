/**************************************************************************/
/* */
/* Copyright (c) 1996-2006 by Express Logic Inc. */
/* */
/* This software is copyrighted by and is the sole property of Express */
/* Logic, Inc. All rights, title, ownership, or other interests */
/* in the software remain the property of Express Logic, Inc. This */
/* software may only be used in accordance with the corresponding */
/* license agreement. Any unauthorized use, duplication, transmission, */
/* distribution, or disclosure of this software is expressly forbidden. */
/* */
/* This Copyright notice may not be removed or modified without prior */
/* written consent of Express Logic, Inc. */
/* */
/* Express Logic, Inc. reserves the right to modify this software */
/* without notice. */
/* */
/* Express Logic, Inc. info@expresslogic.com */
/* 11423 West Bernardo Court http://www.expresslogic.com */
/* San Diego, CA 92127 */
/* */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/** */
/** FileX Component */
/** */
/** RAM Disk Driver */
/** */
/**************************************************************************/
/**************************************************************************/
/* Include necessary system files. */
#include "tx_api.h"
#include "fx_api.h"
#include "r_qspi.h"
#include "qspi_filex_io_driver.h"
#include "r_qspi_subsector_erase.h"
#include "system_thread.h"


static LONG last_subsect_num = -1;
static UCHAR last_subsect_data[QSPI_SUB_SECTOR_SIZE] BSP_ALIGN_VARIABLE(8);

UINT usb_flush_media(void);

UINT usb_media_read (void * storage, ULONG lun, UCHAR * p_dest, ULONG blocks, ULONG lba, ULONG * status)
{
    SSP_PARAMETER_NOT_USED(storage);
    SSP_PARAMETER_NOT_USED(lun);
    SSP_PARAMETER_NOT_USED(status);

    tx_semaphore_ceiling_put(&g_usb_qspi_active, 1);

    UINT ret;

    for (ULONG offset = 0; offset < blocks; offset++)
    {
        /* Cache has the same sub sector */
        if ((LONG) ((lba + offset) >> 3) == last_subsect_num)
        {
            /* Read data from cache */
            memcpy(p_dest + (offset * MEDIA_SECT_SIZE), &last_subsect_data[((lba + offset) & 0x7) * MEDIA_SECT_SIZE], MEDIA_SECT_SIZE);
        }

        /* Cache has different subsector or is empty */
        else
        {
            /* Read data directly from QSPI */

            ret = g_qspi.p_api->read(g_qspi.p_ctrl,
            (uint8_t *) (QSPI_MEDIA_BASE + ((lba + offset) * MEDIA_SECT_SIZE)),
            (uint8_t *) p_dest + (offset * MEDIA_SECT_SIZE), MEDIA_SECT_SIZE);
            if (ret)
                return(ret);

        }
    }

    return (UX_SUCCESS);
}

UINT usb_media_write (void * storage, ULONG lun, UCHAR * p_src, ULONG blocks, ULONG lba, ULONG * status)
{
    SSP_PARAMETER_NOT_USED(storage);
    SSP_PARAMETER_NOT_USED(lun);
    SSP_PARAMETER_NOT_USED(status);

    tx_semaphore_ceiling_put(&g_usb_qspi_active, 1);

    UINT ret;

    for (ULONG offset = 0; offset < blocks; offset++)
    {
        /* Cache empty */
        if (-1 == last_subsect_num)
        {
            /* Cache the subsector and modify it */
            last_subsect_num = (LONG) ((lba + offset) >> 3);

             ret = g_qspi.p_api->read(g_qspi.p_ctrl,
            (uint8_t *) (QSPI_MEDIA_BASE + (last_subsect_num * QSPI_SUB_SECTOR_SIZE)),
            (uint8_t *) last_subsect_data, QSPI_SUB_SECTOR_SIZE);
            if (ret)
                return(ret);

            memcpy(&last_subsect_data[((lba + offset) & 0x7) * MEDIA_SECT_SIZE], p_src + (offset * MEDIA_SECT_SIZE), MEDIA_SECT_SIZE);
        }

        /* Cache has the same subsector */
        else if ((LONG) ((lba + offset) >> 3) == last_subsect_num)
        {
            /* Modify the subsector in cache */
            memcpy(&last_subsect_data[((lba + offset) & 0x7) * MEDIA_SECT_SIZE], p_src + (offset * MEDIA_SECT_SIZE), MEDIA_SECT_SIZE);
        }

        /* Cache has different subsector */
        else
        {
            /* Write current cache to the last subsector */
            bool not_ready = true;

            ret = R_QSPI_SubSectorErase(g_qspi.p_ctrl,
            (uint8_t *) (QSPI_MEDIA_BASE + (last_subsect_num * QSPI_SUB_SECTOR_SIZE)));
            if (ret)
                return(ret);

            while (not_ready)
            {
                ret = g_qspi.p_api->statusGet(g_qspi.p_ctrl, &not_ready);
                if (ret)
                    return(ret);
            }

            for (LONG page = 0; page < (QSPI_SUB_SECTOR_SIZE / QSPI_PAGE_SIZE); page++)
            {
                not_ready = true;

                ret = g_qspi.p_api->pageProgram(g_qspi.p_ctrl,
                (uint8_t *) (QSPI_MEDIA_BASE + ((last_subsect_num * QSPI_SUB_SECTOR_SIZE) + (page * QSPI_PAGE_SIZE))),
                (uint8_t *) &last_subsect_data[page * QSPI_PAGE_SIZE], QSPI_PAGE_SIZE);
                if (ret)
                    return(ret);

                while (not_ready)
                {
                    ret = g_qspi.p_api->statusGet(g_qspi.p_ctrl, &not_ready);
                    if (ret)
                        return(ret);
                }
            }

                /* Cache new subsector and modify it */
            last_subsect_num = (LONG) ((lba + offset) >> 3);

            ret = g_qspi.p_api->read(g_qspi.p_ctrl,
            (uint8_t *) (QSPI_MEDIA_BASE + (last_subsect_num * QSPI_SUB_SECTOR_SIZE)),
            (uint8_t *) last_subsect_data, QSPI_SUB_SECTOR_SIZE);
            if (ret)
                return(ret);

            memcpy(&last_subsect_data[((lba + offset) & 0x7) * MEDIA_SECT_SIZE], p_src + (offset * MEDIA_SECT_SIZE), MEDIA_SECT_SIZE);
        }
    }

    return (UX_SUCCESS);
}

UINT usb_media_status (void * storage, ULONG lun, ULONG id, ULONG * status)
{
    SSP_PARAMETER_NOT_USED(storage);
    SSP_PARAMETER_NOT_USED(lun);
    SSP_PARAMETER_NOT_USED(id);
    SSP_PARAMETER_NOT_USED(status);

    tx_semaphore_ceiling_put(&g_usb_qspi_active, 1);
    *status = 0;

    return (UX_SUCCESS);
}

UINT usb_flush_media(void)
{
    UINT ret;

    /* Cache empty */
    if (-1 == last_subsect_num)
        return(UX_SUCCESS);

    /* Write current cache to the last subsector */
    bool not_ready = true;

    ret = R_QSPI_SubSectorErase(g_qspi.p_ctrl, (uint8_t *) (QSPI_MEDIA_BASE + (last_subsect_num * QSPI_SUB_SECTOR_SIZE)));
    if (ret)
        return(ret);

    while (not_ready)
    {
        ret = g_qspi.p_api->statusGet(g_qspi.p_ctrl, &not_ready);
        if (ret)
            return(ret);
    }

    for (LONG page = 0; page < (QSPI_SUB_SECTOR_SIZE / QSPI_PAGE_SIZE); page++)
    {
        not_ready = true;

        ret = g_qspi.p_api->pageProgram(g_qspi.p_ctrl,
        (uint8_t *) (QSPI_MEDIA_BASE + ((last_subsect_num * QSPI_SUB_SECTOR_SIZE) + (page * QSPI_PAGE_SIZE))),
        (uint8_t *) &last_subsect_data[page * QSPI_PAGE_SIZE], QSPI_PAGE_SIZE);
        if (ret)
            return(ret);

        while (not_ready)
        {
            ret = g_qspi.p_api->statusGet(g_qspi.p_ctrl, &not_ready);
            if (ret)
                return(ret);
        }
    }

    last_subsect_num = -1;
    /* Clean up the cache */
    memset(last_subsect_data, 0, QSPI_SUB_SECTOR_SIZE);

    return (UX_SUCCESS);
}

/**************************************************************************/
/* */
/* FUNCTION RELEASE */
/* */
/* _fx_ram_driver PORTABLE C */
/* 5.0 */
/* AUTHOR */
/* */
/* William E. Lamie, Express Logic, Inc. */
/* */
/* DESCRIPTION */
/* */
/* This function is the entry point to the generic RAM disk driver */
/* that is delivered with all versions of FileX. The format of the */
/* RAM disk is easily modified by calling fx_media_format prior */
/* to opening the media. */
/* */
/* This driver also serves as a template for developing FileX drivers */
/* for actual devices. Simply replace the read/write sector logic with */
/* calls to read/write from the appropriate physical device */
/* */
/* FileX RAM/FLASH structures look like the following: */
/* */
/* Physical Sector Contents */
/* */
/* 0 Boot record */
/* 1 FAT Area Start */
/* +FAT Sectors Root Directory Start */
/* +Directory Sectors Data Sector Start */
/* */
/* */
/* INPUT */
/* */
/* media_ptr Media control block pointer */
/* */
/* OUTPUT */
/* */
/* None */
/* */
/* CALLS */
/* */
/* _fx_utility_memory_copy Copy sector memory */
/* */
/* CALLED BY */
/* */
/* FileX System Functions */
/* */
/* RELEASE HISTORY */
/* */
/* DATE NAME DESCRIPTION */
/* */
/* 12-12-2005 William E. Lamie Initial Version 5.0 */
/* */
/**************************************************************************/
VOID _fx_qspi_driver(FX_MEDIA *media_ptr)
{
    UCHAR *source_address;
    UCHAR *dest_address;
    UINT size;
    ssp_err_t err;
    bool qspi_we_active; /* is the QSPI write/erase active */
    qspi_instance_t  *qspi = media_ptr->fx_media_driver_info;
    ULONG lba;

    /* Process the driver request specified in the media control block. */
    switch(media_ptr -> fx_media_driver_request)
    {
        case FX_DRIVER_READ:
        {
            lba = (media_ptr -> fx_media_driver_logical_sector + media_ptr ->fx_media_hidden_sectors);
            err = usb_media_read (NULL, 0, media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_driver_sectors, lba, NULL);
            if(SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_WRITE:
        {

            lba = (media_ptr -> fx_media_driver_logical_sector + media_ptr ->fx_media_hidden_sectors);

            err = usb_media_write (NULL, 0, media_ptr->fx_media_driver_buffer, media_ptr->fx_media_driver_sectors,  lba, NULL);
            if(SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_FLUSH:
        {
            usb_flush_media();
            /* Return driver success. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_ABORT:
        {
            /* Return driver success. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_INIT:
        {
            last_subsect_num = -1;

            err = qspi->p_api->open(qspi->p_ctrl, qspi->p_cfg);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Select Bank Zero */
            err = qspi->p_api->bankSelect(0);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_UNINIT:
        {
            usb_flush_media();
            err = qspi->p_api->close(qspi->p_ctrl);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_BOOT_READ:
        {
            /* Read the boot record and return to the caller. */

            /* Select Bank Zero */
            err = qspi->p_api->bankSelect(0);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            size = QSPI_SUB_SECTOR_SIZE;
            err = qspi->p_api->read(qspi->p_ctrl,(uint8_t *)(ADDRESS_ZERO + QSPI_HIDDEN_SECTORS) ,media_ptr -> fx_media_driver_buffer, size);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* determine if the boot record is valid. */
            if ((media_ptr -> fx_media_driver_buffer[0] != (UCHAR) 0xEB) ||
                    (media_ptr -> fx_media_driver_buffer[2] != (UCHAR) 0x90))
            {
                /* Invalid boot record, return an error! */
                media_ptr -> fx_media_driver_status = FX_MEDIA_INVALID;
                return;
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break;
        }
        case FX_DRIVER_BOOT_WRITE:
        {
            /* Write the boot record and return to the caller. */

            /* Select Bank Zero - this is in QSPI peripheral - not the external QSPI device */
            /* I think this is only required when accessing the QSPI via  XIP in the CPU address */
            /* space, not when writing via address and command */
            err = qspi->p_api->bankSelect(0);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* The boot sector is at address zero in the QSPI, after the hidden sectors */
            /* Erase the 4Kb sector */
            dest_address = (uint8_t *)(ADDRESS_ZERO + QSPI_HIDDEN_SECTORS);
            err =  R_QSPI_SubSectorErase(qspi->p_ctrl, dest_address);
            // err = qspi->p_api->sectorErase(qspi->p_ctrl, dest_address);
            if (SSP_SUCCESS != err)
            {
                /* return an error! */
                media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                return;
            }

            /* Wait for the erase operation to complete */
            do{
                err = qspi->p_api->statusGet(qspi->p_ctrl, &qspi_we_active);
                if(SSP_SUCCESS != err)
                {
                    /* return an error! */
                    media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                    return;
                }
            } while (true == qspi_we_active);

            /* Write the sector */
            size = media_ptr -> fx_media_bytes_per_sector;
            source_address = media_ptr -> fx_media_driver_buffer;

            while (size > QSPI_ZERO_BYTES)
            {

                if (size > QSPI_PAGE_SIZE)
                {
                    err = qspi->p_api->pageProgram(qspi->p_ctrl, (uint8_t *)dest_address, (uint8_t *)source_address, QSPI_PAGE_SIZE);
                    size -= QSPI_PAGE_SIZE;
                    source_address += QSPI_PAGE_SIZE;
                    dest_address += QSPI_PAGE_SIZE;
                }
                else
                { /* size is QSPI_PAGE_SIZE or less, only one page left to program */
                    err = qspi->p_api->pageProgram(qspi->p_ctrl, (uint8_t *)dest_address, (uint8_t *)source_address, size);
                    size -= size;
                    /* no need to increment the addresses, as we have finished */
                }

                if(SSP_SUCCESS != err)
                {
                    /* return an error! */
                    media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                    return;
                }

                do{
                    err = qspi->p_api->statusGet(qspi->p_ctrl, &qspi_we_active);
                    if(SSP_SUCCESS != err)
                    {
                        /* return an error! */
                        media_ptr -> fx_media_driver_status = FX_IO_ERROR;
                        return;
                    }
                } while (true == qspi_we_active);
            }

            /* Successful driver request. */
            media_ptr -> fx_media_driver_status = FX_SUCCESS;
            break ;
        }
        default:
        {
            /* Invalid driver request. */
            media_ptr -> fx_media_driver_status = FX_IO_ERROR;
            break;
        }
    }
}



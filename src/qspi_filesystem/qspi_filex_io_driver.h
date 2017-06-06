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

#ifndef QSPI_FILEX_IO_DRIVER_H_
#define QSPI_FILEX_IO_DRIVER_H_

VOID _fx_qspi_driver(FX_MEDIA *media_ptr);

#define QSPI_SECTOR_SIZE        (64 * 1024)
#define QSPI_SUB_SECTOR_SIZE    (4 * 1024)
#define QSPI_NUM_SECTORS        4096       /* Not the actual number, smaller to start with */
#define QSPI_HIDDEN_SECTORS     0

#define ADDRESS_ZERO            BSP_PRV_QSPI_DEVICE_PHYSICAL_ADDRESS
#define QSPI_PAGE_SIZE          256
#define QSPI_ZERO_BYTES         0

#define MEDIA_SECT_SIZE     (0x200)
#define MEDIA_TOTAL_SECTORS (32768)

#define QSPI_MEDIA_BASE    BSP_PRV_QSPI_DEVICE_PHYSICAL_ADDRESS

#endif /* QSPI_FILEX_IO_DRIVER_H_ */

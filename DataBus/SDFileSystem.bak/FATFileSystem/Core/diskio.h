/*-----------------------------------------------------------------------
/  Low level disk interface module include file  R0.06   (C)ChaN, 2007
/-----------------------------------------------------------------------*/

//Modified by Thomas Hamilton, Copyright 2010

#ifndef _DISKIO
#define _DISKIO

#define _READONLY   0
#define _USE_IOCTL  1

#include "integer.h"
#include "FATFileSystem.h"
#include <stdio.h>

/* Status of Disk Functions */
typedef BYTE    DSTATUS;
#define STA_NOINIT  0x01    /* Drive not initialized */
#define STA_NODISK  0x02    /* No medium in the drive */
#define STA_PROTECT 0x04    /* Write protected */
/* Results of Disk Functions */
typedef enum
{
    RES_OK = 0,     /* 0: Successful */
    RES_ERROR,      /* 1: R/W Error */
    RES_WRPRT,      /* 2: Write Protected */
    RES_NOTRDY,     /* 3: Not Ready */
    RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;

/* Prototypes for disk control functions */
DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);
DRESULT disk_read (BYTE, BYTE*, DWORD, BYTE);
#if    _READONLY == 0
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
#endif
DRESULT disk_ioctl (BYTE, BYTE, void*);
/* Command code for disk_ioctrl() */
#define CTRL_SYNC           0   /* Mandatory for read/write configuration */
#define GET_SECTOR_COUNT    1   /* Mandatory for only f_mkfs() */
#define GET_SECTOR_SIZE     2
#define GET_BLOCK_SIZE      3   /* Mandatory for only f_mkfs() */
#define CTRL_POWER          4
#define CTRL_LOCK           5
#define CTRL_EJECT          6
#define MMC_GET_TYPE        10  /* MMC/SDC command */
#define MMC_GET_CSD         11
#define MMC_GET_CID         12
#define MMC_GET_OCR         13
#define MMC_GET_SDSTAT      14
#define ATA_GET_REV         20  /* ATA/CF command */
#define ATA_GET_MODEL       21
#define ATA_GET_SN          22

#endif
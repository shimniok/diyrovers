/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

//Modified by Thomas Hamilton, Copyright 2010

#include "diskio.h"

DSTATUS disk_initialize(BYTE drv) 
{
    if (FATFileSystem::DriveArray[drv])
    {
        return (DSTATUS)FATFileSystem::DriveArray[drv]->disk_initialize();
    }
    else
    {
        return STA_NOINIT;
    }
}

DSTATUS disk_status(BYTE drv) 
{
    if (FATFileSystem::DriveArray[drv])
    {
        return (DSTATUS)FATFileSystem::DriveArray[drv]->disk_status();
    }
    else
    {
        return STA_NOINIT;
    }
}

DRESULT disk_read(BYTE drv, BYTE* buff, DWORD sector, BYTE count)
{
    if (FATFileSystem::DriveArray[drv])
    {
        return (DRESULT)FATFileSystem::DriveArray[drv]->disk_read((unsigned char*)buff,
            (unsigned long)sector, (unsigned char)count);
    }
    else
    {
        return RES_NOTRDY;
    }
}

#if _READONLY == 0
DRESULT disk_write(BYTE drv, const BYTE* buff, DWORD sector, BYTE count)
{
    if (FATFileSystem::DriveArray[drv])
    {
        return (DRESULT)FATFileSystem::DriveArray[drv]->disk_write((const unsigned char*)buff,
            (unsigned long)sector, (unsigned char)count);
    }
    else
    {
        return RES_NOTRDY;
    }
}
#endif

DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void* buff)
{
    switch (ctrl)
    {
        case CTRL_SYNC:
            if (FATFileSystem::DriveArray[drv])
            {
                return (DRESULT)FATFileSystem::DriveArray[drv]->disk_sync();
            }
            else
            {
                return RES_NOTRDY;
            } 

        case GET_SECTOR_SIZE:
            if (FATFileSystem::DriveArray[drv])
            {
                WORD Result = FATFileSystem::DriveArray[drv]->disk_sector_size();
                if (Result > 0)
                {
                    *((WORD*)buff) = Result;
                    return RES_OK;
                }
                else
                {
                    return RES_ERROR;
                }
            }
            else
            {
                return RES_NOTRDY;
            }

        case GET_SECTOR_COUNT:
            if (FATFileSystem::DriveArray[drv])
            {
                DWORD Result = FATFileSystem::DriveArray[drv]->disk_sector_count();
                if (Result > 0)
                {
                    *((DWORD*)buff) = Result;
                    return RES_OK;
                }
                else
                {
                    return RES_ERROR;
                }
            }
            else
            {
                return RES_NOTRDY;
            }

        case GET_BLOCK_SIZE:
            if (FATFileSystem::DriveArray[drv])
            {
                DWORD Result = FATFileSystem::DriveArray[drv]->disk_block_size();
                if (Result > 0)
                {
                    *((DWORD*)buff) = Result;
                    return RES_OK;
                }
                else
                {
                    return RES_ERROR;
                }
            }
            else
            {
                return RES_NOTRDY;
            }

        default:
            return RES_PARERR;
    }
}
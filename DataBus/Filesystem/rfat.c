/*
 * Copyright (c) 2014 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#define LM4F120_DISK_CONFIG_CRC       0
#define LM4F120_DISK_CONFIG_OPTIMIZE  1

#include "rfat.h"
#include "lm4f120h5qr.h"

/***********************************************************************************************************************/
/***********************************************************************************************************************/

int strcmp(const char *s1, const char *s2)
{
    const uint8_t *us1 = (const uint8_t*)s1;
    const uint8_t *us2 = (const uint8_t*)s2;
    int diff = 0;
    int cc;

    do
    {
	cc = *(us1++);
	diff = cc - (int)(*us2++);

	if (diff != 0)
	{
	    break;
	}
    } 
    while (cc != '\0');

    return diff;
}

char *strcat(char * restrict s1, const char * restrict s2)
{
    uint8_t *us1 = (uint8_t*)s1;
    const uint8_t *us2 = (const uint8_t*)s2;
    int cc;

    do
    {
	cc = *(us1++);
    }
    while (cc != '\0');

    us1--;

    do
    {
	cc = *(us1++) = *(us2++);
    }
    while (cc != '\0');

    return s1;
}


size_t strlen(const char *s)
{
    const uint8_t *us = (const uint8_t*)s;
    int cc;

    do
    {
	cc = *(us++);
    }
    while (cc != '\0');

    return (size_t)((const char*)us - s);
}

void *memset(void *s1, int c, size_t n)
{
    if (n != 0)
    {
	uint8_t * restrict us1 = (uint8_t*)s1;
	
	do 
	{
	    *(us1++) = (uint8_t)c;
	}
	while (--n != 0);
    }

    return s1;
}

void *memcpy(void * restrict s1, const void * restrict s2, size_t n)
{
    if (n != 0)
    {
	uint8_t * restrict us1 = (uint8_t*)s1;
	const uint8_t * restrict us2 = (const uint8_t*)s2;
	
	do 
	{
	    *(us1++) = *(us2++);
	}
	while (--n != 0);
    }

    return s1;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/

/*
 * Driver Interface (MMC/SPI, MMC/SDIO):
 *
 * Errors:
 *
 * F_ERR_INITFUNC
 *
 *    Generic error during initialization (malloc failure, RTOS failure)
 *
 * F_ERR_CARDREMOVED
 *
 *   Card did not repsond within timeout. This error is sticky till rfat_disk_release() is called.
 *
 * F_ERR_ONDRIVE
 *
 *   An error showed up while sending a command to the card. Anything, including CRC is covered.
 *
 * F_ERR_INVALIDSECTOR
 *
 *   ECC error during read.
 *
 * F_ERR_READ
 *
 *   A read did not succeed. Internal card error as well as CRC.
 *
 * F_ERR_WRITE
 *
 *   A write did not succeed. Internal card error as well as CRC.
 *
 * F_ERR_INVALIDMEDIA
 *
 *   Card did not respond or is not supported during the initial initialization.
 *
 *
 * A read_multiple/write_mulitple operation is always open. If it is terminated (by another command),
 * there can only be a F_ERR_CARDREMOVED (timed out on write, or command error for CMD_STOP_TRANSFER).
 * CRC errors can be retried. ECC errors on read can be retried. Timeouts in commands can be retried.
 *
 *
 * rfat_disk_read() is used for FAT reads only.
 *
 * rfat_disk_write() is used for FAT/DIR updates only.
 *
 * rfat_disk_read_multiple() is used for DIR/DATA reads with the assumption that prefetching will be in effect.
 *
 * rfat_disk_write_multiple() is used for DATA writes with the assumption that write buffering will be in effect.
 *
 * rfat_disk_release() clears a sticky F_ERR_CARDREMOVED. However it still can return a F_ERR_CARDREMOVED if a timeout
 * occurs while cleaning up pending operations.   
 *
 *
 * Return values for rfat_disk_status():
 *
 * RFAT_DISK_STATUS_PRESENT
 * RFAT_DISK_STATUS_WRITEPROTECT
 *
 * API:
 *
 * int rfat_disk_init(rfat_volume_t *volume);
 * uint32_t rfat_disk_status(rfat_volume_t *volume);
 * int rfat_disk_capacity(rfat_volume_t *volume, uint32_t * p_block_count, uint32_t * p_block_size);
 * int rfat_disk_serial(rfat_volume_t *volume, uint32_t * p_serial);
 * int rfat_disk_read(rfat_volume_t *volume, uint32_t address, void *data);
 * int rfat_disk_read_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, void *data);
 * int rfat_disk_write(rfat_volume_t *volume, uint32_t address, const void *data);
 * int rfat_disk_write_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, const void *data);
 * int rfat_disk_release(rfat_volume_t *volume);
 */

#define RFAT_DISK_STATUS_PRESENT      1
#define RFAT_DISK_STATUS_WRITEPROTECT 2

/***********************************************************************************************************************/
/***********************************************************************************************************************/

/*
 * clsno/blkno/blkno_e addressing:
 *
 *
 * (1) seek:
 *
 * if (file->length == 0)
 * {
 *     file->clsno = CLUSTER_NONE;
 *     file->blkno = BLKNO_NONE;
 *     file->blkno_e = BLKNO_NONE;
 * }
 * else
 * {
 *     if (file->position == 0)
 *     {
 *         file->clsno = file->first_clsno;
 *         file->blkno = volume->cls_blk_offset + (file->clsno << volume->cls_blk_shift);
 *         file->blkno_e = file->blkno + volume->cls_blk_size;
 *     }
 *     else
 *     {
 *         file->clsno = clsno(file->position -1);
 *
 *         if (!(file->position & volume->cls_mask))
 *         {
 *             file->blkno = volume->cls_blk_offset + ((file->clsno +1) << volume->cls_blk_shift);
 *         }
 *         else
 *         {
 *            file->blkno = volume->cls_blk_offset + (file->clsno << volume->cls_blk_shift) + ((file->position & volume->cls_mask) >> BLK_SHIFT);
 *         }
 *
 *         file->blkno_e = volume->cls_blk_offset + ((file->clsno +1) << volume->cls_blk_shift);
 *     }
 * }
 *
 *
 * (2) step:
 *
 * if (file->blkno == file->blkno_e)
 * {
 *      _rfat_cluster_chain_seek(volume, position->clsno, 1, &clsno);
 * 
 *      file->clsno = clsno; 
 *      file->blkno = volume->cls_blk_offset + (file->clsno << volume->cls_blk_shift);
 *      file->blkno_e = file->blkno + volume->cls_blk_size;
 * }
 *
 */

/***********************************************************************************************************************/

typedef struct _rfat_cache_entry_t      rfat_cache_entry_t;
typedef struct _rfat_cluster_procs_t    rfat_cluster_procs_t;
typedef struct _rfat_dir_t              rfat_dir_t;
typedef struct _rfat_file_t             rfat_file_t;
typedef struct _rfat_volume_t           rfat_volume_t;

static int rfat_disk_init(rfat_volume_t *volume);
static uint32_t rfat_disk_status(rfat_volume_t *volume);
static int rfat_disk_capacity(rfat_volume_t *volume, uint32_t * p_block_count, uint32_t * p_block_size);
static int rfat_disk_serial(rfat_volume_t *volume, uint32_t * p_serial);
static int rfat_disk_read(rfat_volume_t *volume, uint32_t address, void *data);
static int rfat_disk_read_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, void *data);
static int rfat_disk_write(rfat_volume_t *volume, uint32_t address, const void *data);
static int rfat_disk_write_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, const void *data);
static int rfat_disk_release(rfat_volume_t *volume);
static int rfat_disk_zero(rfat_volume_t *volume, uint32_t address, uint32_t length);

static int rfat_volume_init(rfat_volume_t *volume);
static int rfat_volume_mount(rfat_volume_t *volume);
static int rfat_volume_unmount(rfat_volume_t *volume);
static int rfat_volume_format(rfat_volume_t *volume);
static int rfat_volume_lock(rfat_volume_t *volume);
static int rfat_volume_lock_noinit(rfat_volume_t *volume);
static int rfat_volume_lock_nomount(rfat_volume_t *volume);
static void rfat_volume_unlock(rfat_volume_t *volume);

static int rfat_fat_cache_read(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry);
static int rfat_fat_cache_write(rfat_volume_t *volume, rfat_cache_entry_t *entry);
static void rfat_fat_cache_modify(rfat_volume_t *volume, rfat_cache_entry_t *entry);
static int rfat_fat_cache_flush(rfat_volume_t *volume);

static void rfat_dir_cache_zero(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry);
static int rfat_dir_cache_read(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry);
static int rfat_dir_cache_write(rfat_volume_t *volume, rfat_cache_entry_t *entry);

static int rfat_data_cache_read(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, rfat_cache_entry_t ** p_entry);
static int rfat_data_cache_check(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, rfat_cache_entry_t ** p_entry);
static int rfat_data_cache_write(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry);
static void rfat_data_cache_modify(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry);
static void rfat_data_cache_clean(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry);
static int rfat_data_cache_flush(rfat_volume_t *volume, rfat_file_t *file);
static int rfat_data_cache_invalidate(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, uint32_t blkcnt, int flush);

static int rfat_cluster_read_fat12(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
static int rfat_cluster_write_fat12(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata);
static int rfat_cluster_read_fat16(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
static int rfat_cluster_write_fat16(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata);
static int rfat_cluster_read_fat32(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
static int rfat_cluster_write_fat32(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata);
static int rfat_cluster_chain_seek(rfat_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno);
static int rfat_cluster_chain_create(rfat_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno);
static int rfat_cluster_chain_destroy(rfat_volume_t *volume, uint32_t clsno, int truncate);

static int rfat_path_parse_filename(const uint8_t **p_path, uint8_t *dirname, int wildcard);
static int rfat_path_match_filename(const rfat_dir_t *dir, const uint8_t *dirname, int wildcard);
static void rfat_path_convert_filename(const uint8_t *dirname, uint8_t *filename);
static int rfat_path_merge_filename(const uint8_t *dirname, uint8_t *pathname);
static int rfat_path_parse_label(const uint8_t *label, uint8_t *dirname);
static void rfat_path_convert_label(const uint8_t *dirname, uint8_t *label);
static void rfat_path_setup_directory(rfat_volume_t *volume, rfat_dir_t *dir, const uint8_t *dirname, uint8_t attr, uint32_t clsno, uint32_t length);
static int rfat_path_create_entry(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry);
static int rfat_path_destroy_entry(rfat_volume_t *volume, uint32_t blkno, uint32_t offset);
static int rfat_path_rename_entry(rfat_volume_t *volume, uint8_t *dirname, uint32_t blkno, uint32_t offset);
static int rfat_path_check_busy(rfat_volume_t *volume);
static int rfat_path_check_locked(rfat_volume_t *volume, uint32_t blkno, uint32_t offset);
static int rfat_path_check_empty(rfat_volume_t *volume, uint32_t clsno);
static int rfat_path_find_directory(rfat_volume_t *volume, int wildcard, const uint8_t *path, uint8_t *pathname, uint8_t *dirname, uint32_t *p_clsno);
static int rfat_path_find_entry(rfat_volume_t *volume, int wildcard, uint32_t clsno, uint32_t blkno, uint32_t offset, uint8_t *dirname, uint32_t *p_clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry);
static int rfat_path_find_file(rfat_volume_t *volume, const uint8_t *path, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry);
static int rfat_path_find_label(rfat_volume_t *volume, uint32_t *p_clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry);
static int rfat_path_find_next(rfat_volume_t *volume, F_FIND *find);

static int rfat_file_seek(rfat_volume_t *volume, rfat_file_t *file, uint32_t position);
static int rfat_file_shrink(rfat_volume_t *volume, rfat_file_t *file, uint32_t length);
static int rfat_file_extend(rfat_volume_t *volume, rfat_file_t *file, uint32_t length, int truncate);
static int rfat_file_open(rfat_volume_t *volume, const uint8_t *filename, uint32_t mode, uint32_t length, rfat_file_t **p_file);
static void rfat_file_close(rfat_volume_t *volume, rfat_file_t *file);
static int rfat_file_flush(rfat_volume_t *volume, rfat_file_t *file, int close);
static int rfat_file_read(rfat_volume_t *volume, rfat_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count);
static int rfat_file_write(rfat_volume_t *volume, rfat_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count);


/***********************************************************************************************************************/

#define RFAT_INLINE   __inline__ __attribute__((always_inline))
#define RFAT_NOINLINE __attribute__((noinline))
#define RFAT_NORETURN __attribute__((noreturn))

#define FALSE  0
#define TRUE   1

#if !defined(NULL)
#define NULL   ((void*)0)
#endif

#define RFAT_PORT_CLOCK(_time, _date) (((_time) = 0), ((_date = 0)))
#define RFAT_PORT_INIT(_volume)              (F_NO_ERROR)
#define RFAT_PORT_LOCK(_volume)              (F_NO_ERROR)
#define RFAT_PORT_UNLOCK(_volume)

#define RFAT_CLUSTER_NONE	      0x00000000u
#define RFAT_CLUSTER_FREE	      0x00000000u
#define RFAT_CLUSTER_FIRST            0x00000002u
#define RFAT_CLUSTER_RESERVED         0x0ffffff6u
#define RFAT_CLUSTER_BAD              0x0ffffff7u
#define RFAT_CLUSTER_LAST	      0x0ffffff8u
#define RFAT_CLUSTER_END_OF_CHAIN     0x0fffffffu

#define RFAT_CLUSTER_RESERVED12R      0x00000ff6u
#define RFAT_CLUSTER_RESERVED16R      0x0000fff6u
#define RFAT_CLUSTER_RESERVED32R      0x0fffffffu

#define RFAT_CLUSTER_LAST16R	      0x0ffffu
#define RFAT_CLUSTER_LAST32R	      0x0fffffffu

#define RFAT_FILE_SIZE_MAX            0xffffffffu

#define RFAT_FILE_MODE_READ           0x01u
#define RFAT_FILE_MODE_WRITE          0x02u
#define RFAT_FILE_MODE_APPEND         0x04u
#define RFAT_FILE_MODE_CREATE         0x08u
#define RFAT_FILE_MODE_TRUNCATE       0x10u
#define RFAT_FILE_MODE_TTY            0x40u
#define RFAT_FILE_MODE_DISK           0x80u

#define BLK_SIZE 512
#define BLK_MASK 511
#define BLK_SHIFT 9
#define BLKNO_NONE    0x00000000
#define BLKNO_INVALID 0xffffffff

#define RFAT_HTOFS(_data)  (_data)
#define RFAT_HTOFL(_data)  (_data)
#define RFAT_FTOHS(_data)  (_data)
#define RFAT_FTOHL(_data)  (_data)

static inline uint32_t rfat_ld_uint16(const uint8_t *address)
{
    return (((uint32_t)(address[0]) << 0) |
	    ((uint32_t)(address[1]) << 8));
}

static inline uint32_t rfat_ld_uint32(const uint8_t *address)
{
    return (((uint32_t)(address[0]) <<  0) |
	    ((uint32_t)(address[1]) <<  8) |
	    ((uint32_t)(address[2]) << 16) |
	    ((uint32_t)(address[3]) << 24));
}

static inline void rfat_st_uint8(void *address, uint32_t offset, uint32_t data)
{
    ((uint8_t*)address)[offset+0] = data;
}

static inline void rfat_st_uint16(uint8_t *address, uint32_t data)
{
    address[0] = data >> 0;
    address[1] = data >> 8;
}

static inline void rfat_st_uint32(uint8_t *address, uint32_t data)
{
    address[0] = data >>  0;
    address[1] = data >>  8;
    address[2] = data >> 16;
    address[3] = data >> 24;
}

#define RFAT_LD_UINT8(_address,_offset) \
  ((uint32_t)(((uint8_t*)((uint8_t*)(_address)+(_offset)))[0]))

#define RFAT_LD_UINT16(_address,_offset) \
  ((uint32_t)(((_offset) & 1) \
	      ? rfat_ld_uint16((uint8_t*)(_address)+(_offset))	\
	      : RFAT_FTOHS(((uint16_t*)((uint8_t*)(_address)+(_offset)))[0])))

#define RFAT_LD_UINT32(_address,_offset) \
  ((uint32_t)(((_offset) & 3) \
	      ? (((_offset) & 1) \
		 ? rfat_ld_uint32((uint8_t*)(_address)+(_offset)) \
		 : (((uint32_t)RFAT_FTOHS(((uint16_t*)((uint8_t*)(_address)+(_offset)))[0])) | \
		    ((uint32_t)RFAT_FTOHS(((uint16_t*)((uint8_t*)(_address)+(_offset)))[1]) << 16))) \
	      : RFAT_FTOHL(((uint32_t*)((uint8_t*)(_address)+(_offset)))[0])))

#define RFAT_ST_UINT8(_address,_offset,_data) \
  ((((uint8_t*)((uint8_t*)(_address)+(_offset)))[0]) = (uint8_t)(_data))

#define RFAT_ST_UINT16(_address,_offset,_data) \
  (((_offset) & 1) \
   ? rfat_st_uint16((uint8_t*)(_address)+(_offset),(_data)) \
   : ((((uint16_t*)((uint8_t*)(_address)+(_offset)))[0]) = RFAT_HTOFS((uint16_t)(_data))))

#define RFAT_ST_UINT32(_address,_offset,_data) \
    (((_offset) & 3) \
     ? (((_offset) & 1) \
	? rfat_st_uint32((uint8_t*)(_address)+(_offset),(_data)) \
	: (((((uint16_t*)((uint8_t*)(_address)+(_offset)))[0]) = RFAT_HTOFS((uint32_t)(_data))), \
	   ((((uint16_t*)((uint8_t*)(_address)+(_offset)))[1]) = RFAT_HTOFS((uint32_t)(_data) >> 16)))) \
     : ((((uint32_t*)((uint8_t*)(_address)+(_offset)))[0]) = RFAT_HTOFL((uint32_t)(_data))))

/* FAT12/FAT16/FAT32 */
#define RFAT_BS_jmpBoot               0    /* 3 0xEB, 0x??, 0x90 or 0xE9, 0x?? 0x?? */
#define RFAT_BS_OEMName               3    /* 8 */
#define RFAT_BPB_BytsPerSec           11   /* 2 */
#define RFAT_BPB_SecPerClus           13   /* 1 */
#define RFAT_BPB_RsvdSecCnt           14   /* 2 */
#define RFAT_BPB_NumFATs              16   /* 1 */
#define RFAT_BPB_RootEntCnt           17   /* 2 */
#define RFAT_BPB_TotSec16             19   /* 2 (must be 0 for FAT32; if 0, then use BPB_TotSec32) */
#define RFAT_BPB_Media                21   /* 1 */
#define RFAT_BPB_FATSz16              22   /* 2 (must by 0 for FAT32) */
#define RFAT_BPB_SecPerTrk            24   /* 2 */
#define RFAT_BPB_NumHeads             26   /* 2 */
#define RFAT_BPB_HiddSec              28   /* 4 */
#define RFAT_BPB_TotSec32             32   /* 4 */
#define RFAT_BS_SignatureWord         510  /* 1, 0x55, 0xAA */

/* FAT12/FAT16 */
#define RFAT_BS_DrvNum                36   /* 1 */
#define RFAT_BS_BootSig               38   /* 1 */
#define RFAT_BS_VolID                 39   /* 4 */
#define RFAT_BS_VolLab                43   /* 11 */
#define RFAT_BS_FilSysType            54   /* 8 */
/* FAT32 */
#define RFAT_BPB_FATSz32              36   /* 4 */
#define RFAT_BPB_ExtFlags             40   /* 2 */
#define RFAT_BPB_FSVer                42   /* 2 */
#define RFAT_BPB_RootClus             44   /* 4 */
#define RFAT_BPB_FSInfo               48   /* 2 */
#define RFAT_BPB_BkBootSec            50   /* 2 */
#define RFAT_BS_DrvNum32              64   /* 1 */
#define RFAT_BS_BootSig32             66   /* 1 */
#define RFAT_BS_VolID32               67   /* 4 */
#define RFAT_BS_VolLab32              71   /* 11 */
#define RFAT_BS_FilSysType32          82   /* 8 */

/* FSI */
#define RFAT_FSI_LeadSig              0    /* 4 0x41615252 */
#define RFAT_FSI_StrucSig             484  /* 4 0x61417272 */
#define RFAT_FSI_Free_Count           488  /* 4 */
#define RFAT_FSI_Nxt_Free             492  /* 4 */
#define RFAT_FSI_TrailSig             508  /* 4 0xaa550000 */

/* MBR */
#define RFAT_MBR_PartitionTable       446   
#define RFAT_MBR_PartitionEntry1      446   
#define   RFAT_MBR_EntryBootIndicator 0    /* 1 */
#define   RFAT_MBR_EntryCHSStart      1    /* 3 */
#define   RFAT_MBR_EntrySystemID      4    /* 1 */
#define   RFAT_MBR_EntryCHSEnd        5    /* 3 */
#define   RFAT_MBR_EntryLBAOffset     8    /* 4 */
#define   RFAT_MBR_EntryLBASize       12   /* 4 */
#define RFAT_MBR_PartitionEntry2      462
#define RFAT_MBR_PartitionEntry3      478
#define RFAT_MBR_PartitionEntry4      494
#define RFAT_MBR_SignatureWord	      510  /* 2, 0x55, 0xAA */

/*
 * MMC/SD spec for MBR: 
 *
 * BootIndicator        0x00 or 0x80
 * SystemID             0x01 or 0x04 or 0x06 (or 0x0b for FAT32)
 * ParitionEntry2       all ZEROs
 * ParitionEntry3       all ZEROs
 * ParitionEntry4       all ZEROs
 */


struct _rfat_dir_t {
    uint8_t                 dir_name[11];
    uint8_t                 dir_attr;
    uint8_t                 dir_nt_reserved;
    uint8_t                 dir_crt_time_tenth;
    uint16_t                dir_crt_time;
    uint16_t                dir_crt_date;
    uint16_t                dir_acc_date;
    uint16_t                dir_clsno_hi;
    uint16_t                dir_wrt_time;
    uint16_t                dir_wrt_date;
    uint16_t                dir_clsno_lo;
    uint32_t                dir_file_size;
};

struct _rfat_cache_entry_t {
    uint32_t                blkno;
    uint8_t                 *data;
};

#define RFAT_FILE_FLAG_DATA_DIRTY    0x01
#define RFAT_FILE_FLAG_DATA_MODIFIED 0x02
#define RFAT_FILE_FLAG_DIR_MODIFIED  0x04

struct _rfat_file_t {
    uint8_t                 mode;
    uint8_t                 flags;
    uint8_t                 reserved[2];    /* ungetc() */
    rfat_cache_entry_t      data_cache;
    uint32_t                dir_blkno;      /* blkno where dir entry resides */ 
    uint32_t                dir_offset;     /* offset within blkno where dir entry resides */
    uint32_t                first_clsno;    /* dir_clsno_hi/dir_clsno_lo from dir entry */
    uint32_t                length;         /* dir_file_size from dir entry */
    uint32_t                position;
    uint32_t                clsno;
    uint32_t                blkno;
    uint32_t                blkno_e; /* exclusive */
};

struct _rfat_cluster_procs_t {
    int                     (*read)(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
    int                     (*write)(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata);
};

#define RFAT_VOLUME_STATE_NONE    0
#define RFAT_VOLUME_STATE_INIT    1
#define RFAT_VOLUME_STATE_MOUNTED 2

#define RFAT_VOLUME_TYPE_NONE     0
#define RFAT_VOLUME_TYPE_FAT12    1
#define RFAT_VOLUME_TYPE_FAT16    2
#define RFAT_VOLUME_TYPE_FAT32    3

#define RFAT_VOLUME_FLAG_NOTAVAILABLE  0x01
#define RFAT_VOLUME_FLAG_WRITEPROECTED 0x02
#define RFAT_VOLUME_FLAG_FAT_DIRTY     0x04


struct _rfat_volume_t {
    uint8_t                 state;
    uint8_t                 type;
    uint8_t                 flags;
    uint8_t                 reserved[1];
    uint32_t                boot_blkno;
    uint32_t                bkboot_blkofs;
    uint32_t                fsinfo_blkofs;
    uint32_t                fat_blkcnt;
    uint32_t                fat1_blkno;                   /* FAT to read/write from/to */
    int32_t                 fat2_blkofs;
    uint32_t                root_clsno;
    uint32_t                root_blkno;
    uint32_t                root_blkcnt;
    uint32_t                free_clsno;
    uint32_t                last_clsno;
    uint32_t                cls_shift;                    /* shift to get the byte offset for a clsno */
    uint32_t                cls_mask;                     /* (1 << cls_shift) -1 */
    uint32_t                cls_size;                     /* (1 << cls_shift) */
    int32_t                 cls_blk_offset;               /* offset to get the blkno for a clsno */
    uint32_t                cls_blk_shift;                /* shift to get the blkno for a clsno */
    uint32_t                cls_blk_size;                 /* 1 << cls_blk_shift */
    void                    *private;                     /* device private */
    rfat_cluster_procs_t    cluster;
    rfat_cache_entry_t      fat_cache;
    rfat_cache_entry_t      dir_cache;
    uint32_t                work_clsno;
    uint8_t                 work_pathname[F_MAXPATH];
    rfat_file_t             file_table[RFAT_CONFIG_MAX_FILES];
};

static int           rfat_last_status;

static rfat_volume_t rfat_volume;

static uint8_t       rfat_cache[(2 + RFAT_CONFIG_MAX_FILES) * 512];

static const uint8_t rfat_dirname_dot[11]    = ".          ";
static const uint8_t rfat_dirname_dotdot[11] = "..         ";

/***********************************************************************************************************************/

static int  rfat_disk_zero(rfat_volume_t *volume, uint32_t blkno, uint32_t blkcnt)
{
    int status = F_NO_ERROR;
    uint8_t *data;

    data = volume->dir_cache.data;
    volume->dir_cache.blkno = BLKNO_INVALID;

    memset(data, 0, BLK_SIZE);

    for (; blkcnt; blkno++, blkcnt--)
    {
        status = rfat_disk_write_multiple(volume, blkno, 1, data);
    }
    
    return status;
}

/***********************************************************************************************************************/

static int rfat_volume_init(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;
    uint8_t *cache;
    rfat_file_t *file, *file_e;

    status = rfat_disk_init(volume);

    if (status == F_NO_ERROR)
    {
        volume->state = RFAT_VOLUME_STATE_INIT;
        volume->type = RFAT_VOLUME_TYPE_NONE;
        volume->flags = 0;

        cache = &rfat_cache[0];

        volume->fat_cache.data = cache;
        cache += BLK_SIZE;

        volume->dir_cache.data = cache;
        cache += BLK_SIZE;
    
        for (file = &volume->file_table[0], file_e = &volume->file_table[RFAT_CONFIG_MAX_FILES]; (file < file_e); file++)
        {
            file->data_cache.data = cache;
            cache += BLK_SIZE;
        }
    }

    return status;
}

static int rfat_volume_mount(rfat_volume_t * volume)
{
    int status = F_NO_ERROR;
    uint8_t *data;
    uint32_t blkno, blkcnt, clscnt, cls_mask, cls_shift, disk_status;
    rfat_file_t *file, *file_e;

    disk_status = rfat_disk_status(volume);

    if (!(disk_status & RFAT_DISK_STATUS_PRESENT))
    {
        status = F_ERR_CARDREMOVED;
    }
    else
    {
        volume->flags = 0;

        if (disk_status & RFAT_DISK_STATUS_WRITEPROTECT)
        {
            volume->flags |= RFAT_VOLUME_FLAG_WRITEPROECTED;
        }

        blkno = 0;

        data = volume->dir_cache.data;
        volume->dir_cache.blkno = BLKNO_INVALID;

        status = rfat_disk_read(volume, blkno, data);

        if (status == F_NO_ERROR)
        {
            if (RFAT_LD_UINT16(data, RFAT_BS_SignatureWord) != 0xaa55)
            {
                status = F_ERR_NOTFORMATTED;
            }
            else
            {
                if (!(RFAT_LD_UINT8(data, RFAT_BS_jmpBoot+0) == 0xe9) &&
                    !((RFAT_LD_UINT8(data, RFAT_BS_jmpBoot+0) == 0xeb) &&
                      (RFAT_LD_UINT8(data, RFAT_BS_jmpBoot+2) == 0x90)))
                {
                    /* On a MMC/SD this tends to be all zero if no PBR is used */
                
                    blkno = RFAT_LD_UINT32(data, RFAT_MBR_PartitionEntry1+RFAT_MBR_EntryLBAOffset);

                    status = rfat_disk_read(volume, blkno, data);

                    if (RFAT_LD_UINT16(data, RFAT_BS_SignatureWord) != 0xaa55)
                    {
                        status = F_ERR_NOTFORMATTED;
                    }
                }
            }

            if (status == F_NO_ERROR)
            {
                if ((RFAT_LD_UINT16(data, RFAT_BS_SignatureWord) != 0xaa55) ||
                    (!((RFAT_LD_UINT8(data, RFAT_BS_BootSig) == 0x29) &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType +0) == 'F') &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType +1) == 'A') &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType +2) == 'T')) &&
                     !((RFAT_LD_UINT8(data, RFAT_BS_BootSig32) == 0x29) &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType32 +0) == 'F') &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType32 +1) == 'A') &&
                       (RFAT_LD_UINT8(data, RFAT_BS_FilSysType32 +2) == 'T'))))
                {
                    status = F_ERR_NOTFORMATTED;
                }
                else
                {
                    /* Here we got a valid boot record */
         
                    if (RFAT_LD_UINT16(data, RFAT_BPB_BytsPerSec) != BLK_SIZE)
                    {
                        status = F_ERR_NOTSUPPSECTORSIZE;
                    }
                    else if (RFAT_LD_UINT8(data, RFAT_BPB_NumFATs) > 2)
                    {
                        status = F_ERR_NOTUSEABLE;
                    }
                    else
                    {
                        volume->boot_blkno = blkno;

                        volume->cls_mask = ((RFAT_LD_UINT8(data, RFAT_BPB_SecPerClus) * BLK_SIZE) - 1);

                        for (cls_mask = 0x8000, cls_shift = 16; !(volume->cls_mask & cls_mask); cls_mask >>= 1, cls_shift--) { }

                        volume->cls_shift = cls_shift;
                        volume->cls_size = (1 << volume->cls_shift);
                        volume->cls_blk_shift = cls_shift - BLK_SHIFT;
                        volume->cls_blk_size = (1 << volume->cls_blk_shift);

                        if (RFAT_LD_UINT16(data, RFAT_BPB_TotSec16) != 0)
                        {
                            blkcnt = RFAT_LD_UINT16(data, RFAT_BPB_TotSec16);
                        }
                        else
                        {
                            blkcnt = RFAT_LD_UINT32(data, RFAT_BPB_TotSec32);
                        }

                        blkno += RFAT_LD_UINT16(data, RFAT_BPB_RsvdSecCnt);
                        blkcnt -= RFAT_LD_UINT16(data, RFAT_BPB_RsvdSecCnt);

                        if (RFAT_LD_UINT16(data, RFAT_BPB_FATSz16) != 0)
                        {
                            volume->fat_blkcnt = RFAT_LD_UINT16(data, RFAT_BPB_FATSz16);
                        }
                        else
                        {
                            volume->fat_blkcnt = RFAT_LD_UINT32(data, RFAT_BPB_FATSz32);
                        }

                        volume->fat1_blkno = blkno;
                        volume->fat2_blkofs = 0;

                        blkno += volume->fat_blkcnt;
                        blkcnt -= volume->fat_blkcnt;

                        if (RFAT_LD_UINT8(data, RFAT_BPB_NumFATs) != 1)
                        {
                            volume->fat2_blkofs = volume->fat_blkcnt;

                            blkno += volume->fat_blkcnt;
                            blkcnt -= volume->fat_blkcnt;
                        }

                        if (RFAT_LD_UINT16(data, RFAT_BPB_RootEntCnt) != 0)
                        {
                            /* FAT12/FAT16 */
                            volume->bkboot_blkofs = 0;
                            volume->fsinfo_blkofs = 0;

                            volume->root_clsno  = RFAT_CLUSTER_NONE;
                            volume->root_blkno  = blkno;
                            volume->root_blkcnt = ((RFAT_LD_UINT16(data, RFAT_BPB_RootEntCnt) * 32) + BLK_MASK) >> BLK_SHIFT;

                            blkno += volume->root_blkcnt;
                            blkcnt -= volume->root_blkcnt;

                            volume->free_clsno = 2;
                        }
                        else
                        {
                            /* FAT32 */
                            volume->bkboot_blkofs = RFAT_LD_UINT16(data, RFAT_BPB_BkBootSec);
                            volume->fsinfo_blkofs = RFAT_LD_UINT16(data, RFAT_BPB_FSInfo);

                            volume->root_clsno  = RFAT_LD_UINT32(data, RFAT_BPB_RootClus);
                            volume->root_blkno  = BLKNO_NONE;
                            volume->root_blkcnt = 0;

                            if (RFAT_LD_UINT16(data, RFAT_BPB_ExtFlags) & 0x0080)
                            {
                                if (RFAT_LD_UINT16(data, RFAT_BPB_ExtFlags) & 0x000f)
                                {
                                    volume->fat1_blkno += volume->fat_blkcnt;
                                    volume->fat2_blkofs = 0;
                                }
                            }

                            volume->free_clsno = 2;

                            if (volume->fsinfo_blkofs != 0)
                            {
                                status = rfat_disk_read(volume, volume->boot_blkno + volume->fsinfo_blkofs, data);

                                if (status == F_NO_ERROR)
                                {
                                    if ((RFAT_LD_UINT32(data, RFAT_FSI_LeadSig)  == 0x41645252) &&
                                        (RFAT_LD_UINT32(data, RFAT_FSI_StrucSig) == 0x61417272) &&
                                        (RFAT_LD_UINT32(data, RFAT_FSI_TrailSig) == 0xaa550000))
                                    {
                                        if (RFAT_LD_UINT32(data, RFAT_FSI_Nxt_Free) != 0xffffffff)
                                        {
                                            volume->free_clsno = RFAT_LD_UINT32(data, RFAT_FSI_Nxt_Free);
                                        }
                                    }
                                }
                            }
                        }

                        if (status == F_NO_ERROR)
                        {
                            volume->cls_blk_offset = blkno - (2 << volume->cls_blk_shift);

                            clscnt = (blkcnt >> volume->cls_blk_shift);

                            volume->last_clsno = (2 + clscnt) - 1;

                            if (clscnt < 4085)
                            {
                                volume->type = RFAT_VOLUME_TYPE_FAT12;

                                volume->cluster.read  = rfat_cluster_read_fat12;
                                volume->cluster.write = rfat_cluster_write_fat12;
                            }
                            else if (clscnt < 65525)
                            {
                                volume->type = RFAT_VOLUME_TYPE_FAT16;

                                volume->cluster.read  = rfat_cluster_read_fat16;
                                volume->cluster.write = rfat_cluster_write_fat16;
                            }
                            else
                            {
                                volume->type = RFAT_VOLUME_TYPE_FAT32;

                                volume->cluster.read  = rfat_cluster_read_fat32;
                                volume->cluster.write = rfat_cluster_write_fat32;
                            }

                            volume->state = RFAT_VOLUME_STATE_MOUNTED;
                        }
                    }
                }
            }
        }
    
        if (status == F_NO_ERROR)
        {
            volume->fat_cache.blkno = BLKNO_INVALID;
            volume->dir_cache.blkno = BLKNO_INVALID;

            volume->work_clsno = RFAT_CLUSTER_NONE;
            volume->work_pathname[0] = F_SEPARATORCHAR;
            volume->work_pathname[1] = '\0';

            for (file = &volume->file_table[0], file_e = &volume->file_table[RFAT_CONFIG_MAX_FILES]; (file < file_e); file++)
            {
                file->mode = 0;
            }
        }

        if (status == F_NO_ERROR)
        {
            volume->state = RFAT_VOLUME_STATE_MOUNTED;
        }
    }

    return status;
}

static int  rfat_volume_unmount(rfat_volume_t * volume)
{
    int status = F_NO_ERROR;
    uint8_t *data;

    if (volume->state == RFAT_VOLUME_STATE_NONE)
    {
        status = F_ERR_INITFUNC;
    }
    else
    {
	if (volume->fsinfo_blkofs != 0)
	{
	    data = volume->dir_cache.data;
            volume->dir_cache.blkno = BLKNO_INVALID;

	    memset(data, 0, BLK_SIZE);

	    RFAT_ST_UINT32(data, RFAT_FSI_LeadSig,    0x41645252);
	    RFAT_ST_UINT32(data, RFAT_FSI_StrucSig,   0x61417272);
	    RFAT_ST_UINT32(data, RFAT_FSI_Free_Count, 0xffffffff);
	    RFAT_ST_UINT32(data, RFAT_FSI_Nxt_Free,   volume->free_clsno);
	    RFAT_ST_UINT32(data, RFAT_FSI_TrailSig,   0xaa550000);
                
	    status = rfat_disk_write(volume, (volume->boot_blkno + volume->fsinfo_blkofs), data);
                
	    if (status == F_NO_ERROR)
	    {
		if (volume->bkboot_blkofs != 0)
		{
		    status = rfat_disk_write(volume, (volume->boot_blkno + volume->bkboot_blkofs + volume->fsinfo_blkofs), data);
		}
	    }
	}

	if (status == F_NO_ERROR)
	{
	    /* Revert the state to be RFAT_VOLUME_STATE_INIT, so that can be remounted */

	    volume->state = RFAT_VOLUME_STATE_INIT;

	    status = rfat_disk_release(volume);
	}
    }

    return status;
}

static int  rfat_volume_format(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;
    unsigned int fattype, partype;
    uint32_t blkcnt, blksz, align_blk_size, cls_blk_shift, clscnt, disk_status;
    uint32_t boot_blkno, fat1_blkno, fat2_blkno, root_blkno, data_blkno, fat_blkcnt, root_blkcnt;
    uint32_t hpc, spt, start_h, start_s, start_c, end_h, end_s, end_c;
    uint32_t serial;
    uint8_t *data;

    disk_status = rfat_disk_status(volume);

    if (!(disk_status & RFAT_DISK_STATUS_PRESENT))
    {
        status = F_ERR_CARDREMOVED;
    }
    else if (disk_status & RFAT_DISK_STATUS_WRITEPROTECT)
    {
        status = F_ERR_WRITEPROTECT;
    }
    else
    {
        volume->flags = 0;

	blkcnt = 0;
	blksz  = 0;
	serial = 0;
	
        status = rfat_disk_capacity(volume, &blkcnt, &blksz);

        if (status == F_NO_ERROR)
        {
            if (blksz == BLK_SIZE)
            {
                status = rfat_disk_serial(volume, &serial);
            }
            else
            {
                status = F_ERR_INVALIDMEDIA;
            }
        }

        if (status == F_NO_ERROR)
        {
            /* Revert the state to be RFAT_VOLUME_STATE_INIT, so that can be remounted */

            volume->state = RFAT_VOLUME_STATE_INIT;

            /*
             * There is a upper limit of 4153344 blocks for SDSC cards, and
             * a lower limit of 4211712 for SDHC cards as per SD spec (CSD description).
             *
             * Hower the real lower limit for SHDC is:
             *
             *    16384 + (65525 * 64) = 4209984
             *    4211712 = 8192 + 8192 + (65552 * 64)
             *
             *    (16384 is the required padding upfront for SDHC at this size)
             *
             * The upper legal limit for SDSC is also slightly different from the
             * spec if one thinks about it:
             *
             *    768 + (65524 * 64) = 4194304
             *    4153344 = 768 + (64884 * 64)
             *
             *    (768 is the required padding upfront for SDSC at this size)
             *
             * Thus use those corrected limits to figure out SDSC/SDHC.
             */
    
            if (blkcnt < 4209984)
            {
                /* SDSC, FAT12/FAT16 */

                /* Limit the number of blocks to not overflow FAT16
                 */
                if (blkcnt > 4194304)
                {
                    blkcnt = 4194304;
                }

                /* Number of Heads and Sectors Per Track ...
                 *
                 * See Microsoft's MBR specs and the term "translation mode".
                 * 
                 * We want to use powers of 2, which means the upper limit for
                 * that representation is 1024 * 128 * 32  = 4194304. That happens
                 * to be also the upper limit for SDSC here. If the size is smaller
                 * than 2 * 16, use "hpc" 2 and "spt" 16, otherwise "spt" is 32, and 
                 * "hpc" is the smallest power of 2.
                 */

                if (blkcnt <= (1024 * 2 * 16))
                {
                    hpc = 2;
                    spt = 16;
                }
                else
                {
                    hpc = 2;
                    spt = 32;
                    
                    while (hpc < 128)
                    {
                        if (blkcnt <= (1024 * hpc * spt))
                        {
                            break;
                        }

                        hpc <<= 1;
                    }
                }

                if (blkcnt > ((1048 * 1024 * 1024) >> BLK_SHIFT))
                {
                    cls_blk_shift = 6;
                }
                else if (blkcnt > ((8 * 1024 * 1024) >> BLK_SHIFT))
                {
                    cls_blk_shift = 5;
                }
                else
                {
                    cls_blk_shift = 4;
                }

                if (blkcnt > ((256 * 1024 * 1024) >> BLK_SHIFT))
                {
                    align_blk_size = 128;
                }
                else if (blkcnt > ((64 * 1024 * 1024) >> BLK_SHIFT))
                {
                    align_blk_size = 64;
                }
                else if (blkcnt > ((8 * 1024 * 1024) >> BLK_SHIFT))
                {
                    align_blk_size = 32;
                }
                else
                {
                    align_blk_size = 16;
                }

                /*
                 * FAT12/FAT16 layout requires that the MBR takes up "align_blk_size" blocks. There is also 32 blocks for
                 * the root directory. Then there is one boot sector. Hence we can compute the maximum clscnt derived
                 * from blkcnt/align_blk_size/clssz. N.b. that FAT16 has at least 4085 regular FAT entries, plus 2 special ones,
                 * which means 16 blocks.
                 */
        
                root_blkcnt = 32;

                clscnt = (blkcnt - align_blk_size - ((1 + 2 * 16 + root_blkcnt + (align_blk_size-1)) & ~(align_blk_size-1))) >> cls_blk_shift;

                if (clscnt < 4085)
                {
                    fattype = RFAT_VOLUME_TYPE_FAT12;
            
                    clscnt = (blkcnt - align_blk_size - ((1 + 2 * 2 + root_blkcnt + (align_blk_size-1)) & ~(align_blk_size-1))) >> cls_blk_shift;

                    fat_blkcnt = (((((clscnt + 2) * 3) + 1) / 2) + (BLK_SIZE-1)) >> BLK_SHIFT;
                }
                else
                {
                    fattype = RFAT_VOLUME_TYPE_FAT16;

                    fat_blkcnt = ((clscnt + 2) * 2 + (BLK_SIZE-1)) >> BLK_SHIFT;
                }

                data_blkno = align_blk_size + ((1 + 2 * fat_blkcnt + root_blkcnt + (align_blk_size-1)) & ~(align_blk_size-1));
                root_blkno = (data_blkno - 32);
                fat2_blkno = root_blkno - fat_blkcnt;
                fat1_blkno = fat2_blkno - fat_blkcnt;
                boot_blkno = fat1_blkno - 1;

                /* Recompute blkcnt based upon clscnt with truncation
                 */
                clscnt = (blkcnt - data_blkno) >> cls_blk_shift;
                blkcnt = data_blkno + (clscnt << cls_blk_shift);

                /* See Microsoft's MBR specs and the term "logical drive".
                 */
                if ((blkcnt - boot_blkno) < 32680)
                {
                    partype = 0x01;
                }
                else if ((blkcnt - boot_blkno) < 65536)
                {
                    partype = 0x04;
                }
                else
                {
                    partype = 0x06;
                }
            }
            else
            {
                /* SDHC/SDXC, FAT32 */

                /* Number of Heads and Sectors Per Track ...
                 *
                 * See Microsoft's MBR specs and the term "translation mode".
                 */

                if (blkcnt <= (1024 * 128 * 64))
                {
                    hpc = 128;
                    spt = 63;
                }
                else
                {
                    hpc = 255;
                    spt = 63;
                }

                cls_blk_shift = 6;
                align_blk_size = 8192;

                /* FAT32 is layed out different than FAT12/FAT16. There is no root_blkno, and we know
                 * that there are at least 65525 clusters. Also there are 9 reserved blocks minimum.
                 */

                root_blkcnt = 0;

                clscnt = (blkcnt - align_blk_size - ((9 + 2 * 512 + (align_blk_size-1)) & ~(align_blk_size-1))) >> cls_blk_shift;
        
                fattype = RFAT_VOLUME_TYPE_FAT32;

                fat_blkcnt = ((clscnt + 2) * 4 + (BLK_SIZE-1)) >> BLK_SHIFT;

                data_blkno = align_blk_size + ((9 + 2 * fat_blkcnt + (align_blk_size-1)) & ~(align_blk_size-1));
                root_blkno = data_blkno; /* really the first data cluster */
                fat2_blkno = data_blkno - fat_blkcnt;
                fat1_blkno = fat2_blkno - fat_blkcnt;
                boot_blkno = align_blk_size;

                /* Recompute blkcnt based upon clscnt with truncation
                 */
                clscnt = (blkcnt - data_blkno) >> cls_blk_shift;
                blkcnt = data_blkno + (clscnt << cls_blk_shift);

                /* Select partype to get either CHS/LBA or LBA.
                 */
                if (blkcnt <= 16450560)
                {
                    partype = 0x0b;
                }
                else
                {
                    partype = 0x0c;
                }
            }

            /* CHS has max legal values (0..1023) * (0..254) * (1..63). If the LBA is outside
             * this CHS range, then use the maximum value. This can only happen with FAT32,
             * in which case the partition type signals to use LBA anyway.
             */
            if (blkcnt <= 16450560)
            {
                start_c = boot_blkno / (hpc * spt);
                start_h = (boot_blkno - (start_c * hpc * spt)) / spt;
                start_s = boot_blkno - (start_c * hpc * spt) - (start_h * spt) + 1;
                
                end_c = (blkcnt-1) / (hpc * spt);
                end_h = ((blkcnt-1) - (end_c * hpc * spt)) / spt;
                end_s = (blkcnt-1) - (end_c * hpc * spt) - (end_h * spt) + 1;
            }
            else
            {
                start_c = 1023;
                start_h = 254;
                start_s = 63;
                
                end_c   = 1023;
                end_h   = 254;
                end_s   = 63;
            }

            if (status == F_NO_ERROR)
            {
                /* Ok beyond here there cannot be any logical error. We can always format.
                 * So first blow away the MBR / PBR to not have any inconsistent state.
                 */
                
                status = rfat_disk_zero(volume, 0, 1);
            }

            if (status == F_NO_ERROR)
            {
                if (fattype == RFAT_VOLUME_TYPE_FAT32)
                {
                    /* PBR and FAT1/FAT2/Root are in 2 separate ranges ...
                     */

                    status = rfat_disk_zero(volume, boot_blkno, 9);
                
                    if (status == F_NO_ERROR)
                    {
                        status = rfat_disk_zero(volume, fat1_blkno, (data_blkno - fat1_blkno) + (1 << cls_blk_shift));
                    }
                }
                else
                {
                    /* PBR/FAT1/FAT2/Root are all in one linear range, so clean them in
                     * one sweep.
                     */

                    status = rfat_disk_zero(volume, boot_blkno, (data_blkno - boot_blkno));
                }
            }

            data = volume->dir_cache.data;
            volume->dir_cache.blkno = BLKNO_INVALID;

            if (status == F_NO_ERROR)
            {
                /* Write first FAT1/FAT2 entry */
                memset(data, 0, BLK_SIZE);

                if (fattype == RFAT_VOLUME_TYPE_FAT12)
                {
                    RFAT_ST_UINT8(data, 0, 0xf8);
                    RFAT_ST_UINT8(data, 1, 0xff);
                    RFAT_ST_UINT8(data, 2, 0xff);
                }
                else if (fattype == RFAT_VOLUME_TYPE_FAT16)
                {
                    RFAT_ST_UINT8(data, 0, 0xf8);
                    RFAT_ST_UINT8(data, 1, 0xff);
                    RFAT_ST_UINT8(data, 2, 0xff);
                    RFAT_ST_UINT8(data, 3, 0xff);
                }
                else
                {
                    RFAT_ST_UINT32(data, 0, 0x0ffffff8);
                    RFAT_ST_UINT32(data, 4, 0x0fffffff);
                    RFAT_ST_UINT32(data, 8, 0x0fffffff); /* root_clsno ... EOC */
                }

                status = rfat_disk_write(volume, fat1_blkno, data);

                if (status == F_NO_ERROR)
                {
                    status = rfat_disk_write(volume, fat2_blkno, data);
                }
            }

            if (status == F_NO_ERROR)
            {
                if (fattype == RFAT_VOLUME_TYPE_FAT32)
                {
                    /* Write the FSInfo Record for FAT32 */

                    memset(data, 0, BLK_SIZE);

                    RFAT_ST_UINT32(data, RFAT_FSI_LeadSig,    0x41645252);
                    RFAT_ST_UINT32(data, RFAT_FSI_StrucSig,   0x61417272);
                    RFAT_ST_UINT32(data, RFAT_FSI_Free_Count, 0xffffffff);
                    RFAT_ST_UINT32(data, RFAT_FSI_Nxt_Free,   0x00000003);
                    RFAT_ST_UINT32(data, RFAT_FSI_TrailSig,   0xaa550000);
                
                    status = rfat_disk_write(volume, boot_blkno+1, data);
                
                    if (status == F_NO_ERROR)
                    {
                        status = rfat_disk_write(volume, boot_blkno+7, data);
                    }
                }
            }

            if (status == F_NO_ERROR)
            {
                /* Write the PBR */
                
                memset(data, 0, BLK_SIZE);

                RFAT_ST_UINT8(data, RFAT_BS_jmpBoot +0,      0xeb);
                RFAT_ST_UINT8(data, RFAT_BS_jmpBoot +1,      0x00);
                RFAT_ST_UINT8(data, RFAT_BS_jmpBoot +2,      0x90);
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +0,      'M');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +1,      'S');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +2,      'D');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +3,      'O');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +4,      'S');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +5,      '5');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +6,      '.');
                RFAT_ST_UINT8(data, RFAT_BS_OEMName +7,      '0');
                RFAT_ST_UINT16(data, RFAT_BPB_BytsPerSec,    BLK_SIZE);
                RFAT_ST_UINT8(data, RFAT_BPB_SecPerClus,     (1 << cls_blk_shift));
                RFAT_ST_UINT16(data, RFAT_BPB_RsvdSecCnt,    fat1_blkno - boot_blkno);
                RFAT_ST_UINT8(data, RFAT_BPB_NumFATs,        2);
                RFAT_ST_UINT16(data, RFAT_BPB_RootEntCnt,    ((fattype == RFAT_VOLUME_TYPE_FAT32) ? 0 : 512));
                RFAT_ST_UINT16(data, RFAT_BPB_TotSec16,      (((blkcnt - boot_blkno) <= 65535) ? (blkcnt - boot_blkno) : 0));
                RFAT_ST_UINT8(data, RFAT_BPB_Media,          0xf8);
                RFAT_ST_UINT16(data, RFAT_BPB_FATSz16,       ((fattype == RFAT_VOLUME_TYPE_FAT32) ? 0 : fat_blkcnt));
                RFAT_ST_UINT16(data, RFAT_BPB_SecPerTrk,     spt);
                RFAT_ST_UINT16(data, RFAT_BPB_NumHeads,      hpc);
                RFAT_ST_UINT32(data, RFAT_BPB_HiddSec,       boot_blkno);
                RFAT_ST_UINT32(data, RFAT_BPB_TotSec32,      (((blkcnt - boot_blkno) <= 65535) ? 0 : (blkcnt - boot_blkno)));
                RFAT_ST_UINT16(data, RFAT_BS_SignatureWord,  0xaa55);
                
                if (fattype != RFAT_VOLUME_TYPE_FAT32)
                {
                    RFAT_ST_UINT8(data, RFAT_BS_DrvNum,         0x80);
                    RFAT_ST_UINT8(data, RFAT_BS_BootSig,        0x29);
                    RFAT_ST_UINT32(data, RFAT_BS_VolID,          serial);
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +0,       'N');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +1,       'O');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +2,       ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +3,       'N');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +4,       'A');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +5,       'M');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +6,       'E');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +7,       ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +8,       ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +9,       ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab +10,      ' ');
                    
                    if (fattype == RFAT_VOLUME_TYPE_FAT12)
                    {
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +0,   'F');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +1,   'A');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +2,   'T');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +3,   '1');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +4,   '2');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +5,   ' ');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +6,   ' ');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +7,   ' ');
                    }
                    else
                    {
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +0,   'F');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +1,   'A');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +2,   'T');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +3,   '1');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +4,   '6');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +5,   ' ');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +6,   ' ');
                        RFAT_ST_UINT8(data, RFAT_BS_FilSysType +7,   ' ');
                    }

                    status = rfat_disk_write(volume, boot_blkno, data);
                }
                else
                {
                    RFAT_ST_UINT32(data, RFAT_BPB_FATSz32,        fat_blkcnt);
                    RFAT_ST_UINT16(data, RFAT_BPB_ExtFlags,       0);
                    RFAT_ST_UINT16(data, RFAT_BPB_FSVer,          0);
                    RFAT_ST_UINT32(data, RFAT_BPB_RootClus,       2);
                    RFAT_ST_UINT16(data, RFAT_BPB_FSInfo,         1);
                    RFAT_ST_UINT16(data, RFAT_BPB_BkBootSec,      6);
                    RFAT_ST_UINT8(data, RFAT_BS_DrvNum32,       0x80);
                    RFAT_ST_UINT8(data, RFAT_BS_BootSig32,      0x29);
                    RFAT_ST_UINT32(data, RFAT_BS_VolID32,        serial);
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +0,     'N');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +1,     'O');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +2,     ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +3,     'N');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +4,     'A');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +5,     'M');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +6,     'E');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +7,     ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +8,     ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +9,     ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_VolLab32 +10,    ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +0, 'F');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +1, 'A');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +2, 'T');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +3, '3');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +4, '2');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +5, ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +6, ' ');
                    RFAT_ST_UINT8(data, RFAT_BS_FilSysType32 +7, ' ');

                    /* Write Backup Boot Record first, so if that fails there
                     * is no valid Primary Boot Record.
                     */

                    status = rfat_disk_write(volume, boot_blkno+6, data);

                    if (status == F_NO_ERROR)
                    {
                        status = rfat_disk_write(volume, boot_blkno, data);
                    }
                }
            }

            if (status == F_NO_ERROR)
            {
                /* Write the MBR */

                memset(data, 0, BLK_SIZE);

                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryBootIndicator, 0x00);
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSStart +0, start_h);
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSStart +1, ((start_c << 6) | start_s));
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSStart +2, (start_c >> 2));
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntrySystemID, partype);
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSEnd +0, end_h);
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSEnd +1, ((end_c << 6) | end_s));
                RFAT_ST_UINT8(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryCHSEnd +2, (end_c >> 2));
                RFAT_ST_UINT32(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryLBAOffset, boot_blkno);
                RFAT_ST_UINT32(data, RFAT_MBR_PartitionEntry1 + RFAT_MBR_EntryLBASize, (blkcnt - boot_blkno));
                RFAT_ST_UINT16(data, RFAT_MBR_SignatureWord, 0xaa55);

                status = rfat_disk_write(volume, 0, data);
            }
        }
    }

    return status;
}

static int rfat_volume_lock(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (RFAT_PORT_LOCK(volume) == F_NO_ERROR)
    {
        /*
         * Check here whether the volume is mounted. If not mount it. If it cannot
         * be mounted, throw an error.
         */

        if (volume->state != RFAT_VOLUME_STATE_MOUNTED)
        {
            if (volume->state == RFAT_VOLUME_STATE_INIT)
            {
                status = rfat_volume_mount(volume);
            }
            else
            {
                status = F_ERR_INITFUNC;
            }
        }
    }
    else
    {
        status = F_ERR_BUSY;
    }

    return status;
}

static int rfat_volume_lock_noinit(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (RFAT_PORT_LOCK(volume) == F_NO_ERROR)
    {
        if (volume->state != RFAT_VOLUME_STATE_NONE)
        {
            status = F_ERR_BUSY;
        }
    }
    else
    {
        status = F_ERR_BUSY;
    }

    return status;
}

static int  rfat_volume_lock_nomount(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (RFAT_PORT_LOCK(volume) == F_NO_ERROR)
    {
        if (volume->state == RFAT_VOLUME_STATE_NONE)
        {
            status = F_ERR_INITFUNC;
        }
    }
    else
    {
        status = F_ERR_BUSY;
    }

    return status;
}

static void rfat_volume_unlock(rfat_volume_t *volume)
{
    RFAT_PORT_UNLOCK(volume);
}

/***********************************************************************************************************************/

static int rfat_fat_cache_find(rfat_volume_t *volume, uint32_t blkno)
{
    int status = F_NO_ERROR;

    if (volume->flags & RFAT_VOLUME_FLAG_FAT_DIRTY)
    {
        status = rfat_fat_cache_write(volume, &volume->fat_cache);
    }

    if (status == F_NO_ERROR)
    {
        status = rfat_disk_read(volume, blkno, volume->fat_cache.data);
            
        if (status == F_NO_ERROR)
        {
            volume->fat_cache.blkno = blkno;
        }
    }

    return status;
}

static int rfat_fat_cache_read(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->fat_cache.blkno != blkno)
    {
	status = rfat_fat_cache_find(volume, blkno);
    }

    *p_entry = &volume->fat_cache;

    return status;
}

static int rfat_fat_cache_write(rfat_volume_t *volume, rfat_cache_entry_t *entry)
{
    int status = F_NO_ERROR;

    status = rfat_disk_write(volume, entry->blkno, entry->data);
                
    if (status == F_NO_ERROR)
    {
#if (RFAT_CONFIG_DISABLE_FAT_MIRROR != 1)
	if (volume->fat2_blkofs != 0)
	{
	    status = rfat_disk_write(volume, entry->blkno + volume->fat2_blkofs, entry->data);
	}

	if (status == F_NO_ERROR)
#endif /* RFAT_CONFIG_DISABLE_FAT_MIRROR != 1 */
	{
	    volume->flags &= ~RFAT_VOLUME_FLAG_FAT_DIRTY;
	}
    }

    return status;
}

static inline void rfat_fat_cache_modify(rfat_volume_t *volume, rfat_cache_entry_t *entry)
{
    volume->flags |= RFAT_VOLUME_FLAG_FAT_DIRTY;
}

static int rfat_fat_cache_flush(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->flags & RFAT_VOLUME_FLAG_FAT_DIRTY)
    {
	status = rfat_fat_cache_write(volume, &volume->fat_cache);
    }

    return status;
}

/***********************************************************************************************************************/

static int rfat_dir_cache_find(rfat_volume_t *volume, uint32_t blkno)
{
    int status = F_NO_ERROR;

    status = rfat_disk_read_multiple(volume, blkno, 1, volume->dir_cache.data);
    
    if (status == F_NO_ERROR)
    {
        volume->dir_cache.blkno = blkno;
    }

    return status;
}

static int rfat_dir_cache_read(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
	status = rfat_dir_cache_find(volume, blkno);
    }

    *p_entry = &volume->dir_cache;

    return status;
}

static int rfat_dir_cache_write(rfat_volume_t *volume, rfat_cache_entry_t * entry)
{
    int status = F_NO_ERROR;

    status = rfat_disk_write(volume, entry->blkno, entry->data);

    return status;
}

static void rfat_dir_cache_zero(rfat_volume_t *volume, uint32_t blkno, rfat_cache_entry_t **p_entry)
{
    memset(volume->dir_cache.data, 0, BLK_SIZE);

    volume->dir_cache.blkno = blkno;

    *p_entry = &volume->dir_cache;
}

/***********************************************************************************************************************/

static int rfat_data_cache_write(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry)
{
    int status = F_NO_ERROR;

    status = rfat_disk_write_multiple(volume, entry->blkno, 1, entry->data);
        
    if (status == F_NO_ERROR)
    {
        file->flags &= ~RFAT_FILE_FLAG_DATA_DIRTY;
    }

    return status;
}

static int rfat_data_cache_find(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, int refill)
{
    int status = F_NO_ERROR;

    if (file->flags & RFAT_FILE_FLAG_DATA_DIRTY)
    {
        status = rfat_data_cache_write(volume, file, &file->data_cache);
    }

    if (refill)
    {
        if (status == F_NO_ERROR)
        {
            status = rfat_disk_read_multiple(volume, blkno, 1, file->data_cache.data);
            
            if (status == F_NO_ERROR)
            {
                file->data_cache.blkno = blkno;
            }
        }
    }

    return status;
}

static int rfat_data_cache_read(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, rfat_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (file->data_cache.blkno != blkno)
    {
	status = rfat_data_cache_find(volume, file, blkno, TRUE);
    }

    *p_entry = &file->data_cache;
    
    return status;
}

static int rfat_data_cache_check(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, rfat_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (file->data_cache.blkno != blkno)
    {
	status = rfat_data_cache_find(volume, file, blkno, FALSE);
    }

    *p_entry = &file->data_cache;
    
    return status;
}

static inline void rfat_data_cache_modify(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry)
{
    file->flags |= (RFAT_FILE_FLAG_DATA_DIRTY | RFAT_FILE_FLAG_DATA_MODIFIED);
}

static inline void rfat_data_cache_clean(rfat_volume_t *volume, rfat_file_t *file, rfat_cache_entry_t *entry)
{
    file->flags &= ~RFAT_FILE_FLAG_DATA_DIRTY;
}

static int rfat_data_cache_flush(rfat_volume_t *volume, rfat_file_t *file)
{
    int status = F_NO_ERROR;

    if (file->flags & RFAT_FILE_FLAG_DATA_DIRTY)
    {
        status = rfat_data_cache_write(volume, file, &file->data_cache);
    }

    return status;
}

static int rfat_data_cache_invalidate(rfat_volume_t *volume, rfat_file_t *file, uint32_t blkno, uint32_t blkcnt, int flush)
{
    int status = F_NO_ERROR;

    if ((blkno <= file->data_cache.blkno) && (file->data_cache.blkno < (blkno + blkcnt)))
    {
        if (file->flags & RFAT_FILE_FLAG_DATA_DIRTY)
        {
            status = rfat_data_cache_write(volume, file, &file->data_cache);
        }
        
        if (status == F_NO_ERROR)
        {
            file->data_cache.blkno = BLKNO_INVALID;

            file->flags &= ~RFAT_FILE_FLAG_DATA_DIRTY;
        }
    }

    return status;
}

/***********************************************************************************************************************/

static int rfat_cluster_read_fat12(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno, clsdata;
    uint8_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_READ");
    }
#endif
    
    offset = clsno + (clsno >> 1);
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint8_t*)(entry->data + (offset & BLK_MASK));

        if (clsno & 1)
        {
            clsdata = ((uint32_t)(*fat_data) >> 4);
        }
        else
        {
            clsdata = ((uint32_t)(*fat_data));
        }

        if ((offset & BLK_MASK) == BLK_MASK)
        {
            status = rfat_fat_cache_read(volume, (blkno+1), &entry);

            if (status == F_NO_ERROR)
            {
                fat_data = (uint8_t*)(entry->data + 0);
	    }
	}
	else
	{
	    fat_data++;
	}

	if (status == F_NO_ERROR)
	{
            if (clsno & 1)
            {
                clsdata |= ((uint32_t)(*fat_data) << 4);
            }
            else
            {
                clsdata |= (((uint32_t)(*fat_data) & 0x0000000fu) << 8);
            }
        }

	if (clsdata >= RFAT_CLUSTER_RESERVED12R)
	{
            clsdata += (RFAT_CLUSTER_RESERVED32R - RFAT_CLUSTER_RESERVED12R);
	}

        *p_clsdata = clsdata;
    }
    
    return status;
}

static int rfat_cluster_write_fat12(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno;
    uint8_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_WRITE");
    }
#endif

    offset = clsno + (clsno >> 1);
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint8_t*)(entry->data + (offset & BLK_MASK));

        if (clsno & 1)
        {
            *fat_data = (*fat_data & 0x0f) | (clsdata << 4);
        }
        else
        {
            *fat_data = clsdata;
        }

        if ((offset & BLK_MASK) == BLK_MASK)
        {
            rfat_fat_cache_modify(volume, entry);

            status = rfat_fat_cache_read(volume, (blkno+1), &entry);

            if (status == F_NO_ERROR)
            {
                fat_data = (uint8_t*)(entry->data + 0);
	    }
	}
	else
	{
	    fat_data++;
	}
        
	if (status == F_NO_ERROR)
	{
            if (clsno & 1)
            {
                *fat_data = clsdata >> 4;
            }
            else
            {
		*fat_data = (*fat_data & 0xf0) | ((clsdata >> 8) & 0x0f);
            }

            rfat_fat_cache_modify(volume, entry);
        }
    }
    
    return status;
}

static int rfat_cluster_read_fat16(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno, clsdata;
    uint16_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_READ");
    }
#endif

    offset = clsno << 1;
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint16_t*)(entry->data + (offset & BLK_MASK));

	clsdata = RFAT_FTOHS(*fat_data);
        
	if (clsdata >= RFAT_CLUSTER_RESERVED16R)
	{
            clsdata += (RFAT_CLUSTER_RESERVED32R - RFAT_CLUSTER_RESERVED16R);
	}

        *p_clsdata = clsdata;
    }
    
    return status;
}

static int rfat_cluster_write_fat16(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno;
    uint16_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_WRITE");
    }
#endif

    offset = clsno << 1;
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint16_t*)(entry->data + (offset & BLK_MASK));

	*fat_data = RFAT_HTOFS(clsdata);

        rfat_fat_cache_modify(volume, entry);
    }
    
    return status;
}

static int rfat_cluster_read_fat32(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno, clsdata;
    uint32_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_READ");
    }
#endif

    offset = clsno << 2;
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint32_t*)(entry->data + (offset & BLK_MASK));

	clsdata = RFAT_FTOHL(*fat_data) & 0x0fffffff;

        *p_clsdata = clsdata;
    }
    
    return status;
}

static int rfat_cluster_write_fat32(rfat_volume_t *volume, uint32_t clsno, uint32_t clsdata)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    uint32_t *fat_data;
    rfat_cache_entry_t *entry;

#if defined(DEBUG)
    if ((clsno <= 1) || (clsno > volume->last_clsno))
    {
	assert(!"CLUSTER_WRITE");
    }
#endif

    offset = clsno << 2;
    blkno = volume->fat1_blkno + (offset >> BLK_SHIFT);

    status = rfat_fat_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
	fat_data = (uint32_t*)(entry->data + (offset & BLK_MASK));

	*fat_data = (*fat_data & 0xf0000000) | (RFAT_HTOFL(clsdata) & 0x0fffffff);

        rfat_fat_cache_modify(volume, entry);
    }
    
    return status;
}

static int rfat_cluster_chain_seek(rfat_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno)
{
    int status = F_NO_ERROR;
    uint32_t clsdata_r;

    *p_clsno  = RFAT_CLUSTER_END_OF_CHAIN;

    if ((clsno >= RFAT_CLUSTER_FIRST) && (clsno < RFAT_CLUSTER_RESERVED))
    {
	do
	{
	    status = (*volume->cluster.read)(volume, clsno, &clsdata_r);

	    if (status == F_NO_ERROR)
            {
                if (clsdata_r < RFAT_CLUSTER_LAST)
                {
                    clsno = clsdata_r;
                    clscnt--;
                }
	    }
	}
	while ((status == F_NO_ERROR) && (clsdata_r < RFAT_CLUSTER_LAST) && (clscnt != 0));

        if (clscnt == 0)
        {
            *p_clsno = clsno;
        }
    }

    return status;
}

static int rfat_cluster_chain_create(rfat_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno)
{
    int status = F_NO_ERROR;
    uint32_t clsno_s, clsno_a, clsno_f, clsno_n, clsno_l, clsdata_r;

    clsno_f = RFAT_CLUSTER_FREE;
    clsno_s = clsno;

    clsno_n = clsno_l = volume->free_clsno;

    do
    {
        do
        {
            clsno_a = clsno_n;

	    status = (*volume->cluster.read)(volume, clsno_a, &clsdata_r);

	    if (status == F_NO_ERROR)
            {
		if (clsdata_r == RFAT_CLUSTER_FREE)
		{
		    status = (*volume->cluster.write)(volume, clsno_a, RFAT_CLUSTER_END_OF_CHAIN);
		}

                if (status == F_NO_ERROR)
                {
		    clsno_n++;

                    if (clsno_n == volume->last_clsno)
                    {
                        clsno_n = 2; 
                    }
                }
            }
        }
        while ((status == F_NO_ERROR) && (clsdata_r != RFAT_CLUSTER_FREE) && (clsno_n != clsno_l));

        if (status == F_NO_ERROR)
        {
            if (clsdata_r == RFAT_CLUSTER_FREE)
            {
                if (clsno_f == RFAT_CLUSTER_FREE)
                {
                    clsno_f = clsno_a;
                }
                
                if (clsno_s != RFAT_CLUSTER_FREE)
                {
                    status = (*volume->cluster.write)(volume, clsno_s, clsno_a);
                }

                if (status == F_NO_ERROR)
                {
		    clscnt--;
                }
            }
        }
    }
    while ((status == F_NO_ERROR) && (clsdata_r == RFAT_CLUSTER_FREE) && (clscnt != 0));

    if (status == F_NO_ERROR)
    {
        if (clscnt != 0)
        {
            status = rfat_cluster_chain_destroy(volume, clsno_f, FALSE);

            if (status == F_NO_ERROR)
            {
                if (clsno != RFAT_CLUSTER_NONE)
                {
                    status = (*volume->cluster.write)(volume, clsno, RFAT_CLUSTER_END_OF_CHAIN);
                }
            }

            status = F_ERR_NOMOREENTRY;
        }
        else
        {
            volume->free_clsno = clsno_n;

            *p_clsno = clsno_f;
        }
    }

    return status;
}

static int rfat_cluster_chain_destroy(rfat_volume_t *volume, uint32_t clsno, int truncate)
{
    int status = F_NO_ERROR;
    uint32_t clsdata_r, clsdata_w;

    clsdata_w = (truncate ? RFAT_CLUSTER_END_OF_CHAIN : RFAT_CLUSTER_FREE);

    do
    {
	status = (*volume->cluster.read)(volume, clsno, &clsdata_r);

	if (status == F_NO_ERROR)
	{
	    status = (*volume->cluster.write)(volume, clsno, clsdata_w);

            if (clsdata_r < RFAT_CLUSTER_LAST)
            {
                clsno = clsdata_r;
                clsdata_w = RFAT_CLUSTER_FREE;
            }
	}
    }
    while ((status == F_NO_ERROR) && (clsdata_r < RFAT_CLUSTER_LAST));

    return status;
}

/***********************************************************************************************************************/

static int rfat_path_parse_filename(const uint8_t **p_path, uint8_t *dirname, int wildcard)
{
    int status = F_NO_ERROR;
    unsigned int c, n, nmax, nwildcard;
    const uint8_t *path;

    memset(dirname, ' ', 11);

    path = *p_path;
    c = *path++;

    if (c == '.')
    {
        if ((path[0] == F_SEPARATORCHAR) || (path[0] == '\0'))
        {
            /* special entry "."
             */
            dirname[0] = '.';
            path++;
        }
        else if ((path[0] == '.') && ((path[1] == F_SEPARATORCHAR) || (path[1] == '\0')))
        {
            /* special entry ".."
             */
            dirname[0] = '.';
            dirname[1] = '.';
            path += 2;
        }
        else
        {
            status = F_ERR_INVALIDNAME;
        }
    }
    else
    {
        for (n = 0, nmax = 8, nwildcard = 0; (status == F_NO_ERROR) &&  (c != F_SEPARATORCHAR) && (c != '\0'); c = *path++)
        {
            if (c == '.')
            {
                if (nmax == 8)
                {
                    n = 8;
                    nmax = 11;
                }
                else
                {
                    status = F_ERR_INVALIDNAME;
                }
            }
            else
            {
                if ((n == nmax) ||
                    ((n == 0) && (c == 0x20)) ||
                    (c < 0x20) ||
                    (c == '"') ||
                    (c == ',') ||
                    (c == '-') ||
                    (c == ':') ||
                    (c == ';') ||
                    (c == '<') ||
                    (c == '=') ||
                    (c == '>') ||
                    (c == '@') ||
                    (c == '[') ||
                    (c == '\\') ||
                    (c == ']') ||
                    (c == '|'))
                {
                    status = F_ERR_INVALIDNAME;
                }
                else
                {
                    if (c == '*')
		    {
                        if (wildcard)
                        {
                            /* Expand wildcard '*' to a sequence of '?'
                             */
                            for (; n < nmax; n++)
                            {
                                dirname[n] = '?';
                            }

                            nwildcard++;
                        }
                        else
                        {
                            status = F_ERR_INVALIDNAME;
                        }
		    }
                    else if (c == '?')
		    {
                        if (wildcard)
                        {
                            dirname[n++] = c;

                            nwildcard++;
                        }
                        else
                        {
                            status = F_ERR_INVALIDNAME;
                        }
                    }
		    else
		    {
			/* This is just a bad toupper() implementation.
			 * Should be at least iso8859-1 compatible.
			 */
			if ((c >= 'a') && (c <= 'z'))
			{
			    c -= ('a' - 'A');
			}
			
			if (c == 0xe5)
			{
			    c = 0x05;
			}
			
			dirname[n++] = c;
		    }
                }
            }
        }

        /* It's an error if the filename is empty, or if there
         * were wildcards in there and it's not the last
         * element of a path.
         */
        if ((n == 0) || ((c != '\0') && (nwildcard != 0)))
        {
            status = F_ERR_INVALIDNAME;
        }
    }

    if (status == F_NO_ERROR)
    {
        *p_path = path -1;
    }

    return status;
}

static int rfat_path_match_filename(const rfat_dir_t *dir, const uint8_t *dirname, int wildcard)
{
    unsigned int n;
    int match = TRUE;

    for (n = 0; n < 11; n++)
    {
        match = ((dirname[n] != '?') ? (dir->dir_name[n] == dirname[n]) : wildcard);

        if (!match)
        {
            break;
        }
    }

    return match;
}

static void rfat_path_convert_filename(const uint8_t *dirname, uint8_t *filename)
{
    unsigned int n;

    for (n = 7; n != 0; n--)
    {
        if (dirname[n] != ' ')
        {
            break;
        }
    }

    memcpy(filename, dirname, (n+1));

    filename += (n+1);

    for (n = 10; n != 7; n--)
    {
        if (dirname[n] != ' ')
        {
            break;
        }
    }
    
    if (n != 7)
    {
        *filename++ = '.';

        memcpy(filename, dirname+8, (n-7));
        
        filename += (n-7);
    }

    *filename++ = '\0';
}

static int rfat_path_merge_filename(const uint8_t *dirname, uint8_t *pathname)
{
    int status = F_NO_ERROR;
    uint8_t *filename, *pathsplit;

    for (filename = pathname, pathsplit = NULL; *filename != '\0'; filename++)
    {
        if (*filename == F_SEPARATORCHAR)
        {
            pathsplit = filename;
        }
    }

    if (dirname[0] == '.')
    {
        if (dirname[1] == '.')
        {
            /* ".." means the last path element needs to be stripped off */

	    if (pathname == pathsplit)
	    {
		pathname[0] = F_SEPARATORCHAR;
		pathname[1] = '\0';
	    }
	    else
	    {
		pathsplit[0] = '\0';
	    }
        }
        else
        {
            /* "." is redundant */
        }        
    }
    else
    {
        if ((F_MAXPATH - (filename - pathname)) < 13)
        {
            // ### not sure what to do. This is used for f_chdir(). It should
            // return NOTFOUND in that case ...
            status = F_ERR_NOTFOUND;
        }
        else
        {
	    if (*(filename-1) != F_SEPARATORCHAR)
	    {
		*filename++ = F_SEPARATORCHAR;
	    }

            rfat_path_convert_filename(dirname, filename);
            
        }
    }

    return status;
}

static int rfat_path_parse_label(const uint8_t *label, uint8_t *dirname)
{
    int status = F_NO_ERROR;
    unsigned int c, n;

    memset(dirname, ' ', 11);

    c = *label++;

    for (n = 0; (status == F_NO_ERROR) &&  (c != F_SEPARATORCHAR) && (c != '\0'); c = *label++)
    {
	if ((n == 11) ||
	    ((n == 0) && (c == 0x20)) ||
	    (c < 0x20) ||
	    (c == '"') ||
	    (c == ',') ||
	    (c == '-') ||
	    (c == '.') ||
	    (c == ':') ||
	    (c == ';') ||
	    (c == '<') ||
	    (c == '=') ||
	    (c == '>') ||
	    (c == '@') ||
	    (c == '[') ||
	    (c == '\\') ||
	    (c == ']') ||
	    (c == '|'))
	{
	    status = F_ERR_INVALIDNAME;
	}
	else
	{
	    /* This is just a bad toupper() implementation.
	     * Should be at least iso8859-1 compatible.
	     */
	    if ((c >= 'a') && (c <= 'z'))
	    {
		c -= ('a' - 'A');
	    }
	    
	    if (c == 0xe5)
	    {
		c = 0x05;
	    }
	    
	    dirname[n++] = c;
	}
    }
     
    /* It's an error if the volname is empty.
     */
    if (n == 0)
    {
	status = F_ERR_INVALIDNAME;
    }

    return status;
}

static void rfat_path_convert_label(const uint8_t *dirname, uint8_t *label)
{
    unsigned int n;

    for (n = 10; n != 0; n--)
    {
        if (dirname[n] != ' ')
        {
            break;
        }
    }

    memcpy(label, dirname, (n+1));

    label[n+1] = '\0';
}

static void rfat_path_setup_directory(rfat_volume_t *volume, rfat_dir_t *dir, const uint8_t *dirname, uint8_t attr, uint32_t clsno, uint32_t length)
{
    uint16_t crt_time, crt_date;

    RFAT_PORT_CLOCK(crt_time, crt_date);

    memcpy(dir->dir_name, dirname, 11);
    dir->dir_attr = attr;
    dir->dir_nt_reserved = 0;
    dir->dir_crt_time_tenth = 0;
                        
    if (volume->type == RFAT_VOLUME_TYPE_FAT32)
    {
        dir->dir_crt_time = crt_time;
        dir->dir_crt_date = crt_date;
        dir->dir_acc_date = 0;
    }
    else
    {
        dir->dir_crt_time = 0;
        dir->dir_crt_date = 0;
        dir->dir_clsno_hi = 0;
    }

    dir->dir_acc_date = 0;
    dir->dir_clsno_hi = RFAT_HTOFS(clsno >> 16);
    dir->dir_wrt_time = crt_time;
    dir->dir_wrt_date = crt_date;
    dir->dir_clsno_lo = RFAT_HTOFS(clsno & 0xffff);
    dir->dir_file_size = RFAT_HTOFL(length);
}

static int rfat_path_create_entry(rfat_volume_t *volume, uint32_t clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    blkno = BLKNO_NONE;
    offset = 0;

    status = rfat_cluster_chain_create(volume, clsno, 1, &clsno);

    if (status == F_NO_ERROR)
    {
        blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);

        status = rfat_disk_zero(volume, blkno, volume->cls_blk_size);

        if (status == F_NO_ERROR)
        {
            rfat_dir_cache_zero(volume, blkno, p_entry);
        }
    }
    
    *p_blkno = blkno;
    *p_offset = offset;

    return status;
}

static int rfat_path_destroy_entry(rfat_volume_t *volume, uint32_t blkno, uint32_t offset)
{
    int status = F_NO_ERROR;
    uint32_t clsno;
    rfat_dir_t *dir;
    rfat_cache_entry_t *entry;

    status = rfat_dir_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
        dir = (rfat_dir_t*)(entry->data + offset);
        
        if (volume->type == RFAT_VOLUME_TYPE_FAT32)
        {
            clsno = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
        }
        else
        { 
            clsno = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
        }

        dir->dir_name[0] = 0xe5;
        
        status = rfat_dir_cache_write(volume, entry);
        
        if (status == F_NO_ERROR)
        {
            if (clsno != RFAT_CLUSTER_NONE)
            {
                status = rfat_cluster_chain_destroy(volume, clsno, FALSE);
            }
        }
    }

    return status;
}

static int rfat_path_rename_entry(rfat_volume_t *volume, uint8_t *dirname, uint32_t blkno, uint32_t offset)
{
    int status = F_NO_ERROR;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;

    status = rfat_dir_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
        dir = (rfat_dir_t*)(entry->data + offset);

	memcpy(dir->dir_name, dirname, 11);

        status = rfat_dir_cache_write(volume, entry);
    }

    return status;
}

static int rfat_path_check_busy(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;
    rfat_file_t *file, *file_e;

    for (file = &volume->file_table[0], file_e = &volume->file_table[RFAT_CONFIG_MAX_FILES]; file < file_e; file++)
    {
	if (file->mode & (RFAT_FILE_MODE_READ | RFAT_FILE_MODE_WRITE))
	{
	    status = F_ERR_BUSY;

	    break;
	}
    }
    
    return status;
}

static int rfat_path_check_locked(rfat_volume_t *volume, uint32_t blkno, uint32_t offset)
{
    int status = F_NO_ERROR;
    rfat_file_t *file, *file_e;

    for (file = &volume->file_table[0], file_e = &volume->file_table[RFAT_CONFIG_MAX_FILES]; file < file_e; file++)
    {
	if ((file->dir_blkno == blkno) && (file->dir_offset == offset))
	{
	    status = F_ERR_LOCKED;

	    break;
	}
    }
    
    return status;
}

/* Simply return F_ERR_NOTEMPTY if the directory is not empty, or
 * F_NO_ERROR if it's empty.
 */
static int rfat_path_check_empty(rfat_volume_t *volume, uint32_t clsno)
{
    int status = F_NO_ERROR;
    int done;
    uint32_t blkno, blkno_e;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir, *dir_e;

    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
    blkno_e = blkno + volume->cls_blk_size;

    done = FALSE;

    while ((status == F_NO_ERROR) && !done)
    {
	if (blkno == blkno_e)
	{
	    status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);
			
	    if (status == F_NO_ERROR)
	    {
		if (clsno >= RFAT_CLUSTER_LAST)
		{
		    done = TRUE;
		}

		if (!done)
		{
		    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
		    blkno_e = blkno + volume->cls_blk_size;
		}
	    }
	}

	if (status == F_NO_ERROR)
	{
	    if (!done)
	    {
		status = rfat_dir_cache_read(volume, blkno, &entry);
                            
		if (status == F_NO_ERROR)
		{
		    dir = (rfat_dir_t*)entry->data;
		    dir_e = (rfat_dir_t*)(entry->data + BLK_SIZE);
                    
		    while ((status == F_NO_ERROR) && !done && (dir != dir_e))
		    {
			if (dir->dir_name[0] == 0x00)
			{
			    /* A 0x00 in dir_name[0] says it's a free entry, and all subsequent entries
			     * are also free. So we can stop searching.
			     */
			    done = TRUE;
			}
			else if (dir->dir_name[0] == 0xe5)
			{
			    /* A 0xe5 in dir_name[0] says it's a free entry.
			     */
                            
			    dir++;
			}
			else if (dir->dir_name[0] == '.')
			{
			    /* A '.' in dir_name[0] says it's either "." or "..".
			     */
                            
			    dir++;
			}
			else
			{
			    status = F_ERR_NOTEMPTY;
			}
		    }
		}
	    }
	}
    }

    return status;
}


/*
 * pathname   merged pathname including last element (if NULL, no merging takes place)
 * dirname    last path element
 * p_clsno    clsno of parent directory
 */

static int rfat_path_find_directory(rfat_volume_t *volume, int wildcard, const uint8_t *path, uint8_t *pathname, uint8_t *dirname, uint32_t *p_clsno)
{
    int status = F_NO_ERROR;
    int done;
    uint32_t clsno, clsno_d, clsno_n, blkno, blkno_e;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir, *dir_e;

    clsno_d = RFAT_CLUSTER_NONE;

    if (*path == '\0')
    {
	status = F_ERR_INVALIDNAME;
    }
    else if (*path == F_SEPARATORCHAR)
    {
        path++;

        if (pathname != NULL)
        {
            pathname[0] = F_SEPARATORCHAR;
            pathname[1] = '\0';
        }
    }
    else
    {
	if (volume->work_clsno != RFAT_CLUSTER_END_OF_CHAIN)
	{
	    clsno_d = volume->work_clsno;
	    
	    if (pathname != NULL)
	    {
		memcpy(pathname, volume->work_pathname, F_MAXPATH);
	    }
	}
	else
	{
	    status = F_ERR_INVALIDDIR;
	}
    }

    while ((status == F_NO_ERROR) && (*path != '\0'))
    {
        status = rfat_path_parse_filename(&path, dirname, wildcard);

        if (pathname != NULL)
        {
            if (status == F_NO_ERROR)
            {
                status = rfat_path_merge_filename(dirname, pathname);
            }
        }

        if (status == F_NO_ERROR)
        {
            if (*path != '\0')
            {
		if ((dirname[0] == '.') && (dirname[1] == ' '))
		{
		    path++;
		}
		else
		{
		    if (clsno_d == RFAT_CLUSTER_NONE)
		    {
			if (volume->type == RFAT_VOLUME_TYPE_FAT32)
			{
			    clsno = volume->root_clsno;
			    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			    blkno_e = blkno + volume->cls_blk_size;
			}
			else
			{
			    clsno = RFAT_CLUSTER_NONE;
			    blkno = volume->root_blkno;
			    blkno_e = blkno + volume->root_blkcnt;
			}
		    }
		    else
		    {
			clsno = clsno_d;
			blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			blkno_e = blkno + volume->cls_blk_size;
		    }

		    done = FALSE;
		    dir = NULL;
                
		    while ((status == F_NO_ERROR) && !done)
		    {
			if (blkno == blkno_e)
			{
			    if (clsno == RFAT_CLUSTER_NONE)
			    {
				done = TRUE;
				dir = NULL;
			    }
			    else
			    {
				status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno_n);
                            
				if (status == F_NO_ERROR)
				{
				    if (clsno_n >= RFAT_CLUSTER_LAST)
				    {
					done = TRUE;
					dir = NULL;
				    }
				    else
				    {
					clsno = clsno_n;
					blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
					blkno_e = blkno + volume->cls_blk_size;
				    }
				}
			    }
			}

			if (status == F_NO_ERROR)
			{
			    if (!done)
			    {
				status = rfat_dir_cache_read(volume, blkno, &entry);
                            
				if (status == F_NO_ERROR)
				{
				    dir = (rfat_dir_t*)entry->data;
				    dir_e = (rfat_dir_t*)(entry->data + BLK_SIZE);
                        
				    while ((status == F_NO_ERROR) && !done && (dir != dir_e))
				    {
					if (dir->dir_name[0] == 0x00)
					{
					    /* A 0x00 in dir_name[0] says it's a free entry, and all subsequent entries
					     * are also free. So we can stop searching.
					     */
                                
					    done = TRUE;
					    dir = NULL;
					}
					else if (dir->dir_name[0] == 0xe5)
					{
					    /* A 0xe5 in dir_name[0] says it's a free entry.
					     */
                                        
					    dir++;
					}
					else if ((dir->dir_attr & F_ATTR_DIR) && rfat_path_match_filename(dir, dirname, FALSE))
					{
					    done = TRUE;
					}
					else
					{
					    dir++;
					}
				    }
				}
			    }
			}
		    }

		    if (status == F_NO_ERROR)
		    {
			if (dir != NULL)
			{
			    if (volume->type == RFAT_VOLUME_TYPE_FAT32)
			    {
				clsno_d = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
			    }
			    else
			    { 
				clsno_d = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
			    }
                        
			    path++;
			}
			else
			{
			    //### status = F_ERR_NOTFOUND;
			    status = F_ERR_INVALIDDIR;
			}
		    }
		}
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        *p_clsno = clsno_d;
    }

    return status;
}

static int rfat_path_find_entry(rfat_volume_t *volume, int wildcard, uint32_t clsno, uint32_t blkno, uint32_t offset, uint8_t *dirname, uint32_t *p_clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    int done;
    uint32_t clsno_n, blkno_e, blkno_f, offset_f;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir, *dir_e;

    if (clsno == RFAT_CLUSTER_NONE)
    {
	if (volume->type == RFAT_VOLUME_TYPE_FAT32)
	{
	    clsno = volume->root_clsno;
	    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
	    blkno_e = blkno + volume->cls_blk_size;

            offset = 0;
	}
	else
	{
	    blkno_e = volume->root_blkno + volume->root_blkcnt;

            if (blkno == BLKNO_NONE)
            {
                blkno = volume->root_blkno;
                offset = 0;
            }
            else
            {
                offset += sizeof(rfat_dir_t);
            }
	}
    }
    else
    {
        blkno_e = (volume->cls_blk_offset + ((clsno + 1) << volume->cls_blk_shift));

	if (blkno == BLKNO_NONE)
	{
	    blkno = (volume->cls_blk_offset + (clsno << volume->cls_blk_shift));
	    offset = 0;
	}
	else
	{
	    offset += sizeof(rfat_dir_t);
	}
    }

    if (offset == BLK_SIZE)
    {
        blkno++;
        offset = 0;
    }

    blkno_f = BLKNO_NONE;
    offset_f = 0;
    
    done = FALSE;
    entry = NULL;

    while ((status == F_NO_ERROR) && !done)
    {
	if (blkno == blkno_e)
	{
	    if (clsno == RFAT_CLUSTER_NONE)
	    {
		done = TRUE;
		entry = NULL;
	    }
	    else
	    {
		status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno_n);
		
		if (status == F_NO_ERROR)
		{
		    if (clsno_n >= RFAT_CLUSTER_LAST)
		    {
			done = TRUE;
			entry = NULL;
		    }
                    else
                    {
                        clsno = clsno_n;
			blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			blkno_e = blkno + volume->cls_blk_size;
                    }
		}
	    }
        }

        if (status == F_NO_ERROR)
        {
            if (!done)
            {
                status = rfat_dir_cache_read(volume, blkno, &entry);
                
                if (status == F_NO_ERROR)
                {
                    dir = (rfat_dir_t*)(entry->data + offset);
                    dir_e = (rfat_dir_t*)(entry->data + BLK_SIZE);
                
                    offset = 0;

                    while ((status == F_NO_ERROR) && !done && (dir != dir_e))
                    {
                        if (dir->dir_name[0] == 0x00)
                        {
                            /* A 0x00 in dir_name[0] says it's a free entry, and all subsequent entries
                             * are also free. So we can stop searching.
                             */

                            blkno_f = entry->blkno;
                            offset_f = (uint8_t*)dir - entry->data;
                            
                            done = TRUE;
                            entry = NULL;
                        }
                        else if (dir->dir_name[0] == 0xe5)
                        {
                            /* A 0xe5 in dir_name[0] says it's a free entry.
                             */

                            blkno_f = entry->blkno;
                            offset_f = (uint8_t*)dir - entry->data;
                            
                            dir++;
                        }
                        else if (!(dir->dir_attr & F_ATTR_VOLUME) && rfat_path_match_filename(dir, dirname, wildcard))
                        { 
                            offset = (uint8_t*)dir - entry->data;
                            
                            done = TRUE;
                        }
                        else
                        {
                            dir++;
			}
		    }
		}
	    }
	}
    }

    if (status == F_NO_ERROR)
    {
        if (p_clsno)
        {
            *p_clsno = clsno;
        }

        if (p_blkno)
        {
	    if (entry != NULL)
	    {
		*p_blkno = blkno;
		*p_offset = offset;
	    }
	    else
	    {
		*p_blkno = blkno_f;
		*p_offset = offset_f;
	    }
	}

        *p_entry = entry;
    }

    return status;
}

static int rfat_path_find_file(rfat_volume_t *volume, const uint8_t *path, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    uint32_t clsno;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];

    status = rfat_path_find_directory(volume, FALSE, path, NULL, path_dirname, &clsno);

    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, NULL, p_blkno, p_offset, p_entry);

	if (*p_entry == NULL)
	{
	    status = F_ERR_NOTFOUND;
	}
    }

    return status;
}

static int rfat_path_find_label(rfat_volume_t *volume, uint32_t *p_clsno, uint32_t *p_blkno, uint32_t *p_offset, rfat_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    int done;
    uint32_t clsno, clsno_n, blkno, blkno_e, blkno_f, offset_f;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir, *dir_e;

    if (volume->type == RFAT_VOLUME_TYPE_FAT32)
    {
	clsno = volume->root_clsno;
	blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
	blkno_e = blkno + volume->cls_blk_size;
    }
    else
    {
	clsno = RFAT_CLUSTER_NONE;
	blkno = volume->root_blkno;
	blkno_e = blkno + volume->root_blkcnt;
    }

    blkno_f = BLKNO_NONE;
    offset_f = 0;
    
    done = FALSE;
    entry = NULL;

    while ((status == F_NO_ERROR) && !done)
    {
	if (blkno == blkno_e)
	{
	    if (clsno == RFAT_CLUSTER_NONE)
	    {
		done = TRUE;
		dir = NULL;
	    }
	    else
	    {
		status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno_n);
		
		if (status == F_NO_ERROR)
		{
		    if (clsno_n >= RFAT_CLUSTER_LAST)
		    {
			done = TRUE;
			entry = NULL;
		    }
                    else
                    {
                        clsno = clsno_n;
			blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			blkno_e = blkno + volume->cls_blk_size;
                    }
		}
	    }
        }

        if (status == F_NO_ERROR)
        {
            if (!done)
            {
                status = rfat_dir_cache_read(volume, blkno, &entry);
                
                if (status == F_NO_ERROR)
                {
                    dir = (rfat_dir_t*)entry->data;
                    dir_e = (rfat_dir_t*)(entry->data + BLK_SIZE);
                
                    while ((status == F_NO_ERROR) && !done && (dir != dir_e))
                    {
                        if (dir->dir_name[0] == 0x00)
                        {
                            /* A 0x00 in dir_name[0] says it's a free entry, and all subsequent entries
                             * are also free. So we can stop searching.
                             */

                            blkno_f = entry->blkno;
                            offset_f = (uint8_t*)dir - entry->data;
                            
                            done = TRUE;
                            entry = NULL;
                        }
                        else if (dir->dir_name[0] == 0xe5)
                        {
                            /* A 0xe5 in dir_name[0] says it's a free entry.
                             */

                            blkno_f = entry->blkno;
                            offset_f = (uint8_t*)dir - entry->data;
                            
                            dir++;
                        }
                        else if (dir->dir_attr & F_ATTR_VOLUME)
                        { 
                            done = TRUE;
                        }
                        else
                        {
                            dir++;
			}
		    }
		}
	    }
	}
    }

    if (status == F_NO_ERROR)
    {
        if (p_clsno)
        {
            *p_clsno = clsno;
        }

        if (p_blkno)
        {
	    *p_blkno = blkno_f;
	    *p_offset = offset_f;
	}

        *p_entry = entry;
    }

    return status;
}

static int rfat_path_find_next(rfat_volume_t *volume, F_FIND *find)
{
    int status = F_NO_ERROR;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;

    status = rfat_path_find_entry(volume, TRUE, find->find_clsno, find->find_blkno, find->find_offset, find->find_pattern, &find->find_clsno, &find->find_blkno, &find->find_offset, &entry);
	
    if (status == F_NO_ERROR)
    {
        if (entry != NULL)
        {
            dir = (rfat_dir_t*)(entry->data + find->find_offset);

            rfat_path_convert_filename(dir->dir_name, (uint8_t*)&find->filename[0]);
		
            memcpy(&find->name[0], &dir->dir_name[0], F_MAXNAME);
            memcpy(&find->ext[0], &dir->dir_name[F_MAXNAME], F_MAXEXT);
		
            find->attr = dir->dir_attr;
            find->ctime = RFAT_FTOHS(dir->dir_wrt_time);
            find->cdate = RFAT_FTOHS(dir->dir_wrt_date);

            if (volume->type == RFAT_VOLUME_TYPE_FAT32)
            {
                find->cluster = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
            }
            else
            {
                find->cluster = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
            }
		
            find->filesize = RFAT_FTOHL(dir->dir_file_size);
        }
        else
        {
            status = F_ERR_NOTFOUND;
        }
    }

    return status;
}


/***********************************************************************************************************************/

static int rfat_file_flush(rfat_volume_t *volume, rfat_file_t *file, int close)
{
    int status = F_NO_ERROR;
    uint16_t wrt_time, wrt_date;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;

    status = rfat_data_cache_flush(volume, file);

    if (status == F_NO_ERROR)
    {
        status = rfat_fat_cache_flush(volume);

        if (status == F_NO_ERROR)
        {
	    if ((file->flags & RFAT_FILE_FLAG_DIR_MODIFIED) || (close && (file->flags & RFAT_FILE_FLAG_DATA_MODIFIED)))
            {
                status = rfat_dir_cache_read(volume, file->dir_blkno, &entry);

                if (status == F_NO_ERROR)
                {
                    dir = (rfat_dir_t*)(entry->data + file->dir_offset);

                    RFAT_PORT_CLOCK(wrt_time, wrt_date);

                    dir->dir_attr |= F_ATTR_ARC;
                    dir->dir_wrt_time = RFAT_HTOFS(wrt_time);
                    dir->dir_wrt_date = RFAT_HTOFS(wrt_date);
                    dir->dir_clsno_lo = RFAT_HTOFS(file->first_clsno & 0xffff);
                    dir->dir_file_size = RFAT_HTOFL(file->length);

		    if (volume->type == RFAT_VOLUME_TYPE_FAT32)
		    {
			dir->dir_clsno_hi = RFAT_HTOFS(file->first_clsno >> 16);
		    }

                    status = rfat_dir_cache_write(volume, entry);
            
                    if (status == F_NO_ERROR)
                    {
			file->flags &= ~(RFAT_FILE_FLAG_DATA_MODIFIED | RFAT_FILE_FLAG_DIR_MODIFIED);
                    }
                }
            }
        }
    }

    return status;
}

static int rfat_file_seek(rfat_volume_t *volume, rfat_file_t *file, uint32_t position)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clscnt;

    if (file->position != position)
    {
        if (file->length == 0)
        {
            file->position = position;
            file->clsno = RFAT_CLUSTER_NONE;
            file->blkno = BLKNO_NONE;
            file->blkno_e = BLKNO_NONE;
        }
        else
        {
            if (position == 0)
            {
                file->position = 0;
                file->clsno = file->first_clsno;
                file->blkno = volume->cls_blk_offset + (file->clsno << volume->cls_blk_shift);
                file->blkno_e = file->blkno + volume->cls_blk_size;
            }
            else if (position <= file->length)
            {
                if ((file->position == 0) || (file->position > position))
                {
                    clsno = file->first_clsno;
                    clscnt = ((position -1) >> volume->cls_shift);
                }
                else
                {
                    clsno = file->clsno;
                    clscnt = ((position -1) >> volume->cls_shift) - ((file->position -1) >> volume->cls_shift);
                }

                if (clscnt != 0)
                {
                    status = rfat_cluster_chain_seek(volume, clsno, clscnt, &clsno);
                }

                if (status == F_NO_ERROR)
                {
                    file->position = position;
                    file->clsno = clsno;

                    if (!(position & volume->cls_mask))
                    {
                        file->blkno = volume->cls_blk_offset + ((clsno+1) << volume->cls_blk_shift);
                    }
                    else
                    {
                        file->blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift) + ((position & volume->cls_mask) >> BLK_SHIFT);
                    }

                    file->blkno_e = volume->cls_blk_offset + ((clsno+1) << volume->cls_blk_shift);
                }
            }
            else
            {
                if (file->position >= file->length)
                {
                    /* Nothing to do as we are already past the file->length
                     */
                    file->position = position;
                }
                else
                {
                    if (file->position == 0)
                    {
                        clsno = file->first_clsno;
                        clscnt = ((file->length -1) >> volume->cls_shift);
                    }
                    else
                    {
                        clsno = file->clsno;
                        clscnt = ((file->length -1) >> volume->cls_shift) - ((file->position -1) >> volume->cls_shift);
                    }

                    if (clscnt != 0)
                    {
                        status = rfat_cluster_chain_seek(volume, clsno, clscnt, &clsno);
                    }
                    
                    if (status == F_NO_ERROR)
                    {
                        file->position = position;
                        file->clsno = clsno;
                        
                        if (!(file->length & volume->cls_mask))
                        {
                            file->blkno = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
                        }
                        else
                        {
                            file->blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift) + ((file->length & volume->cls_mask) >> BLK_SHIFT);
                        }

                        file->blkno_e = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
                    }
                }
            }
        }
    }

    return status;
}

static int rfat_file_shrink(rfat_volume_t *volume, rfat_file_t *file, uint32_t length)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clscnt;

    if (length == 0)
    {
	status = rfat_cluster_chain_destroy(volume, file->first_clsno, FALSE);

	if (status == F_NO_ERROR)
	{
	    file->first_clsno = RFAT_CLUSTER_NONE;

	    file->length = 0;
	    file->position = 0;

	    /* file->position is 0 here, but clsno/blkno/blkno_e
	     * point to the first cluster, which just got deleted.
	     */
	    file->clsno = RFAT_CLUSTER_NONE;
	    file->blkno = BLKNO_NONE;
	    file->blkno_e = BLKNO_NONE;
	    
	    file->flags |= RFAT_FILE_FLAG_DIR_MODIFIED;
	}
    }
    else
    {
        if (file->position <= length)
        {
            clsno = file->clsno;
            clscnt = ((length -1) >> volume->cls_shift) - ((file->position -1) >> volume->cls_shift);
        }
        else
        {
            clsno = file->first_clsno;
            clscnt = ((length-1) >> volume->cls_shift);
        }

	if (clscnt != 0)
	{
	    status = rfat_cluster_chain_seek(volume, clsno, clscnt, &clsno);
	}

	if (status == F_NO_ERROR)
	{
	    status = rfat_cluster_chain_destroy(volume, clsno, TRUE);
	
	    if (status == F_NO_ERROR)
	    {
		file->length = length;
                file->position = length;

                file->clsno = clsno;

                if (file->length == 0)
                {
                    file->blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
                    file->blkno_e = file->blkno + volume->cls_blk_size;
                }
                else
                {
                    if (!(file->position & volume->cls_mask))
                    {
                        file->blkno = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
                    }
                    else
                    {
                        file->blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift) + ((file->position & volume->cls_mask) >> BLK_SHIFT);
                    }

                    file->blkno_e = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
                }

		file->flags |= RFAT_FILE_FLAG_DIR_MODIFIED;
	    }
	}
    }

    // rfat_fat_cache_flush(volume);

    return status;
}

static int rfat_file_extend(rfat_volume_t *volume, rfat_file_t *file, uint32_t length, int truncate)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clscnt, clsno_n, blkno, blkno_e, blkno_n, blkcnt, count;
    uint32_t zero_address, zero_length;

    if (file->length == 0)
    {
        clsno = RFAT_CLUSTER_NONE;
        clscnt = (((length -1) + volume->cls_mask) >> volume->cls_shift);
    }
    else
    {
        clsno = file->clsno;
        
        /* If the position is beyond the length, a previous seek did put us there,
         * which means it already did a seek to the end of the file. So no need to
         * seek to the file end. If they are equal, there is also no need to scan.
         */
        if (file->length > file->position)
        {
            clscnt = ((file->length -1) >> volume->cls_shift) - ((file->position -1) >> volume->cls_shift);
            
            if (clscnt != 0)
            {
                status = rfat_cluster_chain_seek(volume, clsno, clscnt, &clsno);
            }
        }
        
        if (status == F_NO_ERROR)
        {
            clscnt = ((length -1) >> volume->cls_shift) - ((file->length -1) >> volume->cls_shift);
        }
    }
    
    if (status == F_NO_ERROR)
    {
        if (clscnt != 0)
        {
            status = rfat_cluster_chain_create(volume, clsno, clscnt, &clsno_n);
	    
	    if (status == F_NO_ERROR)
	    {
		if (file->length == 0)
		{
		    file->first_clsno = clsno_n;
		    file->clsno = clsno_n;
		    file->blkno = volume->cls_blk_offset + (clsno_n << volume->cls_blk_shift);
		    file->blkno_e = file->blkno + volume->cls_blk_size;
		}
	    }
        }
    }

    if (status == F_NO_ERROR)
    {
	if (file->position > file->length)
	{
	    if (file->length == 0)
	    {
		clsno = file->clsno;
		blkno = file->blkno;
		blkno_e = file->blkno_e;
	    }
	    else
	    {
		if (!(file->length & volume->cls_mask))
		{
		    blkno = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
		}
		else
		{
		    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift) + ((file->length & volume->cls_mask) >> BLK_SHIFT);
		}
            
		blkno_e = volume->cls_blk_offset + ((clsno +1) << volume->cls_blk_shift);
	    }

	    if (truncate)
	    {
		count = length - file->length;
	    }
	    else
	    {
		count = file->position - file->length;
	    }

	    if (count)
	    {
		zero_address = BLKNO_INVALID;
		zero_length = 0;

		if (blkno == blkno_e)
		{
		    status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);
                
		    if (status == F_NO_ERROR)
		    {
			blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			blkno_e = blkno + volume->cls_blk_size;
		    }
		}
            
		if (status == F_NO_ERROR)
		{
		    /* Skip the first block if it is already partial covered, as then
		     * it had to be zeroed out before.
		     */
		    if (file->length & BLK_MASK)
		    {
			if (count >= (BLK_SIZE - (file->length & BLK_MASK)))
			{
			    count -= (BLK_SIZE - (file->length & BLK_MASK));
			    blkno++;
			}
			else
			{
			    count = 0;
			}

		    }

		    while ((status == F_NO_ERROR) && (count != 0))
		    {
			if (blkno == blkno_e)
			{
			    status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);
                        
			    if (status == F_NO_ERROR)
			    {
				blkno_n = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
				blkno_e = blkno_n + volume->cls_blk_size;
                            
				if (blkno != blkno_n)
				{
				    if (zero_length != 0)
				    {
					status = rfat_disk_zero(volume, zero_address, zero_length);

					zero_address = BLKNO_INVALID;
					zero_length = 0;
				    }

				    blkno = blkno_n;
				}

				if (status == F_NO_ERROR)
				{
				    if (count < BLK_SIZE)
				    {
					if (zero_length == 0)
					{
					    zero_address = blkno;
					}

					zero_length++;

					count = 0;
				    }
				    else
				    {
					blkcnt = count >> BLK_SHIFT;

					if (blkcnt > (blkno_e - blkno))
					{
					    blkcnt = (blkno_e - blkno);
					}

					if (zero_length == 0)
					{
					    zero_address = blkno;
					}

					zero_length += blkcnt;

					count += (blkcnt << BLK_SHIFT);
					blkno += blkcnt;
				    }
				}
			    }
			}
		    }

		    if (status == F_NO_ERROR)
		    {
			if (zero_length != 0)
			{
			    status = rfat_disk_zero(volume, zero_address, zero_length);
			}
		    }
		}
	    }

	    if (status == F_NO_ERROR)
	    {
		file->clsno = clsno;
		file->blkno = blkno;
		file->blkno_e = blkno_e;
	    }
	}
    }


    if (status == F_NO_ERROR)
    {
	/* For the append, we just scanned forward to file->position.
	 * For the truncate case we scanned forward to the new file->length.
	 * Hence store clsno/blkno/blkno_e in "file" and update file->position
	 * for the non-append case.
	 */
	
	file->length = length;
	
	if (truncate)
	{
	    file->position = length;
	}
	
	
	file->flags |= RFAT_FILE_FLAG_DIR_MODIFIED;
    }

    // rfat_fat_cache_flush(volume);

    return status;
}

static int rfat_file_open(rfat_volume_t *volume, const uint8_t *filename, uint32_t mode, uint32_t length, rfat_file_t **p_file)
{
    int status = F_NO_ERROR;
    unsigned int n;
    uint32_t clsno, blkno, offset;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_file_t *file;
    rfat_dir_t *dir;
    rfat_cache_entry_t *entry;

    file = NULL;

    for (n = 0; n < RFAT_CONFIG_MAX_FILES; n++)
    {
	if (!(volume->file_table[n].mode & RFAT_FILE_MODE_DISK))
        {
            file = &volume->file_table[n];
            break;
        }
    }

    if (file)
    {
        status = rfat_path_find_directory(volume, FALSE, filename, NULL, path_dirname, &clsno);
	
        if (status == F_NO_ERROR)
        {
            status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, &clsno, &blkno, &offset, &entry);

            if (status == F_NO_ERROR)
            {
                if (entry != NULL)
                {
                    dir = (rfat_dir_t*)(entry->data + offset);
                    
                    if (dir->dir_attr & F_ATTR_DIR)
                    {
                        status = F_ERR_INVALIDDIR;
                    }
                    else
                    {
                        if ((mode & RFAT_FILE_MODE_WRITE) && (dir->dir_attr & F_ATTR_READONLY))
                        {
                            status = F_ERR_ACCESSDENIED;
                        }
                        else
                        {
                            for (n = 0; n < RFAT_CONFIG_MAX_FILES; n++)
                            {
                                if ((volume->file_table[n].dir_blkno == blkno) && (volume->file_table[n].dir_offset == offset))
                                {
                                    if (mode & RFAT_FILE_MODE_WRITE)
                                    {
                                        status = F_ERR_LOCKED;
                                    }
                                    else
                                    {
                                        if (volume->file_table[n].mode & RFAT_FILE_MODE_WRITE)
                                        {
                                            status = F_ERR_LOCKED;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (status == F_NO_ERROR)
                {
                    if (!(mode & (RFAT_FILE_MODE_CREATE | RFAT_FILE_MODE_APPEND)))
                    {
                        if (entry == NULL)
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                    else
                    {
                        if (entry == NULL)
                        {
                            if (blkno == BLKNO_NONE)
                            {
                                status = rfat_path_create_entry(volume, clsno, &blkno, &offset, &entry);
                            }
                            else
                            {
                                status = rfat_dir_cache_read(volume, blkno, &entry);
                            }

                            if (status == F_NO_ERROR)
                            {
                                dir = (rfat_dir_t*)(entry->data + offset);
                            
                                rfat_path_setup_directory(volume, dir, path_dirname, 0, RFAT_CLUSTER_NONE, 0);
                                
                                status = rfat_dir_cache_write(volume, entry);
                            }
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        file->flags = 0;
                        file->dir_offset = offset;
                        file->dir_blkno = blkno;

			dir = (rfat_dir_t*)(entry->data + offset);
                    
                        if (volume->type == RFAT_VOLUME_TYPE_FAT32)
                        {
                            file->first_clsno = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                        }
                        else
                        {
                            file->first_clsno = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                        }
                    
                        file->length = RFAT_FTOHL(dir->dir_file_size);
                        file->position = 0;

                        if (file->length == 0)
                        {
                            file->clsno = RFAT_CLUSTER_NONE;
                            file->blkno = BLKNO_NONE;
                            file->blkno_e = BLKNO_NONE;
                        }
                        else
                        {
                            file->clsno = file->first_clsno;
                            file->blkno = volume->cls_blk_offset + (file->clsno << volume->cls_blk_shift);
                            file->blkno_e = volume->cls_blk_offset + ((file->clsno +1) << volume->cls_blk_shift);
                        }

                        if (mode & RFAT_FILE_MODE_TRUNCATE)
                        {
                            if (file->length != length)
                            {
                                if (file->length < length)
                                {
                                    status = rfat_file_extend(volume, file, length, TRUE);
                                }
                                else
                                {
                                    status = rfat_file_shrink(volume, file, length);
                                }
                            }
                        }
                        else if (mode & RFAT_FILE_MODE_CREATE)
                        {
                            if (file->length != 0)
                            {
                                status = rfat_file_shrink(volume, file, 0);
                            }
                        }
                        else if (mode & RFAT_FILE_MODE_APPEND)
                        {
                            status = rfat_file_seek(volume, file, file->length);
                        }

                        if (status == F_NO_ERROR)
                        {
                            file->mode = RFAT_FILE_MODE_DISK | mode;
                            file->data_cache.blkno = BLKNO_INVALID;

			    status = rfat_file_flush(volume, file, FALSE);
                        }
                    }
                }
            }
        }
    }

    *p_file = file;

    return status;
}

static void rfat_file_close(rfat_volume_t *volume, rfat_file_t *file)
{
    file->mode = 0;
    file->dir_blkno = BLKNO_NONE;
    file->dir_offset = 0;
}

static int rfat_file_read(rfat_volume_t *volume, rfat_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count)
{
    int status = F_NO_ERROR;
    uint32_t blkno, blkno_e, blkcnt, clsno, position, total, size;
    rfat_cache_entry_t *entry;

    *p_count = 0;

    if (file->position >= file->length)
    {
	count = 0;
    }
    else
    {
	if (count > (file->length - file->position))
	{
	    count = (file->length - file->position);
	}
    }

    if (count != 0)
    {
	total = count;

        position = file->position;
        clsno = file->clsno;
        blkno = file->blkno;
        blkno_e = file->blkno_e;

        if (blkno == blkno_e)
        {
            status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);

            if (status == F_NO_ERROR)
            {
                blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
                blkno_e = blkno + volume->cls_blk_size;
            }
        }
            
        if (status == F_NO_ERROR)
        {
            if (((position & BLK_MASK) + count) < BLK_SIZE)
            {
                status = rfat_data_cache_read(volume, file, blkno, &entry);
	    
                if (status == F_NO_ERROR)
                {
                    memcpy(data, entry->data + (position & BLK_MASK), count);

                    position += count;
                    count = 0;

                    if (!(position & BLK_MASK))
                    {
                        blkno++;
                    }
                }
            }
            else
            {
                if (position & BLK_MASK)
                {
                    status = rfat_data_cache_read(volume, file, blkno, &entry);
	    
                    if (status == F_NO_ERROR)
                    {
                        size = (BLK_SIZE - (position & BLK_MASK));

                        memcpy(data, entry->data + (BLK_SIZE - size), size);

                        position += size;
                        data += size;
                        count -= size;

                        blkno++;
                    }
                }

                while ((status == F_NO_ERROR) && (count != 0))
                {
                    if (blkno == blkno_e)
                    {
                        status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);
                        
                        if (status == F_NO_ERROR)
                        {
                            blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
                            blkno_e = blkno + volume->cls_blk_size;
                        }
                    }
                
                    if (status == F_NO_ERROR)
                    {
                        if (count < BLK_SIZE)
                        {
                            status = rfat_data_cache_read(volume, file, blkno, &entry);
                    
                            if (status == F_NO_ERROR)
                            {
                                memcpy(data, entry->data, count);

                                position += count;
                                data += count;
                                count = 0;
                            }
                        }
                        else
                        {
                            size = volume->cls_size - (count & (volume->cls_mask & ~BLK_MASK));
                            
                            if (size > count)
                            {
                                size = count & ~BLK_MASK;
                            }

                            blkcnt = size >> BLK_SHIFT;

                            status = rfat_data_cache_invalidate(volume, file, blkno, blkcnt, TRUE);

                            if (status == F_NO_ERROR)
                            {
                                status = rfat_disk_read_multiple(volume, blkno, blkcnt, data);

                                if (status == F_NO_ERROR)
                                {
                                    position += size;
                                    data += size;
                                    count -= size;

                                    blkno += blkcnt;
                                }

                            }
                        }
                    }
                }
            }
        
            if (status == F_NO_ERROR)
            {
                file->position = position;
                file->clsno = clsno;
                file->blkno = blkno;
                file->blkno_e = blkno_e;
            }
        }

	*p_count = total - count;
    }

    return status;
}

static int rfat_file_write(rfat_volume_t *volume, rfat_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count)
{
    int status = F_NO_ERROR;
    uint32_t blkno, blkno_e, blkcnt, clsno, position, length, length_n, total, size;
    rfat_cache_entry_t *entry;

    *p_count = 0;

    if ((file->mode & RFAT_FILE_MODE_APPEND) && (file->position != file->length))
    {
	status = rfat_file_seek(volume, file, file->length);
    }

    if (status == F_NO_ERROR)
    {
	if (count > (RFAT_FILE_SIZE_MAX - file->position))
	{
	    count = (RFAT_FILE_SIZE_MAX - file->position);
	}

	if (count != 0)
	{
	    total = count;
            length = file->length;
	    length_n = file->position + count;

	    if (length_n > length)
	    {
		if (length == 0)
		{
		    status = rfat_file_extend(volume, file, length_n, FALSE);
		}
		else
		{
		    if ((length != file->position) || (((length -1) & ~volume->cls_mask) != ((length_n - 1) & ~volume->cls_mask)))
		    {
			status = rfat_file_extend(volume, file, length_n, FALSE);
		    }
		    else
		    {
			file->length = length_n;

                        file->flags |= RFAT_FILE_FLAG_DIR_MODIFIED;
		    }
		}
	    }

	    if (status == F_NO_ERROR)
	    {
		position = file->position;
		clsno = file->clsno;
		blkno = file->blkno;
		blkno_e = file->blkno_e;

		if (blkno == blkno_e)
		{
		    status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);

		    if (status == F_NO_ERROR)
		    {
			blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
			blkno_e = blkno + volume->cls_blk_size;
		    }
		}

		if (status == F_NO_ERROR)
		{
		    if (((position & BLK_MASK) + count) < BLK_SIZE)
		    {
			/* All data is within one block, simply go throu the cache.
                         * If the block is beyond the old file->length, is has to 
                         * be all zeros. Otherwise got throu the normal cache.
                         */

			if ((position & ~BLK_MASK) >= length)
			{
			    status = rfat_data_cache_check(volume, file, blkno, &entry);

			    if (status == F_NO_ERROR)
			    {
				memset(entry->data, 0, BLK_SIZE);
                        
				entry->blkno = blkno;
			    }
			}
			else
			{
                            status = rfat_data_cache_read(volume, file, blkno, &entry);
			}
	    
			if (status == F_NO_ERROR)
			{
			    memcpy(entry->data + (position & BLK_MASK), data, count);

                            rfat_data_cache_modify(volume, file, entry);

                            position += count;
			    count = 0;

                            if (!(position & BLK_MASK))
                            {
                                blkno++;
                            }
			}
		    }
		    else
		    {
			if (position & BLK_MASK)
			{
                            if ((position & ~BLK_MASK) >= length)
                            {
                                status = rfat_data_cache_check(volume, file, blkno, &entry);
                                
                                if (status == F_NO_ERROR)
                                {
                                    memset(entry->data, 0, BLK_SIZE);
                                    
                                    entry->blkno = blkno;
                                }
                            }
                            else
                            { 
                                status = rfat_data_cache_read(volume, file, blkno, &entry);
                            }

			    if (status == F_NO_ERROR)
			    {
				size = (BLK_SIZE - (position & BLK_MASK));

                                memcpy(entry->data + (BLK_SIZE - size), data, size);

                                status = rfat_data_cache_write(volume, file, entry);

                                if (status == F_NO_ERROR)
                                {
                                    position += size;
                                    data += size;
                                    count -= size;
                                    
                                    blkno++;
                                }
			    }
			}

			while ((status == F_NO_ERROR) && (count != 0))
			{
			    if (blkno == blkno_e)
			    {
				status = rfat_cluster_chain_seek(volume, clsno, 1, &clsno);

				if (status == F_NO_ERROR)
				{
				    blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);
				    blkno_e = blkno + volume->cls_blk_size;
                                }
                            }

			    if (status == F_NO_ERROR)
			    {
                                if (count < BLK_SIZE)
                                {
                                    if (position >= length)
                                    {
                                        status = rfat_data_cache_check(volume, file, blkno, &entry);
                                        
                                        if (status == F_NO_ERROR)
                                        {
                                            memset(entry->data, 0, BLK_SIZE);
                                                
                                            entry->blkno = blkno;
                                        }
                                    }
                                    else
                                    {
                                        status = rfat_data_cache_read(volume, file, blkno, &entry);
                                    }

                                    if (status == F_NO_ERROR)
                                    {
                                        memcpy(entry->data, data, count);

                                        status = rfat_data_cache_write(volume, file, entry);

                                        if (status == F_NO_ERROR)
                                        {
                                            position += count;
                                            data += count;
                                            count = 0;
                                        }
                                    }
                                }
                                else
                                {
                                    size = volume->cls_size - (count & (volume->cls_mask & ~BLK_MASK));
			    
                                    if (size > count)
                                    {
                                        size = count & ~BLK_MASK;
                                    }

                                    blkcnt = size >> BLK_SHIFT;

                                    status = rfat_data_cache_invalidate(volume, file, blkno, blkcnt, FALSE);

                                    if (status == F_NO_ERROR)
                                    {
                                        status = rfat_disk_write_multiple(volume, blkno, blkcnt, data);

                                        if (status == F_NO_ERROR)
                                        {
                                            position += size;
                                            data += size;
                                            count -= size;
                                            
                                            blkno += blkcnt;
                                        }
                                    }
                                }
                            }
                        }
		    }
		}

		if (status == F_NO_ERROR)
		{
		    file->position = position;
		    file->clsno = clsno;
		    file->blkno = blkno;
		    file->blkno_e = blkno_e;
		}
	    }

	    if (total != count)
	    {
		file->flags |= RFAT_FILE_FLAG_DATA_MODIFIED;
	    }

	    *p_count = total - count;
	}
    }

    return status;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/

int f_initvolume(void)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock_noinit(volume);
    
    if (status == F_NO_ERROR)
    {
        status = rfat_volume_init(volume);
        
	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_delvolume(void)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock_nomount(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_check_busy(volume);

	if (status == F_NO_ERROR)
	{
	    status = rfat_volume_unmount(volume);

	    if (status == F_NO_ERROR)
	    {
		volume->state = RFAT_VOLUME_STATE_NONE;
	    }
	}
        
	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_format(int fattype)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock_nomount(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_check_busy(volume);

	if (status == F_NO_ERROR)
	{
	    status = rfat_volume_format(volume);
        }
        
	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_getfreespace(F_SPACE *pspace)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clsno_e, clscnt_total, clscnt_free, clscnt_used, clscnt_bad, clsdata;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        clscnt_total = 0;
        clscnt_free  = 0;
        clscnt_used  = 0;
        clscnt_bad   = 0;

        for (clsno = 2, clsno_e = volume->last_clsno; ((status == F_NO_ERROR) && (clsno <= clsno_e)); clsno++)
        {
            status = (*volume->cluster.read)(volume, clsno, &clsdata);

            if (status == F_NO_ERROR)
            {
                clscnt_total++;

                if (clsdata == RFAT_CLUSTER_FREE)
                {
                    clscnt_free++;
                }
                else if (clsdata == RFAT_CLUSTER_BAD)
                {
                    clscnt_bad++;
                }
                else
                {
                    clscnt_used++;
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            pspace->total      = clscnt_total << volume->cls_shift;
            pspace->free       = clscnt_free  << volume->cls_shift;
            pspace->used       = clscnt_used  << volume->cls_shift;
            pspace->bad        = clscnt_bad   << volume->cls_shift;
            pspace->total_high = clscnt_total >> (32 - volume->cls_shift);
            pspace->free_high  = clscnt_free  >> (32 - volume->cls_shift);
            pspace->used_high  = clscnt_used  >> (32 - volume->cls_shift);
            pspace->bad_high   = clscnt_bad   >> (32 - volume->cls_shift);
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_setlabel(const char *label)
{
    int status = F_NO_ERROR;
    uint32_t clsno, blkno, offset;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_dir_t *dir;
    rfat_cache_entry_t *entry;
    rfat_volume_t *volume;

    volume = &rfat_volume;
    
    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_parse_label((const uint8_t*)label, path_dirname);

	if (status == F_NO_ERROR)
	{
	    status = rfat_path_find_label(volume, &clsno, &blkno, &offset, &entry);

	    if (status == F_NO_ERROR)
	    {
		if (entry == NULL)
		{
		    if (blkno == BLKNO_NONE)
		    {
			status = rfat_path_create_entry(volume, clsno, &blkno, &offset, &entry);
		    }
		    else
		    {
			status = rfat_dir_cache_read(volume, blkno, &entry);
		    }
		}

		if (status == F_NO_ERROR)
		{
                    dir = (rfat_dir_t*)(entry->data + offset);

		    rfat_path_setup_directory(volume, dir, path_dirname, F_ATTR_VOLUME, RFAT_CLUSTER_NONE, 0);
		
		    status = rfat_dir_cache_write(volume, entry);
		}
	    }
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_getlabel(char *label, int maxlen)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_dir_t *dir;
    rfat_cache_entry_t *entry;
    rfat_volume_t *volume;

    volume = &rfat_volume;
    
    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (maxlen < 12)
        {
            status = F_ERR_TOOLONGNAME;
        }
        else
        {
	    status = rfat_path_find_label(volume, NULL, &blkno, &offset, &entry);

	    if (status == F_NO_ERROR)
	    {
		if (entry == NULL)
		{
		    status = rfat_dir_cache_read(volume, volume->boot_blkno, &entry);

		    if (status == F_NO_ERROR)
		    {
			if (volume->type == RFAT_VOLUME_TYPE_FAT32)
			{
                            rfat_path_convert_label(entry->data + RFAT_BS_VolLab32, (uint8_t*)label);
			}
			else
			{
			    rfat_path_convert_label(entry->data + RFAT_BS_VolLab, (uint8_t*)label);
			}
		    }
		}
		else
		{
                    dir = (rfat_dir_t*)(entry->data + offset);

		    rfat_path_convert_label(dir->dir_name, (uint8_t*)label);
		}
	    }
	}

        rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}


int f_mkdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clsno_e, clsno_s, blkno, blkno_s, offset;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_dir_t *dir;
    rfat_cache_entry_t *entry;
    rfat_volume_t *volume;

    volume = &rfat_volume;
    
    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if ((dirname[0] == F_SEPARATORCHAR) && (dirname[1] == '\0'))
        {
            status = F_ERR_INVALIDDIR;
        }
        else
        {
            status = rfat_path_find_directory(volume, FALSE, (const uint8_t*)dirname, NULL, path_dirname, &clsno);
            
            if (status == F_NO_ERROR)
            {
                if (path_dirname[0] == '.')
                {
		    status = F_ERR_NOTFOUND;
		}
		else
		{
		    status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, &clsno_e, &blkno, &offset, &entry);

		    if (status == F_NO_ERROR)
		    {
			if (entry == NULL)
			{
			    status = rfat_cluster_chain_create(volume, RFAT_CLUSTER_NONE, 1, &clsno_s);

			    if (status == F_NO_ERROR)
			    {
				blkno_s = volume->cls_blk_offset + (clsno_s << volume->cls_blk_shift);
                            
				status = rfat_disk_zero(volume, blkno_s, volume->cls_blk_size);

                                if (status == F_NO_ERROR)
                                {
                                    if (blkno == BLKNO_NONE)
                                    {
                                        status = rfat_path_create_entry(volume, clsno_e, &blkno, &offset, &entry);
                                    }
                                    else
                                    {
                                        status = rfat_dir_cache_read(volume, blkno, &entry);
                                    }
                                    
                                    dir = (rfat_dir_t*)(entry->data + offset);
                                    
                                    rfat_path_setup_directory(volume, dir, path_dirname, F_ATTR_DIR, clsno_s, 0);
                                    
                                    status = rfat_dir_cache_write(volume, entry);
                        
                                    if (status == F_NO_ERROR)
                                    {
                                        rfat_dir_cache_zero(volume, blkno_s, &entry);
                                        
                                        dir = (rfat_dir_t*)(entry->data + 0);
                                        
                                        rfat_path_setup_directory(volume, dir, rfat_dirname_dot, F_ATTR_DIR, clsno_s, 0);
                                        
                                        dir = (rfat_dir_t*)(entry->data + sizeof(rfat_dir_t));
                                        
                                        rfat_path_setup_directory(volume, dir, rfat_dirname_dotdot, F_ATTR_DIR, clsno, 0);
                                        
                                        status = rfat_dir_cache_write(volume, entry);
                                    }
                                }
                            }
                        }
                        else
			{
			    status = F_ERR_DUPLICATED;
			}
		    }
		}
            }
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_rmdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, blkno, offset;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;
    
    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if ((dirname[0] == F_SEPARATORCHAR) && (dirname[1] == '\0'))
        {
            status = F_ERR_INVALIDDIR;
        }
        else
        {
            status = rfat_path_find_directory(volume, FALSE, (const uint8_t*)dirname, NULL, path_dirname, &clsno);
            
            if (status == F_NO_ERROR)
            {
		if (path_dirname[0] == '.')
		{
		    status = F_ERR_NOTFOUND;
		}
		else
		{
		    status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, NULL, &blkno, &offset, &entry);

		    if (status == F_NO_ERROR)
		    {
			if (entry != NULL)
			{
                            dir = (rfat_dir_t*)(entry->data + offset);

			    if (dir->dir_attr & F_ATTR_DIR)
			    {
				if (volume->type == RFAT_VOLUME_TYPE_FAT32)
				{
				    clsno = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
				}
				else
				{ 
				    clsno = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
				}
			    
				status = rfat_path_check_empty(volume, clsno);
                            
				if (status == F_NO_ERROR)
				{
				    status = rfat_path_destroy_entry(volume, blkno, offset);

				    if (status == F_NO_ERROR)
				    {
					if (volume->work_clsno == clsno)
					{
					    volume->work_clsno = RFAT_CLUSTER_END_OF_CHAIN;
					}
				    }
				}
			    }
			    else
			    {
				status = F_ERR_INVALIDDIR;
			    }
			}
			else
			{
			    status = F_ERR_NOTFOUND;
			}
		    }
		}
            }
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_chdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, blkno, offset;
    uint8_t path_pathname[F_MAXPATH];
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;
    
    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if ((dirname[0] == F_SEPARATORCHAR) && (dirname[1] == '\0'))
        {
            /* rfat_path_find_directory cannot return a vaild "dir" for the "/",
             * so special code this here.
             */
            volume->work_clsno = RFAT_CLUSTER_NONE;
            volume->work_pathname[0] = F_SEPARATORCHAR;
            volume->work_pathname[1] = '\0';
        }
        else
        {
            status = rfat_path_find_directory(volume, FALSE, (const uint8_t*)dirname, path_pathname, path_dirname, &clsno);
            
            if (status == F_NO_ERROR)
            {
                if (path_dirname[0] == '.')
                {
                    if (path_dirname[1] == ' ')
                    {
                        /* "." */
                        volume->work_clsno = clsno;
                    }
                    else
                    {
                        /* ".." */

                        if (clsno != RFAT_CLUSTER_NONE)
                        {
                            blkno = volume->cls_blk_offset + (clsno << volume->cls_blk_shift);

                            status = rfat_dir_cache_read(volume, blkno, &entry);
                                                
                            if (status == F_NO_ERROR)
                            {
                                dir = (rfat_dir_t*)(entry->data + sizeof(rfat_dir_t));
                                
                                if (volume->type == RFAT_VOLUME_TYPE_FAT32)
                                {
                                    volume->work_clsno = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                                }
                                else
                                {
                                    volume->work_clsno = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                                }
                            }
                        }
                        else
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                }
                else
                {
                    status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, NULL, &blkno, &offset, &entry);

                    if (status == F_NO_ERROR)
                    {
                        if (entry != NULL)
                        {
                            dir = (rfat_dir_t*)(entry->data + offset);
                            
                            if (dir->dir_attr & F_ATTR_DIR)
                            {
                                if (volume->type == RFAT_VOLUME_TYPE_FAT32)
                                {
                                    volume->work_clsno = ((uint32_t)RFAT_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                                }
                                else
                                {
                                    volume->work_clsno = (uint32_t)RFAT_FTOHS(dir->dir_clsno_lo);
                                }
                            }
                            else
                            {
                                status = F_ERR_INVALIDDIR;
                            }
                        }
                        else
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                }

                if (status == F_NO_ERROR)
                {
                    memcpy(volume->work_pathname, path_pathname, F_MAXPATH);
                }
            }
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_getcwd(char *dirname, int maxlen)
{
    int status = F_NO_ERROR;
    unsigned int n;
    rfat_volume_t *volume;
    
    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        for (n = 0; n < F_MAXPATH; n++)
        {
            if (volume->work_pathname[n] == '\0')
            {
                break;
            }
        }

        if ((n+1) <= maxlen)
        {
            memcpy(dirname, volume->work_pathname, n);

            dirname[n] = '\0';
        }
        else
        {
            status = F_ERR_NOTUSEABLE;
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_rename(const char *filename, const char *newname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, blkno, offset;
    uint8_t path_dirname[F_MAXNAME+F_MAXEXT];
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = rfat_path_find_directory(volume, FALSE, (const uint8_t*)filename, NULL, path_dirname, &clsno);
            
        if (status == F_NO_ERROR)
        {
            status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, NULL, &blkno, &offset, &entry);

            if (status == F_NO_ERROR)
            {
                if (entry != NULL)
                {
                    dir = (rfat_dir_t*)(entry->data + offset);

                    if (dir->dir_attr & F_ATTR_READONLY)
                    {
                        status = F_ERR_ACCESSDENIED;
                    }
                    else
                    {
                        status = rfat_path_check_locked(volume, blkno, offset);

                        if (status == F_NO_ERROR)
                        {
                            status = rfat_path_parse_filename((const uint8_t**)&newname, path_dirname, FALSE);

                            if (status == F_NO_ERROR)
                            {
                                status = rfat_path_find_entry(volume, FALSE, clsno, BLKNO_NONE, 0, path_dirname, NULL, NULL, NULL, &entry);

                                if (status == F_NO_ERROR)
                                {
                                    if (entry != NULL)
                                    {
                                        status = F_ERR_DUPLICATED;
                                    }
                                    else
                                    {
                                        status = rfat_path_rename_entry(volume, path_dirname, blkno, offset);
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    status = F_ERR_NOTFOUND;
                }
            }
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_delete(const char *filename)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
            
        if (status == F_NO_ERROR)
        {
            dir = (rfat_dir_t*)(entry->data + offset);
            
            if (dir->dir_attr & F_ATTR_DIR)
            {
                status = F_ERR_NOTFOUND;
            }
            else if (dir->dir_attr & F_ATTR_READONLY)
            {
                status = F_ERR_ACCESSDENIED;
            }
            else
            {
                status = rfat_path_check_locked(volume, blkno, offset);
                
                if (status == F_NO_ERROR)
                {
                    status = rfat_path_destroy_entry(volume, blkno, offset);
                }
            }
        }

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

long f_filelength(const char *filename)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset, length;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
            
	if (status == F_NO_ERROR)
	{
	    dir = (rfat_dir_t*)(entry->data + offset);

	    length = dir->dir_file_size;
	}

	rfat_volume_unlock(volume);
    }

    return (status == F_NO_ERROR) ? length : 0;
}

int f_findfirst(const char *filename, F_FIND *find)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = rfat_path_find_directory(volume, TRUE, (const uint8_t*)filename, NULL, find->find_pattern, &find->find_clsno);
            
        if (status == F_NO_ERROR)
        {
            find->find_blkno = BLKNO_NONE;
            find->find_offset = 0;

            status = rfat_path_find_next(volume, find);
        }
        
	rfat_volume_unlock(volume);
    }
    
    rfat_last_status = status;

    return status;
}

int f_findnext(F_FIND *find)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = rfat_path_find_next(volume, find);

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_settimedate(const char *filename, unsigned short ctime, unsigned short cdate)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
	
	if (status == F_NO_ERROR)
	{
            dir = (rfat_dir_t*)(entry->data + offset);

	    dir->dir_wrt_time = RFAT_HTOFS(ctime);
	    dir->dir_wrt_date = RFAT_HTOFS(cdate);
	    
	    status = rfat_dir_cache_write(volume, entry);
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_gettimedate(const char *filename, unsigned short *ctime, unsigned short *cdate)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
	
	if (status == F_NO_ERROR)
	{
            dir = (rfat_dir_t*)(entry->data + offset);

	    *ctime = RFAT_FTOHS(dir->dir_wrt_time);
	    *cdate = RFAT_FTOHS(dir->dir_wrt_date);
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_setattr(const char *filename, unsigned char attr)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
	
	if (status == F_NO_ERROR)
	{
            dir = (rfat_dir_t*)(entry->data + offset);

	    dir->dir_attr = ((dir->dir_attr & ~(F_ATTR_ARC | F_ATTR_SYSTEM | F_ATTR_HIDDEN | F_ATTR_READONLY)) |
			     (attr & (F_ATTR_ARC | F_ATTR_SYSTEM | F_ATTR_HIDDEN | F_ATTR_READONLY)));
	    
	    status = rfat_dir_cache_write(volume, entry);
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

int f_getattr(const char *filename, unsigned char *attr)
{
    int status = F_NO_ERROR;
    uint32_t blkno, offset;
    rfat_cache_entry_t *entry;
    rfat_dir_t *dir;
    rfat_volume_t *volume;

    volume = &rfat_volume;

    status = rfat_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
	status = rfat_path_find_file(volume, (const uint8_t*)filename, &blkno, &offset, &entry);
	
	if (status == F_NO_ERROR)
	{
            dir = (rfat_dir_t*)(entry->data + offset);

	    *attr = dir->dir_attr;
	}

	rfat_volume_unlock(volume);
    }

    rfat_last_status = status;

    return status;
}

F_FILE * f_open(const char *filename, const char *mode)
{
    int status = F_NO_ERROR;
    uint32_t mode_f;
    rfat_file_t *file;
    rfat_volume_t *volume;

    mode_f = 0;
    
    if (mode[1] == '\0')
    {
        if (mode[0] == 'r')
        {
            mode_f = RFAT_FILE_MODE_READ;
        }
        else if (mode[0] == 'w')
        {
            mode_f = RFAT_FILE_MODE_WRITE | RFAT_FILE_MODE_CREATE;
        }
        else if (mode[0] == 'a')
        {
            mode_f = RFAT_FILE_MODE_WRITE | RFAT_FILE_MODE_APPEND;
        }
    }
    else if ((mode[1] == '+') && (mode[2] == '\0'))
    {
        if (mode[0] == 'r')
        {
            mode_f = RFAT_FILE_MODE_READ | RFAT_FILE_MODE_WRITE;
        }
        else if (mode[0] == 'w')
        {
            mode_f = RFAT_FILE_MODE_READ | RFAT_FILE_MODE_WRITE | RFAT_FILE_MODE_CREATE;
        }
        else if (mode[0] == 'a')
        {
            mode_f = RFAT_FILE_MODE_READ | RFAT_FILE_MODE_WRITE | RFAT_FILE_MODE_APPEND;
        }
    }

    if (mode_f == 0)
    {
        status = F_ERR_NOTUSEABLE;
    }
    else
    {
        volume = &rfat_volume;
        
        status = rfat_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            status = rfat_file_open(volume, (const uint8_t*)filename, mode_f, 0, &file);

            rfat_volume_unlock(volume);
        }
    }

    rfat_last_status = status;
    
    return (status == F_NO_ERROR) ? file : NULL;
}

F_FILE * f_truncate(const char *filename, long length)
{
    int status = F_NO_ERROR;
    rfat_file_t *file;
    rfat_volume_t *volume;

    if (length < 0)
    {
        status = F_ERR_NOTUSEABLE;
    }
    else
    {
        volume = &rfat_volume;

        status = rfat_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            status = rfat_file_open(volume, (const uint8_t*)filename, (RFAT_FILE_MODE_READ | RFAT_FILE_MODE_WRITE | RFAT_FILE_MODE_TRUNCATE), length, &file);

            rfat_volume_unlock(volume);
        }
    }
        
    rfat_last_status = status;

    return (status == F_NO_ERROR) ? file : NULL;
}

int f_close(F_FILE *file)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        volume = &rfat_volume;
    
        status = rfat_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            if (file->mode & RFAT_FILE_MODE_WRITE)
            {
                status = rfat_file_flush(volume, file, TRUE);
            }

            if (status == F_NO_ERROR)
            {
		rfat_file_close(volume, file);
            }

            rfat_volume_unlock(volume);
        }
    }

    rfat_last_status = status;

    return status;
}

long f_write(const void *buffer, long size, long count, F_FILE *file)
{
    int status = F_NO_ERROR;
    long result = 0;
    uint32_t total;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & RFAT_FILE_MODE_WRITE))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            if ((size > 0) && (count > 0))
            {
                volume = &rfat_volume;

                status = rfat_volume_lock(volume);
                
                if (status == F_NO_ERROR)
                {
		    status = rfat_file_write(volume, file, (uint8_t*)buffer, (unsigned long)count * (unsigned long)size, &total);

		    result = total / (unsigned long)size;
                    
                    rfat_volume_unlock(volume);
                }
            }
        }
    }

    rfat_last_status = status;

    return result;
}

long f_read(void *buffer, long size, long count, F_FILE *file)
{
    int status = F_NO_ERROR;
    long result = 0;
    uint32_t total;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & RFAT_FILE_MODE_READ))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            if ((size > 0) && (count > 0))
            {
                volume = &rfat_volume;

                status = rfat_volume_lock(volume);
            
                if (status == F_NO_ERROR)
                {
		    status = rfat_file_read(volume, file, (uint8_t*)buffer, (unsigned long)count * (unsigned long)size, &total);

		    result = total / (unsigned long)size;

                    rfat_volume_unlock(volume);
                }
            }
        }
    }

    rfat_last_status = status;

    return result;
}

int f_seek(F_FILE *file, long offset, int whence)
{
    int status = F_NO_ERROR;
    uint32_t position;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        switch (whence) {
        case F_SEEK_CUR:
	    if ((offset >= 0)
		? ((uint32_t)offset > (RFAT_FILE_SIZE_MAX - file->position))
		: ((uint32_t)(0 - offset) > file->position))
	    {
		status = F_ERR_NOTUSEABLE;
	    }
	    else
	    {
		position = file->position + offset;
	    }
            break;

        case F_SEEK_END:
            if ((offset >= 0)
                ? ((uint32_t)offset > (RFAT_FILE_SIZE_MAX - file->length))
                : ((uint32_t)(0 - offset) > file->length))
            {
                status = F_ERR_NOTUSEABLE;
            }
            else
            {
                position = file->length + offset;
            }
            break;

        case F_SEEK_SET:
            if ((offset < 0) || (offset > RFAT_FILE_SIZE_MAX))
            {
                status = F_ERR_NOTUSEABLE;
            }
            else
            {
                position = offset;
            }
            break;

        default:
            status = F_ERR_NOTUSEABLE;
            break;
        }
            
        if (status == F_NO_ERROR)
        {
            volume = &rfat_volume;

            status = rfat_volume_lock(volume);
    
            if (status == F_NO_ERROR)
            {
                status = rfat_file_seek(volume, file, position);

                rfat_volume_unlock(volume);
            }
        }
    }

    rfat_last_status = status;

    return status;
}

long f_tell(F_FILE *file)
{
    int status = F_NO_ERROR;
    uint32_t position;
    
    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        position = file->position;
    }

    rfat_last_status = status;

    return (status == F_NO_ERROR) ? (long)position : -1;
}

int f_eof(F_FILE *file)
{
    int status = F_NO_ERROR;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (file->position >= file->length)
        {
            status = F_ERR_EOF;
        }
    }

    rfat_last_status = status;

    return status;
}

int f_rewind(F_FILE *file)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        volume = &rfat_volume;

	status = rfat_volume_lock(volume);
    
	if (status == F_NO_ERROR)
	{
            status = rfat_file_seek(volume, file, 0);

            rfat_volume_unlock(volume);
	}
    }

    rfat_last_status = status;

    return status;
}

int f_seteof(F_FILE *file)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        volume = &rfat_volume;

        status = rfat_volume_lock(volume);
        
        if (status == F_NO_ERROR)
        {
            if (file->length != file->position)
            {
                if (file->length < file->position)
                {
                    status = rfat_file_extend(volume, file, file->position, TRUE);
                }
                else
                {
                    status = rfat_file_shrink(volume, file, file->position);
                }
            }

            rfat_volume_unlock(volume);
        }
    }

    rfat_last_status = status;

    return status;
}

int f_putc(int c, F_FILE *file)
{
    int status = F_NO_ERROR;
    int result = -1;
    uint8_t data;
    uint32_t total;
    rfat_cache_entry_t *entry;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & RFAT_FILE_MODE_WRITE))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            volume = &rfat_volume;

            entry = &file->data_cache;

            if ((entry->blkno == file->blkno) && ((file->mode & RFAT_FILE_MODE_APPEND) ? (file->position == file->length) : (file->position <= file->length)))
            {
                *(entry->data + (file->position & BLK_MASK)) = c;

                rfat_data_cache_modify(volume, file, entry);
                
                file->position++;
                
                if (!(file->position & BLK_MASK))
                {
                    file->blkno++;
                }
                
                if (file->length < file->position)
                {
                    file->length = file->position;
                }

                file->flags |= RFAT_FILE_FLAG_DATA_MODIFIED;

		result = c;
            }
            else
            {
                status = rfat_volume_lock(volume);
                
                if (status == F_NO_ERROR)
                {
		    data = c;
                
		    status = rfat_file_write(volume, file, &data, 1, &total);

		    if (total == 1)
		    {
			result = c;
		    }

                    rfat_volume_unlock(volume);
                }
            }
        }
    }

    rfat_last_status = status;

    return result;
}

int f_getc(F_FILE *file)
{
    int status = F_NO_ERROR;
    int result = -1;
    uint8_t data;
    uint32_t total;
    rfat_cache_entry_t *entry;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & RFAT_FILE_MODE_READ))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            volume = &rfat_volume;

            entry = &file->data_cache;

            if ((entry->blkno == file->blkno) && (file->position < file->length))
            {
                result = *(entry->data + (file->position & BLK_MASK));
                
                file->position++;
                
                if (!(file->position & BLK_MASK))
                {
                    file->blkno++;
                }
            }
            else
            {
                status = rfat_volume_lock(volume);
    
                if (status == F_NO_ERROR)
                {
                    status = rfat_file_read(volume, file, &data, 1, &total);
                    
                    if (total == 1)
                    {
                        result = data;
                    }

                    rfat_volume_unlock(volume);
                }
            }
        }
    }

    rfat_last_status = status;

    return result;
}

int f_flush(F_FILE *file)
{
    int status = F_NO_ERROR;
    rfat_volume_t *volume;

    if (!file || !(file->mode & RFAT_FILE_MODE_DISK))
    {
	status = F_ERR_NOTOPEN;
    }
    else
    {
        volume = &rfat_volume;

	status = rfat_volume_lock(volume);
        
	if (status == F_NO_ERROR)
	{
	    status = rfat_file_flush(volume, file, FALSE);
	    
	    rfat_volume_unlock(volume);
	}
    }

    rfat_last_status = status;

    return status;
}

int f_getlasterror(void)
{
    return rfat_last_status;
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

typedef struct _rfat_disk_device_t rfat_disk_device_t;

#define RFAT_DISK_STATE_NONE            0
#define RFAT_DISK_STATE_READY           1
#define RFAT_DISK_STATE_BUSY            2
#define RFAT_DISK_STATE_READ_CONTINUE   3
#define RFAT_DISK_STATE_WRITE_CONTINUE  4
#define RFAT_DISK_STATE_REMOVED         5

#define RFAT_DISK_TYPE_NONE             0
#define RFAT_DISK_TYPE_SDSC             1
#define RFAT_DISK_TYPE_SDHC             2

#define RFAT_DISK_MODE_NONE             0
#define RFAT_DISK_MODE_IDENTIFY         1
#define RFAT_DISK_MODE_DATA_TRANSFER    2

struct _rfat_disk_device_t {
    uint8_t  state;
    uint8_t  type;
    uint32_t address;
};

#define CMD_GO_IDLE_STATE           (0x40+0)
#define CMD_SEND_OP_COND            (0x40+1)
#define CMD_SEND_IF_COND            (0x40+8)
#define CMD_SEND_CSD                (0x40+9)
#define CMD_SEND_CID                (0x40+10)
#define CMD_STOP_TRANSMISSION       (0x40+12)
#define CMD_SEND_STATUS             (0x40+13)
#define CMD_SET_BLOCKLEN            (0x40+16)
#define CMD_READ_SINGLE_BLOCK       (0x40+17)
#define CMD_READ_MULTIPLE_BLOCK     (0x40+18)
#define CMD_WRITE_SINGLE_BLOCK      (0x40+24)
#define CMD_WRITE_MULTIPLE_BLOCK    (0x40+25)
#define CMD_APP_CMD                 (0x40+55)
#define CMD_READ_OCR                (0x40+58)
#define CMD_CRC_ON_OFF              (0x40+59)

#define ACMD_SD_STATUS              (0x40+13)
#define ACMD_SET_WR_BLK_ERASE_COUNT (0x40+23)
#define ACMD_SD_SEND_OP_COND        (0x40+41)
#define ACMD_SET_CLR_CARD_DETECT    (0x40+42)
#define ACMD_SEND_SCR               (0x40+51)

/***********************************************************************************************************************/
/***********************************************************************************************************************/

extern uint32_t SystemCoreClock;

#define FALSE  0
#define TRUE   1

#if !defined(NULL)
#define NULL   ((void*)0)
#endif

/*
 * STX      PA5 (DI)   (Host to card data signal)
 * SRX      PA4 (DO)   (Card to host data signal)
 * SFSS     PA3 (CS)   (Host to card chip select signal)
 * SCLK     PA2 (SCLK) (Host to card clock signal)
 */

#define LM4F120_DISK_FIFO_COUNT      8

#if (LM4F120_DISK_CONFIG_CRC == 1)

static const uint8_t lm4f120_crc7_table[256]= {
    0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
    0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
    0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
    0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
    0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
    0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
    0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
    0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
    0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
    0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
    0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
    0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
    0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
    0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
    0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
    0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
    0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
    0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
    0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
    0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
    0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
    0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
    0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
    0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
    0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
    0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
    0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
    0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
    0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
    0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
    0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
    0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

static const uint16_t lm4f120_crc16_table[256]= {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static RFAT_DISK_INLINE uint8_t lm4f120_step_crc7(uint8_t data, uint8_t crc7)
{
    return lm4f120_crc7_table[(crc7 << 1) ^ data];
}

static uint8_t lm4f120_compute_crc7(const uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint8_t crc7 = 0;

    for (n = 0; n < count; n++)
    {
        crc7 = lm4f120_step_crc7(data[n], crc7);
    }
    
    return crc7;
}

static RFAT_DISK_INLINE uint16_t lm4f120_step_crc16(uint8_t data, uint16_t crc16)
{
    return (crc16 << 8) ^ lm4f120_crc16_table[((crc16 >> 8) ^ data)];
}

static uint16_t lm4f120_compute_crc16(const uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint16_t crc16 = 0;

    for (n = 0; n < count; n++)
    {
        crc16 = lm4f120_step_crc16(data[n], crc16);
    }
    
    return crc16;
}

#endif /* LM4F120_DISK_CONFIG_CRC == 1 */


/***********************************************************************************************************************/
/***********************************************************************************************************************/

/*
 * int      lm4f120_disk_present(void)
 * int      lm4f120_disk_writeprotect(void)
 * void     lm4f120_disk_select(void)
 * void     lm4f120_disk_unselect(void)
 * uint32_t lm4f120_disk_mode(uint32_t mode)
 * void     lm4f120_disk_send(uint8_t)
 * uint8_t  lm4f120_disk_send_data(const uint8_t *data, uint32_t count)
 * uint8_t  lm4f120_disk_send_command(uint8_t index, const uint32_t argument)
 * uint8_t  lm4f120_disk_receive(void)
 * int      lm4f120_disk_receive_data(const uint8_t *data, uint32_t count)
 */

static uint32_t lm4f120_disk_ssi_cr0;
static uint32_t lm4f120_disk_ssi_cpsr;

static int      lm4f120_disk_present(void);
static int      lm4f120_disk_writeprotect(void);
static void     lm4f120_disk_select(void);
static void     lm4f120_disk_unselect(void);
static uint32_t lm4f120_disk_mode(uint32_t mode);
static void     lm4f120_disk_send(uint8_t);
static uint8_t  lm4f120_disk_send_data(const uint8_t *data, uint32_t count);
static uint8_t  lm4f120_disk_send_command(uint8_t index, const uint32_t argument);
static uint8_t  lm4f120_disk_receive(void);
static int      lm4f120_disk_receive_data(uint8_t *data, uint32_t count);


#define RFAT_PORT_DISK_PRESENT()                         lm4f120_disk_present()
#define RFAT_PORT_DISK_WRITEPROTECT()                    lm4f120_disk_writeprotect()
#define RFAT_PORT_DISK_SELECT()                          lm4f120_disk_select()
#define RFAT_PORT_DISK_UNSELECT()                        lm4f120_disk_unselect()
#define RFAT_PORT_DISK_MODE(_mode)                       lm4f120_disk_mode((_mode))
#define RFAT_PORT_DISK_SEND(_data)                       lm4f120_disk_send((_data))
#define RFAT_PORT_DISK_SEND_DATA(_data,_count)           lm4f120_disk_send_data((_data),(_count))
#define RFAT_PORT_DISK_SEND_COMMAND(_command,_argument)  lm4f120_disk_send_command((_command),(_argument))
#define RFAT_PORT_DISK_RECEIVE()                         lm4f120_disk_receive()
#define RFAT_PORT_DISK_RECEIVE_DATA(_data,_count)        lm4f120_disk_receive_data((_data),(_count))

/*
 * lm4f120_disk_send(uint8_t data)
 *
 * Send one byte, discard read data. The assumption
 * is that at this point both TX and RX FIFOs are
 * empty, so that a write is always possible. On
 * the read part there is a wait for the RX FIFO to
 * become not empty.
 */

static void lm4f120_disk_send(uint8_t data)
{
    SSI2_DR_R = data;

    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }

    SSI2_DR_R;
}

/*
 * lm4f120_disk_receive()
 *
 * Receive one byte, send 0xff as data. The assumption
 * is that at this point both TX and RX FIFOs are
 * empty, so that a write is always possible. On
 * the read part there is a wait for the RX FIFO to
 * become not empty.
 */

static uint8_t lm4f120_disk_receive(void)
{
    SSI2_DR_R = 0xff;

    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }

    return SSI2_DR_R;
}

static int lm4f120_disk_present(void)
{
    return 1;
}

static int lm4f120_disk_writeprotect(void)
{
    return 0;
}

static void lm4f120_disk_select(void)
{
    /* Setup/Enable SPI port for shared access.
     */
    SSI2_CR0_R  = lm4f120_disk_ssi_cr0;
    SSI2_CPSR_R = lm4f120_disk_ssi_cpsr;
    SSI2_CR1_R  = SSI_CR1_SSE;

    /* CS output, drive CS to L */
    *(GPIO_PORTA_DATA_BITS_R + 0x10) = 0x00;

    /* 7.5.1.1 states that the card will not
     * drive DO for one more clock after CS goes L,
     * but will accept data right away. The first
     * thing after a select will be always either
     * a command (send_command), or a "Stop Token".
     * In both cases there will be a byte over the
     * bus, and hence DO will be stable.
     */
}

static void lm4f120_disk_unselect(void)
{
    /* CS is output, drive CS to H */

    while (SSI2_SR_R & SSI_SR_BSY) { continue; }
	
    *(GPIO_PORTA_DATA_BITS_R + 0x10) = 0x10;

    /* 7.5.1.1 states that the card drives
     * the DO line at least one more clock
     * cycle after CS goes H. Hence send
     * one extra byte over the bus, if we get
     * here while SSI was enabled.
     */
    
    lm4f120_disk_send(0xff);

    while (SSI2_SR_R & SSI_SR_BSY) { continue; }

    /* Disable SPI port for shared access
     */
    SSI2_CR1_R = 0;
}



/* 
 * lm4f120_disk_mode(uint32_t mode)
 *
 * RFAT_DISK_MODE_NONE           disable SPI, CS is input
 * RFAT_DISK_MODE_IDENTIFY       400kHz mode, send 74 clocks while CS/DI are set to H
 * RFAT_DISK_MODE_DATA_TRANSFER  25MHz mode (or less)
 *
 * Returns the SPI clock ...
 */

static uint32_t lm4f120_disk_mode(uint32_t mode)
{
    unsigned int n;
    uint32_t speed, cpsdvsr, scr;

    /* Enable SSI2 */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    SYSCTL_RCGCSSI_R;

    // Enable GPIOA/GPIOB
    SYSCTL_RCGCGPIO_R |= (SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1);
    SYSCTL_RCGCGPIO_R;

    /* This device is a composite with CS being on PA4, while PB5/PA3 are CS for TFT/EEPROM.
     * So we need to enable and tie them high as well.
     */

    if (mode == RFAT_DISK_MODE_NONE)
    {
	/* Disable SSI2 */
	SSI2_CR1_R = 0;

	GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~0xf0) | (0x20);

	/* Disable SSI2/AFSEL */
	GPIO_PORTB_AFSEL_R = (GPIO_PORTB_AFSEL_R & ~0xf0) | (0xd0);
	
	/* Select the proper SSI2 settings in PCTL */
	GPIO_PORTB_PCTL_R = ((GPIO_PORTB_PCTL_R & ~(GPIO_PCTL_PB4_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M)) |
			     (GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX));
	
	/* Enable 8MA drive to meet timings */
	GPIO_PORTB_DR8R_R = (GPIO_PORTB_DR8R_R & ~0xf0) | (0xf0);

	GPIO_PORTB_PDR_R = (GPIO_PORTB_PDR_R & ~0xf0) | (0x00);
	GPIO_PORTB_PUR_R = (GPIO_PORTB_PUR_R & ~0xf0) | (0x00);

	/* Set DO/DI/CS/SCLK to H */
	*(GPIO_PORTB_DATA_BITS_R + 0xf0) = 0xf0;

	/* Enable/Set DEN (PB7/PB6/PB5/PB4 are enabled) */
	GPIO_PORTB_DEN_R = (GPIO_PORTB_DEN_R & ~0xf0) | (0xf0);


	/* Enable/Set DIR (CS/PA4 is output (PA3 as well (CS2)) */
	GPIO_PORTA_DIR_R = (GPIO_PORTA_DIR_R & ~0x18) | (0x18);

	/* Disable AFSEL on PA4 (plus PA3) */
	GPIO_PORTA_AFSEL_R = (GPIO_PORTA_AFSEL_R & ~0x18) | (0x00);

	/* Enable 8MA drive to meet timings */
	GPIO_PORTA_DR8R_R = (GPIO_PORTA_DR8R_R & ~0x18) | (0x18);

	/* Set CS to H (plus PA4) */
	*(GPIO_PORTA_DATA_BITS_R + 0x18) = 0x18;

	/* Enable/Set DEN (PA4 plus PA3) */
	GPIO_PORTA_DEN_R = (GPIO_PORTA_DEN_R & ~0x18) | (0x18);

        speed = 0;
    }
    else
    {
	if (mode == RFAT_DISK_MODE_IDENTIFY)
	{
	    speed = 400000;
	}
	else
	{
	    speed = 20000000;
	    
	    lm4f120_disk_unselect();
	}
	
	/* 
	 * "speed" cannot be above SystemCoreClock / 2 !
	 *
	 *     speed = (SystemCoreClock / (cpsdvsr * (1 + scr));
	 *
	 * The idea is now to compute the minimum "cpsdvsr" that does 
	 * not overflow "scr" (0..255):
	 *
	 *     cpsdvsr * (1 + scr) = SystemCoreClock / speed;
	 *     cpsdvsr = max(2, ((SystemCoreClock / speed) / (1 + scr_max) +1) & ~1)) = max(2, ((SystemCoreClock / speed) / 256 + 1) & ~1);
	 *
	 * With that a "scr" can be computed:
	 *
	 *     (1 + scr) = (SystemCoreClock / speed) / cpsdvsr;
	 *     scr = (SystemCoreClock / speed) / cpsdvsr -1;
	 *
	 * However this is all pretty pointless. Lets assume we have a 50MHz SystemCoreClock:
	 *
	 *     speed  c s
	 *  25000000  2 0
	 *  12500000  2 1
	 *   8333333  2 2
	 *   6250000  2 3
	 *   5000000  2 4
	 *    400000  2 62
	 */

	cpsdvsr = ((SystemCoreClock / speed) / 256 +1) & ~1;

	if (cpsdvsr == 0)
	{
	    cpsdvsr = 2;
	}

	scr = (SystemCoreClock / speed + (cpsdvsr -1)) / cpsdvsr -1;

        speed = SystemCoreClock / (cpsdvsr * (1 + scr));

        lm4f120_disk_ssi_cr0  = ((scr << 8) | SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8);
        lm4f120_disk_ssi_cpsr = cpsdvsr;

	SSI2_CR1_R  = 0;
	SSI2_CR0_R  = lm4f120_disk_ssi_cr0;
	SSI2_CPSR_R = lm4f120_disk_ssi_cpsr;
	SSI2_IM_R   = 0u;
	SSI2_ICR_R  = ~0u;	
	SSI2_CR1_R  = SSI_CR1_SSE;

	if (mode == RFAT_DISK_MODE_IDENTIFY)
	{
	    /* CS is H high alread from the MODE_NONE setting above.
	     * Just set DI (MOSI) to H (MOSI == PB7).
	     */

	    GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~0x80) | (0x80);

	    /* Disable SSI2/AFSEL */
	    GPIO_PORTB_AFSEL_R = (GPIO_PORTB_AFSEL_R & ~0x80) | (0x00);

	    /* Here CS/DI are driven both to H.
	     *
	     * Specs says to issue 74 clock cycles in SPI mode while CS/DI are H,
	     * so simply send 80 bytes over the clock line.
	     */

	    *(GPIO_PORTB_DATA_BITS_R + 0x80) = 0x80;

	    for (n = 0; n < 10; n++)
	    {
		lm4f120_disk_receive();
	    }

	    while (SSI2_SR_R & SSI_SR_BSY) { continue; }

	    /* Enable SSI2/AFSEL */
	    GPIO_PORTB_AFSEL_R = (GPIO_PORTB_AFSEL_R & ~0x80) | (0x80);

	    GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~0x80) | (0x00);
	}

	lm4f120_disk_select();
    }

    return speed;
}

/*
 * lm4f120_disk_send_data(const uint8_t *data, uint32_t count)
 *
 * Returns a "Data Response Token".
 */

static uint8_t lm4f120_disk_send_data(const uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint8_t response;
    uint16_t crc16;

    GPIO_PORTF_DATA_R |= 0x02;

#if (LM4F120_DISK_CONFIG_CRC == 1)
    crc16 = lm4f120_compute_crc16(data, count);
#else /* LM4F120_DISK_CONFIG_CRC == 1 */
    crc16 = 0xffff;
#endif /* LM4F120_DISK_CONFIG_CRC == 1 */

#if (LM4F120_DISK_CONFIG_OPTIMIZE == 1)

    /*
     * Idea is to stuff first up data into the TX FIFO till it's full
     * (or better said till there ae no more splots in the RX FIFO).
     * Then wait for at least one item in the RX FIFO to read it back,
     * and refill the TX FIFO. At the end, the RX FIFO is drained.
     */

    for (n = 0; n < LM4F120_DISK_FIFO_COUNT; n++)
    {
        SSI2_DR_R = data[n];
    }
    
    for (n = LM4F120_DISK_FIFO_COUNT; n < count; n++)
    {
        while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
        
        SSI2_DR_R;
        SSI2_DR_R = data[n];
    }
    
    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
    
    SSI2_DR_R;
    SSI2_DR_R = crc16 >> 8;
    
    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
    
    SSI2_DR_R;
    SSI2_DR_R = crc16;
    
    for (n = 0; n < LM4F120_DISK_FIFO_COUNT; n++)
    {
        while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
        
        SSI2_DR_R;
    }
    
#else /* LM4F120_DISK_CONFIG_OPTIMIZE */

    for (n = 0; n < count; n++)
    {
        lm4f120_disk_send(data[n]);
    } 
    
    lm4f120_disk_send(crc16 >> 8);
    lm4f120_disk_send(crc16 >> 0);
    
#endif /* LM4F120_DISK_CONFIG_OPTIMIZE */

    /* At last read back the "Data Response Token":
     *
     * 0x05 No Error
     * 0x0b CRC Error
     * 0x0d Write Error
     */
    
    response = lm4f120_disk_receive() & 0x1f;

    GPIO_PORTF_DATA_R &= ~0x02;
    
    return response;
}


/*
 * lm4f120_disk_receive_data(const uint8_t *data, uint32_t count)
 *
 * Returns 1 on success, and 0 on a CRC error.
 */

static int lm4f120_disk_receive_data(uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint16_t crc16;

    GPIO_PORTF_DATA_R |= 0x08;
    
#if (LM4F120_DISK_CONFIG_OPTIMIZE == 1)

    /*
     * Idea is to stuff first up data into the TX FIFO till it's full
     * (or better said till there ae no more splots in the RX FIFO).
     * Then wait for at least one item in the RX FIFO to read it back,
     * and refill the TX FIFO. At the end, the RX FIFO is drained.
     */
    
    for (n = 0; n < LM4F120_DISK_FIFO_COUNT; n++)
    {
        SSI2_DR_R = 0xff;
    }
    
    for (n = 0; n < (count + 2 - LM4F120_DISK_FIFO_COUNT); n++)
    {
        while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
	
        data[n] = SSI2_DR_R;
        SSI2_DR_R = 0xff;
    }
    
    for (; n < count; n++)
    {
        while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
	
        data[n] = SSI2_DR_R;
    }
    
    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
    
    crc16 = (SSI2_DR_R & 0xff) << 8;
    
    while (!(SSI2_SR_R & SSI_SR_RNE)) { continue; }
    
    crc16 |= (SSI2_DR_R & 0xff);
    
#else /* LM4F120_DISK_CONFIG_OPTIMIZE */
    
    for (n = 0; n < count; n++)
    {
        data[n] = lm4f120_disk_receive();
    } 
    
    crc16 = lm4f120_disk_receive() << 8;
    crc16 |= lm4f120_disk_receive();
    
#endif /* LM4F120_DISK_CONFIG_OPTIMIZE */

    GPIO_PORTF_DATA_R &= ~0x08;

#if (LM4F120_DISK_CONFIG_CRC == 1)
    return (crc16 == lm4f120_compute_crc16(data, count));
#else /* LM4F120_DISK_CONFIG_CRC == 1 */
    return 1;
#endif /* LM4F120_DISK_CONFIG_CRC == 1 */
}

/*
 * lm4f120_disk_send_command(uint8_t command, const uint32_t argument)
 *
 * A return value with bit7 set means a timeout on reading a response. Otherwise the 7 LSBs
 * are the error code (mostly R1 format).
 */

static uint8_t lm4f120_disk_send_command(uint8_t command, const uint32_t argument)
{
    unsigned int n;
    uint8_t data[5], response, crc7;

    data[0] = command;
    data[1] = argument >> 24;
    data[2] = argument >> 16;
    data[3] = argument >> 8;
    data[4] = argument >> 0;

#if (LM4F120_DISK_CONFIG_CRC == 1)

    crc7 = (lm4f120_compute_crc7(data, 5) << 1) | 0x01;

#else /* LM4F120_DISK_CONFIG_CRC == 1 */

    if (command == CMD_GO_IDLE_STATE)
    {
        crc7 = 0x95;
    }
    else if (command == CMD_SEND_IF_COND)
    {
        crc7 = 0x87;
    } 
    else
    {
        crc7 = 0xff;
    }

#endif /* LM4F120_DISK_CONFIG_CRC == 1 */

    /* A command needs at least one back to back idle cycle.
     * Unless it's a STOP_TRANSMISSION, which should proceed
     * as early as possible.
     */

    if (command != CMD_STOP_TRANSMISSION)
    {
        lm4f120_disk_receive();
    }

    lm4f120_disk_send(data[0]);
    lm4f120_disk_send(data[1]);
    lm4f120_disk_send(data[2]);
    lm4f120_disk_send(data[3]);
    lm4f120_disk_send(data[4]);
    lm4f120_disk_send(crc7);

    /* NCR is 1..8 bytes, so simply always discard the first byte,
     * and then read up to 8 bytes or till a vaild response
     * was seen. N.b that STOP_TRANSMISSION specifies that
     * the first byte on DataOut after reception of the command
     * is a stuffing byte that has to be ignored. The discard
     * takes care of that here.
     */

    lm4f120_disk_receive();

    for (n = 0; n < 8; n++)
    {
	response = lm4f120_disk_receive();
        
        if (!(response & 0x80))
        {
            /*
             * A STOP_TRANSMISSION can be issued after the card
             * had send a "Data Error Token" for that last block
             * that we are not really intrested in. This would result
             * in an "Illegal Command Error" and/or "Parameter Error".
             * Just ignore those cases.
             */
            if (command == CMD_STOP_TRANSMISSION)
            {
                // response &= ~0x44;
                response = 0x00;
            }
            break;
        }
    }

    return response;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/

/*
 * rfat_disk_wait_ready(rfat_disk_device_t *device)
 *
 * Wait till DO transitions from BUSY (0x00) to READY (0xff).
 */

static int rfat_disk_wait_ready(rfat_disk_device_t *device)
{
    int status = F_NO_ERROR;
    unsigned int n;
    uint8_t response;
    SYSTIM systim1, systim2;

    /* 7.5.1.2 states that while waiting for non busy (not 0x00)
     * the host can release the CS line to let somebody else 
     * access to bus. However there needs to be an extra clock
     * cycle after driving CS to H for the card to release DO,
     * as well as one extra clock cycle after driving CS to L
     * before the data is valid again.
     *
     * Idea would be to scan the bus for <n> cycles, and then sleep
     * for 1ms at a time.
     */

    for (n = 0; n < 64; n++)
    {
        response = RFAT_PORT_DISK_RECEIVE();

        if (response == 0xff)
        {
            break;
        }
    }

    if (response != 0xff)
    {
        get_tim(&systim1);

        do
        {
            response = RFAT_PORT_DISK_RECEIVE();

            if (response != 0xff)
            {
                get_tim(&systim2);

                if ((uint32_t)((uint32_t)systim2.ltime - (uint32_t)systim1.ltime) > 1000)
                {
                    break;
                }

                RFAT_PORT_DISK_UNSELECT();

                rot_rdq(TSK_SELF);

                RFAT_PORT_DISK_SELECT();

                /* 7.5.3.1 implies that there needs to be one extra clock
                 * before busy is asserted after CS goes low ...
                 */
                RFAT_PORT_DISK_RECEIVE();
            }
        }
        while (response != 0xff);
    }

    if (response != 0xff)
    {
        RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);
                    
        device->state = RFAT_DISK_STATE_REMOVED;
        
        status = F_ERR_CARDREMOVED;
    }

    return status;
}

/*
 * rfat_disk_wait_token(rfat_disk_device_t *device, uint8_t *token)
 *
 * Wait for a "Start Block Token", or a "Data Error Token".
 */

static int rfat_disk_wait_token(rfat_disk_device_t *device, uint8_t *token)
{
    int status = F_NO_ERROR;
    unsigned int n;
    uint8_t response;
    SYSTIM systim1, systim2;

    /* Before we waited for a non 0xff token.
     * If a "Start Block Token" (0xfe) zips by then
     * a data block will follow. Otherwise a "Data Error Token"
     * shows up:
     *
     * 0x01 Error
     * 0x02 CC Error
     * 0x04 Card ECC failed
     * 0x08 Out of Range
     *
     * The maximum documented timeout is 100ms for SDHC.
     */

    for (n = 0; n < 64; n++)
    {
        response = RFAT_PORT_DISK_RECEIVE();

        if (response != 0xff)
        {
            break;
        }
    }

    if (response == 0xff)
    {
        get_tim(&systim1);

        do
        {
            response = RFAT_PORT_DISK_RECEIVE();

            if (response == 0xff)
            {
                get_tim(&systim2);

                if ((uint32_t)((uint32_t)systim2.ltime - (uint32_t)systim1.ltime) > 100)
                {
                    break;
                }
            }
        }
        while (response == 0xff);
    }

    if (response == 0xff)
    {
        RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);
	    
        device->state = RFAT_DISK_STATE_REMOVED;
        
        status = F_ERR_CARDREMOVED;
    }

    *token = response;

    return status;
}

static int rfat_disk_send_command(rfat_disk_device_t *device, uint8_t command, uint32_t argument)
{
    int status = F_NO_ERROR;
    uint8_t response;

    response = RFAT_PORT_DISK_SEND_COMMAND(command, argument);

    if (response != 0x00)
    {
        if (response & 0x80)
        {
            RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);
	    
            device->state = RFAT_DISK_STATE_REMOVED;
            
            status = F_ERR_CARDREMOVED;
        }
        else
        {
            status = F_ERR_ONDRIVE;
        }
    }

    return status;
}

static int rfat_disk_attach(rfat_disk_device_t *device)
{
    int status = F_NO_ERROR;
    unsigned int type, n;
    uint8_t response;
    uint8_t data[16];
    SYSTIM systim1, systim2;

    if (!RFAT_PORT_DISK_PRESENT())
    {
        status = F_ERR_INVALIDMEDIA;
    }
    else
    {
        type = RFAT_DISK_TYPE_NONE;

        RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_IDENTIFY);

        /* Apply an initial trial for CMD_GO_IDLE_STATE, so that
         * the card is out of data read/write mode, and can
         * properly respond.
         */
	for (n = 0; n < 2048; n++)
	{
	    response = RFAT_PORT_DISK_RECEIVE();

            if (response == 0xff)
            {
                n += 8;

                response = RFAT_PORT_DISK_SEND_COMMAND(CMD_GO_IDLE_STATE, 0);

                if (response == 0x01)
                {
                    break;
                }
            }
        }

        if (response == 0x01)
        {
            response = RFAT_PORT_DISK_SEND_COMMAND(CMD_SEND_IF_COND, 0x000001aa);

	    if (response == 0x01)
            {
                data[0] = RFAT_PORT_DISK_RECEIVE();
                data[1] = RFAT_PORT_DISK_RECEIVE();
                data[2] = RFAT_PORT_DISK_RECEIVE();
                data[3] = RFAT_PORT_DISK_RECEIVE();

		if ((data[0] == 0x00) && (data[1] == 0x00) && (data[2] == 0x01) && (data[3] == 0xaa))	      
		{
		    type = RFAT_DISK_TYPE_SDHC;
		}
		else
		{
		    type = RFAT_DISK_TYPE_NONE;
		}
            }
            else
            {
                type = RFAT_DISK_TYPE_SDSC;
            }

	    if (type != RFAT_DISK_TYPE_NONE)
	    {
		/* Send ACMD_SD_SEND_OP_COND, and wait till the initialization process is done.
		 */

                get_tim(&systim1);
                
		do
		{
		    response = RFAT_PORT_DISK_SEND_COMMAND(CMD_APP_CMD, 0);

		    if (response == 0x01)
		    {
			response = RFAT_PORT_DISK_SEND_COMMAND(ACMD_SD_SEND_OP_COND, ((type == RFAT_DISK_TYPE_SDHC) ? 0x40000000 : 0x00000000));

                        if (response == 0x01)
                        {
                            get_tim(&systim2);

                            if ((uint32_t)((uint32_t)systim2.ltime - (uint32_t)systim1.ltime) > 1000)
                            {
                                break;
                            }
                        }
		    }
		}
		while (response == 0x01);

		if (response != 0x00)
		{
		    /* The card did not respond, so it's not an SD card.
		     */
		    type = RFAT_DISK_TYPE_NONE;
		}
		else
		{
		    if (type == RFAT_DISK_TYPE_SDHC)
		    {
			/* How it's time to find out whether we really have a SDHC, or a SDSC supporting V2.
			 */
			response = RFAT_PORT_DISK_SEND_COMMAND(CMD_READ_OCR, 0x00000000);
		    
			if (response == 0x00)
			{
                            data[0] = RFAT_PORT_DISK_RECEIVE();
                            data[1] = RFAT_PORT_DISK_RECEIVE();
                            data[2] = RFAT_PORT_DISK_RECEIVE();
                            data[3] = RFAT_PORT_DISK_RECEIVE();

			    if (!(data[0] & 0x40))
			    {
				type = RFAT_DISK_TYPE_SDSC;
			    }
			}
			else
			{
			    type = RFAT_DISK_TYPE_NONE;
			}
		    }

#if (LM4F120_DISK_CONFIG_CRC == 1)
		    if (type != RFAT_DISK_TYPE_NONE)
		    {
			response = RFAT_PORT_DISK_SEND_COMMAND(CMD_CRC_ON_OFF, 1);

			if (response != 0x00)
			{
			    type = RFAT_DISK_TYPE_NONE;
			}
		    }
#endif /* LM4F120_DISK_CONFIG_CRC == 1 */

		    if (type != RFAT_DISK_TYPE_NONE)
		    {
			response = RFAT_PORT_DISK_SEND_COMMAND(CMD_SET_BLOCKLEN, 512);
                        
			if (response != 0x00)
			{
			    type = RFAT_DISK_TYPE_NONE;
			}
		    }
		}
	    }
	}

        if (type != RFAT_DISK_TYPE_NONE)
        {
	    device->state = RFAT_DISK_STATE_READY;
            device->type = type;

	    RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_DATA_TRANSFER);
        }
        else
        {
	    RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);

            status = F_ERR_INVALIDMEDIA;
        }
    }
    
    return status;
}

static int rfat_disk_detach(rfat_disk_device_t *device)
{
    int status = F_NO_ERROR;

    RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);

    return status;
}

/*
 * rfat_disk_select(rfat_disk_device_t *device, int state, uint32_t address)
 *
 * Select the card. Handle all device->state processing,
 * including initialization. Afterward (errors permissing)
 * the card is in idle state.
 */

static int rfat_disk_select(rfat_disk_device_t *device, int state, uint32_t address)
{
    int status = F_NO_ERROR;

    if (device->state == RFAT_DISK_STATE_REMOVED)
    {
        status = F_ERR_CARDREMOVED;
    }
    else if (device->state == RFAT_DISK_STATE_NONE)
    {
        status = rfat_disk_attach(device);
    }
    else
    {
        RFAT_PORT_DISK_SELECT();

        if ((state == device->state) && (device->address == address))
        {
            /* Continuation of a previous read_multiple/write_multiple ... */
        }
        else
        {
            if (device->state == RFAT_DISK_STATE_READ_CONTINUE)
            {
                status = rfat_disk_send_command(device, CMD_STOP_TRANSMISSION, 0);

                if (status == F_NO_ERROR)
                {
                    device->state = RFAT_DISK_STATE_READY;
                }
            }
            
            if (device->state == RFAT_DISK_STATE_WRITE_CONTINUE)
            {
                status = rfat_disk_wait_ready(device);

                if (status == F_NO_ERROR)
                {
                    /* 7.5.3.2 implies that there need to be 8 clocks
                     * before the "Stop Transfer Token", and 8 clocks
                     * after it, before the card signals busy properly.
                     */
                    RFAT_PORT_DISK_RECEIVE();
                    RFAT_PORT_DISK_SEND(0xfd);
                    RFAT_PORT_DISK_RECEIVE();

                    device->state = RFAT_DISK_STATE_BUSY;
                }
            }

            if (device->state == RFAT_DISK_STATE_BUSY)
            {
                status = rfat_disk_wait_ready(device);

                if (status == F_NO_ERROR)
                {
                    device->state = RFAT_DISK_STATE_READY;
                }
            }
        }
    }

    return status;
}

/*
 * rfat_disk_unselect(rfat_disk_device_t *device)
 *
 * Deselect the card.
 */

static void rfat_disk_unselect(rfat_disk_device_t *device)
{
    if (device->state != RFAT_DISK_STATE_REMOVED)
    {
        RFAT_PORT_DISK_UNSELECT();
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/

static rfat_disk_device_t _rfat_disk_device;

int rfat_disk_init(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;

    volume->private = &_rfat_disk_device;

    _rfat_disk_device.state = RFAT_DISK_STATE_NONE;
    _rfat_disk_device.type = RFAT_DISK_TYPE_NONE;
    _rfat_disk_device.address = 0;
    
    RFAT_PORT_DISK_MODE(RFAT_DISK_MODE_NONE);

    return status;
}

uint32_t rfat_disk_status(rfat_volume_t *volume)
{
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint32_t status;

    status = 0;

    if (RFAT_PORT_DISK_WRITEPROTECT())
    {
        status |= RFAT_DISK_STATUS_WRITEPROTECT;
    }

    if (device->state != RFAT_DISK_STATE_REMOVED)
    {
        if (device->state == RFAT_DISK_STATE_NONE)
        {
            if (RFAT_PORT_DISK_PRESENT())
            {
                status |= RFAT_DISK_STATUS_PRESENT;
            }
        }
        else
        {
            status |= RFAT_DISK_STATUS_PRESENT;
        }
    }

    return status;
}

int rfat_disk_capacity(rfat_volume_t *volume, uint32_t *p_block_count, uint32_t *p_block_size)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t token;
    uint8_t data[16];
    uint32_t c_size, c_size_mult, read_bl_len;

    status = rfat_disk_select(device, RFAT_DISK_STATE_READY, 0);

    if (status == F_NO_ERROR)
    {
	/* Read the CSD to get to the size.
	 */
	status = rfat_disk_send_command(device, CMD_SEND_CSD, 0);

        if (status == F_NO_ERROR)
	{
            status = rfat_disk_wait_token(device, &token);

            if (status == F_NO_ERROR)
            {
                if ((token != 0xfe) || !RFAT_PORT_DISK_RECEIVE_DATA(data, 16))
                {
                    status = F_ERR_ONDRIVE;
                }
                else
                {
                    if ((data[0] & 0xc0) == 0)
                    {
                        /* SDSC */
                        
                        read_bl_len = (uint32_t)(data[5] & 0x0f);
                        c_size = ((uint32_t)(data[6] & 0x03) << 10) | ((uint32_t)data[7] << 2) | ((uint32_t)data[8] >> 6);
                        c_size_mult = ((uint32_t)(data[9] & 0x03) << 1) | ((uint32_t)(data[10] & 0x80) >> 7);
                        
                        *p_block_count = ((c_size + 1) << (c_size_mult + 2)) << (read_bl_len - 9);
                    }
                    else
                    {
                        /* SDHC */
                        
                        c_size = ((uint32_t)(data[7] & 0x3f) << 16) | ((uint32_t)data[8] << 8) | ((uint32_t)data[9]);
                        
                        *p_block_count = (c_size + 1) << (19 - 9);
                    }
                    
                    *p_block_size  = 512;
                }
	    }
	}

        rfat_disk_unselect(device);
    }

    return status;
}

int rfat_disk_serial(rfat_volume_t *volume, uint32_t *p_serial)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t token;
    uint8_t data[16];

    status = rfat_disk_select(device, RFAT_DISK_STATE_READY, 0);

    if (status == F_NO_ERROR)
    {
	/* Read the CID to get to the serial.
	 */
	status = rfat_disk_send_command(device, CMD_SEND_CID, 0);

        if (status == F_NO_ERROR)
	{
            status = rfat_disk_wait_token(device, &token);

            if (status == F_NO_ERROR)
            {
                if ((token != 0xfe) || !RFAT_PORT_DISK_RECEIVE_DATA(data, 16))
                {
                    status = F_ERR_ONDRIVE;
                }
                else
                {
                    *p_serial = (((uint32_t)data[9]  << 24) |
                                 ((uint32_t)data[10] << 16) |
                                 ((uint32_t)data[11] <<  8) |
                                 ((uint32_t)data[12] <<  0));
                }
	    }
	}

        rfat_disk_unselect(device);
    }

    return status;
}

int rfat_disk_read(rfat_volume_t *volume, uint32_t address, void *data)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t token;

    status = rfat_disk_select(device, RFAT_DISK_STATE_READY, 0);

    if (status == F_NO_ERROR)
    {
        status = rfat_disk_send_command(device, CMD_READ_SINGLE_BLOCK, ((device->type == RFAT_DISK_TYPE_SDHC) ? address : (address << 9)));

	if (status == F_NO_ERROR)
	{
            status = rfat_disk_wait_token(device, &token);

            if (status == F_NO_ERROR)
            {
                if ((token != 0xfe) || !RFAT_PORT_DISK_RECEIVE_DATA(data, 512))
                {
                    if (token == 0x04)
                    {
                        status = F_ERR_INVALIDSECTOR;
                    }
                    else
                    {
                        status = F_ERR_READ;
                    }
                }
            }
        }

        rfat_disk_unselect(device);
    }

    return status;
}

int rfat_disk_read_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, void *data)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t token;

    status = rfat_disk_select(device, RFAT_DISK_STATE_READ_CONTINUE, address);

    if (status == F_NO_ERROR)
    {
        device->address = address + length;

        if (device->state != RFAT_DISK_STATE_READ_CONTINUE)
        {
            status = rfat_disk_send_command(device, CMD_READ_MULTIPLE_BLOCK, ((device->type == RFAT_DISK_TYPE_SDHC) ? address : (address << 9)));
        }

        if (status == F_NO_ERROR)
        {
            do
            {
                status = rfat_disk_wait_token(device, &token);
            
                if (status == F_NO_ERROR)
                {
                    if ((token != 0xfe) || !RFAT_PORT_DISK_RECEIVE_DATA(data, 512))
                    {
                        status = rfat_disk_send_command(device, CMD_STOP_TRANSMISSION, 0);
            
                        if (status != F_ERR_CARDREMOVED)
                        {
                            device->state = RFAT_DISK_STATE_READY;
                        
                            if (token == 0x04)
                            {
                                status = F_ERR_INVALIDSECTOR;
                            }
                            else
                            {
                                status = F_ERR_READ;
                            }
                        }
                    }
                }

                data += 512;
                length--;
            }
            while ((status == F_NO_ERROR) && (length != 0));

            if (status == F_NO_ERROR)
            {
                device->state = RFAT_DISK_STATE_READ_CONTINUE;
            }
        }

        rfat_disk_unselect(device);
    }

    return status;
}

int rfat_disk_write(rfat_volume_t *volume, uint32_t address, const void *data)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t response;

    status = rfat_disk_select(device, RFAT_DISK_STATE_READY, 0);

    if (status == F_NO_ERROR)
    {
        status = rfat_disk_send_command(device, CMD_WRITE_SINGLE_BLOCK, ((device->type == RFAT_DISK_TYPE_SDHC) ? address : (address << 9)));

        if (status == F_NO_ERROR)
	{
            /* After the CMD_WRITE_SINGLE_BLOCK there needs to be 8 clocks before sending
             * the Start Block Token.
             */
            RFAT_PORT_DISK_RECEIVE();
            RFAT_PORT_DISK_SEND(0xfe);

            response = RFAT_PORT_DISK_SEND_DATA(data, 512);

            if (response == 0x05)
            {
                device->state = RFAT_DISK_STATE_BUSY;
            }
            else
            {
                status = F_ERR_WRITE;
            }
        }

        rfat_disk_unselect(device);
    }

    return status;
}

int rfat_disk_write_multiple(rfat_volume_t *volume, uint32_t address, uint32_t length, const void *data)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;
    uint8_t response;

    status = rfat_disk_select(device, RFAT_DISK_STATE_WRITE_CONTINUE, address);

    if (status == F_NO_ERROR)
    {
        device->address = address + length;

        if (device->state != RFAT_DISK_STATE_WRITE_CONTINUE)
        {
            status = rfat_disk_send_command(device, CMD_WRITE_MULTIPLE_BLOCK, ((device->type == RFAT_DISK_TYPE_SDHC) ? address : (address << 9)));
        }

        if (status == F_NO_ERROR)
        {
            do
            {
                status = rfat_disk_wait_ready(device);

                if (status == F_NO_ERROR)
                {
                    RFAT_PORT_DISK_SEND(0xfc);

                    response = RFAT_PORT_DISK_SEND_DATA(data, 512);

                    if (response != 0x05)
                    {
                        /* 7.3.3.1 states that a non-accepted "Data Response Token" in
                         * a write multiple block operation shall be followed by a 
                         * CMD_STOP_TRANSMISSION (not a "Stop Token").
                         */

                        status = rfat_disk_send_command(device, CMD_STOP_TRANSMISSION, 0);
            
                        if (status != F_ERR_CARDREMOVED)
                        {
                            device->state = RFAT_DISK_STATE_READY;

                            status = F_ERR_WRITE;
                        }
                    }
                }

                data += 512;
                length--;
            }
            while ((status == F_NO_ERROR) && (length != 0));

            if (status == F_NO_ERROR)
            {
                device->state = RFAT_DISK_STATE_WRITE_CONTINUE;
            }
        }

        rfat_disk_unselect(device);
    }

    return status;
}

/* 
 * rfat_disk_release(rfat_volume_t *volume)
 *
 * Essentially recover from a sticky REMOVED state into NONE (allowing
 * for reinitialization). If there was no REMOVED pending, simply
 * detech the device.
 */

int rfat_disk_release(rfat_volume_t *volume)
{
    int status = F_NO_ERROR;
    rfat_disk_device_t *device = (rfat_disk_device_t*)volume->private;

    if (device->state == RFAT_DISK_STATE_REMOVED)
    {
	device->state = RFAT_DISK_STATE_NONE;
    }
    else
    {
        status = rfat_disk_select(device, RFAT_DISK_STATE_READY, 0);
        
        if (status == F_NO_ERROR)
        {
            status = rfat_disk_detach(device);
        }
    }

    return status;
}


/***********************************************************************************************************************/
/***********************************************************************************************************************/

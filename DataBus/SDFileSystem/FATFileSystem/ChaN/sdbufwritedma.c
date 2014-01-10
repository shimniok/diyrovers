/*
 * sdbufwritedma.c
 *
 *  Created on: Jan 9, 2014
 *      Author: mes
 */

/** http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=66583&start=20
 *
 * One important point to remember that the buffer in the memory which will be written to
 *  the SD card needs to be aligned with /4 address which meaning it can be divided by 4
 *  so that it starts at a new 32-bit memory address of MCU. Otherwise, it won't be written
 *  in page mode. I created a function in file.c based on file_write_buf() standard function
 *   which is alway in the same file. This function will do multi-sector DMA transfer. Use
 *   it to write to SD card. I can get nearly 5MByte/second with SCandisk Ultra III.
 */

//! This function transfer a buffer to a file at the current file position
//!
//! @param buffer data buffer
//! @param u16_buf_size data size
//!
//! @return number of byte write
//! @return 0, in case of error
//!
uint16_t file_write_buf_multi_dma( uint8_t *buffer , uint16_t u16_buf_size )
{
	 uint16_t u16_nb_write_tmp;
	 uint16_t u16_nb_write;
	 uint16_t u16_pos_in_sector;

	if( !fat_check_mount_select_open())
	return FALSE;

	if(!(FOPEN_WRITE_ACCESS & fs_g_nav_entry.u8_open_mode))
	{
		fs_g_status = FS_ERR_READ_ONLY;
		return FALSE;
	}

	u16_nb_write = 0;

	while( 0 != u16_buf_size )
	{
		// The file data sector can been directly transfer from buffer to memory (don't use internal cache)
		u16_pos_in_sector = fs_g_nav_entry.u32_pos_in_file % FS_512B;
		if( (0== u16_pos_in_sector)
				&& (FS_512B <= u16_buf_size)
#if (defined __GNUC__) && (defined __AVR32__) || (defined __ICCAVR32__)
				&& (Test_align((U32)buffer, sizeof(U32)))
#endif
		)
		{
			u16_nb_write_tmp = u16_buf_size / FS_512B; // read a modulo sector size

			// Get and eventually alloc the following sector segment of file
			if( !fat_write_file( FS_CLUST_ACT_SEG , u16_nb_write_tmp ))
			return FALSE;
			// Truncate the segment found if more larger than asked size
			if( u16_nb_write_tmp < fs_g_seg.u32_size_or_pos)
			{
				fs_g_seg.u32_size_or_pos = u16_nb_write_tmp;
			} else {
				u16_nb_write_tmp = fs_g_seg.u32_size_or_pos;
			}

			// Directly data tranfert from buffer to memory
			if( CTRL_GOOD != sd_mmc_mci_dma_multiple_ram_2_mem(fs_g_nav.u8_lun , fs_g_seg.u32_addr, buffer, fs_g_seg.u32_size_or_pos))
			{
				fs_g_status = FS_ERR_HW;
				return u16_nb_write;
			}
			fs_g_seg.u32_addr += fs_g_seg.u32_size_or_pos;
			buffer += FS_512B * fs_g_seg.u32_size_or_pos;
			fs_g_seg.u32_size_or_pos = 0;

			// Translate from sector unit to byte unit
			u16_nb_write_tmp *= FS_512B;
		}
		else
		{
			// The file data can't been directly transfer from buffer to memory, the internal cache must be used

			// Tranfer and eventually alloc a data sector from internal cache to memory
			if((fs_g_nav_entry.u32_pos_in_file == fs_g_nav_entry.u32_size)
					&& (0==u16_pos_in_sector) )
			{
				// Eventually alloc one new sector for the file
				if( !fat_write_file( FS_CLUST_ACT_SEG , 1 ))
				return FALSE;
				// Update the cache
				fs_gu32_addrsector = fs_g_seg.u32_addr;
				if( !fat_cache_read_sector( FALSE ))// The memory is not readed because it is a new sector
				return FALSE;
			} else {
				// The sector must existed then alloc no necessary
				if( !fat_write_file( FS_CLUST_ACT_ONE , 1 ))
				return FALSE;
			}

			// Flag internal cache modified
			fat_cache_mark_sector_as_dirty();

			// Compute the number of data to transfer
			u16_nb_write_tmp = FS_512B - u16_pos_in_sector;// The number is limited at sector size
			if( u16_nb_write_tmp > u16_buf_size )
			u16_nb_write_tmp = u16_buf_size;

			// Tranfer data from buffer to internal cache
			memcpy_ram2ram( &fs_g_sector[ u16_pos_in_sector ], buffer , u16_nb_write_tmp );
			buffer += u16_nb_write_tmp;
		}
		// Update positions
		fs_g_nav_entry.u32_pos_in_file+= u16_nb_write_tmp;
		u16_nb_write += u16_nb_write_tmp;
		u16_buf_size -= u16_nb_write_tmp;
		// Update file size
		if( fs_g_nav_entry.u32_pos_in_file > fs_g_nav_entry.u32_size )
		{
			fs_g_nav_entry.u32_size = fs_g_nav_entry.u32_pos_in_file;
		}
	}
	return u16_nb_write; // All buffer is writed
}

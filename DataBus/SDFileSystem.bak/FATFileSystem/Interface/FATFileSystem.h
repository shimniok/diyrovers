/* mbed Microcontroller Library - FATFileSystem
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010

#ifndef MBED_FATFILESYSTEM_H
#define MBED_FATFILESYSTEM_H

#include "stdint.h"
#include "ff.h"
#include "mbed.h"
#include "FileSystemLike.h"
#include "FATFileHandle.h"
#include "FATDirHandle.h"
#include <stdio.h>

class FATFileSystem : public FileSystemLike
{
    private:
        FATFS FileSystemObject;
        unsigned char Drive;

    public:
        static FATFileSystem* DriveArray[_DRIVES];

        FATFileSystem(const char* SystemName);
        virtual ~FATFileSystem();
        
        int format(unsigned int allocationunit);

        virtual FileHandle* open(const char* filename, int flags);
        virtual int remove(const char* filename);
        virtual int rename(const char* oldname, const char* newname);
        virtual DirHandle* opendir(const char* name);
        virtual int mkdir(const char* name, mode_t mode);

        virtual int disk_initialize() { return 0x00; }
        virtual int disk_status() { return 0x00; }
        virtual int disk_read(unsigned char* buff,
            unsigned long sector, unsigned char count) = 0;
        virtual int disk_write(const unsigned char* buff,
            unsigned long sector, unsigned char count) = 0;
        virtual int disk_sync() { return 0x00; }
        virtual unsigned long disk_sector_count() = 0;
        virtual unsigned short disk_sector_size() { return 512; }
        virtual unsigned long disk_block_size() { return 1; }
};

#endif
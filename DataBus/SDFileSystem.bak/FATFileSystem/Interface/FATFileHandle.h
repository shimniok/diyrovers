/* mbed Microcontroller Library - FATFileHandle
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010

#ifndef MBED_FATFILEHANDLE_H
#define MBED_FATFILEHANDLE_H

#include "stdint.h"
#include "ff.h"
#include "mbed.h"
#include "FileHandle.h"
#include <stdio.h>

class FATFileHandle : public FileHandle
{
    private:
        FAT_FIL FileObject;

    public:
        FATFileHandle(FAT_FIL InputFilStr);
        virtual ssize_t write(const void* buffer, size_t length);
        virtual int close();
        virtual ssize_t read(void* buffer, size_t length);
        virtual int isatty();
        virtual off_t lseek(off_t offset, int whence);
        virtual int fsync();
        virtual off_t flen();
};

#endif
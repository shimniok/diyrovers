/* mbed Microcontroller Library - FATDirHandle
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010

#ifndef MBED_FATDIRHANDLE_H
#define MBED_FATDIRHANDLE_H

#include "stdint.h"
#include "ff.h"
#include "mbed.h"
#include "DirHandle.h"
#include <stdio.h>

class FATDirHandle : public DirHandle
{
    private:
        FAT_DIR DirectoryObject;
        struct dirent CurrentEntry;

    public:
        FATDirHandle(FAT_DIR InputDirStr);
        virtual int closedir();
        virtual struct dirent* readdir();
        virtual void rewinddir();
        virtual off_t telldir();
        virtual void seekdir(off_t location);
};

#endif
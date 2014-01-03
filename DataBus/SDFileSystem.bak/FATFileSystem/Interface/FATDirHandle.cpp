/* mbed Microcontroller Library - FATDirHandle
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010
 
#include "FATDirHandle.h"

FATDirHandle::FATDirHandle(FAT_DIR InputDirStr)
{
    DirectoryObject = InputDirStr;
}

int FATDirHandle::closedir()
{
    delete this;
    return 0;
}

struct dirent* FATDirHandle::readdir()
{
    FILINFO FileInfo;
    FRESULT Result = f_readdir(&DirectoryObject, &FileInfo);
    if (Result || !FileInfo.fname[0])
    {
        return NULL;
    }
    else
    {
        for (unsigned char i = 0; i < 13; i++)
        {
            CurrentEntry.d_name[i] = ((char*)FileInfo.fname)[i];
        }
        return &CurrentEntry;
    }
}

void FATDirHandle::rewinddir()
{
    DirectoryObject.index = 0;
}

off_t FATDirHandle::telldir()
{
    return (off_t)DirectoryObject.index;
}

void FATDirHandle::seekdir(off_t location)
{
    DirectoryObject.index = (WORD)location;
}
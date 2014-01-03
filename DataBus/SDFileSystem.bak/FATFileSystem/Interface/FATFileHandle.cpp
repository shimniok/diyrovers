/* mbed Microcontroller Library - FATFileHandle
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010

#include "FATFileHandle.h"

FATFileHandle::FATFileHandle(FAT_FIL InputFilStr)
{
    FileObject = InputFilStr;
}

ssize_t FATFileHandle::write(const void* buffer, size_t length)
{
    UINT ByteWritten;
    if (f_write(&FileObject, buffer, (UINT)length, &ByteWritten))
    { 
        return -1;
    }
    else
    {
        return (ssize_t)ByteWritten;
    }
}

int FATFileHandle::close()
{
    if (f_close(&FileObject))
    {
        return -1;
    }
    else
    {
        delete this;
        return 0;
    }
}

ssize_t FATFileHandle::read(void* buffer, size_t length)
{
    UINT ByteRead;
    if (f_read(&FileObject, buffer, (UINT)length, &ByteRead))
    {
        return -1;
    }
    else
    {
        return (ssize_t)ByteRead;
    }
}

int FATFileHandle::isatty()
{
    return 0;
}

off_t FATFileHandle::lseek(off_t offset, int whence)
{
    switch (whence)
    {
        case SEEK_CUR: offset += FileObject.fptr; break;
        case SEEK_END: offset += FileObject.fsize; break;
    }
    if (f_lseek(&FileObject, (DWORD)offset))
    {
        return -1;
    }
    else
    {
        return (off_t)FileObject.fptr;
    }
}

int FATFileHandle::fsync()
{
    if (f_sync(&FileObject))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

off_t FATFileHandle::flen()
{
    return (off_t)FileObject.fsize;
}
/* mbed Microcontroller Library - FATFileSystem
   Copyright (c) 2008, sford */

//Modified by Thomas Hamilton, Copyright 2010

#include "FATFileSystem.h"

DWORD get_fattime(void)
{
    return 35719201;
}

FATFileSystem* FATFileSystem::DriveArray[_DRIVES] = {0};

FATFileSystem::FATFileSystem(const char* SystemName) : FileSystemLike(SystemName)
{
    for (unsigned char i = 0; i < _DRIVES; i++)
    {
        if(!DriveArray[i])
        {
            DriveArray[i] = this;
            Drive = i;
            f_mount((BYTE)i, &FileSystemObject);
            return;
        }
    }
}

FATFileSystem::~FATFileSystem()
{
    for (unsigned char i = 0; i < _DRIVES; i++)
    {
        if (DriveArray[i] == this)
        {
            DriveArray[i] = NULL;
            f_mount((BYTE)i, NULL);
        }
    }
    delete this;
}

FileHandle* FATFileSystem::open(const char* filename, int flags)
{
    FAT_FIL FileObject;
    char FileName[64];
    BYTE ModeFlags = 0;

    sprintf(FileName, "%d:/%s", Drive, filename);
    switch (flags & 3)
    {
        case O_RDONLY: ModeFlags = FA_READ; break;
        case O_WRONLY: ModeFlags = FA_WRITE; break;
        case O_RDWR: ModeFlags = FA_READ | FA_WRITE; break;
    }
    if(flags & O_CREAT)
    {
        if(flags & O_TRUNC)
        {
            ModeFlags |= FA_CREATE_ALWAYS;
        }
        else
        {
            ModeFlags |= FA_OPEN_ALWAYS;
        }
    }
    else
    {
        ModeFlags |= FA_OPEN_EXISTING;
    }
    if (f_open(&FileObject, (const TCHAR*)FileName, ModeFlags))
    { 
        return NULL;
    }
    else
    {
        if (flags & O_APPEND)
        {
            f_lseek(&FileObject, (DWORD)FileObject.fsize);
        }
        return new FATFileHandle(FileObject);
    }
}

int FATFileSystem::remove(const char* filename)
{
    char FileName[64];

    sprintf(FileName, "%d:/%s", Drive, filename);
    if (f_unlink((const TCHAR*)FileName))
    { 
        return -1;
    }
    else
    {
        return 0;
    }
}

int FATFileSystem::rename(const char* oldname, const char* newname)
{
    char OldName[64];

    sprintf(OldName, "%d:/%s", Drive, oldname);
    if (f_rename((const TCHAR*)OldName, (const TCHAR*)newname))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

DirHandle* FATFileSystem::opendir(const char* name)
{
    FAT_DIR DirectoryObject;
    char DirectoryName[64];

    sprintf(DirectoryName, "%d:%s", Drive, name);
    if (f_opendir(&DirectoryObject, (const TCHAR*)DirectoryName))
    {
        return NULL;
    }
    else
    {
        return new FATDirHandle(DirectoryObject);
    }
}

int FATFileSystem::mkdir(const char* name, mode_t mode)
{
    char DirectoryName[64];

    sprintf(DirectoryName, "%d:%s", Drive, name);
    if (f_mkdir((const TCHAR*)DirectoryName))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int FATFileSystem::format(unsigned int allocationunit)
{
    if (f_mkfs(Drive, 0, allocationunit))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
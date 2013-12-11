/* mbed SDFileSystem Library, for providing file access to SD cards
 * Copyright (c) 2008-2010, sford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_SDHCFILESYSTEM_H
#define MBED_SDHCFILESYSTEM_H

/** Implements FAT32 filesystem on SDHC cards, Copyright (c) 2008-2010, sford */

#include "mbed.h"
#include "FATFileSystem.h"

/* Double Words */
typedef unsigned long long uint64_t;
typedef long long sint64_t;

/** Access the filesystem on an SD Card using SPI
 *
 * @code
 * #include "mbed.h"
 * #include "SDFileSystem.h"
 *
 * SDFileSystem sd(p5, p6, p7, p12, "sd"); // mosi, miso, sclk, cs
 *  
 * int main() {
 *     FILE *fp = fopen("/sd/myfile.txt", "w");
 *     fprintf(fp, "Hello World!\n");
 *     fclose(fp);
 * }
 */
class SDFileSystem : public FATFileSystem {
public:

    /** Create the File System for accessing an SD Card using SPI
     *
     * @param mosi SPI mosi pin connected to SD Card
     * @param miso SPI miso pin conencted to SD Card
     * @param sclk SPI sclk pin connected to SD Card
     * @param cs   DigitalOut pin used as SD Card chip select
     * @param name The name used to access the virtual filesystem
     */
    SDFileSystem(PinName mosi, PinName miso, PinName sclk, PinName cs, const char* name);
    virtual int disk_initialize();
    virtual int disk_status();
    virtual int disk_read(unsigned char *buffer, unsigned long block_number, unsigned char count);    
    virtual int disk_write(const unsigned char *buffer, unsigned long block_number, unsigned char count);
    virtual int disk_sync();
    virtual unsigned long disk_sector_count();

protected:

    int _cmd(int cmd, int arg);
    int _cmdx(int cmd, int arg);
    int _cmd8();
    int _cmd58();
    int initialise_card();
    int initialise_card_v1();
    int initialise_card_v2();
    
    int _read(unsigned char *buffer, int length);
    int _write(const unsigned char *buffer, int length);
    unsigned long _sd_sectors();
    unsigned long _sectors;
    
    SPI _spi;
    DigitalOut _cs; 
    int cdv;    
};

#endif

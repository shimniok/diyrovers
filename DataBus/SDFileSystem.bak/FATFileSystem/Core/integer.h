//Modified by Thomas Hamilton, Copyright 2010

/*--------------------------------------------------------------------*/
/* Integer type definitions for FatFs module  R0.08     (C)ChaN, 2010 */
/*--------------------------------------------------------------------*/

#ifndef _INTEGER
#define _INTEGER

#include "stdint.h"

/* These types must be 16-bit, 32-bit or larger integer */
typedef int             INT;
typedef unsigned int    UINT;

/* These types must be 8-bit integer */
typedef char            CHAR;
typedef unsigned char   UCHAR;
typedef unsigned char   BYTE;

/* These types must be 16-bit integer */
typedef short           SHORT;
typedef unsigned short  USHORT;
typedef unsigned short  WORD;
typedef unsigned short  WCHAR;

/* These types must be 32-bit integer */
typedef long            LONG;
typedef unsigned long   ULONG;
typedef unsigned long   DWORD;

/* Boolean type */
typedef enum { FALSE = 0, TRUE } BOOL;

#endif
/**************************************************************************/ 
/*                                                                        */ 
/*            Copyright (c) 1996-2003 by Express Logic Inc.               */ 
/*                                                                        */ 
/*  This software is copyrighted by and is the sole property of Express   */ 
/*  Logic, Inc.  All rights, title, ownership, or other interests         */ 
/*  in the software remain the property of Express Logic, Inc.  This      */ 
/*  software may only be used in accordance with the corresponding        */ 
/*  license agreement.  Any unauthorized use, duplication, transmission,  */ 
/*  distribution, or disclosure of this software is expressly forbidden.  */ 
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */ 
/*  written consent of Express Logic, Inc.                                */ 
/*                                                                        */ 
/*  Express Logic, Inc. reserves the right to modify this software        */ 
/*  without notice.                                                       */ 
/*                                                                        */ 
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** FileX Component                                                       */
/**                                                                       */
/**   STDIO Wrapper (API)                                                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  STDIO WRAPPER DEFINITION                               RELEASE        */ 
/*                                                                        */ 
/*    fx_stdio.h                                          PORTABLE C      */ 
/*                                                           3.1          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    William E. Lamie, Express Logic, Inc.                               */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file defines the basic STDIO Interface (API) to the            */ 
/*    high-performance FileX MS-DOS compatible embedded file system.      */ 
/*    All service prototypes and data structure definitions are defined   */ 
/*    in this file.                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  xx-xx-2003     William E. Lamie         Initial Version 3.1x          */ 
/*                                                                        */
/**************************************************************************/ 

#ifndef  FX_STDIO_H
#define  FX_STDIO_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif


/* Bring in necessary include files.  */

#include "tx_api.h"
#include "fx_api.h"
#include <stdio.h>


/* Define constants for the FileX STDIO wrapper.  */

#ifndef     FX_STDIO_MAX_DRIVES
#define     FX_STDIO_MAX_DRIVES         4
#endif

#ifndef     FX_STDIO_MAX_OPEN_FILES
#define     FX_STDIO_MAX_OPEN_FILES     10
#endif

/* Define data structures for the FileX STDIO wrapper.  */

typedef struct FX_STDIO_DRIVE_STRUCT
{
    CHAR        fx_stdio_drive_letter;
    FX_MEDIA   *fx_stdio_filex_media;
} FX_STDIO_DRIVE;


typedef struct FX_STDIO_FILE_STRUCT
{
    CHAR        fx_stdio_mode;
    CHAR        fx_stdio_update;
    CHAR        fx_stdio_binary;
    FX_FILE     fx_stdio_filex_file;
} FX_STDIO_FILE;


/* Define function mapping to the FileX STDIO wrapper.  */

#define fopen                               _fx_stdio_fopen
#define fclose                              _fx_stdio_fclose
#define fputc                               _fx_stdio_fputc
#define fgetc                               _fx_stdio_fgetc
#define fseek                               _fx_stdio_fseek
#define fgets                               _fx_stdio_fgets
#ifdef feof
#undef feof
#endif
#define feof                                _fx_stdio_feof
#define fread                               _fx_stdio_fread
#define stat                                _fx_stdio_stat
#define remove                              _fx_stdio_remove
#define fwrite                              _fx_stdio_fwrite

/* Define function prototypes to the FileX STDIO wrapper.  */

void    _fx_stdio_initialize(void);
int     _fx_stdio_drive_register(char drive_letter, FX_MEDIA *filex_media_ptr);
FILE    *_fx_stdio_fopen(char *filename, char *mode);
int     _fx_stdio_fclose(FILE *stream);
int     _fx_stdio_fgetc(FILE *stream);
int     _fx_stdio_fputc(char c, FILE *stream);
int     _fx_stdio_fseek(FILE *stream, long int offset, int wherefrom);

int     _fx_stdio_fread(char * buff, int size, int count, FILE *stream);
char * _fx_stdio_fgets(char *dst, int max, FILE *stream);
int    _fx_stdio_feof(FILE *stream);
int find_first_file(const char * path, char * return_filename);
int     _fx_stdio_remove(const char * path);
size_t _fx_stdio_fwrite (const void * ptr, size_t size, size_t count, FILE * stream);


//typedef struct stat stat_t;
//int     _fx_stdio_stat(const char * path, struct stat * st);

/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */


#ifdef   __cplusplus
        }
#endif

#endif

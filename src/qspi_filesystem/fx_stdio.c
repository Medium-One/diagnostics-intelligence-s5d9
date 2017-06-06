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
/**   STDIO Wrapper                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include the FileX STDIO include file.  */

#include "fx_stdio.h"
#include <sys/stat.h>


/* Define the main data structures for the STDIO wrapper.  */

FX_STDIO_DRIVE  _fx_stdio_drives_array[FX_STDIO_MAX_DRIVES];
FX_STDIO_FILE   _fx_stdio_files_array[FX_STDIO_MAX_OPEN_FILES];

int             _fx_stdio_default_drive =  0;       /* Index into the drive array.  */


/* Define ThreadX semaphore that is needed for mutual exclusion.  */

TX_SEMAPHORE    _fx_stdio_semaphore;

void  _fx_stdio_initialize(void)
{

    /* Clear the drive array.  */
    memset(&_fx_stdio_drives_array[0], 0, sizeof(_fx_stdio_drives_array));

    /* Clear the file array.  */
    memset(&_fx_stdio_files_array[0], 0, sizeof(_fx_stdio_files_array));

    /* Setup the wrapper semaphore.  */
    tx_semaphore_create(&_fx_stdio_semaphore, "FileX STDIO Semaphore", 1);
}


int  _fx_stdio_drive_register(char drive_letter, FX_MEDIA *filex_media_ptr)
{

int     i;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Search for an empty slot in the drive mapping table.  */
    i =  0;
    while (i < FX_STDIO_MAX_DRIVES)
    {

        /* See if this entry is available.  */
        if (_fx_stdio_drives_array[i].fx_stdio_filex_media == FX_NULL)
        {

            /* We found an available entry.  Populate it with the 
               appropriate information.  */
            _fx_stdio_drives_array[i].fx_stdio_filex_media =   filex_media_ptr;
            _fx_stdio_drives_array[i].fx_stdio_drive_letter =  drive_letter;

            /* Release semaphore. */
            tx_semaphore_put(&_fx_stdio_semaphore);

            /* Return success.  */
            return(FX_SUCCESS);
        }

        /* Move to next entry.  */
        i++;
    }

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Return error.  */
    return(FX_NO_MORE_ENTRIES);
}


FILE  *_fx_stdio_fopen(char *filename, char *mode)
{

int             i;
UINT            status;
FX_STDIO_FILE   *file_ptr;
FX_MEDIA        *media_ptr;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* First, determine if there is an available file entry.  */
    i =  0;
    while (i < FX_STDIO_MAX_OPEN_FILES)
    {

        /* See if this entry is available.  */
        if (_fx_stdio_files_array[i].fx_stdio_mode == 0)
        {

            /* Yes, we have an available array entry.  */
            _fx_stdio_files_array[i].fx_stdio_mode =    *mode;
            _fx_stdio_files_array[i].fx_stdio_update =  *(mode+1);       
            _fx_stdio_files_array[i].fx_stdio_binary =  *(mode+2);

            /* Get out of the loop.  */
            break;
        }

        /* Move to next entry.  */
        i++;
    }

    /* Determine if there was an available slot for this file.  */
    if (i == FX_STDIO_MAX_OPEN_FILES)
    {

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* No more entries available, return NULL.  */
        return(FX_NULL);
    }

    /* Setup the file pointer.  */
    file_ptr =  &_fx_stdio_files_array[i];

    /* Clear the media pointer.  */
    media_ptr =  FX_NULL;

    /* Otherwise, we now have an entry.  Now we need to find the
       appropriate drive the file open is to occur on.  */
    if (*(filename+1) == ':')
    {

        /* Yes, a drive is specified.  Search the drive mapping to
           find the drive.  */
        i =  0;
        while (i < FX_STDIO_MAX_DRIVES)
        {

            /* See if this entry is available.  */
            if (_fx_stdio_drives_array[i].fx_stdio_drive_letter == (*filename))
            {
    
                /* Yes, we have a match.  */
                media_ptr =  _fx_stdio_drives_array[i].fx_stdio_filex_media;

                /* Get out of loop.  */
                break;
            }

            /* Look at next drive.  */
            i++;
        }

        /* Otherwise, all is okay, adjust the pointer past the drive specification.  */
        filename = filename + 2;
    }
    else
    {

        /* No drive specified, use the default drive.  */
        media_ptr =  _fx_stdio_drives_array[_fx_stdio_default_drive].fx_stdio_filex_media;
    }

    /* Check for a valid drive.  */
    if (media_ptr == FX_NULL)
    {
        
        /* Clear the file entry out.  */
        file_ptr -> fx_stdio_mode =  0;

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Return a NULL because of the error.  */
        return(FX_NULL);
    } 

    /* Now we can actually attempt to open the file.  */

    /* Check to see if the file should be open for read only or for writing.  */
    if ((file_ptr -> fx_stdio_mode == 'r') && (file_ptr -> fx_stdio_update != '+'))
    {

        /* File should be open for only reading.  */
        status =  fx_file_open(media_ptr, &(file_ptr -> fx_stdio_filex_file), filename, FX_OPEN_FOR_READ);
    }
    else
    {

        /* Determine if a file create should be done in advance of the open.  */
        if ((file_ptr -> fx_stdio_mode == 'w') || (file_ptr -> fx_stdio_mode == 'a'))
        {

            /* Create a file, this will fail if the file is already there.  */
            status = fx_file_create(media_ptr, filename);

            if (status == FX_SUCCESS)
                fx_media_flush(media_ptr);
        }

        /* File should be open for writing.  */
        status =  fx_file_open(media_ptr, &(file_ptr -> fx_stdio_filex_file), filename, FX_OPEN_FOR_WRITE);

        /* Now see if we need to truncate the file.  */
        if (file_ptr -> fx_stdio_mode == 'w')
        {

            /* Truncate the file's contents.  */
            status += fx_file_truncate(&(file_ptr -> fx_stdio_filex_file), 0);
        }

        /* Determine if we should seek to the end of the file.  */
        if (file_ptr -> fx_stdio_mode == 'a')
        {

            /* Yes, seek to the end of the file.  */
            status =  fx_file_seek(&(file_ptr -> fx_stdio_filex_file), 0xFFFFFFFFUL);
        }
    }

    /* Determine if the file open was successful.  */
    if (status != FX_SUCCESS)
    {
        
        /* Clear the file entry out.  */
        file_ptr -> fx_stdio_mode =  0;

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Return a NULL because of the error.  */
        return(FX_NULL);
    } 
    else
    {

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Otherwise, the file open is successful return the handle to the caller.  */
        return((FILE *) file_ptr);
    }
}


int  _fx_stdio_fclose(FILE *stream)
{

UINT            status;
FX_STDIO_FILE   *file_ptr;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    /* Close the file.  */
    status =  fx_file_close(&(file_ptr -> fx_stdio_filex_file));

    /* Clear the file entry out.  */
    file_ptr -> fx_stdio_mode =  0;

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Check the return status.  */
    if (status)
    {

        /* Return an EOF error.  */
        return(EOF);
    }
    else
    {

        /* Return success!  */
        return(FX_SUCCESS);
    }
}


int  _fx_stdio_fgetc(FILE *stream)
{

UCHAR           buffer;
UINT            status;
ULONG           actual_size;
FX_STDIO_FILE   *file_ptr;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    /* Read one character from the file.  */
    status =  fx_file_read(&(file_ptr -> fx_stdio_filex_file), &buffer, 1, &actual_size);

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Determine if the read was successful.  */
    if (status != FX_SUCCESS)
    {

        /* Error, return an EOF.  */
        return(EOF);
    }
    else
    {

        /* Success, return the character as an integer.  */
        return((int) buffer);
    }
}


int  _fx_stdio_fputc(char c, FILE *stream)
{

UINT            status;
FX_STDIO_FILE   *file_ptr;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    /* Write one character to the file.  */
    status =  fx_file_write(&(file_ptr -> fx_stdio_filex_file), &c, 1);

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Determine if the read was successful.  */
    if (status != FX_SUCCESS)
    {

        /* Error, return an EOF.  */
        return(EOF);
    }
    else
    {

        /* Success, return the character as an integer.  */
        return((int) c);
    }
}


int  _fx_stdio_fseek(FILE *stream, long int offset, int wherefrom)
{

UINT            status;
FX_STDIO_FILE   *stdio_file_ptr;
FX_FILE         *file_ptr;


    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Cast the file pointer into the wrapper pointer.  */
    stdio_file_ptr =  (FX_STDIO_FILE *) stream;

    /* Pickup the FileX file pointer.  */
    file_ptr =  &(stdio_file_ptr -> fx_stdio_filex_file);

    /* Determine how to seek in the file.  */
    if (wherefrom == SEEK_SET)
    {

        /* Check for negative offset at the beginning.  */
        if (offset < 0)
        {
            
            /* Seek to beginning of the file.  */
            status =  fx_file_seek(file_ptr, 0);
        }
        else
        {

            /* Seek to the specified offset.  */
            status =  fx_file_seek(file_ptr, (ULONG) offset);
        }
    } 
    else if (wherefrom == SEEK_CUR)
    {

        /* Determine if we should seek forward or backward.  */
        if (offset < 0)
        {

            /* Seek backward from current position.  */

            /* Convert offset to a positive number.  */
            offset =  offset * -1;

            /* Determine if the backward offset is greater than the current file offset.  */
            if (((ULONG) offset) >= file_ptr -> fx_file_current_file_offset)
            {

                /* Yes, just position the file to the beginning.  */
                status =  fx_file_seek(file_ptr, 0);
            }
            else
            {

                /* Seek backward relative to the current position.  */
                status =  fx_file_seek(file_ptr, (ULONG)(file_ptr -> fx_file_current_file_offset - ((ULONG) offset)));
            }
        }
        else
        {

            /* Seek forward from current position.  */
            status =  fx_file_seek(file_ptr, (ULONG)(file_ptr -> fx_file_current_file_offset + ((ULONG) offset)));
        }
    }
    else 
    {

        /* Default to SEEK_END  */

        /* Determine if we should seek forward or backward.  */
        if (offset < 0)
        {

            /* Seek backward from end of file.  */

            /* Convert offset to a positive number.  */
            offset =  offset * -1;

            /* Determine if the backward offset is greater than the current file offset.  */
            if (((ULONG) offset) >= file_ptr -> fx_file_current_file_size)
            {

                /* Yes, just position the file to the beginning.  */
                status =  fx_file_seek(file_ptr, 0);
            }
            else
            {

                /* Seek backward relative to the end of the file.  */
                status =  fx_file_seek(file_ptr, (ULONG) (file_ptr -> fx_file_current_file_size - ((ULONG) offset)));
            }
        }
        else
        {

            /* A seek forward from end position causes a file write of unknown contents for the 
               specified size.  */
            status =  fx_file_seek(file_ptr, 0xFFFFFFFFUL);
            status =  fx_file_write(file_ptr, &status,(ULONG) (((ULONG) offset) - file_ptr -> fx_file_current_file_offset));
        }
    }

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Return completion code.  */
    return((int)status);
}

int _fx_stdio_fread(char * buff, int size, int count, FILE *stream)
{
    UINT            status;
    ULONG           actual_size;
    FX_STDIO_FILE   *file_ptr;

        /* Get protection semaphore.  */
        tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

        /* Cast the file pointer into the wrapper pointer.  */
        file_ptr =  (FX_STDIO_FILE *) stream;

        /* Read requested number of characters from the file.  */
        status =  fx_file_read(&(file_ptr -> fx_stdio_filex_file), buff, (ULONG)(size * count), &actual_size);

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Determine if the read was successful.  */
        if (status != FX_SUCCESS)
        {
            /* Error, return an EOF.  */
            return(EOF);
        }
        else
        {
            /* Success, return the actual number of characters read.  */
            return((int)actual_size);
        }
}

int _fx_stdio_stat(const char * path, struct stat * st)
{
    FX_MEDIA *media_ptr;
    UINT status;
    UINT attributes;
    FX_FILE   file;

        /* Get protection semaphore.  */
        tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

        media_ptr =  _fx_stdio_drives_array[_fx_stdio_default_drive].fx_stdio_filex_media;

        // remove the trailing '/'
        char local_path[500 /*MAX_PATH_SIZE in mongoose.h*/];
        strcpy(local_path, path);

        int len = (int) strlen(local_path);
        if (len > 1 && local_path[len - 1] == '/')
            local_path[len - 1] = 0;

        status = fx_directory_attributes_read(media_ptr, local_path, &attributes);
        if (status == FX_SUCCESS)
        {
            st->st_mode = S_IFDIR;
        }
        else if (status == FX_NOT_DIRECTORY)
        {
            status = fx_file_attributes_read(media_ptr, local_path, &attributes);
            if (status == FX_SUCCESS)
            {
                st->st_mode = 0;
                status =  fx_file_open(media_ptr, &file, local_path, FX_OPEN_FOR_READ);
                if (status == FX_SUCCESS)
                    st->st_size = (off_t) file.fx_file_current_file_size;
                fx_file_close(&file);
            }
        }
        else if (strcmp(local_path, "/") == 0)
        {
            st->st_mode = S_IFDIR;
            status = FX_SUCCESS;
        }

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);
        return (int) status;
}

char * _fx_stdio_fgets(char *dst, int max, FILE *stream)
{
    char c;
    char *p;
    ULONG actual_size;
    UINT status;
    FX_STDIO_FILE   *file_ptr;

    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    /* get max bytes or until a newline or eof */

    for (p = dst, max--; max > 0; max--)
    {
        status =  fx_file_read(&(file_ptr -> fx_stdio_filex_file), &c, 1, &actual_size);
        if (FX_END_OF_FILE == status)
            break;
        if (FX_SUCCESS == status)
            *p++ = c;
        if (c == '\n')
            break;
    }
    *p = 0;
    if ((p == dst) || (FX_END_OF_FILE == status))
        p = NULL;

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    return (p);
}

int    _fx_stdio_feof(FILE *stream)
{
    FX_STDIO_FILE   *file_ptr;

    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    return (file_ptr->fx_stdio_filex_file.fx_file_current_file_offset == file_ptr->fx_stdio_filex_file.fx_file_current_file_size);
}

int _fx_stdio_remove(const char * filename) {
    int             i;
    UINT            status;
    FX_STDIO_FILE   *file_ptr;
    FX_MEDIA        *media_ptr;

    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    /* Clear the media pointer.  */
    media_ptr =  FX_NULL;

    /* Otherwise, we now have an entry.  Now we need to find the
       appropriate drive the file open is to occur on.  */
    if (*(filename+1) == ':')
    {

        /* Yes, a drive is specified.  Search the drive mapping to
           find the drive.  */
        i =  0;
        while (i < FX_STDIO_MAX_DRIVES)
        {

            /* See if this entry is available.  */
            if (_fx_stdio_drives_array[i].fx_stdio_drive_letter == (*filename))
            {

                /* Yes, we have a match.  */
                media_ptr =  _fx_stdio_drives_array[i].fx_stdio_filex_media;

                /* Get out of loop.  */
                break;
            }

            /* Look at next drive.  */
            i++;
        }

        /* Otherwise, all is okay, adjust the pointer past the drive specification.  */
        filename = filename + 2;
    }
    else
    {

        /* No drive specified, use the default drive.  */
        media_ptr =  _fx_stdio_drives_array[_fx_stdio_default_drive].fx_stdio_filex_media;
    }

    /* Check for a valid drive.  */
    if (media_ptr == FX_NULL)
    {

        /* Clear the file entry out.  */
        file_ptr -> fx_stdio_mode =  0;

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Return a NULL because of the error.  */
        return(-1);
    }

    status = fx_file_delete(media_ptr, (char *)filename);

    /* Determine if the file delete was successful.  */
    if (status != FX_SUCCESS)
    {
        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Return a NULL because of the error.  */
        return(-1);
    }
    else
    {

        /* Release semaphore. */
        tx_semaphore_put(&_fx_stdio_semaphore);

        /* Otherwise, the file open is successful return the handle to the caller.  */
        return(0);
    }
}

size_t _fx_stdio_fwrite ( const void * ptr, size_t size, size_t count, FILE * stream ) {
    size_t written = 0;
    FX_STDIO_FILE   *file_ptr;
    UINT ret;


    /* Cast the file pointer into the wrapper pointer.  */
    file_ptr =  (FX_STDIO_FILE *) stream;

    /* Get protection semaphore.  */
    tx_semaphore_get(&_fx_stdio_semaphore, TX_WAIT_FOREVER);

    ret = fx_file_write(&file_ptr->fx_stdio_filex_file, (void *)ptr, size * count);
    if (ret == FX_SUCCESS)
        written = count;

    /* Release semaphore. */
    tx_semaphore_put(&_fx_stdio_semaphore);

    /* Otherwise, the file open is successful return the handle to the caller.  */
    return written;

}

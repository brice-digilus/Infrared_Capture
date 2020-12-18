/** Nano TIFF library.

	Write uncompressed 16 bit grayscale tiff files
	
    Copyright 2020 Brice Dubost

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h> // pulls in declaration for strlen.
#include <errno.h>
#include "nano_tiff_lib.h"

//http://paulbourke.net/dataformats/tiff/

void WriteHexString(FILE *fptr, char const *s)
{
  unsigned int i, c;
  char hex[3];

  for (i = 0; i < strlen(s); i += 2)
  {
    hex[0] = s[i];
    hex[1] = s[i + 1];
    hex[2] = '\0';
    sscanf(hex, "%X", &c);
    putc(c, fptr);
  }
}


void WriteAsciiString(FILE *fptr, char const *s, int max_len)
{
  //max len includes termination byte
  unsigned int i, c;
  char cc;
  int write_len;
  int zero_padding_len;
  if(strlen(s)< max_len)
  {
    write_len = strlen(s);
  }
  else
  {
    write_len = max_len-1;
  }
  zero_padding_len = max_len-write_len;
  

  for (i = 0; i < write_len; i ++)
  {
    putc(s[i], fptr);
  }
  for (i = 0; i < zero_padding_len; i ++)
  {
    putc('\0', fptr);
  }
}


int write_tiff(uint16_t *Tiff_image, int nx, int ny, char *filename)
{
  //ugly wrapper C- like
  char descr[] = "No title";
  return write_tiff_descr(Tiff_image, nx, ny, filename, descr);

}


int write_tiff_descr(uint16_t *Tiff_image, int nx, int ny, char *filename, char *description)
{

  printf("Writing tiff to '''%s'''", filename);
  FILE *fptr;
  fptr = fopen(filename, "w");
  if (fptr == NULL)
  {
    printf("File not created okay, errno = %d\n", errno);
    return 1;
  }

  //http://paulbourke.net/dataformats/tiff/
  //buggy at first modified to be working and grayscale 16bit per px
  int offset;

  /* Write the header */
  WriteHexString(fptr, "4d4d002a"); /* Big endian & TIFF identifier */
  offset = nx * ny * 2 + 8;
  fputc((offset & 0xff000000) / 16777216, fptr);
  fputc((offset & 0x00ff0000) / 65536, fptr);
  fputc((offset & 0x0000ff00) / 256, fptr);
  fputc((offset & 0x000000ff), fptr);

  /* Write the binary data */
  for (int y = 0; y < ny; y++)
  {
    for (int x = 0; x < nx; x++)
    {
      fputc(Tiff_image[x + y * nx] >> 8, fptr);
      fputc(Tiff_image[x + y * nx] & 0xFF, fptr);
    }
  }

  /* Write the footer */
  //    WriteHexString(fptr,"000e");  /* The number of directory entries (14) */
  WriteHexString(fptr, "000c"); /* The number of directory entries (12) */
  /* subfile type, longt 16 = 0x0010*/
  //tag 00fe   data type 0004 N 0000 0001 dataoofset/data 00 00 00 00 ");
  WriteHexString(fptr, "00fe00040000000100000000");

  /* Width tag, short int */
  WriteHexString(fptr, "0100000300000001");
  fputc((nx & 0xff00) / 256, fptr); /* Image width */
  fputc((nx & 0x00ff), fptr);
  WriteHexString(fptr, "0000");

  /* Height tag, short int */
  WriteHexString(fptr, "0101000300000001");
  fputc((ny & 0xff00) / 256, fptr); /* Image height */
  fputc((ny & 0x00ff), fptr);
  WriteHexString(fptr, "0000");

  //16 bit Gray     2 bytes per pixel 1 sample  per pixel 16 bits per sample

  /* Bits per sample tag, count 1 because 1 sample per pixel short int 16 = 0x0010*/
  WriteHexString(fptr, "01020003000000010010");
  WriteHexString(fptr, "0000");

  /* Compression flag, short int : 1 uncompressed*/
  WriteHexString(fptr, "01030003000000010001");
  WriteHexString(fptr, "0000");

  /* Photometric interpolation tag, short int min is black : 1*/
  WriteHexString(fptr, "01060003000000010001");
  WriteHexString(fptr, "0000");

  //https://www.fileformat.info/format/tiff/egff.htm
  /* Strip offset tag, long int : where to find the first strip: at 8 bytes after the beginning of the file. */
  WriteHexString(fptr, "011100040000000100000008");

  /* Orientation flag, short int 1: top left*/
  WriteHexString(fptr, "01120003000000010001");
  WriteHexString(fptr, "0000");

  /* Sample per pixel tag, short int : 1 for grayscale*/
  WriteHexString(fptr, "01150003000000010001");
  WriteHexString(fptr, "0000");

  /* Strip byte count flag, long int : One value because we have one strip and it is the size of the data*/
  WriteHexString(fptr, "0117000400000001");
  offset = nx * ny * 2;
  putc((offset & 0xff000000) / 16777216, fptr);
  putc((offset & 0x00ff0000) / 65536, fptr);
  putc((offset & 0x0000ff00) / 256, fptr);
  putc((offset & 0x000000ff), fptr);

  /* Planar configuration tag, short int */
  WriteHexString(fptr, "011c0003000000010001");
  WriteHexString(fptr, "0000");

  /* Image description ASCII*/
  WriteHexString(fptr, "010e0002");
  /*Image description data count (should be even and > longer than data length), DWORD */
  WriteHexString(fptr, "0000");
  int str_size;
  str_size = strnlen(description,65532);// not 65536 because we want an even number < 65536
  str_size += 1; //to take into account the \0 
  str_size = str_size + str_size % 2; //it has to be even for the TIFF norm

  //we write the size on 16 bits should be more than enough
  putc((str_size & 0x0000ff00) / 256, fptr);
  putc((str_size & 0x000000ff), fptr);

  /*Offset : Header 8 + image byte size + IFD header (2) + tags (N*12) + next IFD offset (4) */
  offset = 8 + nx * ny * 2 + 2 + 12*12 + 4;
  putc((offset & 0xff000000) / 16777216, fptr);
  putc((offset & 0x00ff0000) / 65536, fptr);
  putc((offset & 0x0000ff00) / 256, fptr);
  putc((offset & 0x000000ff), fptr);

  /* End of the directory entry ie next ifd offset put to 0 */
  WriteHexString(fptr, "00000000");

  /*Image description goes here, we allocated 8 bytes */
  WriteAsciiString(fptr,description,str_size);

  /* Magic end bytes name of my cat*/
  fputc('n', fptr);
  fputc('e', fptr);
  fputc('o', fptr);

  fclose(fptr);
  printf("   ... File writed successfully\n");
  return 0;
}

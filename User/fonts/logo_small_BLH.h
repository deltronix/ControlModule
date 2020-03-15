/* File 'EA.BMP' as include

 the array starts with a 2 byte header:
  1th Byte: Width of image in dots
  2th Byte: Height of image in dots
  After that image data will follow */

#define EA_LOGO_SMALL_LEN  98

const byte ea_logo_small[EA_LOGO_SMALL_LEN] __attribute__((section(".progmem.data")))=
{
   32, 19,
  254,255,255,255,255,255,191,191,191,191, 63, 63, 30,  0,  0,  0,
    0,128,224,248,254,255,255,255,255,254,248,224,128,  0,  0,  0,
  255,255,255,255,255,255,239,239,239,239,231,224,192,  0,192,240,
  252,255,255,255,255,255,243,243,255,255,255,255,255,252,240,192,
    3,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  3,  0,  3,  7,
    7,  7,  7,  3,  1,  1,  1,  1,  1,  1,  3,  7,  7,  7,  7,  3
};

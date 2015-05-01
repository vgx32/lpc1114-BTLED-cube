
#ifndef __BUFFER
#define __BUFFER

#define BUF_SIZE    50
// #define HDR_CLEAR   'c' // clears everything in memory on the cube
#define HDR_SINGLE  's' // received a single still frame
#define HDR_LINK    'l' // received a frame of an animation
#define CUBE_SIZE   8   // number of bytes necessary to store display data
#define PKT_SIZE    9
/*
  packet size: 9 bytes
  what to do if received --
    HDR_SINGLE packet:
      clear everything that is being displayed on the cube
      update cube display to only show the HDR_SINGLE frame
    HDR_LINK:
      append HDR_LINK image to the end of list of cube display framesbk
*/

// |B0B1 B2B3 B4B5 B6B7 |end
// | L0   L1   L2   L3  | cube levels

typedef struct {
  char data[BUF_SIZE];
  unsigned int end;
  unsigned int start;
} Buffer;


void resetBuf(Buffer* buf);

void writeCharToBuf(char c, Buffer* buf);

// reads a single char out of buf, incrementing the start ptr
char readBuf(Buffer* buf);

// returns the char at the front of the buffer, without incrementing start
char peekBuf(Buffer* buf);

int getNumBytesToRead(Buffer* buf);

#endif

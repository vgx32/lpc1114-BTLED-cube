#include "buffer.h"

void resetBuf(Buffer* buf) {
  buf->end =0;
  buf->start = 0;
}

// #pragma registerbank(1)
void writeCharToBuf(char c, Buffer* buf) {
  buf->data[buf->end] = c;
  buf->end++;
  if (buf->end > BUF_SIZE) {
    buf->end = 0;
  }
}

char readBuf(Buffer* buf) {
  char result;
  result = peekBuf(buf);
  buf->start++;
  if(buf->start >= BUF_SIZE) {
    buf->start = 0;
  }
  return result;
}

char peekBuf(Buffer* buf) {
  return buf->data[buf->start];
}

int getNumBytesToRead(Buffer* buf) {
  if(buf->end >= buf->start) {
    return buf->end - buf->start;
  } else {
    return 0;
  }
  // return 8;
}

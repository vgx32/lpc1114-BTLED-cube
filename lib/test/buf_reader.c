
#include <stdio.h>
#include <string.h>
#include "../buffer.h"


Buffer rxBuf;
Buffer displayBuf;
char packetAvailableFlag = 0;

char charValue = 'A';
char readChar (){
  char val;
  // val = charValue;
  // if (val > 'z') {
  //   val = 'A';
  // }
  // val++;
  val = fgetc(stdin);
  return val;
}

void copyRxBufToDisplay(Buffer* rxBuf, Buffer* dispBuf){
  if (packetAvailableFlag) {
    int bytesToRead;
    bytesToRead = getNumBytesToRead(rxBuf);
    printf("display packet detected\n");
    if(bytesToRead > BUF_SIZE - dispBuf->end){
      dispBuf->end = 0; // error?
    }
    int j;
    // printf("%d bytes in rxBuf\n", bytesToRead);
    for (j = 0; bytesToRead - j >= PKT_SIZE; j = j + PKT_SIZE) {
      printf("copying one packet\n");
      char hdr;
      hdr = readBuf(rxBuf);
      printf("HEADER=%c\n", hdr);
      if (hdr == HDR_SINGLE || hdr == HDR_LINK) {
        if(hdr == HDR_SINGLE) {
          resetBuf(dispBuf);
        }
        int i;
        for (i = 0; i < CUBE_SIZE; i++){
          writeCharToBuf(readBuf(rxBuf), dispBuf);
        }
      } else { // did not receive a valid header; reset
        resetBuf(rxBuf);
        break;
      }
    }
  }
}

char display[] = "        \0";
void displayCube(Buffer* dispBuf){
  printf("displayBuf Size = %d\n", getNumBytesToRead(dispBuf));
  if(dispBuf->end >= 8) {
    int i;
    for(i = 0; i < 8; i++) {
      strncpy(display, dispBuf->data, 8);
    }
    printf("Displaying:[%s]\n", display);
  }
}

void receiveRxChar () {
    unsigned char c;
    c = readChar();
    if(c != 10) {

      writeCharToBuf(c, &rxBuf);
      packetAvailableFlag = (getNumBytesToRead(&rxBuf) >= PKT_SIZE);
      printf("char %c (%d), packetAvailableFlag = %d\n", c, c, packetAvailableFlag);
    }
}

int main(int argc, char* argv){
  resetBuf(&rxBuf);
  resetBuf(&displayBuf);
  printf("The current index is %d\n", rxBuf.end);
  while (1) {
    // printf("Looping \n");
    displayCube(&displayBuf);
    receiveRxChar();
    copyRxBufToDisplay(&rxBuf, &displayBuf);
    printf("%d bytes in rxBuf\n", getNumBytesToRead(&rxBuf) );
  }
  return 0;
}

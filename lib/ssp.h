#ifndef __SSP
#define __SSP

#define SSP_FIFOSIZE            8 

void init_ssp(void);

void write_ssp(char* data, uint8_t len);

void test_ssp(void);

void reset_chip(void);
#endif
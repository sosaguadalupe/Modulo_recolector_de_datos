#ifndef TWI1_H_
#define TWI1_H_

#include <stdint.h>

void TWI1_init(void);
void TWI1_start(void);
void TWI1_stop(void);
uint8_t TWI1_write(uint8_t data);
uint8_t TWI1_readACK(void);
uint8_t TWI1_readNACK(void);

#endif

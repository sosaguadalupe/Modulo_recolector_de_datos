#include <avr/io.h>
#include "twi1.h"

void TWI1_init(void) {
    TWSR1 = 0x00;    // Prescaler = 1
    TWBR1 = 72;      // SCL 100kHz con F_CPU=8MHz
    TWCR1 = (1<<TWEN);
}

void TWI1_start(void) {
    TWCR1 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR1 & (1<<TWINT)));
}

void TWI1_stop(void) {
    TWCR1 = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
    while (TWCR1 & (1<<TWSTO));
}

uint8_t TWI1_write(uint8_t data) {
    TWDR1 = data;
    TWCR1 = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR1 & (1<<TWINT)));
    return TW_STATUS;
}

uint8_t TWI1_readACK(void) {
    TWCR1 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while (!(TWCR1 & (1<<TWINT)));
    return TWDR1;
}

uint8_t TWI1_readNACK(void) {
    TWCR1 = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR1 & (1<<TWINT)));
    return TWDR1;
}

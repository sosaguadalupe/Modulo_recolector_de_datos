#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <avr/interrupt.h>   // Librería para interrupciones

// CONTROL DE TIEMPO 
#define SLEEP_CYCLES 7 // Cantidad de ciclos de WDT (8s c/u)
#define LUX_CAL_FACTOR 1.1   // Factor de calibración del lux

// UART 
#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)

void UART_init(void){
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
	UCSR0B = (1<<TXEN0);                 // TX habilitado
}

void UART_send_char(char c){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void UART_send_str(const char *s){
	while(*s) UART_send_char(*s++);
}

// Sigfox 
void sigfox_send_hex_payload(uint16_t lux){
	char buf[32];
	snprintf(buf, sizeof(buf), "AT$SF=%04X\r", lux & 0xFFFF);
	UART_send_str(buf);
	_delay_ms(1500);
}

// ADC
void adc_init(void){
	ADMUX = (1<<REFS0); // AVcc ref
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	_delay_ms(2);
}

uint16_t adc_read(void){
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADC;
}

// EEPROM 
void eeprom_write_word(uint16_t addr, uint16_t value){
	while(EECR & (1<<EEPE));
	EEAR = addr;
	EEDR = (uint8_t)value;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);

	while(EECR & (1<<EEPE));
	EEAR = addr + 1;
	EEDR = (uint8_t)(value >> 8);
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
}

uint16_t eeprom_read_word(uint16_t addr){
	while(EECR & (1<<EEPE));
	EEAR = addr;
	EECR |= (1<<EERE);
	uint8_t low = EEDR;

	EEAR = addr + 1;
	EECR |= (1<<EERE);
	uint8_t high = EEDR;

	return (high<<8) | low;
}

// Watchdog Sleep 
void wdt_init_interrupt(void){
	cli();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);   // 8s
	sei();
}

ISR(WDT_vect){ }

void enter_sleep(void){
	MCUCR |= (1<<SE) | (1<<SM1); // Power Down
	asm volatile("sleep");
	MCUCR &= ~(1<<SE);
}

// MAIN 
int main(void){
	UART_init();
	adc_init();

	cli();
	wdt_init_interrupt();
	sei();

	_delay_ms(500);

	uint16_t ultimo_lux = eeprom_read_word(0);

	while(1){
		uint16_t lux_raw = adc_read();
		uint16_t lux = (uint16_t)(lux_raw * LUX_CAL_FACTOR);  // aplicar factor de calibración

		if(lux != ultimo_lux){
			sigfox_send_hex_payload(lux);
			eeprom_write_word(0, lux);
			ultimo_lux = lux;
		}

		// Sleep configurable según SLEEP_CYCLES
		for(uint8_t i=0; i<SLEEP_CYCLES; i++){
			enter_sleep();
		}
	}
}

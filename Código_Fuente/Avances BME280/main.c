#define F_CPU 8000000UL // Frecuencia del ATmega328pb
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // Interrupciones
#include <avr/eeprom.h> // Librería para eeprom
#include <avr/wdt.h> // Funciones del WDT
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "twi1.h" // Librería propia utilizada para I2C

// Configuración
#define SLEEP_CYCLES 75  // Número de ciclos WDT antes de enviar (se multiplica por 8s y te da el tiempo total)
#define BAUD 9600 // Baudrate UART
#define UBRR_VALUE ((F_CPU/16/BAUD)-1) // Cálculo del valor para registro UBRR

// Inicializa UART0 (TX/RX del microcontrolador) 
void UART_init(void){
	// Inicializa UART con 8N1 y habilita transmisión
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
	UCSR0B = (1<<TXEN0); // Habilita transmisión
}

void UART_send_char(char c){
	// Envía un solo carácter por UART
	while(!(UCSR0A & (1<<UDRE0))); // Espera a que el buffer esté vacío
	UDR0 = c;
}

void UART_send_str(const char *s){
	// Envía un string por UART
	while(*s) UART_send_char(*s++);
}

// BME280 
#define BME280_ADDR 0x76 // Dirección I2C del sensor (depende de la conexión del SDO, VCC= 0x77 / GND= 0x76)
int32_t t_fine; // Variable interna para compensaciones
uint16_t dig_T1; int16_t dig_T2, dig_T3; // Coeficientes calibración temperatura
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; // Presión
uint8_t dig_H1; int16_t dig_H2; uint8_t dig_H3; int16_t dig_H4, dig_H5; int8_t dig_H6; // Humedad

// Escribe un registro del BME280
void BME280_write(uint8_t reg, uint8_t val) {
	TWI1_start();
	TWI1_write(BME280_ADDR << 1);
	TWI1_write(reg);
	TWI1_write(val);
	TWI1_stop();
}

// Lee un registro del BME280
uint8_t BME280_read(uint8_t reg) {
	uint8_t val;
	TWI1_start();
	TWI1_write(BME280_ADDR << 1);
	TWI1_write(reg);
	TWI1_start();
	TWI1_write((BME280_ADDR << 1) | 1);
	val = TWI1_readNACK();
	TWI1_stop();
	return val;
}

// Reset del BME280
void BME280_reset(void) { BME280_write(0xE0, 0xB6); _delay_ms(100); }

// Leer coeficientes de calibración
void BME280_readCalibration(void){
	dig_T1=(uint16_t)(BME280_read(0x88)|(BME280_read(0x89)<<8));
	dig_T2=(int16_t)(BME280_read(0x8A)|(BME280_read(0x8B)<<8));
	dig_T3=(int16_t)(BME280_read(0x8C)|(BME280_read(0x8D)<<8));
	dig_P1=(uint16_t)(BME280_read(0x8E)|(BME280_read(0x8F)<<8));
	dig_P2=(int16_t)(BME280_read(0x90)|(BME280_read(0x91)<<8));
	dig_P3=(int16_t)(BME280_read(0x92)|(BME280_read(0x93)<<8));
	dig_P4=(int16_t)(BME280_read(0x94)|(BME280_read(0x95)<<8));
	dig_P5=(int16_t)(BME280_read(0x96)|(BME280_read(0x97)<<8));
	dig_P6=(int16_t)(BME280_read(0x98)|(BME280_read(0x99)<<8));
	dig_P7=(int16_t)(BME280_read(0x9A)|(BME280_read(0x9B)<<8));
	dig_P8=(int16_t)(BME280_read(0x9C)|(BME280_read(0x9D)<<8));
	dig_P9=(int16_t)(BME280_read(0x9E)|(BME280_read(0x9F)<<8));
	dig_H1=BME280_read(0xA1);
	dig_H2=(int16_t)(BME280_read(0xE1)|(BME280_read(0xE2)<<8));
	dig_H3=BME280_read(0xE3);
	dig_H4=(int16_t)((BME280_read(0xE4)<<4)|(BME280_read(0xE5)&0x0F));
	dig_H5=(int16_t)((BME280_read(0xE6)<<4)|(BME280_read(0xE5)>>4));
	dig_H6=(int8_t)BME280_read(0xE7);
}

// Configuración inicial del BME280
void BME280_init(void){
	BME280_write(0xF2,0x01); // Oversampling humedad x1
	BME280_write(0xF4,0xAF); // Temp x1, Presión x16, Normal mode
	BME280_write(0xF5,0x00); // Filtro off
}

// Leer datos RAW del sensor
void BME280_readRaw(int32_t *T, int32_t *P, int32_t *H){
	uint8_t buf[8];
	TWI1_start();
	TWI1_write(BME280_ADDR<<1);
	TWI1_write(0xF7); // Registro inicial de presión
	TWI1_start();
	TWI1_write((BME280_ADDR<<1)|1);
	for(uint8_t i=0;i<7;i++) buf[i]=TWI1_readACK();
	buf[7]=TWI1_readNACK();
	TWI1_stop();
	*P=((int32_t)buf[0]<<12)|((int32_t)buf[1]<<4)|(buf[2]>>4);
	*T=((int32_t)buf[3]<<12)|((int32_t)buf[4]<<4)|(buf[5]>>4);
	*H=((int32_t)buf[6]<<8)|buf[7];
}

// Compensar temperatura
int32_t compensate_T(int32_t adc_T){
	int32_t var1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*dig_T2)>>11;
	int32_t var2 = (((((adc_T>>4)-dig_T1)*((adc_T>>4)-dig_T1))>>12)*dig_T3)>>14;
	t_fine=var1+var2;
	return (t_fine*5+128)>>8; // Temperatura en centigrados x100
}

// Compensar presión
uint32_t compensate_P(int32_t adc_P){
	int64_t var1,var2,p;
	var1=(int64_t)t_fine-128000;
	var2=var1*var1*(int64_t)dig_P6;
	var2=var2+((var1*(int64_t)dig_P5)<<17);
	var2=var2+((int64_t)dig_P4<<35);
	var1=((var1*var1*(int64_t)dig_P3)>>8)+((var1*(int64_t)dig_P2)<<12);
	var1=(((int64_t)1<<47)+var1)*dig_P1>>33;
	if(var1==0) return 0;
	p=1048576-adc_P;
	p=(((p<<31)-var2)*3125)/var1;
	var1=((int64_t)dig_P9*(p>>13)*(p>>13))>>25;
	var2=((int64_t)dig_P8*p)>>19;
	p=(p+var1+var2)>>8;
	p+=(int64_t)dig_P7<<4;
	return (uint32_t)p;  // Presión en Pa
}

// Compensar humedad
uint32_t compensate_H(int32_t adc_H){
	int32_t v_x1 = t_fine-76800;
	v_x1=((((((adc_H<<14)-((int32_t)dig_H4<<20)-(dig_H5*v_x1))+16384)>>15)*
	(((((((v_x1*dig_H6)>>10)*(((v_x1*dig_H3)>>11)+32768))>>10)+2097152)*dig_H2+8192)>>14)));
	v_x1-=( (((v_x1>>15)*(v_x1>>15))>>7 * dig_H1)>>4);
	if(v_x1<0)v_x1=0;
	if(v_x1>419430400)v_x1=419430400;
	return v_x1>>12; // Humedad relativa %
}

// EEPROM 
typedef struct {
	uint16_t temp;
	uint16_t pres;
	uint16_t hum;
} sensor_data_t;

#define EEPROM_ADDR 0x00

void eeprom_write_data(sensor_data_t *data){
	 // Guardar bloque completo en EEPROM
	eeprom_write_block((const void*)data,(void*)EEPROM_ADDR,sizeof(sensor_data_t));
}

void eeprom_read_data(sensor_data_t *data){
	// Leer bloque completo desde EEPROM
	eeprom_read_block((void*)data,(const void*)EEPROM_ADDR,sizeof(sensor_data_t));
}

// Sigfox 
void sigfox_send_sensor(sensor_data_t *data){
	// Enviar datos como payload hexadecimal concatenado
	char buf[32];
	snprintf(buf,sizeof(buf),"AT$SF=%04X%04X%04X\r",data->temp,data->pres,data->hum);
	UART_send_str(buf);
	_delay_ms(1500); // Delay para evitar sobrecarga en transmisión
}

// Watchdog 
volatile uint8_t wdt_counter = 0;  // Contador de ciclos WDT

ISR(WDT_vect){ wdt_counter++; }  // incrementa contador

void wdt_init_interrupt(void){
	cli();
	WDTCSR |= (1<<WDCE)|(1<<WDE); // Permitir cambios
	WDTCSR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0); // 8s
	sei();
}

// Sleep profundo modo Power-Down
void enter_sleep(void){
	MCUCR |= (1<<SE)|(1<<SM1);
	asm volatile("sleep");
	MCUCR &= ~(1<<SE);
}

// Main
int main(void){
	// Inicializaciones
	UART_init();
	TWI1_init();
	BME280_reset();
	BME280_readCalibration();
	BME280_init();
	wdt_init_interrupt();
	sei();

	sensor_data_t datos, ultimo;
	eeprom_read_data(&ultimo);

	while(1){
		enter_sleep();  // WDT 8s
		if(wdt_counter >= SLEEP_CYCLES){ // Configurable al inicio
			wdt_counter = 0;

			int32_t T_raw,P_raw,H_raw;
			BME280_readRaw(&T_raw,&P_raw,&H_raw); // Leer datos crudos
			// Compensar
			datos.temp = compensate_T(T_raw);
			datos.pres = compensate_P(P_raw)/256;
			datos.hum  = compensate_H(H_raw)/4;

			// Enviar solo si hay cambios respecto al último envío
			if(memcmp(&datos,&ultimo,sizeof(sensor_data_t))!=0){
				sigfox_send_sensor(&datos);
				eeprom_write_data(&datos);
				memcpy(&ultimo,&datos,sizeof(sensor_data_t)); // Guardar último valor

			}
		}
	}
}

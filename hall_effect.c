/*
 * 350_final_project.c
 *
 * Created: 4/15/2021 9:49:03 PM
 * Author : Jack Jenkins
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALAR (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "LCD.h"

char String[25];
volatile unsigned int of_cnt = 0;
volatile unsigned int edge = 0;
volatile unsigned int now = 0;
volatile unsigned int speed_delay = 0;
volatile unsigned int speed_to_display = 0;

volatile float speed_array[50];
volatile float time_array[50];
volatile unsigned int dist_edge = 0; 
volatile unsigned int idx = 0;
volatile unsigned int dist_of = 0;
volatile float distance = 0.0;

int trigger = 0;

void UART_init() {
	UBRR0H = (unsigned char)(BAUD_PRESCALAR>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALAR;
	UCSR0B |= (1<<RXEN0); // enable receiver
	UCSR0B |= (1<<TXEN0); // enable transmitter
	
	/* Set frame format: 2 stop bits, 8 data bits */
	UCSR0C |= (1<<UCSZ00); // 8 data bits
	UCSR0C |= (1<<UCSZ01); 
	
	UCSR0C |= (1<<USBS0); // 2 stop bits
}

void UART_send(unsigned char data) {
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void UART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00)
	{
		UART_send(*StringPtr);
		StringPtr++;
	}
}

void LCD_init() {
	// configure the microprocessor pins for the data lines
	lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 4 data lines - output
	lcd_D6_ddr |= (1<<lcd_D6_bit);
	lcd_D5_ddr |= (1<<lcd_D5_bit);
	lcd_D4_ddr |= (1<<lcd_D4_bit);

	// configure the microprocessor pins for the control lines
	lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
	lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output
}

void Initialize() {
	cli();
	// setup timer 
	// div timer1 by 256
	TCCR1B &= ~(1<<CS10);
	TCCR1B &= ~(1<<CS11);
	TCCR1B |= (1<<CS12);
	
	// set up input capture
	DDRB &= ~(1<<DDB0);
	TCCR1B |= (1<<ICES1); // rising edge selected
	TIFR1 |= (1<<ICF1); // clear input capture interrupt flag 
	
	/* ------------------ interrupts ---------------------*/
	TIMSK1 |= (1<<ICIE1); // enable input capture interrupt
	TIMSK1 |= (1<<TOIE1); // enable timer1 overflow interrupt
	
	DDRB |= (1<<DDB6); // PD6 as output pin
	
	sei();
}

ISR(TIMER1_OVF_vect) {
	of_cnt += 1;
	dist_of += 1;
}

ISR(TIMER1_CAPT_vect) {
	now = ICR1;
	speed_delay += 1;
	TCCR1B ^= (1<<ICES1); // switch edge focus
	float ticks =  ((float) now) + (of_cnt*65535.0) - edge;
	float sec = ticks / 62500.0;
	float speed = (((((3.14159265 * 14) / sec) / 12) / 5280) * 3600)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ;
	speed_array[idx] = speed;
	time_array[idx] = sec;
	idx += 1;
	if (speed_delay == 5) {
		speed_to_display = (int) speed;
		speed_delay = 0;
	}
	
	lcd_write_instruction_4d(lcd_Clear); // clear display RAM
	_delay_ms(4);
	
	sprintf(String, "speed: %u MPH", speed_to_display);
	// display the first line of information
	lcd_write_string_4d(String);
	if (idx == 50) {
		float dist_time = (((now + (dist_of*65535.0) - dist_edge) / 62500.0) / 3600);
		float total_time = dist_time*3600;
		// find average speed of last 50 entries
		float avg_speed = 0.0;
		for (int i = 0; i < 50; i++) {
			avg_speed += ((float) (time_array[i] / total_time)) * speed_array[i];
		}
		sprintf(String, "total_time: %u \n", (int) total_time);
		UART_putstring(String);
		sprintf(String, "avg_speed: %u \n", (int) avg_speed);
		UART_putstring(String);
		// time in hours
		 
		sprintf(String, "dist_time: %u \n", (int) dist_time);
		UART_putstring(String);
		float new_distance = avg_speed * dist_time;
		float print_dist = new_distance*1000;
		sprintf(String, "new_distance*1000: %u \n", (int) print_dist);
		UART_putstring(String);
		distance += new_distance;
		float print_distance = distance*1000;
		sprintf(String, "distance*1000: %u \n", (int) print_distance);
		UART_putstring(String);
		dist_edge = now;
		dist_of = 0;
		idx = 0;
	}
	// set cursor to start of second line
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
	_delay_us(80);                                  // 40 uS delay (min)
	// display the second line of information
	sprintf(String, "distance: %u mi", (int) distance);
	lcd_write_string_4d(String);
	
// 	sprintf(String, "edge: %u \n", edge);
// 	UART_putstring(String);
// 	sprintf(String, "now: %u \n", now);
// 	UART_putstring(String);
// 	sprintf(String, "speed: %u \n", (int) speed);
// 	UART_putstring(String);
	edge = now;
	of_cnt = 0;
	TIFR1 |= (1<<ICF1); // clear input capture interrupt flag
}

int main(void)
{
	UART_init();
	Initialize();
	LCD_init();
	
	// initialize the LCD controller as determined by the defines (LCD instructions)
	lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface
	
	lcd_write_instruction_4d(lcd_Clear); // clear display RAM
	_delay_ms(4);
	
	sprintf(String, "speed: %u MPH", speed_to_display);
	// display the first line of information
	lcd_write_string_4d(String);
	// set cursor to start of second line
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
	_delay_us(80);                                  // 40 uS delay (min)
	// display the second line of information
	sprintf(String, "distance: 0 mi");
	lcd_write_string_4d(String);
	
    while (1) 
	{
	}
}


/*
 * ese350lab3.c
 *
 * Created: 3/25/2021 1:51:18 PM
 * Author : Ellie Chen
 */ 

#include <avr/io.h>


#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
/*--------------------Libraries---------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

char String[25];
volatile int of = 0;

void UART_init(void)
{
	
	// set rate
	UBRR0H = (unsigned char)(BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	// enable Tx and Rx
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	// set frame: 2 stop, 8 data
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8 data
	UCSR0C |= (1<<USBS0); // 2 stop
	
}

void UART_send(unsigned char data)
{
	// Wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));
	// Put data into buffer and send data
	UDR0 = data;
	
}

void UART_putstring(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		UART_send(*StringPtr);
		StringPtr++;
	}
}

void ultrasonic_init(void)
{
	cli();
	
	//PB0 = output 
	DDRB |= (1<<DDB0);
	
	//TIMER 1 CONFIG
	// normal mode (do nothing)
	//prescale timer1 (1024)
	TCCR1B |= (1<<CS10);
	TCCR1B &= ~(1<<CS11);
	TCCR1B |= (1<<CS12);
	
	// trigger pulse (10us)
	PORTB |= (1<<PORTB0);
	_delay_us(10);
	PORTB &= ~(1<<PORTB0);
	
	// WAIT FOR ECHO
	// PB0 = input & pull-up
	DDRB &= ~(1<<DDB0);
	PORTB |= (1<<PORTB0);
	
	// input capture flag 
	TIFR1 |= (1<<ICF1);
	
	// overflow interrupt 
	TIMSK1 |= (1<<TOIE0);
	
	// check for rising edge 
	TCCR1B |= (1<<ICES1);
	
	sei();
}

ISR(TIMER1_OVF_vect){
	of++;
}

void ultrasonic_pulse() {
	//B0 = output for trigger pulse 
	DDRB |= (1<<DDB0);
	
	//set input capture flag = 0
	TIFR1 &= ~(1<<ICF1);
	
	// trigger pulse (10us)
	PORTB |= (1<<PORTB0);
	_delay_us(10);
	PORTB &= ~(1<<PORTB0);
	
	// WAIT FOR ECHO
	// PB0 = input & pull-up
	DDRB &= ~(1<<DDB0);
	PORTB |= (1<<PORTB0);
	
	//reset counter number
	TCNT1 = 0;  
	
	//clear input capture flag 
	TIFR1 |= (1<<ICF1);
	
	// toggle checking for rising/falling
	TCCR1B ^= (1<<ICES1);
}

void buzzer_init(void) {
	cli();
	
	//D6 = output
	DDRD |= (1 << DDD5);
	
	//TIMER CONFIG 
	//fast pwm
	TCCR0A |= (1<<WGM00);
	TCCR0A |= (1<<WGM01);
	TCCR0B |= (1<<WGM02);
	//noninverting
	TCCR0A |= (1<<COM0B1);
	TCCR0A &= ~(1<<COM0B0);
	//prescale by 64
	TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS01);
	TCCR0B &= ~(1<<CS02);
	
	sei();
}

void ADC_init(void) {
	//clear power red
	PRR &= ~(1<PRADC);
	
	//vref = AVcc
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	
	// clock div by 128 
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	
	//ch 0
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	
	// set to free running
	ADCSRA |= (1<<ADATE); //auto trigger adc
	ADCSRB &= ~(1<<ADTS0); // 000 
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	
	// disable digital input buffer 
	DIDR0 |= (1<<ADC0D);
	
	// enable adc 
	ADCSRA |= (1<<ADEN);
	
	// start converting 
	ADCSRA |= (1<<ADSC);
}



int main(void)
{
	UART_init();
	ultrasonic_init();
	buzzer_init();
	ADC_init();
	
	int rise = 0;
	int fall = 0;
	float distance = 0;
	
	sprintf(String, "init\n");
	UART_putstring(String);
	
	while(1) {
		// first edge:
		// check interrupt flag (must b clear)
		sprintf(String, "in loop\n");
		UART_putstring(String);
		while(!(TIFR1 & (1<<ICF1)));
		rise = ICR1 + 65535*of;
		ultrasonic_pulse();
		sprintf(String, "first pulse\n");
		UART_putstring(String);
		// second edge
		// check interrupt flag again (must b clear)
		while(!(TIFR1 & (1<<ICF1)));
		fall = ICR1 + 65535*of;
		ultrasonic_pulse();
		
		distance = (((double) 34000) * ((fall - rise) / (double) 15625));
		sprintf(String, "%u cm\n", (int) distance);
		UART_putstring(String);
		
		if (distance < 2) {
			distance = 2;
			} else if (distance < 31) {
			distance = 31;
			} else if (distance < 60) {
			distance = 60;
		}
		
		if (distance == 2 || distance == 31 || distance == 60) {
			OCR0A = (int) (250000 / ((distance * -1047/208 + 438485/208) - 1));
			sprintf(String,"freq %u Hz\n", OCR0A);
			UART_putstring(String);
			
			if (distance < 60) {
				DDRD &= ~(1 << DDD5);
				_delay_ms(1000);
				DDRD |= (1 << DDD5);
			}
		} else {
			OCR0A = 0;
		}
		OCR0B = 0.5*OCR0A;
	}
}

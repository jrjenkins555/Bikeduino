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


ISR(TIMER1_OVF_vect){
	of++;
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
	buzzer_init();
	ADC_init();
	int duty = 100;
	OCR0A = 50;
	
	while(1) {
		
		
		DDRD |= (1 << DDD5);
		if (ADC > 550) {
			DDRD &= ~(1 << DDD5);
			duty = 0;
		} else if (ADC > 700) {
			duty = 30;
		} else if (ADC > 450) {
			duty = 40;
		} else if (ADC > 300) {
			duty = 60;
		} else if (ADC > 100) {
			duty = 80;
		} else {
			duty = 100;
		} 
		sprintf(String,"ADC: %u, duty cycle: %u %% \n", ADC, duty);
		UART_putstring(String);
		OCR0B = (duty/ (double) 100) * OCR0A;
		_delay_ms(400); 
		
	}
}



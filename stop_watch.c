/*
 * stop_watch.c
 *
 *  Created on: Sep 16, 2021
 *      Author: Omar Gomaa
 *
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


unsigned char tick_sec1 = 0 ;
unsigned char tick_sec2 = 0 ;
unsigned char tick_min1 = 0 ;
unsigned char tick_min2 = 0 ;
unsigned char tick_hrs1 = 0 ;
unsigned char tick_hrs2 = 0 ;

unsigned char flag = 0 ;

void timer_1() ;

ISR (INT0_vect)
{

	tick_sec1 = 0 ;
	tick_sec2 = 0 ;
	tick_min1 = 0 ;
	tick_min2 = 0 ;
	tick_hrs1 = 0 ;
	tick_hrs1 = 0 ;

}

ISR (INT1_vect)
{
	TCCR1A = 0 ;
	TCCR1B = 0 ;
}

ISR (INT2_vect)
{
	timer_1() ;
}


void INT0_init()
{
	cli();
	DDRD &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	GICR |= (1<<INT0);
	MCUCR |=  (1<<ISC01);
	MCUCR &= ~(1<<ISC00);
	sei();
}

void INT1_init()
{
	cli();
	DDRD &= ~(1<<PD3);
	GICR |= (1<<INT1);
	MCUCR |= (1<<ISC10) | (1<<ISC11);
	sei();
}

void INT2_init()
{
	cli();
	DDRD &= ~(1<<PB2);
	PORTB |= (1<<PB2);
	GICR |= (1<<INT2);
	MCUCR &= ~(1<<ISC2);
	sei();
}



void timer_1 (void) {



	TCNT1 = 0 ;

	OCR1A = 999 ;

	TIMSK = (1<<OCIE1A) ;


	TCCR1A=(1<<FOC1A)|(1<<FOC1B);

	TCCR1B=(1<<WGM12)|(1<<CS10)|(1<<CS12);



	SREG|=(1<<7);
}

ISR(TIMER1_COMPA_vect) {

	flag =  1 ;

}

void display() {

	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA0);
	PORTC = (PORTC&0XF0) | (tick_sec1&0X0F) ;
	_delay_ms(2) ;


	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA1);
	PORTC = (PORTC&0XF0) | (tick_sec2&0X0F) ;
	_delay_ms(2) ;


	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA2);
	PORTC = (PORTC&0XF0) | (tick_min1&0X0F) ;
	_delay_ms(2) ;



	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA3);
	PORTC = (PORTC&0XF0) | (tick_min2&0X0F) ;
	_delay_ms(2) ;



	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA4);
	PORTC = (PORTC&0XF0) | (tick_hrs1&0X0F) ;
	_delay_ms(2) ;



	PORTA &= ~ (0X3F) ;
	PORTA |= (1<<PA5);
	PORTC = (PORTC&0XF0) | (tick_hrs2&0X0F) ;
	_delay_ms(2) ;



}


int main () {

	INT0_init();
	INT1_init();
	INT2_init();

	timer_1() ;


	DDRC |= 0X0F ;
	PORTC = 0 ;

	DDRA |= 0X3F ;
	PORTA |= 0X3F ;

	DDRB &= ~ (1<<PB2) ;
	PORTB &= ~ (1<<PB2) ;



	while (1) {


		if (flag == 1) {

			flag = 0  ;
			tick_sec1++ ;

			if (tick_sec1 == 10 ){
				tick_sec1 = 0 ;
				tick_sec2 ++ ;
			}

			if(tick_sec1 == 0 && tick_sec2 ==6 ) {
				tick_sec2 = 0 ;
				tick_min1++;

			}

			if (tick_min1 ==10 ) {
				tick_min1 = 0 ;
				tick_min2 ++ ;
			}

			if(tick_min2 == 6 && tick_min1 ==0) {
				tick_min2 = 0 ;
				tick_hrs1++;
			}
			if (tick_hrs1 == 10 ) {
				tick_hrs2 ++ ;
				tick_hrs1 = 0 ;

			}
		}

		display() ;


	}

}
















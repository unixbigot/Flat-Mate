/* 
 * Quick AVR ADC primitives.
 *
 * Written for ATTINY45, not genericized yet.
 */
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include "avr_adc.h"

#if !defined(__AVR_ATtiny45__) && !defined(__AVR_ATtiny25__)
#error "This library only tested on Tiny45 so far"
#endif

/* 
 *@ Analog to digital converter routines
 */
int adc_read(void)
{
	int adc_val;
	
	// read 10-bit value from ADCH:ADCL.  MUST read low-byte first
	adc_val = ADCL;
	adc_val |= (ADCH << 8);
	return adc_val;
}

void adc_start(void)
{
	// set the 'start conversion' bit
	ADCSRA |= _BV(ADSC);
}

int adc_wait(void)
{
	// FIXME: add a timeout failure case
	while (ADCSRA & _BV(ADSC))
		_delay_ms(1);
	return 0;
}

int adc_poll(void)
{
	adc_start();
	adc_wait();
	return adc_read();
}

void adc_setref(unsigned char ref)
{
	unsigned char admux_copy = ADMUX;
	
	// clear reference settings
	admux_copy &= ~(_BV(REFS1)|_BV(REFS0)|_BV(REFS2));
	// apply new settings
	admux_copy |= ref;
	ADMUX = admux_copy;
}

void adc_setch(unsigned char ch)
{
	unsigned char admux_copy = ADMUX;
	
	// clear channel settings in our copy of ADMUX
	admux_copy &= ~(_BV(MUX3)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0));

	// apply new settings
	if (ch <= 3)
	{
		// FIXME: diff ADC modes, not supported, only single-ended channels [0..3]
		admux_copy |= ch;
	}
	ADMUX = admux_copy;
}

void adc_setprescaler(unsigned char ps)
{
	unsigned char adcsra_copy = ADCSRA;
	adcsra_copy &= ~(_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0));
	adcsra_copy |= ps;
	ADCSRA = adcsra_copy;
}

void adc_enable(unsigned char on)
{
	if (on)
		ADCSRA |= _BV(ADEN);
	else
		ADCSRA &= ~_BV(ADEN);
}

void adc_intenable(unsigned char on)
{
	if (on)
		ADCSRA |= _BV(ADIE);
	else
		ADCSRA &= ~_BV(ADIE);
}

/*
 * Simple Li-poly battery monitor
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#ifdef __AVR_ATtiny45__
/* 
 *@@ Pin assignments for ATtiny45
 * 
 * Pins:
 *
 * BATI - Battery voltage sense input pint 
 * LEDA - battery level alert led (red)
 * LEDB - battery level usable led (orange)
 * LEDC - battery level good led (yellow)
 * LEDD - battery level full led (green)
 *
 * N.B. Code assumes LED[A-D] are adjacent and LEDA is lowest bit value.
 *
 * Interrupts:
 *
 * TICK - timer tick from internal timer 1 (clk/16384) = 8MHz/16k + TOP=122 => 4.0023 Hz
 * ADC  - ADC conversion interrupt
 */

#define TICK_OCR		OCR1C
#define TICK_TOP		122
#define TICK_CLK_DIV_1024	(_BV(CS13)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV_4096	(_BV(CS13)|_BV(CS12)|_BV(CS10))
#define TICK_CLK_DIV_16384	(_BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_MODE		TICK_CLK_DIV_4096
#define TICK_OVF_VECT		TIM1_OVF_vect


// battery voltage level (Vb) input uses Analog input 2 on PB4 (pin 3)
// Vb/4 via voltage-divider is compared against internal 2.56V reference 
#define BATI_DDR		DDRB
#define BATI_PORT		PORTB
#define BATI_BIT		PB4
#define BATI_BV			_BV(BATI_BIT)

// primary output is a red "battery dangerously low" alert on PB0
// secondary output is a three-element bargraph on PB[123]
#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_PIN			PINB
#define LEDA_BIT		PB0
#define LEDB_BIT		PB1
#define LEDC_BIT		PB2
#define LEDD_BIT		PB3

#define LEDA_BV			_BV(LEDA_BIT)
#define LEDB_BV			_BV(LEDB_BIT)
#define LEDC_BV			_BV(LEDC_BIT)
#define LEDD_BV			_BV(LEDD_BIT)
#define LEDALL_BV		(LEDA_BV|LEDB_BV|LEDC_BV|LEDD_BV)



#else
#error unsupported chip
#endif


/* 
 *@ Constants 
 */

/* 
 * Voltage trigger levels.
 * Battery voltage is read through a voltage divider.
 *
 * if 
 *    Vin ----+
 *            R1
 *            +----- Vout
 *            R2
 *            |
 *            =
 *            .  (gnd)
 *
 * Then Vout = Vin * ( R2 / (R1 + R2) ) 
 *
 * 
 * eg. R1=12k R2=1k => Vout = Vin * (1000 / (1000 + 12000))
 *                            Vin * 0.0769
 *
 *     R1=2k2 R2=1k => Vout = Vin * 0.3125
 *     R1=3k3 R2=1k => Vout = Vin * 0.232
 *     R1=3k9 R2=1k => Vout = Vin * 0.204  
 *     R1=4k7 R2=1k => Vout = Vin * 0.175
 *
 * Fully charged LiPo is 4.23v/cell, discharged is 2.7v/cell (nominal voltage 3.7v/cell)
 * For battery endurance, do not discharge below 3.0v/cell (aircraft users commonly use 2.9v/cell as limit)
 *
 * A 3-cell battery (nominally 11.1v) thus varies from 12.9v to 8.1v, with low-volt alert at 9.0v
 *
 */


// We consider 12v+ to be "full", 11v "good", 10v "low" and 9v "critical" (BMV_foo constants are these values in millivolts)
//
// In AVR-worldview, we use 3.9:1 voltage divider (0.204 scale factor), and read 10-bit ADC comparisons versus a 2.56v reference
// 
// So 12v becomes 2.448v when divided.   Compared to 2.56v reference this gives an ADC result of 1024*(2.448/2.56) == 979
// (defun volts2int (v sf) (/ (* 1024.0 (* v sf) ) 2.56))
//
#define BMV_FULL 12000
#define VL_FULL 979

#define BMV_GOOD 11000
#define VL_GOOD   898

#define BMV_LOW 10000
#define VL_LOW   816

#define BMV_CRIT 9000
#define VL_CRIT  734

/* 
 *@ Global data
 * 
 * We take 4 samples (storing results in accumulator) then take an average.
 *
 */
#define SAMPLE_SIZE 4
char nsample;  // current number of samples in accumulator
int sum;       // value of accumulator (sum of samples)
int level;     // voltage level (mean of previous sample set)

/* 
 *@ Application functions 
 */

void update_leds(unsigned int level) 
{
	unsigned char leds = LED_PORT;
	leds &= ~LEDALL_BV;

	if (level >= VL_FULL)
	{
		// battery full
		leds |= (LEDB_BV|LEDC_BV|LEDD_BV);
	}
	else if (level >= VL_GOOD) 
	{
		// battery good
		leds |= LEDB_BV|LEDC_BV;
	}
	else if (level >= VL_LOW) 
	{
		// battery low
		leds |= LEDB_BV;
	}
	else
	{
		// battery critical
		leds |= LEDA_BV;
	}
	LED_PORT = leds;
}

/* 
 *@ Interrupt Service routins
 *
 * At every clock interrupt we trigger an ADC input
 */
ISR(TICK_OVF_VECT)	
{
	/* 
	 * We got our 1/4 second interrupt
	 * 
	 * Kick of an ADC and go back to sleep, the conversion-complete interrupt will do the rest
	 */

	// write a 1 to ADSC, bit stays high till conversion complete, then goes low, triggering interrupt
	// conversion will take around 0.1ms (13 ADC clock cycles at 125kHz ADclk)
	ADCSRA |= ADSC;
}

/* 
 * Interrupt service routine for Analog-to-Digital Conversion Complete
 *
 * We only ever use the one ADC channel (channel 2, PB4).
 */
ISR(ADC_vect)
{
	unsigned int v;
	/* 
	 * An analog-to-digital conversion for pin BATI_BIT (PB0) has completed
	 */
	
	// read 10-bit value from ADCH:ADCL.  MUST read low-byte first
	v = ADCL;
	v |= (ADCH << 8);
	sum += v;
	++nsample;
	if (nsample >= SAMPLE_SIZE) 
	{
		// we have a full set of samples, calculate mean
		level = sum/nsample;
		update_leds(level);
		nsample=0;
	}
}


/* 
 *@ IO init 
 */
void
ioinit (void)	
{
	char i;
	unsigned char l;
	short v;
	
	/* 
	 * Set up BATI pin as analog input
	 */
	BATI_DDR &= ~BATI_BV;
	DIDR0 |= ADC2D; // disable digital input buffer on PB4

	/* 
	 * Set up all LED pins as ouputs
	 *
	 * For debug, cycle a pattern across all leds for 1s
	 */
	LED_DDR |= LEDALL_BV;
	LED_PORT &= LEDALL_BV;
	
	for (v=LEDA_BV,i=0; i<100; i++) {
		l = LED_PORT;
		l &= LEDALL_BV;
		l |= v;
		LED_PORT=v;
		// v is bitmask of a left-rotating single-LED-on pattern
		v <<= 1;
		if (v > LEDD_BV) 
			v = LEDA_BV;
		_delay_ms(10);
	}
	LED_PORT &= ~(LEDALL_BV);

	/* 
	 * Set up timer 1 as clock tick source
	 *
	 * FIXME: really could disable timer interrupts altogether and 
	 * set ADC to triggerr from timer compare match
	 */
	TCCR1 |= _BV(PWM1A); // clear timer on OCR1C match, generate overflow interrupt
	TCCR1 |= TICK_CLK_MODE; // prescaler divider 16384
	TICK_OCR = TICK_TOP;
	TIMSK |= _BV(TOIE1); // overflow interrupt enable 
 
	/* 
	 * Set up analog-to-digital converter 
	 */
	// set reference level by setting REFS2..0 in ADMUX 
	// select 2.56v with no external bypass capacitor
	ADMUX |= REFS2|REFS1;
	
	// select channel by writing MUX3..0 in ADMUX
	// we want channel 2 (PB4, pin 3)
	ADMUX |= MUX1; // 0010 == channel 2

	// set ADC clock prescaler -
	// we have 8MHz clock, we want 50-200kHz ADclk
	// Select divisor 64 (0b110), giving 125kHz
	ADCSRA |= ADPS2|ADPS1;

	// enable ADC unit by settting ADEN in ADCSRA
	ADCSRA |= ADEN;

	// perform one (polled) conversion and discard result
	// (first conversion after ADC enable is less accurate than subsequent)
	ADCSRA |= ADSC;
	LED_PORT |= LEDA_BV;
	while (ADCSRA & ADSC)
		_delay_ms(1);
	LED_PORT &= ~LEDA_BV;

	// enable ADC interrupts
	ADCSRA |= ADIE;
	
	sei ();
}

int
main (void)
{
	// led off initially

	ioinit ();
	
	/* loop forever, the interrupts are doing the rest */
	for (;;)
		sleep_mode();

	return (0);
}

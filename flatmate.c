/*
 * Flat Mate - Simple Li-poly battery monitor
 *
 * DEVICE: attiny45
 * FUSES:  LHX: 0x062 0xDF 0xFF -- 1MHz RC oscillator
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "avr_adc.h"

#ifdef __AVR_ATtiny45__
/* 
 *@@ Pin assignments for ATtiny45
 * 
 * Pins:
 *
 * LEDA - (PB0) battery level ok led (yellow /red bicolour totem-pole arrangement)
 * LEDB - (PB1) battery level low led (yellow)
 * LEDC - (PB2) battery level good led (yellow)
 * LEDD - (PB3) battery level full led (green)
 * BATI - (PB4) Battery voltage sense input 
 *
 * Interrupts:
 *
 * TICK - timer tick from internal timer 1 (clk/2048) = 1MHz/2k + TOP=122 => 4.0023 Hz
 * ADC  - ADC conversion interrupt
 *
 */

#define TICK_OCR		OCR1C
#define TICK_TOP		122
#define TICK_CLK_DIV_1024	(_BV(CS13)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV_2048	(_BV(CS13)|_BV(CS12))
#define TICK_CLK_DIV_4096	(_BV(CS13)|_BV(CS12)|_BV(CS10))
#define TICK_CLK_DIV_16384	(_BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_MODE		TICK_CLK_DIV_2048
#define TICK_OVF_VECT		TIM1_OVF_vect

// Primary output is a "battery OK/Alert" signal on PB0 that drives
// a BiColor LED and/or a MOSFET.
//
// Secondary output is a three-element bargraph on PB[123]
//
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

// battery voltage level (Vb) input uses Analog input 2 on PB4 (pin 3)
// Vb/12 via voltage-divider is compared against internal 1.1V reference 
// (ADC unit has choice of 1.1v, 2.56v and 5v(VCC) references.)
#define BATI_DDR		DDRB
#define BATI_PORT		PORTB
#define BATI_BIT		PB4
#define BATI_BV			_BV(BATI_BIT)

#else
// insert constants for other supported chips here
#error unsupported chip
#endif

/* 
 *@ Configuration constants 
 *
 * 
 */
#define USE_TIMER 1
#define USE_BARGRAPH 1
#define CELL_COUNT 3

/* 
 *@@ Voltage trigger levels.
 *
 * Battery voltage is read through a voltage divider and compared to the internal voltage reference.
 *
 * If 
 *    Vin ----+
 *            R1
 *            +----- Vout (BATI)
 *            R2
 *            |
 *            =
 *            .  (gnd)
 *
 * Then Vout = Vin * ( R2 / (R1 + R2) ) 
 *
 * ; Use this Emacs lisp function to calculate divisors
 * (defun rn2div (rup rdown) (/ (float rdown) (+ rup rdown)))
 *
 * 
 * eg. R1=12k R2=1k => Vout = Vin * (1000 / (1000 + 12000))
 *                            Vin * 0.0769
 *
 *     R1=20k R2=10k => Vout = Vin * 0.3333   Ileak = 0.4mA @ 12v
 *     R1=2k2 R2=1k  => Vout = Vin * 0.3125
 *     R1=3k3 R2=1k  => Vout = Vin * 0.232
 *     R1=3k9 R2=1k  => Vout = Vin * 0.204    Ileak = 2.4mA @ 12v
 *     R1=39k R2=10k => Vout = Vin * 0.204    Ileak = 0.24mA @ 12v
 *     R1=4k7 R2=1k  => Vout = Vin * 0.175
 *     R1=10k R2=1k  => Vout = Vin * 0.0909   Ileak = 1mA @ 12v
 *     R1=12k R2=1k  => Vout = Vin * 0.0769   Ileak = 0.92mA @ 12v
 *
 * Fully charged LiPo is 4.23v/cell, discharged is 2.7v/cell (nominal voltage 3.7v/cell)
 * For battery endurance, do not discharge below 3.0v/cell (aircraft users commonly use 2.9v/cell as limit)
 *
 * A 3-cell battery (nominally 11.1v) thus varies from 12.9v to 8.10v, with low-volt alert at 9.00v
 * A 2-cell battery (nominally 7.46v)  varies     from 8.46v to 5.40v, with low-volt alert at 6.00v
 *
 *
 *@@ Analog read values for defined voltage levels
 *
 * We consider 12v+ to be "full", 11v "good", 10v "low" and 9v "critical" 
 * (BMV_foo constants are these values in millivolts)
 *
 * In AVR-worldview, we use 12:1 voltage divider and read 10-bit ADC comparisons versus AREF (1.1v)
 *  
 * So 12v becomes 1.00V when divided.   
 * Compared to 1.1v reference this gives an ADC result of 1024*(1.0/1.1) == 859
 *
 * An alternative approach is to use a smaller voltage divisor and compare
 * against Vcc (5.0v), but in practice a 12:1 divisor is easier to achieve
 * due to the standard first preference resistor value series.
 *
 * You can use these Emacs lisp defuns to calculate threshold analog values for your voltage levels
 *
 * (defun volts2int (v sf ref) (round (/ (* 1024.0 (* (float v) sf) ) (float ref))))
 * (defun vlist2int (sf ref levels) (mapcar (lambda (v) (volts2int (float v) sf ref)) levels))
 * eg. (volts2int 12 0.333 5.0) => 818
 *     (vlist2int (rn2div 10000 1000) 1.1 '(12 11 10 9)) => (1016 931 846 762)
 *     (vlist2int (rn2div 20000 10000) 5.0 '(12 11 10 9)) => (819 751 683 614)
 *     (vlist2int (rn2div 12000 1000) 1.1 '(12 11 10 9))=> (859 788 716 644)
 *
 *     The above line calculates the VL_* values shown below
 */

#if CELL_COUNT == 3

#define BMV_FULL 12000
#define VL_FULL    859

#define BMV_GOOD 11000
#define VL_GOOD    788

#define BMV_LOW  10000
#define VL_LOW     716

#define BMV_CRIT  9000
#define VL_CRIT    644

#elif CELL_COUNT == 2

#define BMV_FULL  8000
#define VL_FULL    573

#define BMV_GOOD  7300
#define VL_GOOD    523

#define BMV_LOW   6650
#define VL_LOW     476

#define BMV_CRIT  6000
#define VL_CRIT    430

#else
#error "Unsupported cell count"
#endif


/* 
 *@ Global data
 * 
 * We take 4 samples (storing results in accumulator) then take an average.
 *
 */
#define SAMPLE_SIZE 4
char nsample;  /* current number of samples in accumulator */
int sum;       /* value of accumulator (sum of samples) */
int level;     /* voltage level (mean of previous sample set) */


void delay_1s(void)
{
	char i;
	for (i=0;i<100;i++)
		_delay_ms(10);
}


/* 
 *@ Application subroutines
 */
void update_leds(unsigned int level) 
{
	/* 
	 * Copy current port value into temporary storage for modification.
	 * This allows atomic update without turning off the load power.
	 */
	unsigned char leds = LED_PORT;

	leds &= ~LEDALL_BV;
	/*
	 * The critical level LED is independent of the bargraph leds.
	 * It's a biColor RED when crit, and YELLOW for power-on-voltage-OK
	 * The same pin may also be connected to a FET that can turn off the load
	 */
	if (level > VL_CRIT)
	{
		/* battery is OK, yellow LED lit, FET active */
		leds |= LEDA_BV;
	}
	/* otherwise battery is FLAT, red LED lit, FET off */

#if USE_BARGRAPH
	/* 
	 * OPTIONAL 3-element Bargraph showing state-of-charge. 
	 *
	 * >=FULL GYYY_
	 * >=GOOD _YYY_
	 * >=LOW  __YY_
	 * >=CRIT ___Y_
	 * <CRIT  ____R
	 *
	 */
	if (level >= VL_FULL)
	{
		/* battery FULL or higher */
		leds |= (LEDB_BV|LEDC_BV|LEDD_BV);
	}
	else if (level >= VL_GOOD) 
	{
		/* battery GOOD or better */
		leds |= (LEDB_BV|LEDC_BV);
	}
	else if (level >= VL_LOW) 
	{
		/* battery better than LOW */
		leds |= LEDB_BV;
	}
	/* 
	 * Between LOW and CRIT the bargraph will be dark, 
	 * only the BiColour CRIT/GOOD led will light (i.e. YELLOW=power good)
	 */
#endif

	/* Write back the modified port state */
	LED_PORT = leds;
}

void update_sample(int v) 
{
	/* 
	 * bump the global Count and Sum values, 
	 * and when we have accumulated SAMPLE_SIZE samples, compute an average
	 */
	sum += v;
	++nsample;

	if (nsample >= SAMPLE_SIZE) 
	{
		/* 
		 * we have a full set of samples, calculate mean and update display
		 */
		level = sum/nsample;
		update_leds(level);

		/* 
		 * reset accumulator
		 */
		nsample=0;
		sum=0;
	}
}

#if USE_TIMER
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
	 * Read the analog input and update the sample set
	 */
	update_sample(adc_poll());
}

/* 
 * Interrupt service routine for Analog-to-Digital Conversion Complete
 *
 * We only ever use the one ADC channel (channel 2, PB4).
 */
ISR(ADC_vect)
{
	/* 
	 * An analog-to-digital conversion for pin BATI_BIT (PB0) has completed
	 */
	PINB|=LEDB_BV;
	ADCSRA|=_BV(ADIF);
	(void)adc_read();
}
#endif

/* 
 *@ IO init 
 */
void
ioinit (void)	
{
	int i,v;
	unsigned char l;
	
	/* 
	 * Set up BATI pin as analog input
	 */
	BATI_DDR &= ~BATI_BV;

	/* 
	 * Set up all LED pins as ouputs
	 *
	 * For debug, cycle a pattern across all leds for 1s
	 */
#if USE_BARGRAPH
	LED_DDR |= LEDALL_BV;
	LED_PORT &= LEDALL_BV;
	
	v=0;
	for (i=0; i<10; i++) {
		l = LED_PORT;
		l &= ~(LEDALL_BV);
		switch(v)
		{
		case 0:
			l|=LEDA_BV;
			break;
		case 1:
			l|=LEDB_BV;
			break;
		case 2:
			l|=LEDC_BV;
			break;
		case 3:
			l|=LEDD_BV;
			v=-1;
		}
		v++;
		if (v>3)
			v=0;
		LED_PORT = l;
		_delay_ms(100);
	}
	LED_PORT &= ~(LEDALL_BV);
#else
	LED_DDR |= LEDA_BV;
#endif

#if USE_TIMER
	/* 
	 * Set up timer 1 as clock tick source
	 *
	 * FIXME: really could disable timer interrupts altogether and 
	 * set ADC to trigger from timer compare match
	 */
	TCCR1 |= _BV(PWM1A);    /* clear timer on OCR1C match, generate overflow interrupt */
	TCCR1 |= TICK_CLK_MODE; /* prescaler divider 2048 */
	TICK_OCR = TICK_TOP;
	TIMSK |= _BV(TOIE1);    /* overflow interrupt enable */
#endif
 
	/* 
	 * Set up analog-to-digital converter 
	 *
	 * Internal 1v1 reference, using single-ended channel 2 (PB4, pin 3)
	 */
	adc_setref(ADC_VREF_INT_1V1);
	
	ADMUX|=_BV(MUX1); // ADC2(PB4)
	//adc_setch(2);

	/* 
	 * set ADC clock prescaler - we have 1MHz clock, we want 50-200kHz ADclk
	 */
	adc_setprescaler(ADC_PRESCALER_DIV8);

	/* 
	 * enable ADC unit by settting ADEN in ADCSRA
	 */
	adc_enable(1);

	/* 
	 * Perform one (polled) conversion and discard result
	 * (datasheet says first conversion after ADC enable is less accurate than subsequent)
	 */
	(void)adc_poll();

#if 0 && USE_TIMER
	/* 
	 * enable ADC interrupts
	 */
	ADCSRA|=_BV(ADIF);
	ADCSRA&=~_BV(ADATE);
	ADCSRA |= _BV(ADIE);
	//adc_intenable(1);
#endif
	
	sei ();
}

int
main (void)
{
	char i;
	int l;
	
	ioinit ();
	
	/* loop forever, the interrupts are doing the rest */
	for (;;) 
	{
#if USE_TIMER		
		sleep_mode();
#else
		/* 
		 * take an analog sample, save sample, update leds
		 */
		update_sample(adc_poll());
		_delay_ms(250);
#endif		
	}
	
	return (0);
}

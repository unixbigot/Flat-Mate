#ifndef _AVR_ADC_H
#define _AVR_ADC_H

/* 
 *@ ADC control constants
 * 
 */
#define ADC_VREF_VCC 0
#define ADC_VREF_EXT_PB0 _BV(REFS0)
#define ADC_VREF_INT_1V1 _BV(REFS1)
#define ADC_VREF_INT_2V56 (_BV(REFS2)|_BV(REFS1))
#define ADC_VREF_INT_2V56_BYPASS (_BV(REFS2)|_BV(REFS1)|_BV(REFS0))

#define ADC_PRESCALER_DIV2 _BV(ADPS0)
#define ADC_PRESCALER_DIV4 _BV(ADPS1)
#define ADC_PRESCALER_DIV8 (_BV(ADPS1)|_BV(ADPS0))
#define ADC_PRESCALER_DIV16 _BV(ADPS2)
#define ADC_PRESCALER_DIV64 (_BV(ADPS2)|_BV(ADPS1))
#define ADC_PRESCALER_DIV128 (_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0))

/* 
 *@ ADC control function prototypes 
 *
 * A real simple abstraction for the Analog-to-Digital converter
 *
 * These simple functions (many of them 1-liners) may work out _fatter_ than just 
 * inlining the bit-twiddlery, but in cases where you have the room to spare, 
 * the code is much more maintainable.
 *
 */

extern void adc_enable(unsigned char on);
extern void adc_setref(unsigned char ref);
extern void adc_setch(unsigned char ch);
extern void adc_setprescaler(unsigned char ps);
extern void adc_intenable(unsigned char on);

extern void adc_start(void);
extern int adc_wait(void);
extern int adc_read(void);
extern int adc_poll(void);


#endif

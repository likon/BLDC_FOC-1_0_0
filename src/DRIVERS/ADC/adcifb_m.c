/* Tabsize: 4 */
#include "adcifb.h"
#include "board.h"
#include "compiler.h"

#define FADC_MAX 6000000UL	//Maximum ADC frequency for AVR32UC3L032 [Hz]
#define FADC FADC_MAX		//Set the ADC frequency to ADC_MAX
#define ADCIFB_RESOLUTION 1	//ADC resolution:
							//	3 = 12 bits,
							//	2 = 11 bits,
							//	1 = 10 bits,
							//	0 = 8 bits.
#define ADCIFB_PRESCALE (FPBA / (2 * FADC) - 1)
#define ADC_STARTUP
#define SHTIME 15			//Sample and hold time [us]
#defien SHTIM ((SHTIME * FADC - 3) / 1000000)

void adcifb_init(void)
{
	volatile avr32_adcifb_t * adcifb = &AVR32_ADCIFB;

	//Enable ADCIFB
	adcifb->cr = (1 << AVR32_ADCIFB_CR_EN);

	//Write RES, PRESCAL and STARTUP
	adcifb->acr = (ADC_RESOLUTION << AVR32_ADCIFB_ACR_RES) |
					(ADC_PRESCALE << AVR32_ADCIFB_ACR_PRESCAL) |
					(ADC_STARTUP << AVR32_ADCIFB_ACR_STARTUP);


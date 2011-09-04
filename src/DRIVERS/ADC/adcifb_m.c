/* Tabsize: 4 */
#include "adcifb.h"
#include "board.h"
#include "compiler.h"

#define FADC_MAX 6000000UL	//Maximum ADC frequency for AVR32UC3L032 [Hz]
#define FADC FADC_MAX		//Set the ADC frequency to ADC_MAX
#define ADCIFB_RESOLUTION 1	//ADC resolution: 8 bits
							//	3 = 12 bits,
							//	2 = 11 bits,
							//	1 = 10 bits,
							//	0 = 8 bits.
#define ADCIFB_PRESCALE (FPBA / (2 * FADC) - 1)
#define ADC_STARTUP
#define SHTIM 7			//Sample and hold time
#define ADC_TIMEOUT_VALUE 50000	//Randomly chosen timeout value.
//~ #defien SHTIM (((SHTIME * FADC) / 1000000) - 3)

int adcifb_init(void)
{
	volatile avr32_adcifb_t * adcifb = &AVR32_ADCIFB;

	//Enable ADCIFB
	adcifb->cr = (1 << AVR32_ADCIFB_CR_EN);

	//Write RES, PRESCAL and STARTUP
	adcifb->acr = (ADC_RESOLUTION << AVR32_ADCIFB_ACR_RES) |
					(ADC_PRESCALE << AVR32_ADCIFB_ACR_PRESCAL) |
					(ADC_STARTUP << AVR32_ADCIFB_ACR_STARTUP);

	//Wait until ADC is activated
	timeout = ADC_TIMEOUT_VALUE;
	while(!(adcifb->sr & AVR32_ADCIFB_SR_READY_MASK)) {
		if(timeout-- == 0) {
			return ADC_ERROR;
		}
		return ADC_OK;
}



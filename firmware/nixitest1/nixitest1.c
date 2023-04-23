#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>

#define APB_CLOCK SYSTEM_CORE_CLOCK

uint32_t count;

//#define ENABLE_TUNING

#define ABSOLUTE_MAX_ADC_SET 208 // Actually around 190V  (0 to 208 maps to 0 to 190V)

// Do not mess with PWM_ values unless you know what you are willing to go down a very deep rabbit hole. 
#ifndef ENABLE_TUNING
#define PWM_PERIOD 140
#else
int PWM_PERIOD = 140;
#endif
int PWM_MAXIMUM_DUTY = 48;  //This actually gets overwrittenin the first few milliseconds onces a system VDD is read.

#define ERROR_P_TERM 2 // Actually a shift.  Normally we would do the opposite to smooth out, but we can realy bang this around!  It's OK if we rattle like crazy. 

// We can filter
#define ADC_IIR 2
#define VDD_IIR 2

int update_targ_based_on_vdd = 0;
int target_feedback = 0;
int target_feedback_vdd_adjusted = 0;
int lastadc = 0;
int lastvdd = 0;
int fade_enable = 0;
int fade_time0;
int fade_time1;
int fade_disp0;
int fade_disp1;
int fade_place = 0;

static void ApplyOnMask( uint16_t onmask )
{
	GPIOD->OUTDR = onmask >> 8;
	GPIOC->OUTDR = onmask & 0xff;
}

void ADC1_IRQHandler(void) __attribute__((interrupt));
void ADC1_IRQHandler(void)
{
	// This interrupt should happen ~3.5uS on current settings.
	lastadc = ADC1->RDATAR + (lastadc - (lastadc>>ADC_IIR));
	int adc = lastadc>>ADC_IIR;
	int err = target_feedback_vdd_adjusted - adc;
	ADC1->STATR &= ~ADC_EOC;

	if( err < 0 )
		TIM1->CH2CVR = 0;
	else
	{
		err = err << ERROR_P_TERM;
		if( err > PWM_MAXIMUM_DUTY ) err = PWM_MAXIMUM_DUTY;
		TIM1->CH2CVR = err;
	}

	int fadepos = (++count) & 0xff;

	if( fadepos & 1 )
	{
		ADC1->CTLR2 |= ADC_JSWSTART;
	}
	else
	{
		// Use injection channel data to read vref.
		// Ballparks:
		//   0xF0  / 240 for 5V input << lastvdd
		//	 0x175 / 373 for 3.3v input << lastvdd
		// Tunings (experimentally found)
		//  Duty/Period
		//  100/160 will literally cook the LEDs but can get up to 180V @ 3.3V under load.
		//  48/120  is more efficient than 48/96 at 3.3v.  (139V)
		//  60/112  is pretty efficient, too.  (150V)
		//  84/140 = 176V (reported, 180 actual) with 8 at 3.3   <<< This is a really nice thing to run at on 3.3V
		//			The transformer DOES get very warm though.
		//  Backto 5V.
		//  54/140 --> Is what it is is for 5V if period is set to 140.
		//
		//  therefore I want to map, for maximum duty cycle, the following:
		//  373 -> 84 // Ratio is 4.440
		//  240 -> 56 // Ratio is 4.444
		// Wow! That's nice!

		// TODO: Consider filtering lastvdd.
		//lastvdd = ADC1->IDATAR1; // Don't filter VDD
		lastvdd = ADC1->IDATAR1 + (lastvdd - (lastvdd>>VDD_IIR)); // Filter VDD (but now it's 2^VDD_IIR bigger)

#ifndef ENABLE_TUNING

		// IF we aren't enabling tuning, we can update max-on-time with this value.
		//  There's a neat hack where you can divide by weird decimal divisors by adding and subtracing terms.
		//  I apply that weird trick here.
		//  1÷(1÷4−1÷64−1÷128−1÷1024) is roughly equal to dividing by 4.43290
		//  We actually can simplify it for our purposes as: 1÷(1÷4−1÷64−1÷128)
		//
		PWM_MAXIMUM_DUTY = (lastvdd>>(2+VDD_IIR)) - (lastvdd>>(6+VDD_IIR)) - (lastvdd>>(7+VDD_IIR)); // lastvdd / 4.44.  For ~5V, this works out to 45, for ~3.3V it works out to ~70.
#endif
		update_targ_based_on_vdd = 1;
	}

	if( fade_enable )
	{
		if( fadepos < fade_time0 )
			ApplyOnMask( fade_disp0 );
		else if( fadepos == fade_time0 )
			ApplyOnMask( 0 );
		else if( fadepos < fade_time1 )
			ApplyOnMask( fade_disp1 );
		else
			ApplyOnMask( 0 );
	}
}

static void SetupTimer()
{
	// Main inductor is ~5uH.
	// Our peak current is ~200mA
	// Our target cycle duty is ~1/6
	// Our nominal voltage is ~4V
	// 4V / .000005H = 800000A/s / 0.2 = 0.00000025 = 250nS, but we are only on for 1/6 of the time., or 1.5uS.  Let's set our period to be 64/48 = 652nS.

	// GPIO A1 Push-Pull, Auto Function, 50 MHz Drive Current
	GPIOA->CFGLR &= ~(0xf<<(4*1));
	GPIOA->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF)<<(4*1);

	// Enable Timer 1
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	TIM1->PSC = 0x0000;  // Prescalar to 0x0000 (so, 24MHz base clock)
	TIM1->ATRLR = PWM_PERIOD;
	TIM1->SWEVGR = TIM_UG;
	TIM1->CCER = TIM_CC2E | TIM_CC2NP;  // CH2 is control for FET.
	TIM1->CHCTLR1 = TIM_OC2M_2 | TIM_OC2M_1;

	TIM1->CH2CVR = 0;  // Actual duty cycle.

	// Setup TRGO for ADC.  TODO: this should be on update (TIM_MMS_1)
	TIM1->CTLR2 = TIM_MMS_1;

	// Enable TIM1 outputs
	TIM1->BDTR = TIM_MOE;
	TIM1->CTLR1 = TIM_CEN;
}

static void SetupADC()
{
	// Configure ADC.
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
	
	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7;	// 0-9 for 8 ext inputs and two internals
	
	ADC1->ISQR = 8 | (3<<20); //Injection group is 8. NOTE: See note in 9.3.12 (ADC_ISQR) of TRM.

	// set sampling time for chl 7
	ADC1->SAMPTR2 = (4<<(3*7)) | (4<<(3*8));	// 0:7 => 3/9/15/30/43/57/73/241 cycles
		// (4 == 43 cycles), (6 = 73 cycles)  Note these are alrady /2, so 
		// setting this to 73 cycles actually makes it wait 256 total cycles
		// @ 48MHz.

	// turn on ADC and set rule group to sw trig
	ADC1->CTLR2 = ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL | ADC_EXTTRIG; // 0 = Use TRGO event for Timer 1 to fire ADC rule.

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);

	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);

	// enable the ADC Conversion Complete IRQ
	NVIC_EnableIRQ( ADC_IRQn );

	// Enable the End-of-conversion interrupt.
	ADC1->CTLR1 = ADC_EOCIE | ADC_SCAN | ADC_JDISCEN;
}

uint16_t GenOnMask( int segmenton )
{
	if( segmenton > 0 )
	{
		segmenton--;
		if( segmenton < 8 )
			return 1<<segmenton;
		else if( segmenton == 8 )
			return (1<<2)<<8;
		else if( segmenton == 9 )
			return (1<<3)<<8;
		else if( segmenton == 10 )
			return (1<<0)<<8;
		else if( segmenton == 11 )
			return (1<<7)<<8;
	}
	return 0;
}

static void HandleCommand( uint32_t dmdword )
{
	// ./minichlink -s 0x04 0x01110040
	// ./minichlink -g 0x04
	// It is a valid status word back from the PC.
	int command = dmdword & 0x0f;
	switch( command )
	{
	case 1:
	{
		int feedback = dmdword>>16;
		if( feedback > ABSOLUTE_MAX_ADC_SET ) feedback = ABSOLUTE_MAX_ADC_SET;
		target_feedback = feedback;
		break;
	}
	case 2:
	{
		int segmenton = (dmdword>>16)&0x0f;

		// Disable all fading.
		fade_enable = 0;

		ApplyOnMask( GenOnMask( segmenton ) );
		break;
	}
	case 3:
	{
		// Configure a fade.
		int disp0 = ( dmdword >> 8 ) & 0xf;
		int disp1 = ( dmdword >> 12 ) & 0xf;
		int time0 = ( dmdword >> 16 ) & 0xff;
		int time1 = ( dmdword >> 24 ) & 0xff;

		fade_time0 = time0;
		fade_time1 = time1;
		fade_disp0 = GenOnMask( disp0 );
		fade_disp1 = GenOnMask( disp1 );
		fade_enable = 1;

		break;
	}
	case 4:
	{
#ifdef ENABLE_TUNING
		// this is only for tuning.
		if( ( dmdword & 0xff00 ) == 0xaa00 )
		{
			int period = (dmdword>>16)&0xff;
			if( period < 20 ) period = 20;
			PWM_PERIOD = period;
			TIM1->ATRLR = PWM_PERIOD;
			int max_duty = (dmdword>>24)&0xff;
			if( max_duty > period - 14 ) max_duty = period - 14;
			PWM_MAXIMUM_DUTY = max_duty;
		}
#endif
		break;
	}
	}

	*DMDATA0 = ((lastadc>>ADC_IIR) << 12) | ((lastvdd>>VDD_IIR) << 22);
}

int main()
{
	SystemInit48HSI();
	SetupDebugPrintf();
	Delay_Ms( 10 );

	// Enable Peripherals
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA
				   | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;

	GPIOD->CFGLR = 
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6) | // GPIO D6 Push-Pull  (for debug)
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*7) | // DIG_AUX
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*3) | // DIG_9
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*2) | // DIG_8
		(GPIO_Speed_10MHz | GPIO_CNF_IN_FLOATING)<<(4*1) | // Leave PGM pin floating, dont make it an ADC.
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*0);  // DIG_DOT

	GPIOC->CFGLR = 
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*0) | // DIG_0
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*1) | // DIG_1
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*2) | // DIG_2
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*3) | // DIG_3
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*4) | // DIG_4
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*5) | // DIG_5
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*6) | // DIG_6
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*7);  // DIG_7

		

	GPIOC->BSHR = 1<<4;

	SetupADC();
	SetupTimer();

	*DMDATA0 = 0;

	target_feedback = 0;

	while(1)
	{
		GPIOD->BSHR = 1<<6;
		uint32_t dmdword = *DMDATA0;
		if( (dmdword & 0xf0) == 0x40 )
		{
			HandleCommand( dmdword );
		}
		GPIOD->BSHR = (1<<(16+6));
		if( update_targ_based_on_vdd )
		{
			// target_feedback is in volts. 0..200 maps to the physical device voltage.
			// lastvdd = 0xF0  for 5V input.
			// lastvdd = 0x175 for 3.3v input.
			//
			// target_feedback_vdd_adjusted = 408 for ~192V @ 5
			// target_feedback_vdd_adjusted = 680 for ~192V @ 3.3
			//
			// 408 = 192 * 240 / x = (192*240)/408 = 112.941176471
			// 680 = 192 * 373 / x = (373*680)/192 = 105.317647059
			//  Close enough to 128.
			//
			target_feedback_vdd_adjusted = (target_feedback * lastvdd) >> (7+VDD_IIR);
			update_targ_based_on_vdd = 0;
		}
	}
}


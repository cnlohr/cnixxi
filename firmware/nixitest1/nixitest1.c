#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>

// Limits the "ADC Set Value" in volts.
// This prevents us from exceeding 190 volts target.
#define ABSOLUTE_MAX_ADC_SET 190

// Do not mess with PWM_ values unless you know what you are willing to go down
// a very deep rabbit hole. I experimentally determined 140 for this particular
// system was on the more efficient side of things and gave good dynamic range.
// 140 means it's 48MHz / 140 = 342 kHz for the main flyback frequency.
//
// You can explore values by doing ENABLE_TUNING in here and in testnix.c
// #define ENABLE_TUNING
#ifndef ENABLE_TUNING
#define PWM_PERIOD 140
#else
int PWM_PERIOD = 140;
#endif
int PWM_MAXIMUM_DUTY = 48;  //This is changed based on vdd.


// Flyback PID loop Tuning Parameters
//
// This is a PID loop (you should read about this separately). But these terms
// are actually on the order of 2^term. So we are only able to get the tuning
// to a rough ballpark. Thankfully, PID loops are forgiving.
//
// Honestly, this is MUCH more sophisticated than it needs to be!  I was using
// a P-only loop for quite some time without any issues.
#define ERROR_P_TERM 3
#define ERROR_D_TERM -2
#define PID_SAT_MAX 6144
#define PID_SAT_MIN -1024
#define ERROR_I_TERM -6

// We filter our ADC inputs because they are kind of noisy.
//
// We can use Binary-shift IIR filters to filter the incoming ADC signals.
// See later in the code, but, it maps to only about 4 assembly instructions!
// (plus a read-back of the previous value we will be mixing).
#define ADC_IIR 1
#define VDD_IIR 3

// When we get a new vdd measurement, we can update our target_feedback value
// based on VDD. This doesn't need to happen very often at all! But because
// the ADC in the CH32V003 measures between GND and VDD, we have to make sure
// that is taken into account, because the measured voltage feedback value
// from the high voltage side will be scaled, based on incoming VDD.
int update_targ_based_on_vdd = 0;

// Target feedback, set by the user.
int target_feedback = 0;

// Feedback based on what the user set and the part's VDD.
int feedback_vdd = 0;

// Filtered ADC and VDD values.
int lastadc = 0;
int lastrefvdd = 0;

// Code for handling numeric fading, between 2 numbers or alone.
int fade_enable = 0;
int fade_time0;
int fade_time1;
int fade_disp0;
int fade_disp1;
int fade_place = 0;

// Apply a given output mask to the GPIO ports the nixie tubes are hooked into.
static void ApplyOnMask( uint16_t onmask )
{
	GPIOD->OUTDR = onmask >> 8;
	GPIOC->OUTDR = onmask & 0xff;
}

void ADC1_IRQHandler(void) __attribute__((interrupt));
void ADC1_IRQHandler(void)
{
	static uint32_t count;

	// This interrupt should happen ~3.5uS based on current compiler.
	// In many situations this actually will happen slower than that, but what
	// matters is that our ADC sample is ALWAYS ALIGNED to the PWM, that way
	// any ripple and craziness that happens from the chopping of the flyback
	// is completely filtered out because of where we are sampling.

	// This performs a low-pass filter on our data, ADC1->RDATAR
	// the sample rate, always.  As a side note, the value of the IIR
	// (but now it's 2^VDD_IIR bigger)
	int adcraw = ADC1->RDATAR;
	int newadc = adcraw + (lastadc - (lastadc>>ADC_IIR));
	lastadc = newadc;


	int err = feedback_vdd - lastadc;

	static int lasterr;
	static int integral;
	int derivative = -(err - lasterr);
	integral += err;

	// We asymmetrically allow the integral to saturate, to help prevent long-
	// term oscillations.
	integral = ( integral > ((PID_SAT_MAX)<<ADC_IIR) ) ? ((PID_SAT_MAX)<<ADC_IIR) : integral;
	integral = ( integral < ((PID_SAT_MIN)<<ADC_IIR) ) ? ((PID_SAT_MIN)<<ADC_IIR) : integral;

	// This is the heart of the PID loop.
	// General note about shifting: Be sure to combine your shifts.
	//  If you shift right, then left, you will lose bits of precision.
	int plant = 
		(err << ( (-ADC_IIR) + (ERROR_P_TERM) )) +
		(integral >> ( ADC_IIR - (ERROR_I_TERM) )) + 
		(derivative << ( (-ADC_IIR) - (ERROR_D_TERM) ) );
	lasterr = err;
	plant = ( plant > PWM_MAXIMUM_DUTY ) ? PWM_MAXIMUM_DUTY : plant;
	plant = ( plant < 0 ) ? 0 : plant;
	TIM1->CH2CVR = plant;

	int fadepos = (++count) & 0xff;

	// Only bother getting VDDs every other ADC cycle.
	if( fadepos & 1 )
	{
		ADC1->CTLR2 |= ADC_JSWSTART;
	}
	else
	{
		// Use injection channel data to read vref.
		// Ballparks (for unfiltered numbers)  (Just as a note)
		//   0xF0  / 240 for 5V input << lastrefvdd
		//	 0x175 / 373 for 3.3v input << lastrefvdd

		// Do an IIR low-pass filter on VDD. See IIR discussion above.
		lastrefvdd = ADC1->IDATAR1 + (lastrefvdd - (lastrefvdd>>VDD_IIR)); 

#ifndef ENABLE_TUNING

		// If we aren't enabling tuning, we can update max on time here. We
		// want to limit on-time based on DC voltage on the flyback so that
		// we can get close to (But not get into) saturation of the flyback
		// transformer's core.
		//
		// We can compute expected values, but experimenting is better.
		// Transformer inductance is ~6uH.
		// Our peak current is ~500mA
		// The average voltage is ~4V
		//
		//   4V / .000006H = 0.5A / 666666A/s = 750nS but turns out this was
		//    pessemistic.
		//
		// Experimentation showed that the core of the transformer saturates
		// in about 1uS at 5V and 1.4uS at 3.3v.  More specifically the
		// relationhip between our maximum on-time and vref-measured-by-vdd
		// works out to about:
		//
		//    max_on_time_slices = lastrefvdd / 4.44.
		//
		// There's a neat trick where you can divide by weird decimal divisors
		// by adding and subtracing terms. We perform this trick here and below
		//
		// 1÷(1÷4−1÷64−1÷128−1÷1024) is roughly equal to dividing by 4.43290
		// We actually can simplify it for our purposes as: 1÷(1÷4−1÷64−1÷128)
		//
		// You can arbitrarily add and subtract terms to get as closed to your
		// desired target value as possbile.
		//
		// When we divide a value by powers-of-two, it becomes a bit shift.
		//
		// The bit shift and IIR adjustments can be made so that the compiler
		// can optimize out the addition there.
		//
		// The following code actually
		PWM_MAXIMUM_DUTY = 
			  (lastrefvdd>>(2+VDD_IIR)) 
			- (lastrefvdd>>(6+VDD_IIR))
			- (lastrefvdd>>(7+VDD_IIR));
#endif

		// Tell our main loop that we have a new VDD if it wants it.
		update_targ_based_on_vdd = 1;
	}

	if( fade_enable )
	{
		// Digit fade.  Use fade_timeX and fade_dispX to handle fade logic.
		if( fadepos == fade_time0 )
			ApplyOnMask( 0 );
		else if( fadepos == 0 )
			ApplyOnMask( fade_disp0 );
		else if( fadepos == fade_time1 ) 
			ApplyOnMask( 0 ); // Only useful if we want to have two dim segs on
		else if( fadepos == fade_time0 + 1 )
			ApplyOnMask( fade_disp1 );
	}

	// Acknowledge the interrupt. NOTE: Another start request may have fired
	// while we were servicing the interrupt.  That's probably fine.
	ADC1->STATR &= ~ADC_EOC;
}

static void SetupTimer()
{
	// GPIO A1 Push-Pull, Auto Function, 50 MHz Drive Current.
	// This goes to our switching FET for our flyback.
	GPIOA->CFGLR &= ~(0xf<<(4*1));
	GPIOA->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF)<<(4*1);

	// Enable Timer 1
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	TIM1->PSC = 0x0000;  // Prescalar to 0x0000 (so, 48MHz base clock)
	TIM1->ATRLR = PWM_PERIOD;
	TIM1->SWEVGR = TIM_UG;
	TIM1->CCER = TIM_CC2E | TIM_CC2NP;  // CH2 is control for FET.
	TIM1->CHCTLR1 = TIM_OC2M_2 | TIM_OC2M_1;

	TIM1->CH2CVR = 0;  // Actual duty cycle (Off to begin with)

	// Setup TRGO for ADC.  This makes is to the ADC will trigger on timer
	// reset, so we trigger at the same position every time relative to the
	// FET turning on.
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
	
	//Injection group is 8. NOTE: See note in 9.3.12 (ADC_ISQR) of TRM. The
	// group numbers is actually 4-group numbers.
	ADC1->ISQR = 8 | (3<<20);

	// Sampling time for channels. Careful: This has PID tuning implications
	ADC1->SAMPTR2 = (4<<(3*7)) | (2<<(3*8)); 
		// 0:7 => 3/9/15/30/43/57/73/241 cycles
		// (4 == 43 cycles), (6 = 73 cycles)  Note these are alrady /2, so 
		// setting this to 73 cycles actually makes it wait 256 total cycles
		// @ 48MHz.

	// Turn on ADC and set rule group to sw trig
	// 0 = Use TRGO event for Timer 1 to fire ADC rule.
	ADC1->CTLR2 = ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL | ADC_EXTTRIG; 

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);

	// Calibrate ADC
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);

	// enable the ADC Conversion Complete IRQ
	NVIC_EnableIRQ( ADC_IRQn );

	// Enable the End-of-conversion interrupt.
	ADC1->CTLR1 = ADC_EOCIE | ADC_JDISCEN;
}

uint16_t GenOnMask( int segmenton )
{
	// Produce a bit mask with only one bit on. To indicate the IO to turn on
	// to light up a given segment.  If segmenton == 0, then all IO are off.
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
	// You can use minichlink to setup this:
	// ./minichlink -s 0x04 0x00B40041 # Configure for 180V.
	// ./minichlink -s 0x04 0x00030042 # Light digit "8"
	// ./minichlink -s 0x04 0x60303243 # Dimly light 8 and 8.
	// ./minichlink -g 0x04            # Get status.

	// Note: To get here, DEBUG0's LSB must be 0x4x command is that 'x'
	int command = dmdword & 0x0f;

	switch( command )
	{
	case 1:
	{
		int feedback = dmdword >> 16;
		if( feedback > ABSOLUTE_MAX_ADC_SET )
			feedback = ABSOLUTE_MAX_ADC_SET;
		target_feedback = feedback;
		break;
	}
	case 2:
	{
		int segmenton = (dmdword>>16)&0x0f;

		// Disable all fading.
		fade_enable = 0;

		// Allow at least a microsecond or so to turn cathode off.
		ApplyOnMask( 0 );

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

	// Write the status back to the host PC.  Status is our VDD and our FB V
	*DMDATA0 = ((lastadc>>ADC_IIR) << 12) | ((lastrefvdd>>VDD_IIR) << 22);
}

int main()
{
	SystemInit48HSI();

	// For the ability to printf() if we want.
	SetupDebugPrintf();

	// Let signals settle.
	Delay_Ms( 10 );

	// Enable Peripherals
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
		RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;

	GPIOD->CFGLR = 
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6) | // GPIO D6 Debug
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*7) | // DIG_AUX
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*3) | // DIG_9
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*2) | // DIG_8
		(GPIO_Speed_10MHz | GPIO_CNF_IN_FLOATING)<<(4*1) | // PGM Floats.
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

	ApplyOnMask( 0 );

	SetupADC();
	SetupTimer();

	*DMDATA0 = 0;

	target_feedback = 0;

	while(1)
	{
		// DEBUG: Twiddle P6.  We can look on the scope at what's happening
		// so we can guess at how long the interrupts are taking.
		GPIOD->BSHR = 1<<6;
		GPIOD->BSHR = (1<<(16+6));

		uint32_t dmdword = *DMDATA0;
		if( (dmdword & 0xf0) == 0x40 )
		{
			// I think there is a compiler bug here.  For some reason if I put
			// the code in this function right here, it doesn't work right.
			// so I encapsulated the code in a function.
			//
			// This function handles commands we get over the programming
			// interface.  Like 
			HandleCommand( dmdword );
		}
		if( update_targ_based_on_vdd )
		{
			// target_feedback is in volts. 0..200 maps to the device voltage.
			// lastrefvdd = 0xF0  for 5V input.
			// lastrefvdd = 0x175 for 3.3v input.
			//
			// feedback_vdd = 408 for ~192V @ 5
			// feedback_vdd = 680 for ~192V @ 3.3
			//
			// 408 = 192 * 240 / x = (192*240)/408 = 112.941176471
			// 680 = 192 * 373 / x = (373*680)/192 = 105.317647059
			//
			//  More tests showed this value across units is around 117.
			//
			// X This becomes our denominator.
			// feedback_vdd = (current vdd measurement * target voltage) / 117
			// 
			// Further testing identified that the denominator is almost
			// exactly 117.  We can perform a divison by 117 very quickly by
			//
			//   feedback = numerator/128 + numerator/2048 + numerator/4096
			//
			// See note above about the constant division trick.
			//
			// Side-note:
			// The reason we do this in the main loop instead of the interrupt
			// is because it uses a multiply.  The CH32V003 does not natively
			// have a multiply instruction, so this actually calls out to
			// __mulsi3 in libgcc.a.  As a note, it's time complexity is
			// determined by the size of the right-hand value of multiply,
			// which you should try to make the value which is typically
			// the smaller one.

			uint32_t numerator = (lastrefvdd * target_feedback);
			feedback_vdd =
				(numerator>>(7+VDD_IIR-ADC_IIR)) +
				(numerator>>(11+VDD_IIR-ADC_IIR));

			update_targ_based_on_vdd = 0;
		}
	}
}


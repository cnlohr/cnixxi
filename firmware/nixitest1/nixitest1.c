//
// cnixi - ch32v003 base driver for IN-15 nixie tubes.
//
// Fully integrated PI flyback PSU controller + Nixie Tube output
// control with up to 2 channel simultaneous dimming.
//
// Copyright 2023 <>< Charles Lohr, under the MIT-X11 license or NewBSD
// license, you choose.
//
// Note: Before running this, you should run the optionbytes folder script
// in order configure RESET as a GPIO so you can use the AUX input as well
// as forcing the watchdog on by default. 
//

#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>

static uint16_t GenOnMask( int segmenton );
static void ApplyOnMask( uint16_t onmask );
static inline void WatchdogPet();

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
int pwm_max_duty = 48;  //This is changed based on vdd.


// Flyback PID loop Tuning Parameters
//
// This is a PID loop (you should read about this separately). But these terms
// are actually on the order of 2^term. So we are only able to get the tuning
// to a rough ballpark. Thankfully, PID loops are forgiving.
//
// Honestly, this is MUCH more sophisticated than it needs to be!  I was using
// a P-only loop for quite some time without any issues.
#define ERROR_P_TERM 2  // Actually 2^2
#define ERROR_D_TERM -1  // Actually 2^0
#define I_SAT_MAX (4096+2048) // SAT * 2^(-ADC_IIR+ERROR_I_TERM) = Max impact to PWM
#define I_SAT_MIN (-4096)
#define ERROR_I_TERM -5 // Actually 2^-6

// We filter our ADC inputs because they are kind of noisy.
//
// We can use Binary-shift IIR filters to filter the incoming ADC signals.
// See later in the code, but, it maps to only about 4 assembly instructions!
// (plus a read-back of the previous value we will be mixing).
#define ADC_IIR 2
#define VDD_IIR 2

// Target feedback, set by the user.
int target_feedback = 0;

// Feedback based on what the user set and the part's VDD.
int feedback_vdd = 0;

// Filtered ADC and VDD values.
int lastadc = 0;
int lastrefvdd = 0;

// Code for handling numeric fading, between 2 numbers or alone.

uint16_t fade_time0, fade_time1;
uint16_t fade_disp0, fade_disp1;

static uint32_t HandleFade( uint8_t fadepos )  __attribute__((section(".srodata")));
static uint32_t HandleFade( uint8_t fadepos )
{
	// Digit fade.  Use fade_timeX and fade_dispX to handle fade logic.
	if( fadepos < fade_time0 )
		return fade_disp0;
	else if( fadepos < fade_time1 ) 
		return fade_disp1;
	else
		return 0;
}

static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num );
static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num )
{
	// The CH32V003 is an EC core, so no hardware multiply. GCC's way multiply
	// is slow, so I wrote this.
	//
	// This basically does this:
	//	return small_num * big_num;
	//
	// Note: This does NOT check for zero to begin with, though this still
	// produces the correct results, it is a little weird that even if
	// small_num is zero it executes once.
	//
	// Additionally note, instead of the if( m&1 ) you can do the following:
	//  ret += multiplciant & neg(multiplicand & 1).
	//
	// BUT! Shockingly! That is slower than an extra branch! The CH32V003
	//  can branch unbelievably fast.
	//
	// This is functionally equivelent and much faster.
	//
	// Perf numbers, with small_num set to 180V.
	//  No multiply:         21.3% CPU Usage
	//  Assembly below:      42.4% CPU Usage  (1608 bytes for whole program)
	//  C version:           41.4% CPU Usage  (1600 bytes for whole program)
	//  Using GCC (__mulsi3) 65.4% CPU Usage  (1652 bytes for whole program)
	//
	// The multiply can be done manually:
	uint32_t ret = 0;
	uint32_t multiplicand = small_num;
	uint32_t mutliplicant = big_num;
	do
	{
		if( multiplicand & 1 )
			ret += mutliplicant;
		mutliplicant<<=1;
		multiplicand>>=1;
	} while( multiplicand );
	return ret;

	// Which is equivelent to the following assembly (If you were curious)
/*
	uint32_t ret = 0;
	asm volatile( "\n\
		.option   rvc;\n\
	1:	andi t0, %[small], 1\n\
		beqz t0, 2f\n\
		add %[ret], %[ret], %[big]\n\
	2:	srli %[small], %[small], 1\n\
		slli %[big], %[big], 1\n\
		bnez %[small], 1b\n\
	" :
		[ret]"=&r"(ret), [big]"+&r"(big_num), [small]"+&r"(small_num) : :
		"t0" );
	return ret;
*/
}

// FYI You can use functions in ram to make them work faster.  The .srodata
// attribute. This means this function gets placed into RAM. Normally this
// function takes approximately 2.5-3us to execute from flash, but only 2-2.5us
// to execute from RAM.

// This is an interrupt called by an ADC conversion.
void ADC1_IRQHandler(void)
	__attribute__((interrupt))
	__attribute__((section(".srodata")));

void ADC1_IRQHandler(void)
{
	// If you want to see how long this functon takes to run, you can use a
	// scope and then monitor pin D6 if you uncomment this and the bottom copy.
	GPIOD->BSHR = 1<<6;

	// Acknowledge pending interrupts.
	// This will always be ADC_JEOC, so we don't need to check.
	ADC1->STATR = 0;

	// Acknowledge the interrupt.
	
	// It is crucial that our ADC sample is ALWAYS ALIGNED to the PWM, that way
	// any ripple and craziness that happens from the chopping of the flyback
	// is completely filtered out because of where we are sampling.

	// This performs a low-pass filter on our data, ADC1->RDATAR the sample
	// rate, always.  As a side note, the value of the IIR (but now it's
	// 2^VDD_IIR bigger)

	int adcraw = ADC1->RDATAR;
	int newadc = adcraw + (lastadc - (lastadc>>ADC_IIR));
	lastadc = newadc;

	int err = feedback_vdd - lastadc;

	static int integral;
	static int lasterr;
	int derivative = (err - lasterr);
	lasterr = err;
	integral += err;

	// We asymmetrically allow the integral to saturate, to help prevent long-
	// term oscillations.
	integral = ( integral > ((I_SAT_MAX)<<ADC_IIR) ) ? ((I_SAT_MAX)<<ADC_IIR) : integral;
	integral = ( integral < ((I_SAT_MIN)<<ADC_IIR) ) ? ((I_SAT_MIN)<<ADC_IIR) : integral;

	// This is the heart of the PID loop.
	// General note about shifting: Be sure to combine your shifts.
	// If you shift right, then left, you will lose bits of precision.
	// These shifts turn into single left and right immediate shifts.
	int plant = 
		(err << ( (-ADC_IIR) + (ERROR_P_TERM) )) +
		(integral >> ( ADC_IIR - (ERROR_I_TERM) )) +
		(derivative >> ( (ADC_IIR) - (ERROR_D_TERM) ) );
	plant = ( plant > pwm_max_duty ) ? pwm_max_duty : plant;
	plant = ( plant < 0 ) ? 0 : plant;
	TIM1->CH2CVR = plant;

	// Use injection channel data to read vref.  This is needed because we
	// measure all values WRT to VDD and GND.  So we need to measure the vref
	// a lot to make sure we know what value we are are targeting Ballparks
	//   (for unfiltered numbers)  (Just as a note)
	//   0xF0  / 240 for 5V input << lastrefvdd
	//	 0x175 / 373 for 3.3v input << lastrefvdd

	// Do an IIR low-pass filter on VDD. See IIR discussion above.
	uint32_t vdd = lastrefvdd = ADC1->IDATAR1 + (lastrefvdd - (lastrefvdd>>VDD_IIR));

#ifndef ENABLE_TUNING

	// If we aren't enabling tuning, we can update max on time here. We want to
	// limit on-time based on DC voltage on the flyback so that we can get
	// close to (But not get into) saturation of the flyback transformer's core
	//
	// We can compute expected values, but experimenting is better.
	// Transformer inductance is ~6uH.
	// Our peak current is ~500mA
	// The average voltage is ~4V
	//
	//   4V / .000006H = 0.5A / 666666A/s = 750nS but turns out this was
	//    pessemistic.
	//
	// Experimentation showed that the core of the transformer saturates in
	// about 1uS at 5V and 1.4uS at 3.3v.  More specifically the relationhip
	// between our maximum on-time and vref-measured-by-vdd works out to about:
	//
	//    max_on_time_slices = lastrefvdd / 4.44.
	//
	// There's a neat trick where you can divide by weird decimal divisors by
	// adding and subtracing terms. We perform this trick here and below
	//
	// 1÷(1÷4−1÷64−1÷128−1÷1024) is about equal to dividing by 4.43290
	//  It can be simplified it for our purposes as: 1÷(1÷4−1÷64−1÷128)
	//
	// You can arbitrarily add and subtract terms to get as closed to your
	// desired target value as possbile.
	//
	// When we divide a value by powers-of-two, it becomes a bit shift.
	//
	// The bit shift and IIR adjustments can be made so that the compiler can
	// optimize out the addition there.
	//
	// The following code actually
	pwm_max_duty = 
		  (lastrefvdd>>(2+VDD_IIR)) 
		- (lastrefvdd>>(6+VDD_IIR))
		- (lastrefvdd>>(7+VDD_IIR));
#endif

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
	//  More tests showed this value across units is around 120.
	//
	// X This becomes our denominator.
	// feedback_vdd = (current vdd measurement * target voltage) / 120
	// 
	// Further testing identified that the denominator is almost exactly 120.
	// We can perform a divison by 120 very quickly by
	//
	//   feedback = numerator/128 + numerator/2048
	//
	// See note above about the constant division trick.
	//
	// Side-note:
	//
	// This is unintuitively slow becuase is because it uses a multiply. The
	// CH32V003 does not natively have a multiply instruction, If you use a *
	// it calls out to __mulsi3 in libgcc.a. 

	// The following line of code is still *more* than fast enough but, to
	// write it out manually, we can get even faster!
	//
	//	uint32_t numerator = (vdd * target_feedback);

	uint32_t numerator = FastMultiply( vdd, target_feedback );

	feedback_vdd =
		(numerator>>(7+VDD_IIR-ADC_IIR)) +
		(numerator>>(11+VDD_IIR-ADC_IIR));

	// Pet the watchdog.  If we got here, things should be OK.
	WatchdogPet();

	GPIOD->BSHR = (1<<(16+6));
}

static void SetupTimer1()
{
	// Enable Timer 1
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	TIM1->PSC = 0x0000;  // Prescalar to 0x0000 (so, 48MHz base clock)
	TIM1->ATRLR = PWM_PERIOD;
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

static void SetupTimer2()
{
	// Enable Timer 2
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// PD7 will be used as Timer 2, Channel 4 Configure timer 2 to enable this.
	//TIM2->SWEVGR = 0x0001; 			//TIM_PSCReloadMode_Immediate;

	TIM2->PSC = 0x0020;
	TIM2->ATRLR = 255;				// 0..255 (So we can be 100% on)
	TIM2->CHCTLR2 = TIM_OC4M_2 | TIM_OC4M_1;
	TIM2->CCER = TIM_CC4E;
	TIM2->CH4CVR = 0;  			// Actual duty cycle (Off to begin with)

	// Enable TIM1 outputs
	TIM2->BDTR = TIM_MOE;
	TIM2->CTLR1 = TIM_CEN;
}

static void SetupADC()
{
	// Configure ADC.
	// PD4 is analog input chl 7
	GPIOD->CFGLR &= ~(0xf<<(4*4));	// CNF = 00: Analog, MODE = 00: Input
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// ADCCLK = 12 MHz => RCC_ADCPRE divide by 4
	RCC->CFGR0 &= ~RCC_ADCPRE;  // Clear out the bis in case they were set
	RCC->CFGR0 |= RCC_ADCPRE_DIV4;	// set it to 010xx for /4.

	// Set up single conversion on chl 7
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 7;	// 0-9 for 8 ext inputs and two internals
	
	//Injection group is 8. NOTE: See note in 9.3.12 (ADC_ISQR) of TRM. The
	// group numbers is actually 4-group numbers.
	ADC1->ISQR = (8<<15) | (0<<20);

	// Sampling time for channels. Careful: This has PID tuning implications.
	// Note that with 3 and 3,the full loop (and injection) runs at 138kHz.
	ADC1->SAMPTR2 = (3<<(3*7)) | (3<<(3*8)); 
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

	// ADC_JEOCIE: Enable the End-of-conversion interrupt.
	// ADC_JDISCEN | ADC_JAUTO: Force injection after rule conversion.
	// ADC_SCAN: Allow scanning.
	ADC1->CTLR1 = ADC_JEOCIE | ADC_JDISCEN | ADC_SCAN | ADC_JAUTO;
}

// Apply a given output mask to the GPIO ports the nixie tubes are hooked into.
static void ApplyOnMask( uint16_t onmask )
{
	GPIOD->OUTDR = (onmask >> 8) | 0x80;
	GPIOC->OUTDR = onmask & 0xff;
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

		fade_time0 = -1;
		fade_time1 = -1;
		fade_disp0 = GenOnMask(segmenton);
		fade_disp1 = 0;
		break;
	}
	case 3:
	{
		// Configure a fade.
		fade_disp0 = GenOnMask( ( dmdword >> 8 ) & 0xf );
		fade_disp1 = GenOnMask( ( dmdword >> 12 ) & 0xf );
		fade_time0 = ( dmdword >> 16 ) & 0xff;
		fade_time1 = ( dmdword >> 24 ) & 0xff;
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
			pwm_max_duty = max_duty;
		}
#endif
		break;
	}
	case 5:
	{
		// Aux Neon Control
		TIM2->CH4CVR = dmdword>>16;
	}

	}

	// Write the status back to the host PC.  Status is our VDD and our FB V
	*DMDATA0 = ((lastadc>>ADC_IIR) << 12) | ((lastrefvdd>>VDD_IIR) << 22);
}

static inline void WatchdogPet()
{
	// Writing 0xaaaa into the ctlr prevents the watchdog from killing us.
	IWDG->CTLR = 0xAAAA;
}

static inline void WatchdogSetup()
{
	// Setup watchdog.
	IWDG->CTLR = 0x5555; // Go into watchdog setup mode.
	while( IWDG->STATR & IWDG_PVU ); // Wait for PSCR to become recepitve.
	IWDG->PSCR = 1;      // div LSI by 8 (4 seems unreliable)
	IWDG->RLDR = 0xFFF;  // reload watchdog, not important don't need to check.
	IWDG->CTLR = 0xCCCC; // commit registers.
	WatchdogPet();
}

static inline void AdvanceFadePlace()
{
	static uint32_t lastmask = 0;

	// Causes us to cycle through all 256 sequence points every 1.5ms.
	uint32_t fadepos = (SysTick->CNT >> 5) & 0xff;

	// We want to glow the LEDs with a chopping period of less, so we
	// "rotate" the bits.  This has the effect of making the primary
	// switching frequency for the tubes much higher, but, at the same time
	// also jittering the edges in time so you can get a full 8-bit fade.
	// You can rotate more or less to control the periodicity.

	// With this scramble, the period scramble is about 93us.
	fadepos = (fadepos << 4) | ( fadepos >> 4);

	uint32_t mask = HandleFade( fadepos );
	if( mask != lastmask )
	{
		if( lastmask )
		{
			// Make sure we have a short gap with nothing on.
			ApplyOnMask( 0 );
			Delay_Us( 3 );
		}
		ApplyOnMask( mask );
		lastmask = mask;
	}
}

int main()
{
	// Configure a watchdog timer so if the chip goes crazy it will reset.
	WatchdogSetup();

	// Use internall RC oscillator + 2xPLL to generate 48 MHz system clock.
	SystemInit48HSI();

	// For the ability to printf() if we want.
	SetupDebugPrintf();

	// Pet watchdog for the rest of startup.
	WatchdogPet();

	// Enable Peripherals
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
		RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 |
		RCC_APB2Periph_AFIO;

	RCC->APB1PCENR = RCC_APB1Periph_TIM2;

	// I'm paranoid - let's make sure all tube cathodes are high-Z.
	ApplyOnMask( 0 );

	GPIOD->CFGLR = 
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6) | // GPIO D6 Debug
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*7) | // DIG_AUX  (TIM2CH4)
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3) | // DIG_9
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*2) | // DIG_8
		(GPIO_Speed_10MHz | GPIO_CNF_IN_FLOATING)<<(4*1) | // PGM Floats.
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);  // DIG_DOT

	GPIOC->CFGLR = 
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0) | // DIG_0
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*1) | // DIG_1
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*2) | // DIG_2
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3) | // DIG_3
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4) | // DIG_4
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*5) | // DIG_5
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*6) | // DIG_6
		(GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*7);  // DIG_7


	GPIOA->CFGLR =
		(GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF)<<(4*1); //FLYBACK (T1CH2)

	SetupADC();
	SetupTimer1();
	SetupTimer2();

	*DMDATA0 = 0;

	target_feedback = 0;

	// Cause system timer to run and reload when it hits CMP and HCLK/8.
	// Also, don't stop at comparison value.
	SysTick->CTLR = 1;

	while(1)
	{
		uint32_t dmdword = *DMDATA0;
		if( (dmdword & 0xf0) == 0x40 )
		{
			// I think there is a compiler bug here.  For some reason if I put
			// the code in this function right here, it doesn't work right.
			// so I encapsulated the code in a function.
			//
			// This function handles commands we get over the programming
			// interface.  Like "set HV bus" or "set this digit on."
			HandleCommand( dmdword );
		}

		AdvanceFadePlace();

	}
}


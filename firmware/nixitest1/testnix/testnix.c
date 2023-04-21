#include <stdio.h>

#define CNFG_IMPLEMENTATION
#include "rawdraw_sf.h"

#include "../../ch32v003fun/minichlink/minichlink.h"

int targetnum = 0;

#define VOLTAGE_SCALE 2.01

const char * targdisp[] = { " ", "0", "9", "8", "7", "6", "5", "4", "3", "2", "1", ".", "N" };
void HandleKey( int keycode, int bDown )
{
	if( bDown )
	{
	switch( keycode )
	{
		case '~': case '`': targetnum = 0; break;
		case '1': targetnum = 10; break;
		case '2': targetnum = 9; break;
		case '3': targetnum = 8; break;
		case '4': targetnum = 7; break;
		case '5': targetnum = 6; break;
		case '6': targetnum = 5; break;
		case '7': targetnum = 4; break;
		case '8': targetnum = 3; break;
		case '9': targetnum = 2; break;
		case '0': targetnum = 1; break;
		case '-': case '_': targetnum = 11; break;
		case '=': case '+': targetnum = 12; break;
	}
	}
}

int do_set = 0;
int sety = 0;
void HandleButton( int x, int y, int button, int bDown ) { }
void HandleMotion( int x, int y, int mask ) { sety = y; do_set = mask; }
void HandleDestroy() { }

#define VOLTHISTSIZE 2048
float volthist[VOLTHISTSIZE];
int volthisthead = 0;

int main()
{
	void * dev = TryInit_ESP32S2CHFUN();
	if( !dev )
	{
		fprintf( stderr, "Error: Couldn't find programmer\n" );
		return -9;
	}
	SetupAutomaticHighLevelFunctions( dev );
	if( MCF.SetupInterface( dev ) )
	{
		fprintf( stderr, "Error: failed to setup\n" );
		return -9;
	}
	uint32_t rmask = 0x17000040;

	printf( "DEV: %p\n", dev );
	CNFGSetup( "nixitest1 debug app", 640, 480 );
	while(CNFGHandleInput())
	{
		const uint32_t GLOW = 0xFFD010FF;
		short w, h;
		int x, y;
		CNFGClearFrame();
		CNFGGetDimensions( &w, &h );

		if( do_set )
		{
			do_set = 0;
			float set_v = 450 - sety;
			set_v = set_v/2;
			if( set_v > 0 && set_v < 180 )
			{
				rmask = ( ( (uint32_t)(set_v * VOLTAGE_SCALE) ) << 20 ) | 0x40;
			}
		}
		MCF.WriteReg32( dev, 0x04, rmask | targetnum << 16 );
		uint32_t status = 0xffffffff;
		int r = MCF.ReadReg32( dev, 0x04, &status );
		float voltage = ((float)(status>>16))/VOLTAGE_SCALE;
		volthist[volthisthead] = voltage;
		volthisthead = (volthisthead + 1) % VOLTHISTSIZE;
		CNFGColor( (voltage > 180)?0xff0000ff:GLOW ); 
		CNFGPenX = 1;
		CNFGPenY = 1;
		char cts[128];
		sprintf( cts, "HV Line: %3.0f V\nRStatus: %d", voltage, r );
		CNFGDrawText( cts, 4 );	

		for( y = 0; y < 2; y++ ) for( x = 0; x < 2; x++ )
		{
			CNFGPenX = 200+x;
			CNFGPenY = 1+y;
			CNFGDrawText( targdisp[targetnum], 10 );
		}

		int i;
		int vhp = (volthisthead - 1 + VOLTHISTSIZE*100)%VOLTHISTSIZE;
		float vl = voltage;

		CNFGColor( 0xff0000ff );
		CNFGTackSegment( 0, 450-180*2, w, 450-180*2 );
		CNFGPenX = w - 250; CNFGPenY = 450-180*2-10;
		CNFGDrawText( "WARNING: DO NOT EXCEED THIS LINE (180V)", 2 );

		for( i = 0; i < 9; i++ )
		{
			CNFGColor( (i == 0 )?0xD0D0D0FF:0x303030ff );
			CNFGPenX = 1;
			CNFGPenY = 450 - 10 - i * 40;
			sprintf( cts, "%d volts", i * 20 );
			CNFGDrawText( cts, 2 );
			CNFGTackSegment( 0,450 - i * 40, w, 450 - i * 40 );
			CNFGColor( GLOW ); 
		}


		for( i = 0; i < w; i++ )
		{
			float v = volthist[vhp];
			if( v == 0 ) break;
			CNFGTackSegment( i, 450 - vl*2, i+1, 450 - v*2 );
			vhp = (vhp - 1 + VOLTHISTSIZE*100)%VOLTHISTSIZE;
			//printf( "%f\n", v );
			vl = v;
		}

		CNFGSwapBuffers();
	} while( 1 );

	return 0;
}


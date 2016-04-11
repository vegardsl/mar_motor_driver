/*
 * conf_timeout.h
 *
 * Created: 11.04.2016 11:27:15
 *  Author: vegarsl
 */ 

/*
*This example runs at default 2MHz system clock
*The prescaler for TC is 64
*Thus, 2000000/64=31250
*/
#define	TIMER_EXAMPLE_PERIOD 31250
#define TIMER_PORT_LED       TCE0
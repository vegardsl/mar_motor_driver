/**
 * \file
 *
 * \brief USB Standard I/O (stdio) Example
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This example demonstrates how to configure a C-library Standard
 * I/O interface to the ASF common USB Device CDC service. The initialization
 * routines, along with board and clock configuration constants, illustrate how
 * to perform serial I/O via a Communication Device Class (CDC) device protocol
 *
 * \section files Main Files
 * - stdio_usb_example.c: the example application.
 * - conf_board.h: board configuration
 * - conf_clock.h: board configuration
 * - stdio_usb.h: Common USB CDC Standard I/O Implementation
 * - read.c : System implementation function used by standard library
 * - write.c : System implementation function used by standard library
 *
 * \section example_description Description of the example
 *   - Send message on USB CDC device to a Virtual Com Port.
 *   - Performs echo of any received character
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

//#include <asf.h>

#include <board.h>
#include <sysclk.h>
#include <stdio_usb.h>
#include "avr/wdt.h"
#include "ioport.h"
#include "compiler.h"
#include "preprocessor.h"

enum Receiver_State
{
	idle=0,
	after_message,
	receiving			
};

enum Sign
{
	positive=0,
	negative
};


void init_pwm_motor_driver()
{
	PORTC.DIR = 0b00001111;

	//sysclk ting http://asf.atmel.com/docs/2.11.1/xmegaa/html/group__sysclk__group.html#gga8e29b46d7670875f4c509efd7a8d5f1aadc3231d4c5c8161f3c6c3cb8d8a87bff

	TCC0.CTRLA |=(PIN2_bm); //enables compare, with 8 as a prescaler
	TCC0.CTRLB |=(PIN5_bm)|(PIN4_bm)|(PIN1_bm)|(PIN0_bm);

	TCC0.PER = 2000; //TCC0.PER in CCA and CCB is 100% pwm, 0 is 0% pwm

	TCC0.CCB = 0;
	TCC0.CCA = 0;
	
	PORTC.OUT |= (1<<2);
	PORTC.OUT |= (1<<3);
}

enum Sign get_sign(int16_t value)
{
	enum Sign sign;
	if(value < 0)
	{
		sign = negative;
	}
	else
	{
		sign = positive;
	}
	return sign;
}

int16_t calculate_left_wheel_speed(	int16_t linear_setting, 
									int16_t angular_setting, 
									int16_t wheel_radius,
									int16_t diff_shaft_len)
{
	int16_t numerator = (2*linear_setting - angular_setting*diff_shaft_len);
	int16_t denominator = 2*wheel_radius;
	int16_t speed = numerator/denominator;
	return speed;
}

int16_t calculate_right_wheel_speed(int16_t linear_setting,
									int16_t angular_setting,
									int16_t wheel_radius,
									int16_t diff_shaft_len)
{
	int16_t numerator = (2*linear_setting + angular_setting*diff_shaft_len);
	int16_t denominator = 2*wheel_radius;
	int16_t speed = numerator/denominator;
	return speed;
}

void left_set_wheel_speed(int16_t speed)
{
	enum Sign sign = get_sign(speed);
	uint16_t abs_speed = (speed >= 0 ? speed:-speed);
	
	printf("Speed: %d, Sign: %d\n\r",abs_speed,sign);
	
	if(sign == positive)
	{
		PORTC.OUT &= ~(1<<3); // Set directional pin to indicate FORWARD.
		printf("FORWARD\n\r");
	}
	else if (sign == negative)
	{
		PORTC.OUT |= (1<<3); // Set directional pin to indicate REVERSE.
		printf("REVERSE\n\r");
	}
	TCC0.CCB = abs_speed;
}

void right_set_wheel_speed(int16_t speed)
{
	enum Sign sign = get_sign(speed);
	uint16_t abs_speed = (speed >= 0 ? speed:-speed);
	
	printf("Speed: %d, Sign: %d\n\r",abs_speed,sign);
	
	if(sign == positive)
	{
		PORTC.OUT &= ~(1<<2); // Set directional pin to indicate FORWARD.
		printf("FORWARD\n\r");
	}
	else if (sign == negative)
	{
		PORTC.OUT |= (1<<2); // Set directional pin to indicate REVERSE.
		printf("REVERSE\n\r");
	}
	TCC0.CCA = abs_speed;
}

/**
 * \brief main function
 */

int main (void)
{
	/* Initialize basic board support features.
	 * - Initialize system clock sources according to device-specific
	 *   configuration parameters supplied in a conf_clock.h file.
	 * - Set up GPIO and board-specific features using additional configuration
	 *   parameters, if any, specified in a conf_board.h file.
	 */
	
	
	sysclk_init();
	board_init();

	// Initialize interrupt vector table support.
	irq_initialize_vectors();

	// Enable interrupts
	cpu_irq_enable();

	/* Call a local utility routine to initialize C-Library Standard I/O over
	 * a USB CDC protocol. Tunable parameters in a conf_usb.h file must be
	 * supplied to configure the USB device correctly.
	 */
	
	
	stdio_usb_init();

	init_pwm_motor_driver();
	//left_set_wheel_speed(0);
	//right_set_wheel_speed(0);

	uint8_t ch;
	
	enum Receiver_State rx_state = idle;
	
	int16_t left_speed_setting = 600;
	int16_t right_speed_setting = -600;
	
	

	while (true) {		
		switch(rx_state){
			case receiving:
				TCC0.CCA = 600;
			    //left_set_wheel_speed(left_speed_setting);
			    right_set_wheel_speed(right_speed_setting);
				scanf("%c",&ch);
				printf("Echo: %c\n\r",ch); // echo to output
				if (ch == 115) // if "s"
				{
					// Set linear speed;
					scanf("%c",&ch);
					printf("Echo: %c\n",ch); // echo to output
					scanf("%c",&ch);
					printf("Echo: %c\n",ch); // echo to output
				}
				else if (ch == 97) // if "a"
				{
					// Set angular speed;
					scanf("%c",&ch);
					printf("Echo: %c\n",ch); // echo to output
					scanf("%c",&ch);
					printf("Echo: %c\n",ch); // echo to output
				}
				else if (ch == 27) // if "Esc"
				{
					rx_state = after_message;
				}
				//break;
				
			case after_message:
				// Reset watchdog here.
				// Update commands here.
				//left_set_wheel_speed(100);
				//right_set_wheel_speed(100);
				rx_state = idle;
				printf("e\n\r"); // Confirm out of receive mode. 
				//break;
				
			case idle:
				scanf("%c",&ch); // get one input character
				printf("Echo: %c\n\r",ch); // echo to output
				if (ch == 58) { // if ":"
					printf("r\n\r"); // Confirm in receive mode
					rx_state = receiving;
				}
				//break;
		}
		
		
		printf("In loop\n");
	}
}

	
bool setDir (bool);
uint16_t setSpeed (uint16_t, char);
uint16_t directSetSpeed ();
	
int main (void)
{
	/* Initialize basic board support features.
	 * - Initialize system clock sources according to device-specific
	 *   configuration parameters supplied in a conf_clock.h file.
	 * - Set up GPIO and board-specific features using additional configuration
	 *   parameters, if any, specified in a conf_board.h file.
	 */
	sysclk_init();
	board_init();

	// Initialize interrupt vector table support.
	irq_initialize_vectors();
	
	// Enable interrupts
	cpu_irq_enable();

	/* Call a local utility routine to initialize C-Library Standard I/O over
	 * a USB CDC protocol. Tunable parameters in a conf_usb.h file must be
	 * supplied to configure the USB device correctly.
	 */
	stdio_usb_init();

	// Get and echo characters forever.


	//set port C 0..3 to output, PC0 and PC1 for pwm and PC2 and PC3 for directional
	PORTC.DIR = 0b00001111;

	//sysclk ting http://asf.atmel.com/docs/2.11.1/xmegaa/html/group__sysclk__group.html#gga8e29b46d7670875f4c509efd7a8d5f1aadc3231d4c5c8161f3c6c3cb8d8a87bff

	TCC0.CTRLA |=(PIN2_bm); //enables compare, with 8 as a prescaler
	TCC0.CTRLB |=(PIN5_bm)|(PIN4_bm)|(PIN1_bm)|(PIN0_bm);

	TCC0.PER = 2000; //TCC0.PER in CCA and CCB is 100% pwm, 0 is 0% pwm

	TCC0.CCB = 0;
	TCC0.CCA = 0;

	uint8_t ch;
	uint16_t hVal;
	uint16_t vVal;
	bool hDir;
	bool vDir;
	bool first = true;

	hVal = 0;
	vVal = 0;
	hDir = false;
	vDir = false;
	
	while (true) {
		
		//scanf("%c",&ch); // get one input character
		//printf("echo: %c",ch);
		
    
		scanf("%c",&ch); // get one input character
	
		if (ch==104)
		{
			if (first)
			{
				//wdt_enable(WDT_PER_1KCLK_gc); //enable watchdog
				first = false;
			}
			//wdt_reset();
			printf("%c",ch);
			
			hDir = setDir(hDir);
			//sets the directional pin high or low according to hDir
			if (hDir)
			{
				PORTC.OUT &= ~(1<<2);
			}
			else if (hDir==false){
				PORTC.OUT |= (1<<2);
			}
			
			
			scanf("%c",&ch);
			
			if (ch == 46)
			{
				TCC0.CCA = directSetSpeed ();
			}
		
			else
			{
				hVal = setSpeed(hVal, ch);
				TCC0.CCA = hVal;
			}
		printf(";");
		}
	
		else if (ch==118)
		{
			if (first)
			{
				wdt_enable(WDT_PER_1KCLK_gc); //enable watchdog
				first = false;
			}
			wdt_reset();
			printf("%c",ch);
			vDir = setDir(vDir);
			//sets the directional pin high or low according to vDir
			if (vDir)
			{
				PORTC.OUT &= ~(1<<3);
			}
			else if (vDir==false){
				PORTC.OUT |= (1<<3);
			}
			
			
			scanf("%c",&ch);
		
			if (ch == 46)
			{
				printf("%c",ch);
				TCC0.CCB = directSetSpeed ();
			}
			else
			{
				vVal = setSpeed(vVal, ch);
				TCC0.CCB = vVal;
			}
			printf(";");
		}
	
		else if (ch)
		{
			printf("%c",ch); // echo to output
			printf(";");
		}
	}
	
}

bool setDir (bool start){
	bool dir;
	uint8_t ch;
	
	scanf("%c",&ch);
	
	if(ch==43){
		dir = false;
		printf("%c",ch);
	}
	else if(ch==45){
		dir = true;
		printf("%c",ch);
	}
	else{
		dir = start;
		//printf(" + or - for direction ");
	}
	return dir;
}

uint16_t directSetSpeed ()
{
	uint16_t ut = 0;
	uint8_t ch;
	scanf("%c",&ch);
	ut = (ch - 48)*1000;
	printf("%c",ch);
	
	scanf("%c",&ch);
	ut = ut + (ch - 48)*100;
	printf("%c",ch);
	
	scanf("%c",&ch);
	ut = ut + (ch - 48)*10;
	printf("%c",ch);
	
	scanf("%c",&ch);
	ut = ut + (ch - 48);
	printf("%c",ch);
	return ut;
}

uint16_t setSpeed (uint16_t start, char ch){
	uint16_t speed;

	speed = start;
	switch (ch)
	{
		case 48:
		speed = 0;
		printf("%c",ch);
		break;
		
		case 49:
		speed = 200;
		printf("%c",ch);
		break;
		
		case 50:
		speed = 400;
		printf("%c",ch);
		break;
		
		case 51:
		speed = 600;
		printf("%c",ch);
		break;
		
		case 52:
		speed = 800;
		printf("%c",ch);
		break;
		
		case 53:
		speed = 1000;
		printf("%c",ch);
		break;
		
		case 54:
		speed = 1200;
		printf("%c",ch);
		break;
		
		case 55:
		speed = 1400;
		printf("%c",ch);
		break;
		
		case 56:
		speed = 1600;
		printf("%c",ch);
		break;
		
		case 57:
		speed = 1800;
		printf("%c",ch);
		break;
	}
	return speed;
}

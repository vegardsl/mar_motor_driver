/**
 * \file
 *
 * \brief USB Standard I/O (stdio) Example
 *
 * Copyright (c) 2011 - 2012 Atmel Corporation. All rights reserved.
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
 * \main page
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

const int16_t wheel_radius = 5; // 5 cm
const int16_t diff_shaft_len = 44; // 44 cm

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
									int16_t angular_setting)
{
	int16_t tmp = (angular_setting*diff_shaft_len)/100;
	int16_t numerator = (2*linear_setting - tmp);
	int16_t denominator = 2;//*wheel_radius;
	int16_t speed = numerator/denominator;
	return speed;
}

int16_t calculate_right_wheel_speed(int16_t linear_setting,
									int16_t angular_setting)
{
	int16_t tmp = (angular_setting*diff_shaft_len)/100;
	int16_t numerator = (2*linear_setting + tmp);
	int16_t denominator = 2;//*wheel_radius;
	int16_t speed = numerator/denominator;
	return speed;
}

uint16_t speedToMotorSetting(uint16_t speed)
{
	uint16_t motorSetting;
	printf("speed: %d\n\r",speed);
	if(speed < 1)
	{
		return 0;	// This will prevent an invalid negative number in the unsigned int.
	}
	//motorSetting = ( (1250*speed) - 75)/100; // This will overflow when speen exeeds 40;
	motorSetting = (125*speed - 7)/10; // An approximation to the above, to prevent overflow;
	
	printf("motorSetting: %d\n\r",motorSetting);
	return motorSetting;
}

void left_set_wheel_speed(int16_t speed)
{
	enum Sign sign = get_sign(speed);
	uint16_t abs_speed = (speed >= 0 ? speed:-speed);
	
	printf("L Speed: %d, Sign: %d\n\r",abs_speed,sign);
	
	if(sign == negative)
	{
		PORTC.OUT &= ~(1<<3); // Set directional pin to indicate REVERSE.
		//printf("LREVERSE\n\r");
	}
	else if (sign == positive)
	{
		PORTC.OUT |= (1<<3); // Set directional pin to indicate FORWARD.
		//printf("LFORWARD\n\r");
	}
	
	uint16_t motorSetting = speedToMotorSetting(abs_speed);
	TCC0.CCB = motorSetting;
}

void right_set_wheel_speed(int16_t speed)
{
	enum Sign sign = get_sign(speed);
	uint16_t abs_speed = (speed >= 0 ? speed:-speed);
	
	printf("R Speed: %d, Sign: %d\n\r",abs_speed,sign);
	
	if(sign == negative)
	{
		PORTC.OUT &= ~(1<<2); // Set directional pin to indicate REVERSE.
		//printf("RFORWARD\n\r");
	}
	else if (sign == positive)
	{
		PORTC.OUT |= (1<<2); // Set directional pin to indicate FORWARD.
		//printf("RREVERSE\n\r");
	}
	uint16_t motorSetting = speedToMotorSetting(abs_speed);
	TCC0.CCA = motorSetting;
}

int16_t charToMotorSetting(uint8_t input, uint8_t val_sign)
{
	int16_t result;
	
	result = ( (0x00 << 8) | (input) & 0xff );
	
	if (val_sign == 45)
	{
		result = ~result;
	}
	printf("%d\n",result);
	return result;
}

/************************************************************************/
/* Description: Speed commands from ROS are sent as an int32 which is 
/* split into four chars. XMEGA A3BU uses int16_t. This function 
/* performs the conversion from the four chars to int16_t.
/* Assumes that ch1 is the most significant byte. 
/*                                                                      */
/************************************************************************/
int16_t fourCharsToInt(uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4)
{
	
	printf("Chars Before: %d %d %d %d \n\r",ch1,ch2,ch3,ch4);
	
	if ((ch1)&(1 << 7)) // Is MSB set? If so, the sign is negative.
	{
		printf("NEG\n");
		// Bit 7 in ch3 is the new MSB. Set it to indicate negative sign.
		ch4 |= 1 << 7; 
	}
	else
	{ // Otherwise, make sure the new MSB is 0 to indicate positive sign.
		printf("POS\n");
		ch4 &= ~(1 << 7); 
	}
	int16_t result;
	result = ( (ch3 << 8) | (ch4) & 0xff );
	
	printf("Chars AFTER: %d\n %d\n %d\n %d\n \n\r",ch1,ch2,ch3,ch4);
	
	return result;
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
	left_set_wheel_speed(0);
	right_set_wheel_speed(0);

	uint8_t ch;
	
	enum Receiver_State rx_state = idle;
	
	int16_t left_speed_setting = 0;
	int16_t right_speed_setting = 0;
	
	int16_t linear_speed_setting = 0;
	int16_t angular_speed_setting = 0;
	
	uint8_t velocity_command[4];
	

	while (true) {
		/* The state machine below handles incoming commands via the serial port. 
		* Messages are received and read one char at the time. The message 
		* format is as follows:
		* - BEGINNING OF MESSAGE: ascii value for ":"
		* - BEGINNING OF LINEAR SPEED: ascii value for "s"
		* - +/-, LINEAR SPEED: a char with the ascii value for + or -
		* - LINEAR SPEED: Speed setting, a 0 to 127 value.
		* - BEGINNING OF ANGULAR SPEED: ascii value for "a"
		* - +/-, ANGULAR SPEED: a char with the ascii value for + or -
		* - ANGULAR SPEED: Speed setting, a 0 to 127 value.
		* - END OF MESSAGE: ascii value for "ESC"
		*/
		switch(rx_state){
			case receiving:
				printf("Receiving.\n\r");
				scanf("%c",&ch);
				printf("Echo: %c\n\r",ch); // echo to output
				if (ch == 115) // if "s"
				{
					int16_t result;
					uint8_t new_setting, val_sign;
					// Set linear speed;
					scanf("%c",&val_sign);
					//printf("Echo: %c\n",ch); // echo to output
					printf("%c\n",val_sign); // echo to output
					scanf("%c",&new_setting);
					printf("Echo: %c\n",new_setting); // echo to output

					linear_speed_setting = charToMotorSetting(new_setting, val_sign);
					printf("%d\n",result); // echo to output
				}
				else if (ch == 97) // if "a"
				{
					uint8_t new_setting, val_sign;
					int16_t result;
					// Set angular speed;
					scanf("%c",&val_sign);
					//printf("Echo: %c\n",ch); // echo to output
					scanf("%c",&new_setting);

					
					angular_speed_setting = charToMotorSetting(new_setting, val_sign);
				}
				else if (ch == 27) //if "Escape"
				{
					rx_state = after_message;
				}
				break;
				
			case after_message:
				printf("After Message.\n\r");
				// Reset watchdog here.
				// Update commands here.
				left_speed_setting = calculate_left_wheel_speed(linear_speed_setting, 
																angular_speed_setting);
				right_speed_setting = calculate_right_wheel_speed(linear_speed_setting,
																angular_speed_setting);
				left_set_wheel_speed(left_speed_setting);
				right_set_wheel_speed(right_speed_setting);
				rx_state = idle;
				printf("e\n\r"); // Confirm out of receive mode. 
				break;
				
			case idle:
				printf("Idle.\n\r");
				scanf("%c",&ch); // get one input character
				//printf("Echo: %c\n\r",ch); // echo to output
				if (ch == 58) { // if ":"
					printf("r\n\r"); // Confirm in receive mode
					rx_state = receiving;
				}
				//break;
		}
	}
}
/**
  ******************************************************************************
  * @file    mw_com.c
  * @brief   This file provides code for communications.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mw_com.h"

// Private defines

// Private data types

// Private function prototypes

// Public functions

bool mw_com_command(bool dir, uint32_t movement)
{
	char c;
	uint16_t counter = 0;
	uint8_t size = 3;
	char command[size];

	bool ret = false;
	// Wait for command via UART
	if(USART2_Dequeue(&c)!=0)
	{
		bsp_usart_send_char(c);
        //Check first symbol which decides direction
		if(counter == 0)
		{
			if(c == '-')
			{
				dir = false;
			}
			else
			{
				dir = true;
			}
			counter++; // counter = 1;
		}
		// Finish checking first char for direction
		else
		{
			command[counter-1] = c;
			counter++;
			if(counter >(size))
			{
				movement = (uint32_t)atoi(command);
				  /*if(movement > 255)
					{
						movement = 240;
					}
					else if(movement < 0)
					{
					  	movement = 0;
					}*/
			}
		}
	}

	return ret;
}



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
UART_HandleTypeDef huart2;
// Private data types
char RX_BUFFER[BUFSIZE];
int  RX_BUFFER_HEAD, RX_BUFFER_TAIL;
uint8_t rx_data;
// Private function prototypes

// Public functions

void mw_com_command(bool *dir, uint32_t *movement, bool *confirm)
{
	char c;
	uint16_t counter = 0;
	uint8_t size = 2;
	char command[size];
	// Wait for command via UART
	*confirm = false;
	while(1)
	{
	//HAL_Delay(100);
	if(bsp_usart_dequeue(&c)!=0)
	{
		bsp_usart_send_char(c);
        //Check first symbol which decides direction
		if(counter == 0)
		{
			if(c == '-')
			{
				*dir = false;
			}
			else
			{
				*dir = true;
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
				*movement = (uint32_t)atoi(command);

				*confirm = true;
				break;
			}
		}

	}
	}


}

void mw_com_command_all(bool *dirx, bool *diry, uint32_t *movx, uint32_t *movy, bool *confirm)
{
	char c;
	uint16_t counter = 0;
	uint8_t size = 2;
	uint8_t size_all = 5;
	char command[size];
	*confirm = false;
	while(1)
	{
		if(bsp_usart_dequeue(&c)!=0)
		{
			bsp_usart_send_char(c);
			if(counter == 0)
			{
				if(c == '-')
				{
					*dirx = false;
				}
				else
				{
					*dirx = true;
				}
				counter++;
			}
			else if ((counter >0) && (counter <3))
			{
				command[counter-1] = c;
				counter++;
				if(counter >(size))
				{
					*movx = (uint32_t)atoi(command);

					//*confirm = true;
				}
			}
			else if (counter == 3)
			{
				if(c == '-')
				{
					*diry = false;
				}
				else
				{
					*diry = true;
				}
				counter++;
			}
			else
			{
				command[counter-(2+size)] = c;
				counter++;
				if(counter >(size_all))
				{
					*movy = (uint32_t)atoi(command);

					*confirm = true;
					break;
				}
			}
		}


		}
}


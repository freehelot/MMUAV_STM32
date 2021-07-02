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
	uint32_t temp_movx = *movx;
	uint32_t temp_movy = *movy;
	// Mssg
	// 0 - start char
	// 1 - dir x char
	// 2,3 - mov x chars
	// 4 - dir y char
	// 5,6 - mov y chars
	// 7 - end char
	while(1)
	{
		if(bsp_usart_dequeue(&c)!=0)
		{
			// read char from usart
			bsp_usart_send_char(c);
			// if first char and start char
			if( (counter == 0) && (c = 'A'))
			{
				counter ++;
			}
			// dir x char
			else if(counter == 1)
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
			//mov x chars counter 2,3
			else if ((counter > 1) && (counter < 4))
			{
				command[counter-2] = c;
				//2->3, 3->4
				counter++;
				if(counter >3)
				{
					*movx = (uint32_t)atoi(command);

					//*confirm = true;
				}
			}
			// dir y
			else if (counter == 4)
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
			else if ((counter > 4) && (counter < 7))
			{
				// 5 - (size), 5-5=0, 6-5 = 0
				command[counter-5] = c;
				counter++;
				if(counter > 6 )
				{
					*movy = (uint32_t)atoi(command);

				}
			}
			else if( (counter == 7) && (c = 'B'))
			{
				*confirm = true;
				break;
			}
			else if ( (counter == 7) && (c!= 'B'))
			{
				*confirm = false;
				//*movx = 0;
				*movx = temp_movx;
				*movy = temp_movy;
				break;
			}

		}// if something in usart com port
	}// while end
}


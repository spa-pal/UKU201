#define RX_BUFFER_SIZE1 512
#define TX_BUFFER_SIZE1 512

#include "uart1.h"
#include <stm32f10x_conf.h>

//***********************************************
//сюпр1
char bRXIN1;
char UIB1[100];
char flag1;
char rx_buffer1[RX_BUFFER_SIZE1];
char tx_buffer1[TX_BUFFER_SIZE1];
unsigned short rx_wr_index1,rx_rd_index1,rx_counter1;
unsigned short tx_wr_index1,tx_rd_index1,tx_counter1;
char rx_buffer_overflow1;
char tx1_restart;

//-----------------------------------------------
void putchar1(char c)
{
while (tx_counter1 == TX_BUFFER_SIZE1);

tx_buffer1[tx_wr_index1]=c;
   
if (++tx_wr_index1 >= TX_BUFFER_SIZE1) tx_wr_index1=0;


if (tx1_restart) 
	{                               // If transmit interrupt is disabled, enable it
    tx1_restart = 0;
	USART1->CR1 |= USART_FLAG_TXE;		          // enable TX interrupt
  	}
}

//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
void USART1_IRQHandler (void) 
{
volatile unsigned int IIR;
struct buf_st *p;
char data;

IIR = USART1->SR;		
if (IIR & USART_FLAG_RXNE) 
	{                  	
	USART1->SR &= ~USART_FLAG_RXNE;	          // clear interrupt

	data=USART1->DR & 0x00ff;

	rx_buffer1[rx_wr_index1]=data;
   	bRXIN1=1;
   	if (++rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
   	if (++rx_counter1 == RX_BUFFER_SIZE1)
      	{
      	rx_counter1=0;
      	rx_buffer_overflow1=1;
      	}
    }

if (IIR & USART_FLAG_TXE) 
	{
 	USART1->SR &= ~USART_FLAG_TXE;	          // clear interrupt
	
	if (tx_rd_index1 != tx_wr_index1)
		{
   		USART1->DR = tx_buffer1[tx_rd_index1];
		
		if (++tx_rd_index1 >= TX_BUFFER_SIZE1) tx_rd_index1=0;
   		}
	else 
	 	{
		tx1_restart = 1;
		USART1->CR1 &= ~USART_FLAG_TXE;		      // disable TX interrupt if nothing to send
		}
   	}
}
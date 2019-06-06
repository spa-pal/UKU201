  

#include "uart2.h"
#include <stm32f10x_conf.h>
#include <stm32f10x_lib.h> 

//***********************************************
//сюпр2
char bRXIN2;
char UIB2[100];
//char UOB1[100];
char flag2;
char rx_buffer2[RX_BUFFER_SIZE2];
char tx_buffer2[TX_BUFFER_SIZE2];
unsigned short rx_wr_index2,rx_rd_index2,rx_counter2;
unsigned short tx_wr_index2,tx_rd_index2,tx_counter2;
char rx_buffer_overflow2;
char tx2_restart;

//-----------------------------------------------
void putchar2(char c)
{
while (tx_counter2 == TX_BUFFER_SIZE2);

tx_buffer2[tx_wr_index2]=c;
   
if (++tx_wr_index2 >= TX_BUFFER_SIZE2) tx_wr_index2=0;


if (tx2_restart) 
	{                               // If transmit interrupt is disabled, enable it
    tx2_restart = 0;
	USART2->CR1 |= USART_FLAG_TXE;		          // enable TX interrupt
  	}
}

//-----------------------------------------------
void uart_out2 (char num,char data0,char data1,char data2,char data3,char data4,char data5)
{
char i,t=0;
//char *ptr=&data1;
char UOB2[16]; 
UOB2[0]=data0;
UOB2[1]=data1;
UOB2[2]=data2;
UOB2[3]=data3;
UOB2[4]=data4;
UOB2[5]=data5;

for (i=0;i<num;i++)
	{
	t^=UOB2[i];
	}    
UOB2[num]=num;
t^=UOB2[num];
UOB2[num+1]=t;
UOB2[num+2]=0x0a;

for (i=0;i<num+3;i++)
	{
	putchar2(UOB2[i]);
	}   	
}

//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
void USART2_IRQHandler (void) 
{
volatile unsigned int IIR;
//struct buf_st *p;
char data;

IIR = USART2->SR;		
if (IIR & USART_FLAG_RXNE) 
	{                  	
	USART2->SR &= ~USART_FLAG_RXNE;	          // clear interrupt

	data=USART2->DR & 0x00ff;

	rx_buffer2[rx_wr_index2]=data;
   	bRXIN2=1;
   	if (++rx_wr_index2 == RX_BUFFER_SIZE2) rx_wr_index2=0;
   	if (++rx_counter2 == RX_BUFFER_SIZE2)
      	{
      	rx_counter2=0;
      	rx_buffer_overflow2=1;
      	}
    }

if (IIR & USART_FLAG_TXE) 
	{
 	USART2->SR &= ~USART_FLAG_TXE;	          // clear interrupt
	
	if (tx_rd_index2 != tx_wr_index2)
		{
   		USART2->DR = tx_buffer2[tx_rd_index2];
		
		if (++tx_rd_index2 >= TX_BUFFER_SIZE2) tx_rd_index2=0;
   		}
	else 
	 	{
		tx2_restart = 1;
		USART2->CR1 &= ~USART_FLAG_TXE;		      // disable TX interrupt if nothing to send
		}
   	}

if (IIR & USART_FLAG_TC) 
	{
 	USART2->SR &= ~USART_FLAG_TC;	          // clear interrupt
	//GPIOC->ODR^=(1<<6);
	//GPIOA->ODR&=~(1<<11);

   	}
}

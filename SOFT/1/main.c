#include <stm32f10x_conf.h>
#include <string.h>                   /* string and memory functions         */
#include "STM32_Init.h"               /* stm32 initialisation                */
#include <stm32f10x_lib.h>
#include "main.h"
#include "uart1.h"
#include "uart2.h"

#define	putchar putchar2

//***********************************************
//Тайминги
unsigned char t0cnt0, t0cnt1, t0cnt2, t0cnt3, t0cnt4, t0cnt5; 
bool b1000Hz, b100Hz, b50Hz, b1Hz, b10Hz, b5Hz, b2Hz;
bool bFL, bFL2, bFL5;

//***********************************************
//Данные из EEPROM
signed short ICA_MODBUS_ADDRESS;//Адрес ведомого для выравнивания токов по шине MODBUS-RTU
signed short MODBUS_ADRESS=1;
signed int MODBUS_BAUDRATE=115200;

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//Отладка
char plazma;
char plazma_tx_cnt;


#include <stdio.h>

int sendchar(int ch);
struct __FILE {int handle;};
FILE __stdout;


int fputc(int ch, FILE *f) {
return (sendchar(ch));
}

int sendchar(int ch)
{
//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
putchar2(ch);
return 0;
}


//-----------------------------------------------
void delay_us(long del)
{
long temp;
temp=5*del;

while (--temp);
return;
}

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 
{
//GPIOC->ODR^=(1<<6);
b1000Hz=(bool)1;
	
if(++t0cnt0>=20)
     {
     t0cnt0=0;
     b50Hz=(bool)1;
     }
     
if(++t0cnt1>=10)
     {
     t0cnt1=0;
     b100Hz=(bool)1;

     if(++t0cnt2>=10)
	 	{
		t0cnt2=0;
		b10Hz=(bool)1;
		}

     if(++t0cnt3>=20)
	 	{
		t0cnt3=0;
		b5Hz=(bool)1;
		if(bFL5)bFL5=(bool)0;
		else bFL5=(bool)1;     
		}

	if(++t0cnt4>=50)
		{
		t0cnt4=0;
		b2Hz=(bool)1;
		if(bFL2)bFL2=(bool)0;
		else bFL2=(bool)1;
		}         

	if(++t0cnt5>=100)
		{
		t0cnt5=0;
		b1Hz=(bool)1;
		if(bFL)bFL=(bool)0;
		else bFL=(bool)1;
		}
	}
} 

/*----------------------------------------------------------------------------
 *        Main: 
 *---------------------------------------------------------------------------*/

int main (void) 
{



stm32_Init();
//SysTick->LOAD  = 544;  
//                            
//dac_init();
//init_display ();
//plazma1=SD_Reset();
//i2c_init();
//tx1_restart=1;
plazma=1;


while (1) 
	{
	//delay_us(100000); 
	//GPIOB->ODR^=(1<<10)|(1<<11)|(1<<12);
	//GPIOC->ODR^=(1<<6);
	if (b1000Hz) 
		{
		b1000Hz=(bool)0;

		}
	if (b100Hz) 
		{
		b100Hz=(bool)0;
//		GPIOB->ODR^=(1<<10);
		}
	if (b10Hz) 
		{
		b10Hz=(bool)0; 
//		GPIOB->ODR^=(1<<11);
		}   
	if (b5Hz) 
		{
		b5Hz=(bool)0; 
//		
		}
	if (b1Hz) 
		{
		b1Hz=(bool)0;
		plazma_tx_cnt++;
		
		//putchar1('a');
		//uart_out1 (4,'a','b','c',plazma_tx_cnt,0,0);
		//uart_out2 (4,'d','e','f',plazma_tx_cnt,0,0);
		//printf("MAMA MILA RAMU");
		//printf("plazma = %02d\r\n", plazma++);
		printf("rx_wr_index1 = %d\r\n", rx_wr_index1);
		//putchar2('a');
		}
	}
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

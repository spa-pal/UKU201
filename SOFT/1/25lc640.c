#include "25lc640.h"
#include <stm32f10x_lib.h>
#include "main.h"
//#include <stm32f10x_reg.h>

//-----------------------------------------------
char spi2(char in)
{
while (!(SPI2->SR & TXE));
SPI2->DR = in;

while (!(SPI2->SR & RXNE));
return (SPI2->DR);
}


//----------------------------------------------- 
//настройка SPI2
void spi2_config(void)
{ 
/* Enable clock for SPI2, GPIOB, GPIOC and AFIO. */	 RCC->APB1ENR |= ((unsigned long)0x10000000);
RCC->APB2ENR |= 0x00000019;
RCC->APB1ENR |= 0x00004000;



/* Reset SPI remap (use PA4..PA7). */
AFIO->MAPR   &= 0xFFFFFFFf;

/* spi2_NSS is GPIO, output set to high. */
//GPIOA->BSRR = 0x00000010;

/* SPI2_SCK, SPI2_MISO, SPI2_MOSI are SPI pins. */
GPIOB->CRH &= ~0xFFFF0000;
GPIOB->CRH |=  0xB8B30000;

GPIOC->CRH &= ~0x00000f00;
GPIOC->CRH |=  0x00000300;

SPI2->CR1  = 0x0344+3+(5<<3);
SPI2->CR2  = 0x0000;
}

//----------------------------------------------- 
//выключение SPI2
void spi2_unconfig(void)
{ 

RCC->APB1ENR &= ~0x00000400;

}




//----------------------------------------------- 
//Разрешение записи
void lc640_wren(void)
{

spi2_config();

CS_ON

spi2(0x06); 

CS_OFF

spi2_unconfig();
}

//-----------------------------------------------
//Чтение из м-мы регистра состояния
char lc640_rdsr(void)
{
char temp;

spi2_config();
CS_ON
spi2(0x05);
temp=spi2(0xff);
CS_OFF
spi2_unconfig();
return temp;
}

//----------------------------------------------- 
//Чтение из м-мы байта по адр. ADR
int lc640_read(int ADR)
{
int temp;
temp=0;

while(lc640_rdsr()&0x01)
	{
	}
spi2_config();
CS_ON
CS_ON
//temp_short[0]=PINSEL1;	
//
//IO0DIR|=1UL<<17;
//IO0CLR|=1UL<<17;
spi2(0x03);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=spi2(0xff);
//IO0SET|=1UL<<17;
CS_OFF
CS_OFF
spi2_unconfig();
return temp;

}

//----------------------------------------------- 
//Чтение из м-мы слова по адр. ADR
int lc640_read_int(int ADR)
{
char temp;
int temp_i;


//LPC_GPIO0->FIODIR|=0x00000002;
//LPC_GPIO0->FIOSET|=0x00000002;



while(lc640_rdsr()&0x01)
	{
	}

//lc640_rdsr();
//IO0DIR_bit.P0_11=1;
//IO0SET_bit.P0_11=1;
spi2_config();
CS_ON
spi2(0x03);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=spi2(0xff);
temp_i=spi2(0xff);
temp_i<<=8;
temp_i+=temp;
CS_OFF
CS_OFF
spi2_unconfig();

//LPC_GPIO0->FIOCLR|=0x00000002;
return temp_i;
}

//----------------------------------------------- 
//Чтение из м-мы 4 байт по адр. ADR
long lc640_read_long(int ADR)
{
char temp0,temp1,temp2;
long temp_i;
while(lc640_rdsr()&0x01)
	{
	}
spi2_config();
CS_ON
spi2(0x03);
temp0=*(((char*)&ADR)+1);
spi2(temp0);
temp0=*((char*)&ADR);
spi2(temp0);
temp0=spi2(0xff);
temp1=spi2(0xff);
temp2=spi2(0xff);
temp_i=spi2(0xff);
temp_i<<=8;
temp_i+=temp2;
temp_i<<=8;
temp_i+=temp1;
temp_i<<=8;
temp_i+=temp0;
CS_OFF
CS_OFF
spi2_unconfig();
return temp_i;
}

//----------------------------------------------- 
//Чтение из м-мы 4 байт по адр. ADR
void lc640_read_long_ptr(int ADR,char* out_ptr)
{
char temp0/*,temp1,temp2*/;
//long temp_i;
while(lc640_rdsr()&0x01)
	{
	}
spi2_config();
CS_ON
spi2(0x03);
temp0=*(((char*)&ADR)+1);
spi2(temp0);
temp0=*((char*)&ADR);
spi2(temp0);
out_ptr[0]=spi2(0xff);
out_ptr[1]=spi2(0xff);
out_ptr[2]=spi2(0xff);
out_ptr[3]=spi2(0xff);
CS_OFF
CS_OFF
spi2_unconfig();
}

//----------------------------------------------- 
//Чтение из м-мы N байт по адр. ADR
void lc640_read_str(int ADR, char* ram_ptr, char num)
{
char temp0,i;
while(lc640_rdsr()&0x01)
	{
	}
spi2_config();
CS_ON
spi2(0x03);
temp0=*(((char*)&ADR)+1);
spi2(temp0);
temp0=*((char*)&ADR);
spi2(temp0);

for(i=0;i<num;i++)
	{
	*ram_ptr++=spi2(0xff);
	}
CS_OFF
CS_OFF
spi2_unconfig();
}

//-----------------------------------------------
//Запись байта in по адресу ADR
char lc640_write(int ADR,char in)
{
char temp; 

while(lc640_rdsr()&0x01)
	{
	}
lc640_wren();
spi2_config();	
CS_ON
spi2(0x02);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=spi2(in);
CS_OFF
CS_OFF
spi2_unconfig(); 
return temp;
}

//-----------------------------------------------
//Запись слова in по адресу ADR
char lc640_write_int(short ADR,short in)
{
char temp; 
while(lc640_rdsr()&0x01)
	{
	}
lc640_wren();
spi2_config();	
CS_ON
spi2(0x02);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=*((char*)&in);
spi2(temp);
temp=*(((char*)&in)+1);
spi2(temp);
CS_OFF
CS_OFF
spi2_unconfig();
return temp;
}  

//-----------------------------------------------
//Запись 4 байт in по адресу ADR
char lc640_write_long(int ADR,long in)
{
char temp; 

while(lc640_rdsr()&0x01)
	{
	}
lc640_wren();	
spi2_config();
CS_ON
spi2(0x02);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=*((char*)&in);
spi2(temp);
temp=*(((char*)&in)+1);
spi2(temp);
temp=*(((char*)&in)+2);
spi2(temp);
temp=*(((char*)&in)+3);
spi2(temp);           
CS_OFF
CS_OFF  
spi2_unconfig();
return temp;
}

//-----------------------------------------------
//Запись 4 байт in по адресу ADR
char lc640_write_long_ptr(int ADR,char* in)
{
char temp; 

while(lc640_rdsr()&0x01)
	{
	}
lc640_wren();	
spi2_config();
CS_ON
spi2(0x02);
temp=*(((char*)&ADR)+1);
spi2(temp);
temp=*((char*)&ADR);
spi2(temp);
temp=in[0];
spi2(temp);
temp=in[1];
spi2(temp);
temp=in[2];
spi2(temp);
temp=in[3];
spi2(temp);

CS_OFF
CS_OFF  
spi2_unconfig();
return temp;
}		

#include <stm32f10x_lib.h>
#include "25lc640.h"
#include "mcp2515.h"
//#include "LPC17xx.H"
#include "main.h"
#include "full_can.h"

char mcp2515_can_st,mcp2515_can_st_old;
char MCP2515_RXBUFF[40];
char bMCP2515_IN;
char mcp2515_out_buff[8][8];
char mcp2515_buff_wr_ptr;
char mcp2515_buff_rd_ptr;

//-----------------------------------------------
void mcp2515_reset(void)
{
spi2_config();
MCP2515_CS_ON
spi2(0xc0);
MCP2515_CS_OFF

}

//-----------------------------------------------
char mcp2515_write(char addr,char in)
{           
char temp;
spi2_config();       
MCP2515_CS_ON
spi2(0x02);
spi2(addr);
spi2(in);
MCP2515_CS_OFF
return temp;                
}

//-----------------------------------------------
char mcp2515_read(char addr)
{           
char temp;

spi2_config();       
MCP2515_CS_ON
delay_us(10);
spi2(0x03);
spi2(addr);
temp=spi2(0x55);
MCP2515_CS_OFF   
return temp;                
}  

//-----------------------------------------------
void mcp2515_bit_modify(char addr,char mask,char data)
{           
spi2_config();       
MCP2515_CS_ON
spi2(0x05);
spi2(addr);
spi2(mask);
spi2(data);
MCP2515_CS_OFF
} 

//-----------------------------------------------
char mcp2515_read_status(void)
{           
char temp;
//#asm("cli")
spi2_config();       
MCP2515_CS_ON
delay_us(1);
spi2(0xa0);
temp=spi2(0x55);
MCP2515_CS_OFF
//#asm("sei")    
return temp;                
}

//-----------------------------------------------
void mcp2515_rts(char in)
{
//#asm("cli")
spi2_config();       
MCP2515_CS_ON
if(in==0) in=0x81;
else if(in==1) in=0x82;
else if(in==2) in=0x84;
spi2(in);
MCP2515_CS_OFF
//#asm("sei")                    
}

//-----------------------------------------------
void can_mcp2515_init(void)
{
char spi_temp;                 

mcp2515_reset();
spi_temp=mcp2515_read(CANSTAT);
if((spi_temp&0xe0)!=0x80)
	{
	mcp2515_bit_modify(CANCTRL,0xe0,0x80);
	}
delay_us(10);		
mcp2515_write(CNF1,CNF1_init);
mcp2515_write(CNF2,CNF2_init);
mcp2515_write(CNF3,CNF3_init);

mcp2515_write(RXB0CTRL,0x20/*0b00100000*/);
mcp2515_write(RXB1CTRL,0x20/*0b00100000*/);

delay_ms(10);

mcp2515_write(RXM0SIDH, 0xFF); 
mcp2515_write(RXM0SIDL, 0xFF); 
mcp2515_write(RXF0SIDH, 0xFF); 
mcp2515_write(RXF0SIDL, 0xFF); 
mcp2515_write(RXF1SIDH, 0xFF);
mcp2515_write(RXF1SIDL, 0xFF); 

mcp2515_write(RXM1SIDH, 0xff); 
mcp2515_write(RXM1SIDL, 0xe0); 

mcp2515_write(RXF2SIDH, 0x31); 
mcp2515_write(RXF2SIDL, 0xc0); 

mcp2515_write(RXF3SIDH, 0x00); 
mcp2515_write(RXF3SIDL, 0x00); 

mcp2515_write(RXF4SIDH, 0x00); 
mcp2515_write(RXF4SIDL, 0x00); 

mcp2515_write(RXF5SIDH, 0x00); 
mcp2515_write(RXF5SIDL, 0x00); 

mcp2515_write(TXB2SIDH, 0x13); 
mcp2515_write(TXB2SIDL, 0xc0); 

mcp2515_write(TXB1SIDH, 0x13); 
mcp2515_write(TXB1SIDL, 0xc0); 

mcp2515_write(TXB0SIDH, 0x13); 
mcp2515_write(TXB0SIDL, 0xc0); 



mcp2515_bit_modify(CANCTRL,0xe7,0x05/*0b00000101*/);

mcp2515_write(CANINTE,0x06/*0b00000110*/);
delay_ms(100);
mcp2515_write(BFPCTRL,0x00/*0b00000000*/);  

}

//-----------------------------------------------
void mcp2515_transmit(char data0,char data1,char data2,char data3,char data4,char data5,char data6,char data7)
{
if(/*(mcp2515_buff_wr_ptr<0)||*/(mcp2515_buff_wr_ptr>7))mcp2515_buff_wr_ptr=0;

mcp2515_out_buff[0][mcp2515_buff_wr_ptr]=data0;
mcp2515_out_buff[1][mcp2515_buff_wr_ptr]=data1;
mcp2515_out_buff[2][mcp2515_buff_wr_ptr]=data2;
mcp2515_out_buff[3][mcp2515_buff_wr_ptr]=data3;
mcp2515_out_buff[4][mcp2515_buff_wr_ptr]=data4;
mcp2515_out_buff[5][mcp2515_buff_wr_ptr]=data5;
mcp2515_out_buff[6][mcp2515_buff_wr_ptr]=data6;
mcp2515_out_buff[7][mcp2515_buff_wr_ptr]=data7;

mcp2515_buff_wr_ptr++;
if(mcp2515_buff_wr_ptr>7)mcp2515_buff_wr_ptr=0;
} 

//-----------------------------------------------
void can_mcp2515_hndl(void)
{
unsigned char j,c_temp;
static char ch_cnt;

mcp2515_can_st=mcp2515_read_status();
mcp2515_can_st_old=mcp2515_can_st;



if(mcp2515_can_st&0x02)
	{
	
	for(j=0;j<8;j++)
		{
		RXBUFF[j]=mcp2515_read(RXB1D0+j);
		}
	
	mcp2515_bit_modify(CANINTF,0x02,0x00);
     bMCP2515_IN=1;
	 
	}
           
           
else if((!(mcp2515_can_st&0x54)))
	{
	char n;
    mcp2515_bit_modify(CANINTF,0x1c,0x00);
     
     if(mcp2515_buff_rd_ptr!=mcp2515_buff_wr_ptr)
     	{

         	for(n=0;n<8;n++)
			{ 
			mcp2515_write(TXB0D0+n,mcp2515_out_buff[n][mcp2515_buff_rd_ptr]);
			} 
    		mcp2515_write(TXB0DLC,8);
    		mcp2515_rts(0); 
    		
    		mcp2515_buff_rd_ptr++;
    		if(mcp2515_buff_rd_ptr>7)mcp2515_buff_rd_ptr=0;
    		} 
 	} 	
		
}


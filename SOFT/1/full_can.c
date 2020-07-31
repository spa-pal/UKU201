#include <stm32f10x_type.h>
#include "main.h"
#include "full_can.h"
#include "cmd.h"
#include "mess.h"
#include "global_define.h"
//#include "avar_hndl.h"
#include "eeprom_map.h"
#include "control.h"
#include "25lc640.h"


short can_error_cntr;

// Counts number of filters (CAN message objects) used so far
short volatile gCANFilter = 0;

char ptr_can1_tx_wr,ptr_can1_tx_rd;
long can1_info[8];
long can1_id[8];
long can1_data[8];
long can1_datb[8];
																							 
char ptr_can2_tx_wr,ptr_can2_tx_rd;

long can2_info[8];
long can2_id[8];
long can2_data[8];
long can2_datb[8];

unsigned short rotor_can[6];




// FullCAN Message List
FULLCAN_MSG volatile gFullCANList[MAX_FILTERS];

char bR;
char RXBUFF[40],TXBUFF[40];
char bIN,bIN2;
char bd_dumm[25];
char bd[25];
char TX_len;
//char bOUT;
char RXBUFF2[40],TXBUFF2[40];
extern char can_tx_cnt;
extern char can_tx_cnt2;
char bOUT_FREE=1;
char bOUT_FREE2=1;
char rotor_rotor_rotor[2];
char can_tx_cnt;
char can_rotor[10];

char can_debug_plazma[2][10];
volatile unsigned int CANStatus;

char can_reset_cnt=0;

char plazma_can_pal[20];
char cnt_can_pal;
char plazma_can_pal_index;

// char can_reset_cnt=0;
char plazma_can;
char plazma_can1,plazma_can2,plazma_can3,plazma_can4;
short can2_tx_cnt;
char ccc_plazma[20];

//-----------------------------------------------
char CRC1_in(void)
{
char r,j,lb;
lb=(RXBUFF[1]&0x1f)+0x04;
r=RXBUFF[0];
for(j=1;j<(lb+1);j++)
	{
	r=(RXBUFF[j]^Table87[r]);
	}
if(r==0)r=0xFF;
return r;	
} 

//-----------------------------------------------
char CRC2_in(void)
{
char r,j,lb;
lb=(RXBUFF[1]&0x1f)+0x04;
r=RXBUFF[0];
for(j=1;j<(lb+1);j++)
	{
	r=(RXBUFF[j]^Table95[r]);
	}
if(r==0)r=0xFF;
return r;	
}  

//-----------------------------------------------
char CRC1_out(void)
{
char r,j,lb;
lb=(TXBUFF[1]&0x1f)+0x04;
r=TXBUFF[0];
for(j=1;j<(lb+1);j++)
	{
	r=(TXBUFF[j]^Table87[r]);
	}
if(r==0)r=0xFF;
return r;	
} 

//-----------------------------------------------
char CRC2_out(void)
{
char r,j,lb;
lb=(TXBUFF[1]&0x1f)+0x04;
r=TXBUFF[0];
for(j=1;j<(lb+1);j++)
	{
	r=(TXBUFF[j]^Table95[r]);
	}
if(r==0)r=0xFF;
return r;	
}

//-----------------------------------------------
void paking(char* data_ptr,char data_len)
{
char i,ii,iii;
for(i=0;i<data_len;i++)
	{
	ii=data_len+(i/7);
	iii=i-(7*(i/7)); 
	if(iii==0) data_ptr[ii]=0;
	data_ptr[ii]<<=1;
	if(data_ptr[i]&0x01)
		{
		data_ptr[ii]|=0x01;//(1<<(6-iii));
		}                      
	else 
		{
		data_ptr[ii]&=0xfe;//~(1<<(6-iii));
		}              
	data_ptr[i]>>=1;	        
	data_ptr[i]|=0x80;	
	}                       
for(i=data_len;i<(data_len+(data_len/7)+1);i++)
	{
	data_ptr[i]|=0x80;
	}	
}





//-----------------------------------------------
void can_adr_hndl(void)
{
	TXBUFF[2]=RXBUFF[3];
	TXBUFF[3]=RXBUFF[2];
	TXBUFF[4]=((RXBUFF[4]&0xF0)>>4)|((RXBUFF[4]&0x0f)<<4);
	TXBUFF[5]=((RXBUFF[5]&0xF0)>>4)|((RXBUFF[5]&0x0f)<<4);	
}	

//-----------------------------------------------
 void can_in_an1(void)
{
//char i;
//signed short temp_SS;
char slave_num;

//if(!bIN2) goto CAN_IN_AN1_end; 

//can_debug_plazma[1][2]++;
//can_rotor[1]++;


///if((((RXBUFF[0]==sub_ind1)&&(ind==iK_bps))||((RXBUFF[0]==uavt_bps_pntr)&&(uavt_set_stat==uassSTEP3)))&&(RXBUFF[1]==PUTID)&&(RXBUFF[2]==0xdd)&&(RXBUFF[3]==0xdd)/*&&(sub_ind==6)*/)
///	{
///	mess_send(MESS2IND_HNDL,PARAM_U_AVT_GOOD,0,10);
///	can_reset_cnt=0;
///	}


if((RXBUFF[1]==PUTTM1)&&((RXBUFF[0]&0x1f)>=0)&&((RXBUFF[0]&0x1f)<17))
     {
	 can_error_cntr=0;
	 can_rotor[1]++;
	//can_debug_plazma[1][2]++;
     slave_num=RXBUFF[0]&0x1f;
	 can_rotor[2]=slave_num;
     
    if((RXBUFF[0]&0xe0)==0)bps[slave_num]._device=dSRC;
    else if((RXBUFF[0]&0xe0)==0x40)bps[slave_num]._device=dINV;
     	
	bps[slave_num]._buff[0]=RXBUFF[2]; 
	bps[slave_num]._buff[1]=RXBUFF[3];
	bps[slave_num]._buff[2]=RXBUFF[4];
	bps[slave_num]._buff[3]=RXBUFF[5];
	bps[slave_num]._buff[4]=RXBUFF[6];
	bps[slave_num]._buff[5]=RXBUFF[7];


/*	can_slot[slave_num,0]=RXBUFF[0];
	can_slot[slave_num,1]=RXBUFF[1];
	can_slot[slave_num,2]=RXBUFF[2];
	can_slot[slave_num,3]=RXBUFF[3];
	can_slot[slave_num,4]=RXBUFF[4];
	can_slot[slave_num,5]=RXBUFF[5];
	can_slot[slave_num,6]=RXBUFF[6];
	can_slot[slave_num,7]=RXBUFF[7]; */
	
	bps[slave_num]._cnt=0;
	bps[slave_num]._is_on_cnt=10;
	
 	if((bps[slave_num]._cnt==0)&&(bps[slave_num]._av&(1<<3))) avar_bps_hndl(slave_num,3,0);

	can_reset_cnt=0;
     }

if((RXBUFF[1]==PUTTM2)&&((RXBUFF[0]&0x1f)>=0)&&((RXBUFF[0]&0x1f)<12))
 	{
	can_error_cntr=0;
     slave_num=RXBUFF[0]&0x1f;  

    if((RXBUFF[0]&0xe0)==0)bps[slave_num]._device=dSRC;
    else if((RXBUFF[0]&0xe0)==0x40)bps[slave_num]._device=dINV;
     
	bps[slave_num]._buff[6]=RXBUFF[2]; 
	bps[slave_num]._buff[7]=RXBUFF[3];
	bps[slave_num]._buff[8]=RXBUFF[4];
	bps[slave_num]._buff[9]=RXBUFF[5];
	bps[slave_num]._buff[10]=RXBUFF[6];
	bps[slave_num]._buff[11]=RXBUFF[7];	


/*	can_slot[slave_num,8]=RXBUFF[0];
	can_slot[slave_num,9]=RXBUFF[1];
	can_slot[slave_num,10]=RXBUFF[2];
	can_slot[slave_num,11]=RXBUFF[3];
	can_slot[slave_num,12]=RXBUFF[4];
	can_slot[slave_num,13]=RXBUFF[5];
	can_slot[slave_num,14]=RXBUFF[6];
	can_slot[slave_num,15]=RXBUFF[7]; */
	
	bps[slave_num]._cnt=0;
	bps[slave_num]._is_on_cnt=10; 

   	//if((bps[slave_num]._cnt==0)&&(src[slave_num]._av_net)) avar_s_hndl(slave_num,3,0); 
	can_reset_cnt=0;
   	}


if((RXBUFF[1]==PUTTM_IBATMETER)&&(RXBUFF[0]==PUTTM_IBATMETER))
 	{
	can_error_cntr=0;
	ibat_metr_buff_[0]=((signed long)RXBUFF[2])+(((signed long)RXBUFF[3])<<8);
	ibat_metr_buff_[1]=((signed long)RXBUFF[4])+(((signed long)RXBUFF[5])<<8);
	bIBAT_SMKLBR=((signed short)RXBUFF[6])+(((signed short)RXBUFF[7])<<8);
	if(bIBAT_SMKLBR) bIBAT_SMKLBR_cnt=50;
	if(!bIBAT_SMKLBR)
		{
		signed long temp_SL;
		temp_SL=(signed long)ibat_metr_buff_[0];
		temp_SL-=(signed long)ibat_metr_buff_[1];
		temp_SL*=(signed long)Kibat1[0];
		temp_SL/=2000L;
	
		Ib_ips_termokompensat =(signed short)temp_SL;
		if(bIBAT_SMKLBR_cnt)
			{
			bIBAT_SMKLBR_cnt--;
			Ib_ips_termokompensat=Ib_ips_termokompensat_temp;
			}
		else 
			{
			Ib_ips_termokompensat_temp=Ib_ips_termokompensat;
			}
		}
	ibat_metr_cnt=0;
   	}
}

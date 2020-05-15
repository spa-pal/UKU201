#include <stm32f10x_type.h>
#include <stm32f10x_lib.h>
#include "avar_hndl.h"
#include "eeprom_map.h"	   
#include "full_can.h"
#include "25lc640.h"
#include "main.h"
#include "control.h"
//#include "snmp_data_file.h" 

     
//***********************************************
//Аварии
unsigned avar_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
unsigned avar_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
unsigned avar_stat_old;
unsigned avar_stat_new,avar_stat_offed;
//Структура переменных
//1бит  - питающая сеть
//2бита - батареи
//12бит - БПСы
//5бит  - инверторы
//4бита - внешние датчики температуры
//4бита - внешние сухие контакты

char /*av_net*//*,av_bat[2]*//*av_bps[12],*/av_inv[6];//,/*av_dt[4],av_sk[4]*/;

// oleg_start
//Аварии
unsigned rki_avar1_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
unsigned rki_avar1_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
unsigned rki_avar1_stat_old;
unsigned rki_avar1_stat_new, rki_avar1_stat_offed;

unsigned rki_avarI1_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
unsigned rki_avarI1_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
unsigned rki_avarI1_stat_old;
unsigned rki_avarI1_stat_new, rki_avarI1_stat_offed;

unsigned rki_avarI2_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
unsigned rki_avarI2_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
unsigned rki_avarI2_stat_old;
unsigned rki_avarI2_stat_new, rki_avarI2_stat_offed;
// oleg_end

extern char bOUT_FREE2;	






//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//Внешние


//***********************************************
//Состояние первичной сети
//char u_net_av_stat,u_net_av_stat_;
extern signed short net_U,net_Ustore;
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt;
extern signed char unet_drv_cnt;
extern char net_av;



//-----------------------------------------------
void avar_hndl(void)
{
//static unsigned avar_stat_old;
char i;
//unsigned avar_stat_new,avar_stat_offed;

if(net_av)		SET_REG(avar_stat,1,0,1);
else	   			SET_REG(avar_stat,0,0,1);

for(i=0;i<2;i++)
	{
	if(bat[i]._av&3)	SET_REG(avar_stat,1,1+i,1);
	else	   			SET_REG(avar_stat,0,1+i,1);
	}

for(i=0;i<12;i++)
	{
	if(bps[i]._av&0xef)	SET_REG(avar_stat,1,3+i,1);
	else	   		SET_REG(avar_stat,0,3+i,1);
	}



avar_stat_new=(avar_stat^avar_stat_old)&avar_stat;

avar_ind_stat|=avar_stat_new;


avar_stat_offed=~((avar_stat^avar_stat_old)&avar_stat_old);

if(!AV_OFF_AVT)avar_stat_offed|=0xeffffffe;

avar_ind_stat&=avar_stat_offed; 

avar_stat_old=avar_stat;

}

//-----------------------------------------------
void reload_hndl(void)
{

char data[4];
unsigned int event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='U';
data[1]=0;
data[2]='R';
data[3]=0;
lc640_write_long_ptr(lc640_adr,data);

data[0]=0;
data[1]=0;
data[2]=0;
data[3]=0;
lc640_write_long_ptr(lc640_adr+4,data);

data[0]=(char)(BKP->DR1);
data[1]=(char)(BKP->DR2);
data[2]=(char)(BKP->DR3);
data[3]=0;
lc640_write_long_ptr(lc640_adr+8,data);
	
data[0]=time_H;
data[1]=time_M;
data[2]=time_S;
data[3]=0;
lc640_write_long_ptr(lc640_adr+12,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+16,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+20,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+24,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+28,data);				
	
	
}

//-----------------------------------------------
void avar_unet_hndl(char in)
{

char data[4];
unsigned int event_ptr,lc640_adr,event_ptr_find,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1)
	{
	net_av=1;

	//beep_init(0x01L,'O');
	//a.av.bAN=1;


	//beep_init(0x33333333,'A');
	 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='P';
	data[1]=0;
	data[2]='A';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;//*((char*)&Unet_store);
	data[1]=0;//*(((char*)&Unet_store)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
	}

else if(in==2)
	{
	net_av=2;

	//beep_init(0x01L,'O');
	//a.av.bAN=1;


	//beep_init(0x33333333,'A');
	 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='P';
	data[1]=0;
	data[2]='B';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;//*((char*)&Unet_store);
	data[1]=0;//*(((char*)&Unet_store)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
//	snmp_trap_send("Main power alarm, voltage increased",2,2,0);
	}

else if(in==0)
	{
	net_av=0;

  /*
	SET_REG(C2GSR,3,24,8);
	C2MOD=0;
	bOUT_FREE2=1;*/


	//C2MOD=0;

	//can2_init(7,8,CANBitrate250k_60MHz);

     //beep_init(0x02L,'O');
	//a_.av.bAN=0;
	SET_REG(avar_stat,0,0,1);

	//if(AV_OFF_AVT)a.av.bAN=0;
	
	if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,0,1);

     event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
avar_unet_hndl_lbl1:
	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='P')&&(data[1]==0)&&(data[2]=='A')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     	else goto avar_unet_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     		else goto avar_unet_hndl_lbl1;
     		}

     	}	
	
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data); 
	
	data[0]=*((char*)(&net_Ustore));
	data[1]=*(((char*)(&net_Ustore))+1);
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);	
	}
avar_unet_hndl_end:
	__nop();		
}


//-----------------------------------------------
void avar_uout_hndl(char in)
{

char data[4];
unsigned int event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_data=0;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if((in==1)||(in==2))
	{
	uout_av=in;
		
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='Q';
	data[1]='A';
	data[2]='D';
	if(in==2) data[2]='U';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=*((char*)&out_U);
	data[1]=*(((char*)&out_U)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
		
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
	}

else if(in==0)
	{
	uout_av=0;

  /*
	SET_REG(C2GSR,3,24,8);
	C2MOD=0;
	bOUT_FREE2=1;*/


	//C2MOD=0;

	//can2_init(7,8,CANBitrate250k_60MHz);

     //beep_init(0x02L,'O');
	//a_.av.bAN=0;
//	SET_REG(avar_stat,0,0,1);

	//if(AV_OFF_AVT)a.av.bAN=0;
	
	//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,0,1);

    event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
avar_uout_hndl_lbl1:
	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='Q')&&(data[1]==0)&&(data[2]=='A')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_uout_hndl_end;
     	else goto avar_uout_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_uout_hndl_end;
     		else goto avar_uout_hndl_lbl1;
     		}

     	}	
	
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data); 
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);	
	}
avar_uout_hndl_end:
	__nop();		
}

//-----------------------------------------------
void avar_bps_hndl(char dev, char v, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_simbol;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

avar_simbol='T';
if(v==0)avar_simbol='T';
else if(v==1)avar_simbol='U';
else if(v==2)avar_simbol='u';
else if(v==3)avar_simbol='L';

if(in==1)
	{
	//av_src[bps]=1;

//	SET_REG(avar_ind_stat,1,3+dev,1);
//	SET_REG(avar_stat,1,3+dev,1);
	
	if(v==0)bps[dev]._av|=(1<<0);
	else if(v==1) bps[dev]._av|=(1<<1);
	else if(v==2) bps[dev]._av|=(1<<2);
	else if(v==3) bps[dev]._av|=(1<<3);

	bps[dev]._last_avar=v;

	//beep_init(0x33333333,'A');

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='S';
	data[1]=dev; 
	data[2]=avar_simbol;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		

/*
	if(dev==0)
		{
		if(v==0)
		snmp_trap_send("BPS #1 Alarm, overheat",4,1,0);
		else if(v==1)
		snmp_trap_send("BPS #1 Alarm, voltage is up",4,1,1);
		else if(v==2)
		snmp_trap_send("BPS #1 Alarm, voltage is down",4,1,2);
		else if(v==3)
		snmp_trap_send("BPS #1 Alarm, connect is lost",4,1,3);
		}
	else if(dev==1)
		{
		if(v==0)
		snmp_trap_send("BPS #2 Alarm, overheat",4,2,0);
		else if(v==1)
		snmp_trap_send("BPS #2 Alarm, voltage is up",4,2,1);
		else if(v==2)
		snmp_trap_send("BPS #2 Alarm, voltage is down",4,2,2);
		else if(v==3)
		snmp_trap_send("BPS #2 Alarm, connect is lost",4,2,3);
		} 
	else if(dev==2)
		{
		if(v==0)
		snmp_trap_send("BPS #3 Alarm, overheat",4,3,0);
		else if(v==1)
		snmp_trap_send("BPS #3 Alarm, voltage is up",4,3,1);
		else if(v==2)
		snmp_trap_send("BPS #3 Alarm, voltage is down",4,3,2);
		else if(v==3)
		snmp_trap_send("BPS #3 Alarm, connect is lost",4,3,3);
		} */	
			
	}

else if(in==0)
	{      
//	av_bps[bps]=0;

	//SET_REG(avar_stat,0,3+bps,1);
	//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,3+bps,1);

     if(v==0) bps[dev]._av&=(~(1<<0));
     else if(v==1) bps[dev]._av&=(~(1<<1));
	else if(v==2) bps[dev]._av&=(~(1<<2));
	else if(v==3) bps[dev]._av&=(~(1<<3));
     
//	if((bps[dev]._av&0x0f==0))
//	     {
		//SET_REG(avar_stat,0,3+bps,1);
		//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,3+bps,1);
//	     }
 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_src_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='S')&&(data[1]==dev)&&(data[2]==avar_simbol)))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     	else goto avar_src_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     		else goto avar_src_hndl_lbl1;
     		}

     	}	


	
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	}
	
avar_src_hndl_end:
__nop();		
}

//-----------------------------------------------
void wrk_mem_hndl(char b)
{
char data[4];
unsigned short event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;

signed short temp_temp;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(bat[b]._Iintegr_<5) goto wrk_mem_hndl_end;

temp_temp=bat[b]._u_old[((bat_u_old_cnt+6)&0x07)]; 


event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='W';
data[3]=0;

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&bat[b]._Iintegr_);
data[1]=*(((char*)(&bat[b]._Iintegr_))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

lc640_write_long_ptr(lc640_adr+8,(char*)&bat[b]._wrk_date[0]);
	
lc640_write_long_ptr(lc640_adr+12,(char*)&bat[b]._wrk_date[1]);

data[0]=(char)(BKP->DR1);
data[1]=(char)(BKP->DR2);
data[2]=(char)(BKP->DR3);
data[3]=0;
lc640_write_long_ptr(lc640_adr+16,data);
	
data[0]=time_H;
data[1]=time_M;
data[2]=time_S;
data[3]=0;
lc640_write_long_ptr(lc640_adr+20,data);
	


	
wrk_mem_hndl_end:	
__nop();
}  

//-----------------------------------------------
void avar_bat_ips_hndl(char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1)
	{
	bat_ips._av|=1;
    ips_bat_av_stat=1;
	ips_bat_av_vzvod=1;

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
	event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=0;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
	//snmp_trap_send("BAT №1 Alarm, lost",5,1,1);
		
	}

else if(in==0)
	{
	bat_ips._av&=~1;
	ips_bat_av_stat=0;
	ips_bat_av_vzvod=0;
     
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat_ips_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==0)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     	else goto avar_bat_ips_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     		else goto avar_bat_ips_hndl_lbl1;
     		}

     	}
     		
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	

	
	}
	
avar_bat_ips_hndl_end:
__nop();		
}


//-----------------------------------------------
void avar_bat_hndl_(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1)
	{
	bat[b]._av|=1;
    
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=b;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
/*	if(b==0)
		{
		snmp_trap_send("BAT #1 Alarm, lost",5,1,1);
		}
	else if(b==1)
		{
		snmp_trap_send("BAT #2 Alarm, lost",5,2,1);
		} */
			
	}

else if(in==0)
	{
	bat[b]._av&=~1;

     event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==b)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_hndl_end;
     	else goto avar_bat_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_hndl_end;
     		else goto avar_bat_hndl_lbl1;
     		}

     	}
     		
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	

	
	}
	
avar_bat_hndl_end:
__nop();		
}



//-----------------------------------------------
void ke_zvu_mem_hndl(char b,unsigned short in,unsigned short in1)
{
char data[4];
unsigned int event_ptr=0,lc640_adr/*,event_ptr_find*/,event_cnt;
//unsigned char temp,temp_;
//unsigned int tempUI;
//unsigned long tempUL; 
signed temp_temp;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

temp_temp=bat[b]._u_old[bat_u_old_cnt+1]; 

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='K';
data[3]='Z';

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&in);
data[1]=*(((char*)(&in))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

ke_date[0]=lc640_read_long(EE_SPC_KE_DATE0);
lc640_write_long_ptr(lc640_adr+8,(char*)&ke_date[0]);
ke_date[1]=lc640_read_long(EE_SPC_KE_DATE1);	
lc640_write_long_ptr(lc640_adr+12,(char*)&ke_date[1]);

data[0]=(char)(BKP->DR1);
data[1]=(char)(BKP->DR2);
data[2]=(char)(BKP->DR3);
data[3]=*((char*)&in1);
lc640_write_long_ptr(lc640_adr+16,data);
	
data[0]=time_H;
data[1]=time_M;
data[2]=time_S;
data[3]=*(((char*)(&in1))+1);
lc640_write_long_ptr(lc640_adr+20,data);
 
}

//-----------------------------------------------
void ke_mem_hndl(char b,unsigned short in)
{
char data[4];
unsigned int event_ptr=0,lc640_adr/*,event_ptr_find*/,event_cnt;
//unsigned char temp,temp_;
//unsigned int tempUI;
//unsigned long tempUL; 
signed temp_temp;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

temp_temp=bat[b]._u_old[bat_u_old_cnt+1]; 

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='K';
data[3]=0;

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&in);
data[1]=*(((char*)(&in))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

ke_date[0]=lc640_read_long(EE_SPC_KE_DATE0);
lc640_write_long_ptr(lc640_adr+8,(char*)&ke_date[0]);
ke_date[1]=lc640_read_long(EE_SPC_KE_DATE1);	
lc640_write_long_ptr(lc640_adr+12,(char*)&ke_date[1]);

data[0]=(char)(BKP->DR1);
data[1]=(char)(BKP->DR2);
data[2]=(char)(BKP->DR3);
data[3]=0;
lc640_write_long_ptr(lc640_adr+8,data);
	
data[0]=time_H;
data[1]=time_M;
data[2]=time_S;
data[3]=0;
lc640_write_long_ptr(lc640_adr+12,data);
 
}


//-----------------------------------------------
void vz_mem_hndl(unsigned short in)
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1)
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)
	{      
     
vz_mem_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[2]=='Z')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto vz_mem_hndl_end;
     	else goto vz_mem_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto vz_mem_hndl_end;
     		else goto vz_mem_hndl_lbl1;
     		}

     	}	


	
	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	}

vz_mem_hndl_end:
__nop(); 

}

//-----------------------------------------------
void speedz_mem_hndl(unsigned short in)	 //отметка об ускоренном заряде (1-старт, 5-автоматический старт, 0-штатное завершение, 10-прерван вручную)
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if((in==1)||(in==5)) //старт ускоренного заряда
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='s';
	data[1]=0;
	if(in==5)data[1]='a'; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//штатное завершение
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='s')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// прервано оператором
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='s')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}
  

//-----------------------------------------------
void uz_mem_hndl(unsigned short in)	 //отметка об уравнительном заряде
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1) //старт уравнительного заряда
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='u';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//штатное завершение
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='u')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// прервано оператором
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='u')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
				
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}

//-----------------------------------------------
void fz_mem_hndl(unsigned short in)	 //отметка об формовочном заряде
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
long temp_time;
char time_H,time_M,time_S;

temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
time_H=(char)((temp_time)/3600);
time_M=(char)(((temp_time)%3600)/60);
time_S=(char)(((temp_time)%3600)%60);

if(in==1) //старт формовочного заряда
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='f';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=(char)(BKP->DR1);
	data[1]=(char)(BKP->DR2);
	data[2]=(char)(BKP->DR3);
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);
	
	data[0]=time_H;
	data[1]=time_M;
	data[2]=time_S;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//штатное завершение
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='f')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// прервано оператором
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='f')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=(char)(BKP->DR1);
		data[1]=(char)(BKP->DR2);
		data[2]=(char)(BKP->DR3);
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
		
		data[0]=time_H;
		data[1]=time_M;
		data[2]=time_S;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}

#include <stm32f10x_lib.h>
#include "beep.h"
#include "main.h"
#include "avar_hndl.h"
#include "mess.h"
#include "control.h"

//***********************************************
//«вуки
bool bSILENT;
unsigned long beep_stat_temp,beep_stat;
signed short beep_stat_cnt;
char beep_cnt;
char bU_BAT2REL_AV_BAT;


//-----------------------------------------------
void beep_drv(void)	 //”правление бипером, физическое. 10√ц
{
//GPIOC->CRL&=~0xf0000000;GPIOC->CRL|=0x30000000;GPIOC->ODR&=~(1<<7);

if((beep_stat_temp&0x00000001UL)&&(ZV_ON))
    	{
    	GPIOC->ODR|=(1<<7);
    	beep_cnt=6;
    	}
else GPIOC->ODR&=~(1<<7);

beep_stat_temp>>=1;
if(--beep_stat_cnt<=0)
	{
	beep_stat_cnt=32;
	beep_stat_temp=beep_stat;
	}

if(zv_test_cnt)
	{
	zv_test_cnt--;
	GPIOC->ODR|=(1<<7);
	zv_test_sign=1;
	}
else zv_test_sign=0;

//IO1SET|=(1UL<<27);
//FIO1SET|=(1UL<<27);
//GPIOC->ODR|=(1<<7);
}

//-----------------------------------------------
void beep_init(long zvuk,char fl) 
{
if(fl=='O')
	{
	beep_stat_temp=zvuk;
	beep_stat=0x0L;
	beep_stat_cnt=32;
	} 
else if(fl=='A')
	{
	beep_stat_temp=zvuk;
	beep_stat=zvuk;
	beep_stat_cnt=32;
	}	 

else if(fl=='R')
	{
	beep_stat=zvuk;
	}	
		          
else if(fl=='S')
	{
	beep_stat_temp=0x0L;
	beep_stat=0x0L;
	beep_stat_cnt=32;
	}	
}

//-----------------------------------------------
//”правление бипером, логическое, 1√ц
void beep_hndl(void) 
{ 
static char bcnt;
bcnt++; 
if(bcnt>9)bcnt=0;
//bU_BAT2REL_AV_BAT=0;
if((avar_ind_stat)/*//||(ips_bat_av_stat)*/)beep_init(0x33333333,'R');

else if(uout_av) beep_init(0x0033333,'R');

else if ( (((bat[0]._Ub<(USIGN*10))&&(BAT_IS_ON[0]==bisON))||((bat[1]._Ub<(USIGN*10))&&(BAT_IS_ON[1]==bisON)))) 
	{
	if(!bSILENT)
		{
		beep_init(0x01010101,'R');
		mess_send(MESS2RELE_HNDL,PARAM_RELE_BAT_IS_DISCHARGED,1,5);
		}

	//bU_BAT2REL_AV_BAT=1;
	}

else if ( (((bat[0]._Ib<(-IKB))&&(BAT_IS_ON[0]==bisON))||((bat[1]._Ib<(-IKB))&&(BAT_IS_ON[1]==bisON)))) 
	{
	if(!bSILENT)beep_init(0x00010001,'R');
	//bU_BAT2REL_AV_BAT=1;
	}

else if ( ((bat[0]._temper_stat&0x03)||(bat[1]._temper_stat&0x03)) )
	{
	if(!bSILENT) beep_init(0x00000005,'R');
	}


else 
	{
	beep_init(0x00000000,'S');
	bSILENT=FALSE;
	} 


//if(!avar_ind_stat)beep_init(0x00000000,'R');

/*if(K[MNL]==ON)
	{ */
//if(((av_beep&0xffff)&&(ind!=iK)&&(ind!=iTst))
//	/*||((!T_EXT_ZVUK_EN[0])&&((tout_stat[0]==tMIN)||(tout_stat[0]==tMAX)))*/)beep_init(0x55555555,'R'); 

//else if(bUrazr&&(!bUOFF))beep_init(0x00010001,'R');

//else if(bIrazr&&(cnt_beep==0))beep_init(0x01010101,'R');

/*
else if(bTsi||bTsb)
	{
	if(!bcnt)beep_init(0x00000001,'O');
     }

else if(bIbr) beep_init(0x00000001,'R');
*/


//else beep_init(0x00000000,'S');



bU_BAT2REL_AV_BAT=0;

}

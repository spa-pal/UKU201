#include <stm32f10x_type.h>
#include "global_define.h"
#include "main.h"
#include "mess.h"

char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
int mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];


//-----------------------------------------------
void mess_hndl(void)
{
char i;
for(i=0;i<MESS_DEEP;i++)
	{
	if((mess[i])&&(mess[i]==mess_old[i])&&(mess_cnt[i]))
		{
		mess_cnt[i]--;
		if(!mess_cnt[i])mess[i]=MESS_ZERO;
		
		}
	mess_old[i]=mess[i];
	} 
}

//-----------------------------------------------
void mess_send(char _mess, short par0, short par1, char _time)
{
char i;
i=0;

do
	{
	i++;
	}
while((mess[i])&&(mess[i]!=_mess)&&(i<MESS_DEEP));

mess[i]=_mess;  
mess_par0[i]=par0;
mess_par1[i]=par1;
mess_cnt[i]=_time; 
 
}

//-----------------------------------------------
char mess_find(char _mess)
{
char i;
i=0;

do
	{
	i++;
	}
while((mess[i]!=_mess)&&(i<MESS_DEEP));

if(mess[i]==_mess)
	{ 
	mess_data[0]=mess_par0[i];  
	mess_data[1]=mess_par1[i];
	mess[i]=MESS_ZERO;
	mess_cnt[i]=0;
	return 1; 
	}
else return 0; 
 
}

//-----------------------------------------------
char mess_find_unvol(char _mess)
{
char i;
i=0;

do
	{
	i++;
	}
while((mess[i]!=_mess)&&(i<MESS_DEEP));

if(mess[i]==_mess)
	{ 
	mess_data[0]=mess_par0[i];  
	mess_data[1]=mess_par1[i];
	return 1; 
	}
else return 0; 
 
}

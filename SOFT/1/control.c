#include "control.h"
#include <stm32f10x_lib.h>
#include "main.h"

//***********************************************
//ÀÖÏ
unsigned short adc_buff[10][16];
unsigned short adc_buff_[10];
char adc_ch, adc_cnt;
const unsigned ADC_CH_CONST[]={0,1,4,5,6,7,16,17}; 
char adc_bit_zero=1;

short adc_buff_virt_0=578;

//-----------------------------------------------
void adc_drv(void)
{
int temp_S;
char i,ii;

adc_buff[adc_ch][adc_cnt]=(signed short)(ADC1->DR);
//adc_buff_[adc_ch]=adc_buff[adc_ch][adc_cnt];

adc_ch++;
if(adc_ch>=8)
	{
	adc_ch=0;
	adc_cnt++;
	if(adc_cnt>=16)
		{
		adc_cnt=0;
		adc_bit_zero=0;
		}
	//if((adc_cnt&0x03)==0)
		{
		if(adc_bit_zero)
			{
			for(i=0;i<8;i++)
				{
				temp_S=0;
				for(ii=0;ii<adc_cnt;ii++)
					{
					temp_S+=adc_buff[i][ii];
					}
				if(adc_cnt)adc_buff_[i]=temp_S/(adc_cnt+0);
				else adc_buff_[i]=temp_S;
				}
			}
		else
			{
			for(i=0;i<8;i++)
				{
				temp_S=0;
				for(ii=0;ii<16;ii++)
					{
					temp_S+=adc_buff[i][ii];
					}
				adc_buff_[i]=temp_S>>4;
				}
			}
		}
	}
ADC1->CR2  &=  ~0x00500000;
ADC1->SQR3  = ADC_CH_CONST[adc_ch];
ADC1->CR2  |=  0x00500000;
}

//-----------------------------------------------
void adc_init(void)
{
/*ADC_InitTypeDef ADC_InitStructure;

//clock for ADC (max 14MHz --> 60/6=10MHz)
RCC_ADCCLKConfig (RCC_PCLK2_Div6);
// enable ADC system clock
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);*/

//int temp_S;
//char i;

adc_ch=0;
 
RCC->APB2ENR |= (1<<9);                      /* enable periperal clock for ADC1      */

ADC1->SQR1  = (0<<20);//0x00000000;                    /* 6 conversions                    */
ADC1->SQR3  = ADC_CH_CONST[adc_ch];//(5<<0);//|(1<<5)|(4<<10)|(5<<15)|(6<<20)|(7<<25);     				/* chn10  */
//ADC1->SMPR1 = 5;     					/* set sample time (55,5 cycles)        */ 

ADC1->CR1   =  0x00000100;                   /* use independant mode, SCAN mode      */
ADC1->CR2   =  0x000E0003;                   /* data align right, cont. conversion   */
                                               /* EXTSEL = SWSTART                     */ 
                                               /* enable ADC, DMA mode                 */
ADC1->CR2  |=  0x00500000;                   /* start SW conver	   */
}

//-----------------------------------------------
void matemat(void)
{
//signed short temp_SS;
signed long temp_SL/*,temp_SL_*/;
char /*temp,*/i;

temp_SL=(signed long)adc_buff_[0];
temp_SL*=KunetA;
temp_SL/=6000L;
net_Ua=(signed short)temp_SL;
	
temp_SL=(signed long)adc_buff_[1];
temp_SL*=KunetB;
temp_SL/=6000L;
net_Ub=(signed short)temp_SL;
//net_Ub=KunetB;
	
temp_SL=(signed long)adc_buff_[2];
temp_SL*=KunetC;
temp_SL/=6000L;
net_Uc=(signed short)temp_SL;

if((adc_buff_[3]>800)&&(adc_buff_[3]<3800))bat[0]._nd=0;
else bat[0]._nd=1;
temp_SL=(signed long)adc_buff_[3];
temp_SL*=Ktbat[0];
temp_SL/=20000L;
temp_SL-=273L;
bat[0]._Tb=(signed short)temp_SL;

//bat[0]._Tb=(signed short)adc_bit_zero;
bat[0]._Ib=(signed short)adc_bit_zero;
bat[0]._Ub=(signed short)adc_cnt;
//bat[0]._Tb=(signed short)adc_buff_[3]; 

for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._cnt<25)
     	{
     	bps[i]._Ii=bps[i]._buff[0]+(bps[i]._buff[1]*256);
     	bps[i]._Uin=bps[i]._buff[2]+(bps[i]._buff[3]*256);
     	bps[i]._Uii=bps[i]._buff[4]+(bps[i]._buff[5]*256);
     	bps[i]._Ti=(signed)(bps[i]._buff[6]);
     	bps[i]._adr_ee=bps[i]._buff[7];
     	bps[i]._flags_tm=bps[i]._buff[8];
	    bps[i]._rotor=bps[i]._buff[10]+(bps[i]._buff[11]*256);    
     	} 
	else 
     	{
     	bps[i]._Uii=1; 
     	bps[i]._Ii=2;
     	bps[i]._Uin=3;
     	bps[i]._Ti=0;
     	bps[i]._flags_tm=0; 
	    bps[i]._rotor=0;    
     	}
     
     }

temp_SL=(signed long)adc_buff_virt_0;
temp_SL*=(signed long)Kuout;
temp_SL/=1000L;
out_U=(short)temp_SL;
//out_U=246;

//bat[0]._Tb=45;

}

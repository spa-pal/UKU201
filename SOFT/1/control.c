#include "control.h"
#include <stm32f10x_lib.h>

//***********************************************
//ÀÖÏ
unsigned short adc_buff[10][16];
unsigned short adc_buff_[10];
char adc_ch, adc_cnt;
const unsigned ADC_CH_CONST[]={0,1,4,5,6,7,16,17}; 

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
		}
	//if((adc_cnt&0x03)==0)
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


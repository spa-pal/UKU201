#include "rtl.h"
#include "control.h"
#include <stm32f10x_lib.h>
#include "main.h"
#include "beep.h"
#include "mess.h"
#include "full_can.h"
#include "modbus.h"
#define can1_out mcp2515_transmit

//***********************************************
//АЦП
unsigned short adc_buff[10][16];
unsigned short adc_buff_[10];
char adc_ch, adc_cnt;
const unsigned ADC_CH_CONST[]={0,1,4,5,6,7,16,17}; 
char adc_bit_zero=1;

//-----------------------------------------------
//Автокалибровка
avt_klbr_mode_enum avt_klbr_mode=akmOFF;
char avt_klbr_num;
char avt_klbr_phase;
char avt_klbr_main_cnt;
short avt_klbr_real_value;
short avt_klbr_necc_value;
short avt_klbr_err_cnt;
signed short avt_klbr_err;
char avt_klbr_dev_adr;
char avt_klbr_ch;
char avt_klbr_cmd;
char avt_klbr_err_sign;
char avt_klbr_err_sign_old;

char avt_klbr_main_cnt_;
char avt_klbr_mode_ui, avt_klbr_mode_un, avt_klbr_mode_i, avt_klbr_mode_t;
short avt_klbr_real_value_ui, avt_klbr_real_value_un, avt_klbr_real_value_i, avt_klbr_real_value_t; 
short avt_klbr_necc_value_ui, avt_klbr_necc_value_un, avt_klbr_necc_value_i, avt_klbr_necc_value_t; 
char avt_klbr_num_ui, avt_klbr_num_un, avt_klbr_num_i, avt_klbr_num_t;
short avt_klbr_err_cnt_ui, avt_klbr_err_cnt_un, avt_klbr_err_cnt_i, avt_klbr_err_cnt_t;
signed avt_klbr_err_ui, avt_klbr_err_un, avt_klbr_err_i, avt_klbr_err_t;
char avt_klbr_err_sign_ui, avt_klbr_err_sign_un, avt_klbr_err_sign_i, avt_klbr_err_sign_t;
char avt_klbr_err_sign_old_ui, avt_klbr_err_sign_old_un, avt_klbr_err_sign_old_i, avt_klbr_err_sign_old_t;
char avt_klbr_phase_ui, avt_klbr_phase_un, avt_klbr_phase_i, avt_klbr_phase_t;
char avt_klbr_cmd_ui, avt_klbr_cmd_un, avt_klbr_cmd_i, avt_klbr_cmd_t;
char avt_klbr_dev_adr_ui, avt_klbr_dev_adr_un, avt_klbr_dev_adr_i, avt_klbr_dev_adr_t;

short adc_buff_virt_0=578;

//**********************************************
//Работа с БПСами
char num_of_wrks_bps;
char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
char bps_hndl_2sec_cnt;
unsigned short bps_on_mask,bps_off_mask;
char num_necc_up,num_necc_down;
unsigned char sh_cnt0,b1Hz_sh;
signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
signed short mat_temper;

//***********************************************
//Ротация ведущего источника
char numOfForvardBps,numOfForvardBps_old;
char numOfForvardBps_minCnt;
short numOfForvardBps_hourCnt;

//***********************************************
//Управление ШИМом
signed short cntrl_stat=600;
signed short cntrl_stat_old=600;
signed short cntrl_stat_new;
signed short Ibmax;
unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
unsigned char	ch_cnt1,b1_30Hz_ch;
unsigned char	ch_cnt2,b1_10Hz_ch;
unsigned short IZMAX_;
unsigned short IZMAX_70;
unsigned short IZMAX_130;
unsigned short Ubpsmax;
unsigned short cntrl_stat_blck_cnt;
short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];
char cntrl_hndl_plazma;

//-----------------------------------------------
//Блокировка ИПС
signed short ipsBlckSrc;
signed short ipsBlckLog;
signed short ipsBlckStat;

//***********************************************
//Уравнительный заряд
enum_vz1_stat vz1_stat=vz1sOFF, vz1_stat_old=vz1sOFF;
short vz1_stat_cnt;
long vz1_wrk_cnt;
long vz1_up_cnt;
char volt_region;
short volt_region_cnt;

//***********************************************
//Формовочный заряд
enum_vz2_stat vz2_stat=vz2sOFF, vz2_stat_old=vz2sOFF;
short vz2_stat_cnt;
long vz2_wrk_cnt;
long vz2_up_cnt;
signed short vz2_stat_ph2_cnt;

//***********************************************
//Высоковольтный выравнивающий заряд
enum_hv_vz_stat hv_vz_stat=hvsOFF,hv_vz_stat_old;
short hv_vz_stat_cnt;
long hv_vz_wrk_cnt;
long hv_vz_up_cnt;

//***********************************************
// Параллельная работа в случае перегрева источника
char bPARALLEL_NOT_ENOUG;
char bPARALLEL_ENOUG;
char bPARALLEL;

//***********************************************
//Выравнивание токов ИПС
char ica_plazma[10];
char ica_timer_cnt;
signed short ica_my_current;
signed short ica_your_current;
signed short ica_u_necc;
signed short ica_cntrl_hndl;
signed short ica_cntrl_hndl_cnt;
U8 tcp_soc_avg;
U8 tcp_connect_stat;

char avar_bps_reset_cnt;


//***********************************************
//Выравнивание токов
short avg_main_cnt=20;
signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
signed int avg;
char bAVG;
char avg_cnt;  
char avg_num; 
char avg_vektor;

//***********************************************
//Состояние внешних датчиков
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum_tout_stat tout_stat[4];
signed short t_ext[3];

signed char sk_cnt_dumm[4],sk_cnt[4],sk_av_cnt[4];
enum_sk_stat sk_stat[4]={ssOFF,ssOFF,ssOFF,ssOFF},sk_stat_old[4]={ssOFF,ssOFF,ssOFF,ssOFF};
enum_sk_av_stat sk_av_stat[4]={sasOFF,sasOFF,sasOFF,sasOFF},sk_av_stat_old[4];
signed short t_box,t_box_warm,t_box_vent;
char TELECORE2017_EXT_VENT_PWM,TELECORE2017_INT_VENT_PWM;
char ND_EXT[3];

//**********************************************
//Контроль наличия батарей
signed short 	main_kb_cnt;
signed short 	kb_cnt_1lev;
signed short 	kb_cnt_2lev;
char 		kb_full_ver;
char /*kb_start[2],*/kb_start_ips;
signed short ibat_ips,ibat_ips_;
char ips_bat_av_vzvod=0;
char ips_bat_av_stat=0;

//-----------------------------------------------
//Контроль выходного напряжения
signed short outVoltContrHndlCntUp;		//Счетчик, считает в плюс в случае превышения
signed short outVoltContrHndlCntDn;		//Счетчик, считает в плюс в случае занижения
char uout_av;							//Авария по выходному напряжению 0 - норма, 1 - занижено, 2 - завышено

//-----------------------------------------------
void avg_hndl(void)
{ 
char i;

//#define AVGCNTMAX	5
if(avg_main_cnt)
	{
	avg_main_cnt--;
	//goto avg_hndl_end;
	return;
	}                 

avg_main_cnt=5;
avg_num=0;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._state==bsWRK)&&(bps[i]._cnt<20))avg_num++;
	}

/*if((K[NUMI]>=1)&&(bps_state[0]==ssWRK))	avg_num++;
if((K[NUMI]>=2)&&(bps_state[1]==ssWRK))	avg_num++;
if((K[NUMI]>=3)&&(bps_state[2]==ssWRK))	avg_num++;*/

if(avg_vektor) avg_vektor=0;
else avg_vektor=1;
	
if(avg_num<2)
	{
	//goto avg_hndl_end;
	return;
	}
	
else
	{
	i_avg_min=5000;
	i_avg_max=0;
	i_avg_summ=0;
	for(i=0;i<NUMIST;i++)
		{
		if(bps[i]._state==bsWRK)
			{
			if(bps[i]._Ii>i_avg_max)i_avg_max=bps[i]._Ii;
			if(bps[i]._Ii<i_avg_min)i_avg_min=bps[i]._Ii;
			
			i_avg_summ+=bps[i]._Ii;
			}
		}
	i_avg=i_avg_summ/avg_num;	
	
	if(i_avg_min==0)i_avg_min=1;

	avg=i_avg_max;
	avg*=100;
	avg/=i_avg_min;

	if(avg>130) bAVG=1;
	if(avg<110) bAVG=0;

	if(bAVG==1)
		{
		for(i=0;i<NUMIST;i++)
			{
			if(bps[i]._state==bsWRK)
				{
				if((bps[i]._Ii>i_avg)&&(!avg_vektor))bps[i]._x_--;
				if((bps[i]._Ii<i_avg)&&(avg_vektor))bps[i]._x_++;
			
				if(bps[i]._x_<-50)bps[i]._x_=-50;
				if(bps[i]._x_>50)bps[i]._x_=50;	
				}
			}		
		}			
	}   	 


avg_hndl_end:
__nop();  
}

//-----------------------------------------------
//Управление светодиодами, логическое, 10Гц
void led_hndl(void)
{
ledERROR=0;
ledWARNING=0;
ledUOUTGOOD=0;
ledCAN=1;

if((net_av)||(net_av))ledERROR=1;

if(!uout_av)ledUOUTGOOD=1;


if(test_led_stat==1)
	{
	ledERROR=0;
	ledWARNING=0;
	ledUOUTGOOD=1;
	ledCAN=0;
	}
else if(test_led_stat==2)
	{
	ledERROR=0;
	ledWARNING=1;
	ledUOUTGOOD=0;
	ledCAN=0;
	}
else if(test_led_stat==3)
	{
	ledERROR=1;
	ledWARNING=0;
	ledUOUTGOOD=0;
	ledCAN=0;
	}
else if(test_led_stat==4)
	{
	ledERROR=0;
	ledWARNING=0;
	ledUOUTGOOD=0;
	ledCAN=1;
	}
else if(test_led_stat==5)
	{
	ledERROR=0;
	ledWARNING=0;
	ledUOUTGOOD=0;
	ledCAN=0;
	}
if(test_led_cnt)
	{
	test_led_cnt--;
	if(!test_led_cnt)
		{
		test_led_stat=0;
		}
	}

if(factory_settings_led_reg0)
	{
	factory_settings_led_reg0--;
	ledERROR=0;
	ledWARNING=0;
	ledUOUTGOOD=0;
	ledCAN=0;
	}
if((factory_settings_led_reg0==7)||(factory_settings_led_reg0==1)) ledUOUTGOOD=1;
else if((factory_settings_led_reg0==6)||(factory_settings_led_reg0==2)) ledWARNING=1;
else if((factory_settings_led_reg0==5)||(factory_settings_led_reg0==3)) ledERROR=1;
else if(factory_settings_led_reg0==4) ledCAN=1;

if(factory_settings_led_reg1)
	{
	factory_settings_led_reg1--;
	ledERROR=0;
	ledWARNING=0;
	ledUOUTGOOD=0;
	ledCAN=0;
	}

if((factory_settings_led_reg1==5)||(factory_settings_led_reg1==3)||(factory_settings_led_reg1==1))
	{
	ledERROR=1;
	ledWARNING=1;
	ledUOUTGOOD=1;
	ledCAN=1;
	}

}

//-----------------------------------------------
//Управление светодиодами, физическое, 10Гц
void led_drv(void)
{
static char led_drv_main_cnt;
led_drv_main_cnt++;
led_drv_main_cnt&=0x0f;

if(led_drv_main_cnt&0x01)
	{
	}
/*	*/
//Зеленый светодиод "выходное напряжение в норме"
if(ledUOUTGOOD==1)
	{
	GPIOC->ODR&=~(1<<6);
	}
else 
	{
	GPIOC->ODR|=(1<<6);
	}

//Желтый светодиод "тревога в одном из устройств"
if(ledWARNING==1)
	{
	GPIOB->ODR&=~(1<<12);
	}
else 
	{
	GPIOB->ODR|=(1<<12);
	}

//Красный светодиод "авария в одном из устройств"
if(ledERROR==1)
	{
	GPIOB->ODR&=~(1<<11);
	}
else 
	{
	GPIOB->ODR|=(1<<11);
	}

//Зеленый светодиод "связь по КАН в норме"
if(ledCAN==1)
	{ 
	GPIOB->ODR&=~(1<<10);
	}
else 
	{
	GPIOB->ODR|=(1<<10);
	}

}

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

#define KLBR 0xEE

//-----------------------------------------------
void avt_klbr_start(void)   //Старт автоматической колибровки БПСов 
{
avt_klbr_err_sign_old=0;
avt_klbr_err_sign=0;
avt_klbr_phase=1;
}

//-----------------------------------------------
void avt_klbr_start_ui(void)   //Старт автоматической колибровки выходного напряжения БПСов 
{
avt_klbr_err_sign_old_ui=0;
avt_klbr_err_sign_ui=0;
avt_klbr_phase_ui=1;
}

//-----------------------------------------------
void avt_klbr_start_un(void)   //Старт автоматической колибровки напряжения шины БПСов 
{
avt_klbr_err_sign_old_un=0;
avt_klbr_err_sign_un=0;
avt_klbr_phase_un=1;
}

//-----------------------------------------------
void avt_klbr_start_i(void)   //Старт автоматической колибровки тока БПСов 
{
avt_klbr_err_sign_old_i=0;
avt_klbr_err_sign_i=0;
avt_klbr_phase_i=1;
}

//-----------------------------------------------
void avt_klbr_start_t(void)   //Старт автоматической колибровки температуры БПСов 
{
avt_klbr_err_sign_old_t=0;
avt_klbr_err_sign_t=0;
avt_klbr_phase_t=1;
}


//-----------------------------------------------
void avt_klbr_hndl_(void)   //драйвер автоматической колибровки БПСов и шунта, 1Гц
{
if(++avt_klbr_main_cnt_>=4)avt_klbr_main_cnt_=0;


if(avt_klbr_mode_ui)
	{
	avt_klbr_real_value_ui = bps[avt_klbr_num_ui-1]._Uii;
	}
else if(avt_klbr_mode_un)
	{
	avt_klbr_real_value_un = bps[avt_klbr_num_un-1]._Uin;
	}
else if(avt_klbr_mode_i)
	{
	avt_klbr_real_value_i = bps[avt_klbr_num_i-1]._Ii;
	}
else if(avt_klbr_mode_t)
	{
	avt_klbr_real_value_t = bps[avt_klbr_num_t-1]._Ti;
	}

avt_klbr_err_ui= (signed) ((((signed long)avt_klbr_real_value_ui-(signed long)avt_klbr_necc_value_ui)*10000L)/(signed long)((avt_klbr_real_value_ui!=0)? avt_klbr_real_value_ui : 1));
avt_klbr_err_sign_ui=1;
if(avt_klbr_err_ui<0)avt_klbr_err_sign_ui=-1;
if(avt_klbr_err_sign_old_ui==0)avt_klbr_err_sign_old_ui=avt_klbr_err_sign_ui;

avt_klbr_err_un= (signed) ((((signed long)avt_klbr_real_value_un-(signed long)avt_klbr_necc_value_un)*10000L)/(signed long)((avt_klbr_real_value_un!=0)? avt_klbr_real_value_un : 1));
avt_klbr_err_sign_un=1;
if(avt_klbr_err_un<0)avt_klbr_err_sign_un=-1;
if(avt_klbr_err_sign_old_un==0)avt_klbr_err_sign_old_un=avt_klbr_err_sign_un;

avt_klbr_err_i= (signed) ((((signed long)avt_klbr_real_value_i-(signed long)avt_klbr_necc_value_i)*10000L)/(signed long)((avt_klbr_real_value_i!=0)? avt_klbr_real_value_i : 1));
avt_klbr_err_sign_i=1;
if(avt_klbr_err_i<0)avt_klbr_err_sign_i=-1;
if(avt_klbr_err_sign_old_i==0)avt_klbr_err_sign_old_i=avt_klbr_err_sign_i;

avt_klbr_err_t= (signed) ((((signed long)avt_klbr_real_value_t-(signed long)avt_klbr_necc_value_t)*10000L)/(signed long)((avt_klbr_real_value_t!=0)? avt_klbr_real_value_t : 1));
avt_klbr_err_sign_t=1;
if(avt_klbr_err_t<0)avt_klbr_err_sign_t=-1;
if(avt_klbr_err_sign_old_t==0)avt_klbr_err_sign_old_t=avt_klbr_err_sign_t;

if(avt_klbr_phase_ui==2)
	{
	if(abs(avt_klbr_err_ui)<5)
		{
		avt_klbr_mode_ui=0;
		avt_klbr_num_ui=0;
		//*((short*)(((short*)&modbus_register_1000)+modbus_register_offset_ui))=0;
		*((short*)(((short*)&modbus_register_1000)+modbus_register_offset_ui))=0;
		}
	else if((avt_klbr_err_ui>0)&&(avt_klbr_main_cnt==0)) 
		{
		avt_klbr_cmd_ui=4;
		}
	else if((avt_klbr_err_ui<0)&&(avt_klbr_main_cnt==0))
		{
		avt_klbr_cmd_ui=2;
		}
	}
else
	{
	if(avt_klbr_err_ui>0) 
		{
 		avt_klbr_cmd_ui=5;
		}
	else if(avt_klbr_err_ui<0)
		{
		avt_klbr_cmd_ui=3;
		}
	}

if(avt_klbr_phase_un==2)
	{
	if(abs(avt_klbr_err_un)<5)
		{
		avt_klbr_mode_un=0;
		avt_klbr_num_un=0;
		*((short*)(((short*)&modbus_register_1000)+modbus_register_offset_un))=0;
		}
	else if((avt_klbr_err_un>0)&&(avt_klbr_main_cnt==0)) 
		{
		avt_klbr_cmd_un=4;
		}
	else if((avt_klbr_err_un<0)&&(avt_klbr_main_cnt==0))
		{
		avt_klbr_cmd_un=2;
		}
	}
else
	{
	if(avt_klbr_err_un>0) 
		{
 		avt_klbr_cmd_un=5;
		}
	else if(avt_klbr_err_un<0)
		{
		avt_klbr_cmd_un=3;
		}
	}

if(avt_klbr_phase_i==2)
	{
	if(abs(avt_klbr_err_i)<5)
		{
		avt_klbr_mode_i=0;
		avt_klbr_num_i=0;
		*((short*)(((short*)&modbus_register_1000)+modbus_register_offset_i))=0;
		}
	else if((avt_klbr_err_i>0)&&(avt_klbr_main_cnt==0)) 
		{
		avt_klbr_cmd_i=4;
		}
	else if((avt_klbr_err_i<0)&&(avt_klbr_main_cnt==0))
		{
		avt_klbr_cmd_i=2;
		}
	}
else
	{
	if(avt_klbr_err_i>0) 
		{
 		avt_klbr_cmd_i=5;
		}
	else if(avt_klbr_err_i<0)
		{
		avt_klbr_cmd_i=3;
		}
	}

if(avt_klbr_phase_t==2)
	{
	if(abs(avt_klbr_err_t)<5)
		{
		avt_klbr_mode_t=0;
		avt_klbr_num_t=0;
		*((short*)(((short*)&modbus_register_1000)+modbus_register_offset_t))=0;
		}
	else if((avt_klbr_err_t>0)&&(avt_klbr_main_cnt==0)) 
		{
		avt_klbr_cmd_t=4;
		}
	else if((avt_klbr_err_t<0)&&(avt_klbr_main_cnt==0))
		{
		avt_klbr_cmd_t=2;
		}
	}
else
	{
	if(avt_klbr_err_t>0) 
		{
 		avt_klbr_cmd_t=5;
		}
	else if(avt_klbr_err_t<0)
		{
		avt_klbr_cmd_t=3;
		}
	}

if(avt_klbr_mode_ui!=0)
	{
	if(avt_klbr_err_sign_ui!=avt_klbr_err_sign_old_ui)
		{
		avt_klbr_phase_ui=2;
		}
	} 
avt_klbr_err_sign_old_ui=avt_klbr_err_sign_ui;

if(avt_klbr_mode_un!=0)
	{
	if(avt_klbr_err_sign_un!=avt_klbr_err_sign_old_un)
		{
		avt_klbr_phase_un=2;
		}
	} 
avt_klbr_err_sign_old_un=avt_klbr_err_sign_un;

if(avt_klbr_mode_i!=0)
	{
	if(avt_klbr_err_sign_i!=avt_klbr_err_sign_old_i)
		{
		avt_klbr_phase_i=2;
		}
	} 
avt_klbr_err_sign_old_i=avt_klbr_err_sign_i;

if(avt_klbr_mode_t!=0)
	{
	if(avt_klbr_err_sign_t!=avt_klbr_err_sign_old_t)
		{
		avt_klbr_phase_t=2;
		}
	} 
avt_klbr_err_sign_old_t=avt_klbr_err_sign_t;

if(avt_klbr_num_ui==1)avt_klbr_dev_adr_ui=0;
else if(avt_klbr_num_ui==2)avt_klbr_dev_adr_ui=1;
else if(avt_klbr_num_ui==3)avt_klbr_dev_adr_ui=2;
else if(avt_klbr_num_ui==4)avt_klbr_dev_adr_ui=3;
else if(avt_klbr_num_ui==5)avt_klbr_dev_adr_ui=4;
else if(avt_klbr_num_ui==6)avt_klbr_dev_adr_ui=5;
else if(avt_klbr_num_ui==7)avt_klbr_dev_adr_ui=6;

if(avt_klbr_num_un==1)avt_klbr_dev_adr_un=0;
else if(avt_klbr_num_un==2)avt_klbr_dev_adr_un=1;
else if(avt_klbr_num_un==3)avt_klbr_dev_adr_un=2;
else if(avt_klbr_num_un==4)avt_klbr_dev_adr_un=3;
else if(avt_klbr_num_un==5)avt_klbr_dev_adr_un=4;
else if(avt_klbr_num_un==6)avt_klbr_dev_adr_un=5;
else if(avt_klbr_num_un==7)avt_klbr_dev_adr_un=6;

if(avt_klbr_num_i==1)avt_klbr_dev_adr_i=0;
else if(avt_klbr_num_i==2)avt_klbr_dev_adr_i=1;
else if(avt_klbr_num_i==3)avt_klbr_dev_adr_i=2;
else if(avt_klbr_num_i==4)avt_klbr_dev_adr_i=3;
else if(avt_klbr_num_i==5)avt_klbr_dev_adr_i=4;
else if(avt_klbr_num_i==6)avt_klbr_dev_adr_i=5;
else if(avt_klbr_num_i==7)avt_klbr_dev_adr_i=6;

if(avt_klbr_num_t==1)		avt_klbr_dev_adr_t=0;
else if(avt_klbr_num_t==2)	avt_klbr_dev_adr_t=1;
else if(avt_klbr_num_t==3)	avt_klbr_dev_adr_t=2;
else if(avt_klbr_num_t==4)	avt_klbr_dev_adr_t=3;
else if(avt_klbr_num_t==5)	avt_klbr_dev_adr_t=4;
else if(avt_klbr_num_t==6)	avt_klbr_dev_adr_t=5;
else if(avt_klbr_num_t==7)	avt_klbr_dev_adr_t=6;

if((avt_klbr_phase_ui==1) || ((avt_klbr_phase_ui==2) && (avt_klbr_main_cnt==0)))
	{
	if(avt_klbr_mode_ui!=0)can1_out(avt_klbr_dev_adr_ui,avt_klbr_dev_adr_ui,KLBR,(0*16)+avt_klbr_cmd_ui,(0*16)+avt_klbr_cmd_ui,0,0,0);
	}

if((avt_klbr_phase_un==1) || ((avt_klbr_phase_un==2) && (avt_klbr_main_cnt==0)))
	{
	if(avt_klbr_mode_un!=0)can1_out(avt_klbr_dev_adr_un,avt_klbr_dev_adr_un,KLBR,(1*16)+avt_klbr_cmd_un,(1*16)+avt_klbr_cmd_un,0,0,0);
	}

if((avt_klbr_phase_i==1) || ((avt_klbr_phase_i==2) && (avt_klbr_main_cnt==0)))
	{
	if(avt_klbr_mode_i!=0)can1_out(avt_klbr_dev_adr_i,avt_klbr_dev_adr_i,KLBR,(2*16)+avt_klbr_cmd_i,(2*16)+avt_klbr_cmd_i,0,0,0);
	}

if((avt_klbr_phase_t==1) || ((avt_klbr_phase_t==2) && (avt_klbr_main_cnt==0)))
	{
	if(avt_klbr_mode_t!=0)can1_out(avt_klbr_dev_adr_t,avt_klbr_dev_adr_t,KLBR,(3*16)+avt_klbr_cmd_t,(3*16)+avt_klbr_cmd_t,0,0,0);
	}


//printf("mode = %d %d %d %d; num = %d %d %d %d;\r\n", avt_klbr_mode_ui, avt_klbr_mode_un, avt_klbr_mode_i, avt_klbr_mode_t, avt_klbr_num_ui, avt_klbr_dev_adr_ui/*avt_klbr_num_un*/,avt_klbr_cmd_ui/* avt_klbr_num_i*/,/* avt_klbr_num_t*/modbus_register_offset);
//printf("ui phase = %2d; real = %5d; necc = %5d; err = %3d %; err_cnt = %3d;\r\n", avt_klbr_phase_ui, avt_klbr_real_value_ui, avt_klbr_necc_value_ui, avt_klbr_err_ui, avt_klbr_err_cnt_ui);
//printf("un phase = %2d; real = %5d; necc = %5d; err = %3d %; err_cnt = %3d;\r\n", avt_klbr_phase_un, avt_klbr_real_value_un, avt_klbr_necc_value_un, avt_klbr_err_un, avt_klbr_err_cnt_un);
//printf(" i phase = %2d; real = %5d; necc = %5d; err = %3d %; err_cnt = %3d;\r\n", avt_klbr_phase_i, avt_klbr_real_value_i, avt_klbr_necc_value_i, avt_klbr_err_i, avt_klbr_err_cnt_i);
//printf(" t phase = %2d; real = %5d; necc = %5d; err = %3d %; err_cnt = %3d;\r\n", avt_klbr_phase_t, avt_klbr_real_value_t, avt_klbr_necc_value_t, avt_klbr_err_t, avt_klbr_err_cnt_t);

}

//-----------------------------------------------
void avt_klbr_hndl(void)   //драйвер автоматической колибровки БПСов и шунта, 1Гц
{

if(++avt_klbr_main_cnt>=4)avt_klbr_main_cnt=0;


if(avt_klbr_mode==akmUI)
	{
	/*if(avt_klbr_num==1)avt_klbr_real_value = bps[0]._Uii;
	else if(avt_klbr_num==2)avt_klbr_real_value = bps[1]._Uii;*/
	avt_klbr_real_value = bps[avt_klbr_num-1]._Uii;
	}
else if(avt_klbr_mode==akmUN)
	{
	/*if(avt_klbr_num==1)avt_klbr_real_value = bps[0]._Ii;
	else if(avt_klbr_num==2)avt_klbr_real_value = bps[1]._Ii;*/
	avt_klbr_real_value = bps[avt_klbr_num-1]._Uin;
	}
else if(avt_klbr_mode==akmI)
	{
	avt_klbr_real_value = bps[avt_klbr_num-1]._Ii;
	}
else if(avt_klbr_mode==akmT)
	{
	avt_klbr_real_value = bps[avt_klbr_num-1]._Ti;
	}

avt_klbr_err= (signed short) ((((signed long)avt_klbr_real_value-(signed long)avt_klbr_necc_value)*10000L)/(signed long)((avt_klbr_real_value!=0)? avt_klbr_real_value : 1));
avt_klbr_err_sign=1;
if(avt_klbr_err<0)avt_klbr_err_sign=-1;
if(avt_klbr_err_sign_old==0)avt_klbr_err_sign_old=avt_klbr_err_sign;

if(avt_klbr_phase==2)
	{
	if(abs(avt_klbr_err)<5)
		{
		avt_klbr_mode=akmOFF;
		avt_klbr_num=0;
		*((short*)(((short*)&modbus_register_1000)+modbus_register_offset))=0;
		}
	else if((avt_klbr_err>0)&&(avt_klbr_main_cnt==0)) 
		{
		avt_klbr_cmd=4;
		}
	else if((avt_klbr_err<0)&&(avt_klbr_main_cnt==0))
		{
		avt_klbr_cmd=2;
		}
	}
else
	{
	if(avt_klbr_err>0) 
		{
		//avt_klbr_cmd=4;
		//if((avt_klbr_err>10) && (avt_klbr_phase==1) ) avt_klbr_cmd=5;
		/*if(avt_klbr_phase==1)*/ avt_klbr_cmd=5;
		/*else avt_klbr_cmd=4;*/
		}
	else if(avt_klbr_err<0)
		{
	//	avt_klbr_cmd=2;
	//	if((avt_klbr_err<-10) && (avt_klbr_phase==1)) avt_klbr_cmd=3;
		/*if(avt_klbr_phase==1)*/ avt_klbr_cmd=3;
		/*else avt_klbr_cmd=2;*/
		}
	}

if(avt_klbr_mode!=akmOFF)
	{
	if(avt_klbr_err_sign!=avt_klbr_err_sign_old)
		{
		avt_klbr_phase=2;
		if(avt_klbr_phase==5)
			{
			//avt_klbr_phase=10;
			}
		}
	} 

avt_klbr_err_sign_old=avt_klbr_err_sign;

/*if(avt_klbr_phase==10)
	{
	if(avt_klbr_cmd==4)avt_klbr_cmd=2;
	else avt_klbr_cmd=4;
	}
else */

if((avt_klbr_err<2) && (avt_klbr_err>-2))
	{
//	avt_klbr_mode=akmOFF;
//	avt_klbr_num=0;
//	modbus_register_1022=0;
	}


if(avt_klbr_mode==akmUI) avt_klbr_ch=0;
if(avt_klbr_mode==akmUN) avt_klbr_ch=1;
if(avt_klbr_mode==akmI) avt_klbr_ch=2;
if(avt_klbr_mode==akmT) avt_klbr_ch=3;

if(avt_klbr_num==1)avt_klbr_dev_adr=0;
else if(avt_klbr_num==2)avt_klbr_dev_adr=1;
else if(avt_klbr_num==3)avt_klbr_dev_adr=2;

if(avt_klbr_err_cnt)
	{
	avt_klbr_err_cnt--;
	if(avt_klbr_err_cnt==0)
		{
		avt_klbr_mode=akmOFF;
		avt_klbr_num=0;

		}
	}


if((avt_klbr_phase==1) || ((avt_klbr_phase==2) && (avt_klbr_main_cnt==0)))
	{
	if(avt_klbr_mode!=akmOFF)can1_out(avt_klbr_dev_adr,avt_klbr_dev_adr,KLBR,(avt_klbr_ch*16)+avt_klbr_cmd,(avt_klbr_ch*16)+avt_klbr_cmd,0,0,0);

	}

printf("mode = %2d; num = %2d; phase = %2d; cnt = %d; real = %5d; necc = %5d; err = %3d %; err_cnt = %3d;\r\n", avt_klbr_mode, avt_klbr_num, avt_klbr_phase,avt_klbr_main_cnt
	, avt_klbr_real_value, avt_klbr_necc_value, avt_klbr_err, avt_klbr_err_cnt);

} 

//-----------------------------------------------
void apv_start(char in)
{
if(	(bps[in]._apv_timer_1_lev==0)&&
	(bps[in]._apv_cnt_1_lev==0)&&
	(bps[in]._apv_timer_2_lev==0) )
		{
 		bps[in]._apv_timer_1_lev=60;
		bps[in]._apv_cnt_1_lev=3;
		bps[in]._apv_timer_2_lev=(short)(APV_ON2_TIME*3600);
		}
}

//-----------------------------------------------
void apv_stop(char in)
{
bps[in]._apv_timer_1_lev=0;
bps[in]._apv_cnt_1_lev=0;
bps[in]._apv_timer_2_lev=0;
}

//-----------------------------------------------
void apv_drv(void)		//1 Гц
{
for(i=0;i<NUMIST;i++)
	{
	if(APV_ON1==apvOFF)		//если выключен первый уровень АПВ
		{
		bps[i]._apv_timer_1_lev=0;
		bps[i]._apv_cnt_1_lev=0;
		bps[i]._apv_timer_2_lev=0;
		}
	if(APV_ON2==apvOFF)	   //если выключен второй уровень АПВ
		{
		bps[i]._apv_timer_2_lev=0;
		}

	if(	(bps[i]._apv_timer_1_lev!=0)||	//если работает АПВ-1 или
		(bps[i]._apv_cnt_1_lev!=0)||	//работает АПВ-1 или
		(bps[i]._apv_timer_2_lev!=0) )		 //работает АПВ-2
			{
			if(bps[i]._state==bsWRK)
				{
				if(bps[i]._apv_succes_timer<60)
					{
					bps[i]._apv_succes_timer++;
					if(bps[i]._apv_succes_timer>=60)
						{
						apv_stop(i);
						}
					}
				}
			else bps[i]._apv_succes_timer=0;
			}

	if(bps[i]._apv_timer_1_lev)
		{
		bps[i]._apv_timer_2_lev=0;
		bps[i]._apv_timer_1_lev--;
		if(bps[i]._apv_timer_1_lev==0)
			{
			if(bps[i]._apv_cnt_1_lev)
				{
				bps[i]._apv_cnt_1_lev--;
				bps[i]._apv_timer_1_lev=60;
				bps[i]._apv_reset_av_timer=2;
				}
			else
				{
				if(APV_ON2==apvON)
					{
					bps[i]._apv_timer_1_lev=0;
					bps[i]._apv_cnt_1_lev=0;
					bps[i]._apv_timer_2_lev=(short)(APV_ON2_TIME*3600);
					}
				}
			
			}
		}
	if(bps[i]._apv_timer_2_lev)
		{
		bps[i]._apv_timer_2_lev--;
		if(bps[i]._apv_timer_2_lev==0)
			{
			bps[i]._apv_cnt_1_lev=2;
			bps[i]._apv_timer_1_lev=60;
			}
		}

	if(bps[i]._apv_reset_av_timer)bps[i]._apv_reset_av_timer--;
	} 
/*char i;
for(i=0;i<2;i++) 
	{
	if(apv_cnt_sec[i])
		{
		apv_cnt_sec[i]--;
		if(apv_cnt_sec[i]==0)
			{
			cnt_av_umax[i]=0;
			cnt_av_umin[i]=0;
			reset_apv_cnt[i]=600;
			}
		}
	
	if(reset_apv_cnt[i])
		{
		reset_apv_cnt[i]--;
		if(reset_apv_cnt[i]==0)
			{
			apv_cnt[i]=0;
			}
		}	
		
	if(hour_apv_cnt[i])
		{
		hour_apv_cnt[i]--;
		if(hour_apv_cnt[i]==0)
			{
			apv_cnt[i]=0;
			avar_src_reset(i);
			}
		}			
	}




if(apv_cnt_1)
	{
	apv_cnt_1--;
	if(!apv_cnt_1) 
		{
		avar_src_reset(0);
		avar_src_reset(1);
		//cntrl_stat=0;
		}
	}*/		
}

//-----------------------------------------------
void matemat(void)	//Вычисление всех величин, 5Гц
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

if(NUMPHASE==1)
	{
	net_U=net_Ua;
	net_Umax=net_U;
	}
else if(NUMPHASE==3)
	{
	net_U=net_Ua;
	if(net_Ub<net_U)net_U=net_Ub;
	if(net_Uc<net_U)net_U=net_Uc;
	net_Umax=net_Ua;
	if(net_Ub>net_Umax)net_Umax=net_Ub;
	if(net_Uc>net_Umax)net_Umax=net_Uc;
	}

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
     	bps[i]._Uii=0; 
     	bps[i]._Ii=0;
     	bps[i]._Uin=0;
     	bps[i]._Ti=0;
     	bps[i]._flags_tm=0; 
	    bps[i]._rotor=0;    
     	}
     
     }

//Напряжение шины(батареи)
temp_SL=(signed long)adc_buff_[5];
temp_SL*=(signed long)Kuout;
temp_SL/=500L;
out_U=(short)temp_SL;


//Напряжение выпрямителей
temp_SL=(signed long)adc_buff_[4];
temp_SL*=Kubps;
temp_SL/=500L;
bps_U=(signed short)temp_SL;

#ifdef UKU_220_IPS_TERMOKOMPENSAT
load_I=0;

Isumm=0;

for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<5)Isumm+=bps[i]._Ii;
     }  
     
load_I=load_I+Isumm;
if(load_I<0)load_I=0;

#endif

//Суммарный ток выпрямителей
temp_SL=0;
for (i=0;i<NUMIST;i++)
	{
	temp_SL+=((signed long)bps[i]._Ii);
	}
bps_I=(signed short)temp_SL;


Ibmax=Ib_ips_termokompensat;

}

//-----------------------------------------------
//Обработчик БПСов	10Гц
void bps_hndl(void)
{
char ptr__,i;
unsigned short tempUS;

if(sh_cnt0<10)
	{
	sh_cnt0++;
	if(sh_cnt0>=10)
		{
		sh_cnt0=0;
		b1Hz_sh=1;
		}
	}

/*if(sh_cnt1<5)
	{
	sh_cnt1++;
	if(sh_cnt1==5)
		{
		sh_cnt1=0;
		b2Hz_sh=1;
		}
	} */


/*
if(mess_find(MESS_SRC_ON_OFF))
	{
	if(mess_data[0]==_MESS_SRC_MASK_BLOK_2SEC)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))src[i]._ist_blok_cnt=20;
			}
		
		}
	else if(mess_data[0]==_MESS_SRC_MASK_UNBLOK)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))src[i]._ist_blok_cnt=0;
			}
		
		}
	}
	
else if(mess_find(_MESS_SRC_MASK_ON))
	{				
	if(mess_data[0]==_MESS_SRC_MASK_ON)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				src[i]._ist_blok_cnt=0;
				src[i]._flags_tu=2;
				}
			}
		
		}				
	}*/



/*else*/ 
bps_on_mask=0;
bps_off_mask=0;

if(mess_find_unvol(MESS2BPS_HNDL))
	{
	if(mess_data[0]==PARAM_BPS_ALL_OFF_AFTER_2SEC)
		{
		bps_off_mask=0xffff;
		}

	if(mess_data[0]==PARAM_BPS_MASK_OFF_AFTER_2SEC)
		{
		bps_off_mask=mess_data[1];
		}

	if(mess_data[0]==PARAM_BPS_MASK_ON)
		{
		bps_on_mask=mess_data[1];
		}

	if(mess_data[0]==PARAM_BPS_ALL_ON)
		{
		bps_on_mask=0xffff;
		}

	if(mess_data[0]==PARAM_BPS_MASK_ON_OFF_AFTER_2SEC)
		{
		bps_on_mask=mess_data[1];
		bps_off_mask=~(mess_data[1]);
		}


	for(i=0;i<=NUMIST;i++)
		{
		if(bps_off_mask&(1<<i)) bps[i]._blok_cnt++;
		else bps[i]._blok_cnt=0;
		gran(&bps[i]._blok_cnt,0,50);
		if(bps[i]._blok_cnt>20) bps[i]._flags_tu=1;
		if(bps_on_mask&(1<<i)) bps[i]._flags_tu=0;
	     }

	
/*

	if(bps_all_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=1;
	     	}
		}
	else if(bps_mask_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=1;
	     	}
		}	
		
	else if(bps_mask_on_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=1;
			else bps[i]._flags_tu=0;
	     	}
		}
		
	if(mess_data[0]==PARAM_BPS_MASK_ON)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=0;
	     	}
		}
*/										
	}


else if(b1Hz_sh)
	{
	ptr__=0;
     for(i=0;i<NUMIST;i++)
		{
	     bps[i]._flags_tu=1;
	     }	
  	     
  	for(i=0;(i<NUMIST)&&(ptr__<num_necc);i++)
  		{
		char ii,iii;

		ii=(char)NUMIST;
		//if(ii<0)ii=0;
		if(ii>32)ii=32;
		iii=numOfForvardBps;
		//if(iii<0)iii=0;
		if(iii>=NUMIST)iii=0;
		iii+=i;
		iii=iii%ii;
		
  	     if((bps[iii]._state==bsRDY)||(bps[iii]._state==bsWRK))
  	         	{
  	         	bps[iii]._flags_tu=0;
  	         	ptr__++;
  	         	}
			
  	     }
	bps[numOfForvardBps_old]._flags_tu=0;

	if(main_1Hz_cnt<60)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=0;
	     	}	
		}
	if(ipsBlckStat)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=1;
	     	}
		}

     for(i=0;i<=NUMIST;i++)
		{
	    if(bps[i]._flags_tu==1) 	bps[i]._x_=-50;
	   	}	
		 
  	}


for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._ist_blok_host_cnt!=0)
          {
          bps[i]._flags_tu=99;
	     bps[i]._ist_blok_host_cnt--;
          }
     }




b1Hz_sh=0;


num_of_wrks_bps=0;
tempUS=0;
for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._state==bsWRK)
		{
		num_of_wrks_bps++;
		if(bps[i]._Uii>tempUS)tempUS=bps[i]._Uii;
		}
	}
Ubpsmax=tempUS;

bPARALLEL_ENOUG=0;
bPARALLEL_NOT_ENOUG=1;

for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._Ti>=TSIGN)
		{
		bPARALLEL_ENOUG=1;
		}
	if(bps[i]._Ti>=(TSIGN-5))
		{
		bPARALLEL_NOT_ENOUG=0;
		}
	}

if(bPARALLEL_ENOUG==1)
	{
	bPARALLEL=1;
	}
else if(bPARALLEL && bPARALLEL_NOT_ENOUG)
	{
	bPARALLEL=0;
	}
/*
//Пересброс БПСа при потере связи
if(bps[in]._cnt>=10) bps[in]._flags_tu|=BIN8(10000000);
else bps[in]._flags_tu&=BIN8(1111111);

if(avar_bps_reset_cnt) 
	{
	bps[in]._flags_tu|=BIN8(10);
	bps[in]._av=0;
	}
else if(bps[in]._apv_reset_av_timer) bps[in]._flags_tu|=BIN8(10);
else bps[in]._flags_tu&=BIN8(11111101);

if((modbus_register_998)&&(modbus_register_998==(in+1)))
	{
	bps[in]._flags_tu|=BIN8(10);
	} */



if(modbus_register_998)
	{
	for(i=0;i<=NUMIST;i++)
		{
	   	bps[i]._flags_tu=1;
	   	}
	bps[modbus_register_998-1]._flags_tu=2;
	if(modbus_register_997==1)
		{
		bps[modbus_register_998-1]._flags_tu=3;
		}
	}

if(modbus_register_995)
	{
	for(i=0;i<=NUMIST;i++)
		{
	   	bps[i]._flags_tu=0;
	   	}
	bps[modbus_register_995-1]._flags_tu=1;
	}
}

//биты аварий в приходящих сообщениях от источников и инверторов
#define AV_OVERLOAD	0
#define AV_T	1
#define AVUMAX	3
#define AVUMIN	4

//-----------------------------------------------
void bps_drv(char in)  		//Драйвер БПСов 10Гц
{
char temp;

if (bps[in]._device!=dSRC) return;
temp=bps[in]._flags_tm;
if(temp&(1<<AV_T))
	{
	if(bps[in]._temp_av_cnt<1200) 
		{
		bps[in]._temp_av_cnt++;
		if(bps[in]._temp_av_cnt>=1200)
			{
			bps[in]._temp_av_cnt=1200;
		   	if(!(bps[in]._av&(1<<0)))avar_bps_hndl(in,0,1);
			}
		}
	}

else if(!(temp&(1<<AV_T)))
	{
	if(bps[in]._temp_av_cnt) 
		{
		bps[in]._temp_av_cnt--;
		if(!bps[in]._temp_av_cnt)
			{
			if(bps[in]._av&(1<<0))avar_bps_hndl(in,0,0);
			}
		} 	

	}

if((temp&(1<<AVUMAX)))
	{
	if(bps[in]._umax_av_cnt<10) 
		{
		bps[in]._umax_av_cnt++;
		if(bps[in]._umax_av_cnt>=10)
			{ 
			bps[in]._umax_av_cnt=10;
			if(!(bps[in]._av&(1<<1)))avar_bps_hndl(in,1,1);
			apv_start(in);
		  	/*if((K[APV]!=ON)||((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afOFF)))avar_s_hndl(in,1,1);
			if((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afON))
				{
				apv_cnt[in,0]=APV_INIT;
				apv_cnt[in,1]=APV_INIT;
				apv_cnt[in,2]=APV_INIT;
				apv_flags[in]=afOFF;
				}				*/
						
			}
		} 
	}		
else if(!(temp&(1<<AVUMAX)))
	{
	if(bps[in]._umax_av_cnt>0) 
		{
		bps[in]._umax_av_cnt--;
		if(bps[in]._umax_av_cnt==0)
			{
			bps[in]._umax_av_cnt=0;
			avar_bps_hndl(in,1,0);
			//apv_stop(in);
	 //		apv_cnt[in,0]=0;
	//		apv_cnt[in,1]=0;
	 //		apv_cnt[in,2]=0;			
			}
		}
	else if(bps[in]._umax_av_cnt<0) bps[in]._umax_av_cnt=0;		 
	}

if(temp&(1<<AVUMIN))
	{
	if(bps[in]._umin_av_cnt<10) 
		{
		bps[in]._umin_av_cnt++;
		if(bps[in]._umin_av_cnt>=10)
			{ 
			bps[in]._umin_av_cnt=10;
			if(!(bps[in]._av&(1<<2)))avar_bps_hndl(in,2,1);
			apv_start(in);
		  	/*	if((K[APV]!=ON)||((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afOFF)))avar_s_hndl(in,2,1);
			if((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afON))
				{
				apv_cnt[in,0]=APV_INIT;
				apv_cnt[in,1]=APV_INIT;
				apv_cnt[in,2]=APV_INIT;
				apv_flags[in]=afOFF;
				}*/				
			}
		} 
	}	
	
else if(!(temp&(1<<AVUMIN)))
	{
	if(bps[in]._umin_av_cnt) 
		{
		bps[in]._umin_av_cnt--;
		if(bps[in]._umin_av_cnt==0)
			{
			bps[in]._umin_av_cnt=0;
			avar_bps_hndl(in,2,0);
			//apv_stop(in);
		//	apv_cnt[in,0]=0;
		//	apv_cnt[in,1]=0;
		//	apv_cnt[in,2]=0;
			}
		}
	else if(bps[in]._umin_av_cnt>10)bps[in]._umin_av_cnt--;	 
	}

if((bps[in]._Uii<(UB20-DU))&&(bps[in]._state==bsWRK))
	{
	if(bps[in]._umin_av_cnt_uku<300) 
		{
		bps[in]._umin_av_cnt_uku++;
		if(bps[in]._umin_av_cnt_uku>=300)
			{ 
			bps[in]._umin_av_cnt_uku=300;
			if(!(bps[in]._av&(1<<2)))avar_bps_hndl(in,2,1);
		  	/*	if((K[APV]!=ON)||((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afOFF)))avar_s_hndl(in,2,1);
			if((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afON))
				{
				apv_cnt[in,0]=APV_INIT;
				apv_cnt[in,1]=APV_INIT;
				apv_cnt[in,2]=APV_INIT;
				apv_flags[in]=afOFF;
				}*/				
			}
		} 
	}	
	
else if(bps[in]._Uii>=(UB20-DU))
	{
	if(bps[in]._umin_av_cnt_uku) 
		{
		bps[in]._umin_av_cnt_uku--;
		if(bps[in]._umin_av_cnt_uku==0)
			{
			bps[in]._umin_av_cnt_uku=0;
			avar_bps_hndl(in,2,0);
		//	apv_cnt[in,0]=0;
		//	apv_cnt[in,1]=0;
		//	apv_cnt[in,2]=0;
			}
		}
	else if(bps[in]._umin_av_cnt_uku>300)bps[in]._umin_av_cnt_uku=300;	 
	}

//bps[in]._state=bsOFF;

if (bps[in]._av&0x0f)					bps[in]._state=bsAV;
else if ( (net_av) && (bps[in]._cnt>20)/*&& 
		(bps[in]._Uii<200)*/)				bps[in]._state=bsOFF_AV_NET;
else if (bps[in]._flags_tm&BIN8(100000))	bps[in]._state=bsRDY;
else if (bps[in]._cnt<20)				bps[in]._state=bsWRK;



//else if(bps[in]._flags_tm&BIN8(100000)) bps[in]._state=ssBL;
//else if((!(bps[in]._flags_tm&BIN8(100000)))&&(net_U>100))bps[in]._state=ssWRK;
//else bps[0]._state=ssNOT;

//bps[in]._is_ready=0;
//bps[in]._is_wrk=0;
//if(bps[in]._av_net) bps[in]._flags_bp='N';// не подключен
//else if(bps[in]._av_u_max) bps[in]._flags_bp='P';// завышено напряжение(u_.av_.bAS1T)) bps_state[0]=ssAV;
//else if(bps[in]._av_u_min) bps[in]._flags_bp='M';// занижено напряжение
//else if(bps[in]._av_temper) bps[in]._flags_bp='T';// температура
//else if(bps[in]._flags_tm&BIN8(100000)) 
//	{
//	bps[in]._flags_bp='B';// заблокирован
//	bps[in]._is_ready=1;
//	}
//else if((!(bps[in]._flags_tm&BIN8(100000)))&&(net_U>100))
//     {
//     bps[in]._flags_bp='W';// работает
//     bps[in]._is_ready=1;
//     bps[in]._is_wrk=1;
     
//     }
//else bps[in]._is_ready=1;     





/*
bps[in]._flags_tu&=BIN8(11111110);
if(bps[in]._ist_blok_cnt)
	{
	bps[in]._ist_blok_cnt--;
	bps[in]._flags_tu|=BIN8(1);
	}

	   */ 


	
bps[in]._vol_u=cntrl_stat+bps[in]._x_;	
bps[in]._vol_i=1000;
//bps[0]._vol_u=500;
//bps[1]._vol_u=cntrl_stat_pwm; 
}

//-----------------------------------------------
void u_necc_hndl(void)
{
signed long temp_L;
signed long temp_SL;
//signed short temp_SS;

//char i;

//temp_SS=0;
signed short t[2];




if(!TERMOKOMPENS)
	{
	//u_necc=U0B;
	u_necc=UB20;
	//u_necc=1234;
	}
else
	{
	if(bat[0]._nd)mat_temper=20;
	else mat_temper=bat[0]._Tb;

	if(mat_temper<0)temp_SL=UB0; 
	else 
		{
		if(mat_temper>40)mat_temper=40; 
		temp_SL=(UB20-UB0)*10;
		temp_SL*=mat_temper;
		temp_SL/=200;
		temp_SL+=UB0;
		}
/*	if((spc_stat==spcVZ)&&((sk_stat[0]==1)||(VZ_CH_VENT_BLOK==0)))
		{
		temp_SL=UVZ;
		}*/
	u_necc=(unsigned int)temp_SL;
	//u_necc=3456;
	}  

//u_necc=2355;
/*
if((speedChIsOn)||(sp_ch_stat==scsWRK))
	{
	u_necc=speedChrgVolt;
	}
if(hv_vz_stat==hvsWRK)
	{
	u_necc=UVZ;
	}
if(vz1_stat==vz1sWRK)
	{
	u_necc=UZ_U;
	}
if(vz2_stat==vz2sWRK1)
	{
	u_necc=FZ_U1;
	}
if(vz2_stat==vz2sWRK2)
	{
	u_necc=FZ_U2;
	}*/
/*
if((uavt_set_stat==uassSTEP1)||(uavt_set_stat==uassSTEP2))
	{
	u_necc=U_AVT;
	}*/

if(mess_find_unvol(MESS2UNECC_HNDL))
	{		
	if(mess_data[0]==PARAM_UNECC_SET)
		{
		u_necc=mess_data[1];
		}		
	} 

temp_L=(signed long) u_necc;
temp_L*=98L;
temp_L/=100L;
u_necc_dn=(signed short)temp_L;

temp_L=(signed long) u_necc;
temp_L*=102L;
temp_L/=100L;
u_necc_up=(signed short)temp_L;
/*
#ifdef IPS_SGEP_GAZPROM
u_necc=248;
#endif */
}

//-----------------------------------------------
//Вычисление необходимого количества источников 1Гц
void num_necc_hndl(void)
{

static short num_necc_block_cnt;
if(num_necc_block_cnt) num_necc_block_cnt--;

Isumm_=Isumm;

if(bat[0]._Ib<0) Isumm_+=(abs(bat[0]._Ib))/10;
if(bat[1]._Ib<0) Isumm_+=(abs(bat[1]._Ib))/10;

num_necc_up=(Isumm_/((signed short)IMAX))+1;
////Isumm_+=(signed short)((IMAX*(10-KIMAX))/10);
////Isumm_+=(signed short)(IMAX-IMIN);

num_necc_down=(Isumm_/((signed short)IMIN))+1;

if(num_necc_up>num_necc)
	{
	num_necc=num_necc_up;
	num_necc_block_cnt=60;
	}
else if(num_necc_down<num_necc)
	{
	if(!num_necc_block_cnt)
		{
		num_necc=num_necc_down;
		num_necc_block_cnt=60;
		}
	}

if(PAR) num_necc=NUMIST;
#ifdef UKU_220_IPS_TERMOKOMPENSAT
if(bPARALLEL) num_necc=NUMIST;
if(vz1_stat==vz1sWRK)num_necc=NUMIST; //Включаем все источники если уравнительный заряд
if((vz2_stat==vz2sWRK1)||(vz2_stat==vz2sWRK2))num_necc=NUMIST; //Включаем все источники если уравнительный заряд
#endif

gran(&num_necc,1,NUMIST);

}

//-----------------------------------------------
void unet_drv(void)
{
//if(net_av_2min_timer)net_av_2min_timer--;

if(net_U<UMN)
	{
	if((unet_drv_cnt<10)&&(main_1Hz_cnt>15))
		{
		unet_drv_cnt++;
		if(unet_drv_cnt>=10)
			{
			net_Ustore=net_U;
		 	avar_unet_hndl(1);
			
			}
		}
	else if(unet_drv_cnt>=10)unet_drv_cnt=10;

	if(net_U<net_Ustore) net_Ustore=net_U;	
	}

else if(net_U>UMN)
	{                 
	if(unet_drv_cnt)
		{
		unet_drv_cnt--;
		if(unet_drv_cnt<=0)
			{
			avar_unet_hndl(0);
			avar_bps_reset_cnt=10;
			}
		}
	else if(unet_drv_cnt<0)unet_drv_cnt=0;
	
	}
if(net_Umax>UMAXN)
	{
	if((unet_max_drv_cnt<10)&&(main_1Hz_cnt>15))
		{
		unet_max_drv_cnt++;
		if(unet_max_drv_cnt>=10)
			{
			net_Ustore=net_Umax;
		 	avar_unet_hndl(2);
			
			}
		}
	else if(unet_max_drv_cnt>=10)unet_max_drv_cnt=10;

	if(net_Umax>net_Ustore) net_Ustore=net_Umax;	
	}

else if(net_Umax<UMAXN)
	{                 
	if(unet_max_drv_cnt)
		{
		unet_max_drv_cnt--;
		if(unet_max_drv_cnt<=0)
			{
			avar_unet_hndl(0);
			avar_bps_reset_cnt=10;
			}
		}
	else if(unet_max_drv_cnt<0)unet_max_drv_cnt=0;
	
	}
if(avar_bps_reset_cnt)	avar_bps_reset_cnt--;
}


//-----------------------------------------------
void show_mess(char* p1, char* p2, char* p3, char* p4,int m_sec)
{
//bgnd_par(p1,p2,p3,p4);
//tree_up(iSM,sub_ind,sub_ind1,sub_ind2);
//ret((char)(m_sec/100));
//show_mess_cnt=(char)(m_sec/100);
//show_mess_p1=p1;
//show_mess_p2=p2;
//show_mess_p3=p3;
//show_mess_p4=p4;
}

//-----------------------------------------------
void ext_drv(void)
{
char i;

for(i=0;i<1;i++)
	{

	if(!(GPIOC->IDR&=(1<<9)))
		{
		if(sk_cnt[i]<10)
			{
			sk_cnt[i]++;
			if(sk_cnt[i]>=10)
				{
				sk_stat[i]=ssON;
				}
			}
		else 
			{
			sk_cnt[i]=10;
			}
		}
	else
		{
		if(sk_cnt[i]>0)
			{
			sk_cnt[i]--;
			if(sk_cnt[i]<=0)
				{
				sk_stat[i]=ssOFF;
				}
			}
		else 
			{
			sk_cnt[i]=0;
			}
		}
	}

for(i=0;i<1;i++)
	{
	if(((SK_SIGN[i]==0)&&(sk_stat[i]==ssON))||((SK_SIGN[i])&&(sk_stat[i]==ssOFF)) )
		{
		if(sk_av_cnt[i]<10)
			{
			sk_av_cnt[i]++;
			if(sk_av_cnt[i]>=10)
				{
				sk_av_stat[i]=sasON;
				}
			}
		else 
			{
			sk_av_cnt[i]=10;
			}
		}
	else
		{
		if(sk_av_cnt[i]>=0)
			{
			sk_av_cnt[i]--;
			if(sk_av_cnt[i]<=0)
				{
				sk_av_stat[i]=sasOFF;
				}
			}
		else 
			{
			sk_av_cnt[i]=0;
			}
		}
	}
}

//-----------------------------------------------
void speedChargeStartStop(void)
{
spch_plazma[1]++;
/*if(speedChIsOn)
	{
	speedChIsOn=0;
	}

else
	{
	if(speedChrgBlckStat==0)
		{
		speedChIsOn=1;
		speedChTimeCnt=0;
		}
	else
		{
		show_mess(	"     Ускоренный     ",
	          		"       заряд        ",
	          		"    заблокирован!   ",
	          		"                    ",2000);	 
		}
	}*/

if(sp_ch_stat!=scsOFF)
	{
	sp_ch_stat=scsOFF;
	speedz_mem_hndl(10);
	spch_plazma[1]=10;
	}

else
	{
	spch_plazma[1]=20;
	if((speedChrgBlckStat==0)&&(spc_stat==spcOFF)
		&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)
		)
		{
		sp_ch_stat=scsSTEP1;
		speedz_mem_hndl(1);
		}
	else 
		{
		show_mess(	"     Ускоренный     ",
	          		"       заряд        ",
	          		"    заблокирован!   ",
	          		"                    ",2000);
		}
	}
}

//-----------------------------------------------
//Контроль выходного напряжения, 1Гц
void outVoltContrHndl(void)
{ 
if(out_U>U_OUT_KONTR_MAX)
	{
	if(outVoltContrHndlCntUp<U_OUT_KONTR_DELAY)
		{
		outVoltContrHndlCntUp++;
		if(outVoltContrHndlCntUp==U_OUT_KONTR_DELAY)
			{
			if(!uout_av) avar_uout_hndl(2);
			}
		}
	}
else
	{
	if(outVoltContrHndlCntUp)
		{
		outVoltContrHndlCntUp--;
			{
			if(outVoltContrHndlCntUp==0)
				{
				if(uout_av)avar_uout_hndl(0);
				}
			}
		}
	}

if(out_U<U_OUT_KONTR_MIN)
	{
	if(outVoltContrHndlCntDn<U_OUT_KONTR_DELAY)
		{
		outVoltContrHndlCntDn++;
		if(outVoltContrHndlCntDn==U_OUT_KONTR_DELAY)
			{
			if(!uout_av) avar_uout_hndl(1);
			}
		}
	}
else
	{
	if(outVoltContrHndlCntDn)
		{
		outVoltContrHndlCntDn--;
			{
			if(outVoltContrHndlCntDn==0)
				{
				if(uout_av)avar_uout_hndl(0);
				}
			}
		}
	}

if (out_U<(USIGN*10)) 
	{
	if(!bSILENT)
		{
		mess_send(MESS2RELE_HNDL,PARAM_RELE_BAT_IS_DISCHARGED,1,20);
		}

	//bU_BAT2REL_AV_BAT=1;
	}


}


//-----------------------------------------------
void speedChargeHndl(void)
{
/*
if(sp_ch_stat==scsOFF)
	{
	if((sk_stat[1]==1)&&(sk_stat_old[1]=0))
	}*/
	 
if(sp_ch_stat==scsSTEP1)
	{
	if(sp_ch_stat_old!=sp_ch_stat)
		{
		sp_ch_stat_cnt=5;
		if(SP_CH_VENT_BLOK==0)
			{
			sp_ch_stat_cnt=0;
			sp_ch_stat=scsWRK;
			}
		}
	if(sp_ch_stat_cnt)
		{
		sp_ch_stat_cnt--;
		if(sp_ch_stat_cnt==0)
			{
			sp_ch_stat=scsERR1; 	//Не включилась вентиляция;

			}
		}
	if(sk_stat[0]==1)sp_ch_stat=scsWRK;
	}

if(sp_ch_stat==scsWRK)
	{
	if(sp_ch_stat_old!=sp_ch_stat)
		{
		sp_ch_wrk_cnt=(signed long)speedChrgTimeInHour*3600L;
		hv_vz_up_cnt=0;
		}
	sp_ch_wrk_cnt--;
	hv_vz_up_cnt++;
	if(sp_ch_wrk_cnt==0)
		{
		sp_ch_stat=scsOFF;
		speedz_mem_hndl(0);
		}
	#ifdef UKU_220_IPS_TERMOKOMPENSAT
	if((sk_stat[0]==0)&&(SP_CH_VENT_BLOK==1))sp_ch_stat=scsERR2;
	#endif
	}

if(sp_ch_stat==scsERR1)		//Отсутствует вентиляция при включении
	{
	if((sp_ch_stat_old!=sp_ch_stat)||(sp_ch_stat_cnt==0))
		{
		sp_ch_stat_cnt=10;
		}
	sp_ch_stat_cnt--;
	if((sp_ch_stat_cnt==10)||(sp_ch_stat_cnt==9))
		{
		show_mess(	"  УСКОРЕННЫЙ ЗАРЯД  ",
					"   НЕ МОЖЕТ БЫТЬ    ",
					"      ВКЛЮЧЕН       ",
					"  БЕЗ ВЕНТИЛЯЦИИ!!  ",
					5000);
		}
	}
if(sp_ch_stat==scsERR2)		//Пропала вентиляция при работе
	{
	if((sp_ch_stat_old!=sp_ch_stat)||(sp_ch_stat_cnt==0))
		{
		sp_ch_stat_cnt=10;
		}
	sp_ch_stat_cnt--;
	if((sp_ch_stat_cnt==10)||(sp_ch_stat_cnt==9))
		{
		show_mess(	"  УСКОРЕННЫЙ ЗАРЯД  ",
					"    ЗАБЛОКИРОВАН    ",
					"     НЕИСПРАВНА     ",
					"    ВЕНТИЛЯЦИЯ!!!   ",
					5000);
		}
	if(sk_stat[0]==1)sp_ch_stat=scsWRK;
	}


sp_ch_stat_old=sp_ch_stat;



if(speedChrgAvtEn==1)
	{
	if((sp_ch_stat==scsOFF)&&(spc_stat==spcOFF)
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)
		#endif
		)
		{
		if((load_U<u_necc)&&((u_necc-load_U)>speedChrgDU)
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		&&(abs(Ib_ips_termokompensat/10-IZMAX)<5)
		#endif
		#ifdef UKU_220_V2
		&&(abs(bat[0]._Ib/10-IZMAX)<10)
		#endif
		&&(!speedChrgBlckStat))
			{
			speedChargeStartCnt++;
			if(speedChargeStartCnt>=60)
				{
				speedChargeStartStop();
				speedz_mem_hndl(5);
				}
			}
		else speedChargeStartCnt=0;
		}
	else speedChargeStartCnt=0;
	}



/*
if(speedChIsOn)
	{
	speedChTimeCnt++;
	if(speedChTimeCnt>=((signed long)speedChrgTimeInHour*3600L))
		{
		speedChIsOn=0;
		}
	if(speedChrgBlckStat)
		{
		speedChIsOn=0;
		speedChTimeCnt=0;
		}
	}



if(speedChrgAvtEn)
	{
	if(!speedChIsOn)
		{
		if((load_U<u_necc)&&((u_necc-load_U)>speedChrgDU)&&(abs(Ib_ips_termokompensat/10-IZMAX)<5)&&(!speedChrgBlckStat))
			{
			speedChIsOn=1;
			}
		}
	}


*/
//if(/*(speedChrgBlckSrc!=1)&&*/(speedChrgBlckSrc!=2)) speedChrgBlckStat=0;
//else
/*	{
	speedChrgBlckStat=0;
	if(speedChrgBlckSrc==1)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[11]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[11]<2000))) speedChrgBlckStat=1;
		}
	else if(speedChrgBlckSrc==2)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[13]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[13]<2000))) speedChrgBlckStat=1;
		}
	}  */

/*
if(speedChrgBlckStat==1)
	{

	//speedChargeStartStop();

	speedChrgShowCnt++;
	if(speedChrgShowCnt>=30)	
		{
		speedChrgShowCnt=0;
		show_mess(	"     УСКОРЕННЫЙ     ",
					"       ЗАРЯД        ",
					"     ЗАПРЕЩЕН!!!    ",
					"                    ",
					5000);
		}
	} 
else speedChrgShowCnt=0;  */
}

//-----------------------------------------------
//Обслуживание тестовых операций, 10Гц
void tst_hndl(void)
{
/*if(tst_hndl_cnt)
	{
	tst_hndl_cnt--;
	if(tst_hndl_cnt==0)
		{
		test_control_register=0;
		}
	}

if(test_control_register!=test_control_register_old)
	{
	if(test_control_register==0)
		{
		rele_output_stat_test_byte=0;
		rele_output_stat_test_mask=0;
		tst_hndl_cnt=0;
		}
	else
		{
		tst_hndl_cnt=100;
		}

	}
test_control_register_old=test_control_register; */

//Принудительное управление реле1
if(test_hndl_rele1_cnt)
	{
	test_hndl_rele1_cnt--;
	if(test_hndl_rele1_cnt==0)
		{
		test_hndl_rele1_cntrl=0;
		}
	rele_output_stat_test_byte&=0xfe;
	if(test_hndl_rele1_cntrl==1)rele_output_stat_test_byte|=0x01;
	rele_output_stat_test_mask|=0x01;	
	if(test_hndl_rele1_cntrl==0)rele_output_stat_test_mask&=0xfe;
	}
else
	{
	rele_output_stat_test_byte&=0xfe;
	rele_output_stat_test_mask&=0xfe;
	}

//Принудительное управление реле2
if(test_hndl_rele2_cnt)
	{
	test_hndl_rele2_cnt--;
	if(test_hndl_rele2_cnt==0)
		{
		test_hndl_rele2_cntrl=0;
		}
	rele_output_stat_test_byte&=0xfd;
	if(test_hndl_rele2_cntrl==1)rele_output_stat_test_byte|=0x02;
	rele_output_stat_test_mask|=0x02;	
	if(test_hndl_rele2_cntrl==0)rele_output_stat_test_mask&=0xfd;
	}
else
	{
	rele_output_stat_test_byte&=0xfd;
	rele_output_stat_test_mask&=0xfd;
	}

//Принудительное управление реле3
if(test_hndl_rele3_cnt)
	{
	test_hndl_rele3_cnt--;
	if(test_hndl_rele3_cnt==0)
		{
		test_hndl_rele3_cntrl=0;
		}
	rele_output_stat_test_byte&=0xfb;
	if(test_hndl_rele3_cntrl==1)rele_output_stat_test_byte|=0x04;
	rele_output_stat_test_mask|=0x04;	
	if(test_hndl_rele3_cntrl==0)rele_output_stat_test_mask&=0xfb;
	}
else
	{
	rele_output_stat_test_byte&=0xfb;
	rele_output_stat_test_mask&=0xfb;
	}

//Принудительное управление релеHV
if(test_hndl_releHV_cnt)
	{
	test_hndl_releHV_cnt--;
	if(test_hndl_releHV_cnt==0)
		{
		test_hndl_releHV_cntrl=0;
		}
	rele_output_stat_test_byte&=0xf7;
	if(test_hndl_releHV_cntrl==1)rele_output_stat_test_byte|=0x08;
	rele_output_stat_test_mask|=0x08;	
	if(test_hndl_releHV_cntrl==0)rele_output_stat_test_mask&=0xf7;
	}
else
	{
	rele_output_stat_test_byte&=0xf7;
	rele_output_stat_test_mask&=0xf7;
	}

//Управление тестом источников
if(test_hndl_bps_cnt)
	{
	test_hndl_bps_cnt--;
	if(test_hndl_bps_cnt==0)
		{
		test_hndl_bps_number=0;
		test_hndl_bps_state=0;
		}
	else
		{
		if(test_hndl_bps_number==255)	//если управляем всеми источниками
			{
			if(test_hndl_bps_state==1)	//включаем все источники на максимальный шим
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);
				}
			else if(test_hndl_bps_state==2)	//включаем все источники на минимальный шим
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
				}
			else if(test_hndl_bps_state==3)	//включаем все источники на термокоипенсацию
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);
				}
			else if(test_hndl_bps_state==4)	//включаем все источники на автономную работу
				{
 				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
				mess_send(MESS2NET_DRV,PARAM_BPS_NET_OFF,1,10);
				}			
			}
		else
			{
			if(test_hndl_bps_state==0)	//выключаем один источник
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<(test_hndl_bps_number-1)),10);
				}
			else if(test_hndl_bps_state==1)	//включаем один источник на максимальный шим
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<(test_hndl_bps_number-1)),10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);
				}
			else if(test_hndl_bps_state==2)	//включаем один источник на минимальный шим
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<(test_hndl_bps_number-1)),10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
				}
			else if(test_hndl_bps_state==3)	//включаем один источник на термокоипенсацию
				{
				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<(test_hndl_bps_number-1)),10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
				mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);
				}
			else if(test_hndl_bps_state==4)	//включаем один источник на автономную работу
				{
 				mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<(test_hndl_bps_number-1)),10);
				mess_send(MESS2NET_DRV,PARAM_BPS_NET_OFF,1,10);
				}
			}
		}
	}
}

//-----------------------------------------------
//Управление реле, физическое, 10Гц
void rele_drv(void)
{
//static char temp;
//if(temp==1) temp=0;
//else temp=1;

//GPIOC4 -> первое реле с верху, крайнее правое на плате
//GPIOB1 -> второе реле с верху, среднее на плате
//GPIOB0 -> третье реле с верху, крайнее левое на плате
//GPIOC5 -> Реле включения высокого напряжения 
if(rele_output_stat&0x01) 	GPIOC->ODR|=(1<<4);
else 						GPIOC->ODR&=~(1<<4);
if(rele_output_stat&0x02) 	GPIOB->ODR|=(1<<1);
else 						GPIOB->ODR&=~(1<<1);
if(rele_output_stat&0x04) 	GPIOB->ODR|=(1<<0);
else 						GPIOB->ODR&=~(1<<0);
if(rele_output_stat&0x08) 	GPIOC->ODR|=(1<<5);
else 						GPIOC->ODR&=~(1<<5);

}

//-----------------------------------------------
//Управление реле, логическое, 10Гц
void rele_hndl(void)
{
char i;
char temp;
short releset[3];
releset[0]=RELE1SET;
releset[1]=RELE2SET;
releset[2]=RELE3SET;
temp=0;

for(i=0;i<3;i++)
	{
	if((net_av==1)&&(releset[i]&(1<<1)))  temp|=(1<<i);  	//Отработка аварии по заниженному сетевому напряжению
	if((net_av==2)&&(releset[i]&(1<<2)))  temp|=(1<<i);  	//Отработка аварии по завышенному сетевому напряжению
	

	if((uout_av==1)&&(releset[i]&(1<<7))) temp|=(1<<i);  	//Отработка аварии по заниженному выходному напряжению
	if((uout_av==2)&&(releset[i]&(1<<8))) temp|=(1<<i);  	//Отработка аварии по завышенному выходному напряжению
	
	if(releset[i]&(1<<0)) temp^=(1<<i); 					//Инверсия выхода если есть соответствующая установка
	}


rele_output_stat=temp;

rele_output_stat&=~rele_output_stat_test_mask;
rele_output_stat|=(rele_output_stat_test_byte&rele_output_stat_test_mask);
//if()
}

//-----------------------------------------------
//Вычисление рабочего ШИМа 10Гц
void cntrl_hndl(void)
{



IZMAX_=IZMAX;

//cntrl_hndl_plazma=10;

if((speedChIsOn)||(sp_ch_stat==scsWRK))IZMAX_=speedChrgCurr;
if(vz1_stat==vz1sWRK) IZMAX_=UZ_IMAX;
if(vz2_stat==vz2sWRK1) IZMAX_=FZ_IMAX1;
if(vz2_stat==vz2sWRK2) IZMAX_=FZ_IMAX2;
//if(spc_stat==spcVZ) IZMAX_=IMAX_VZ;

if(cntrl_stat_blok_cnt)cntrl_stat_blok_cnt--;
if(cntrl_stat_blok_cnt_)cntrl_stat_blok_cnt_--;

if((bat[0]._temper_stat&0x03)||(bat[1]._temper_stat&0x03))IZMAX_=IZMAX_/10;


if((REG_SPEED<1)||(REG_SPEED>5)) REG_SPEED=1;
if(ch_cnt0<(10*REG_SPEED))
	{
	ch_cnt0++;
	if(ch_cnt0>=10*REG_SPEED)
		{
		ch_cnt0=0;
		b1Hz_ch=1;
		}
	}


if(mess_find_unvol(MESS2CNTRL_HNDL))
	{
	if(mess_data[0]==PARAM_CNTRL_STAT_PLUS)
		{
		cntrl_stat/*_new*/=cntrl_stat_old+mess_data[1];
		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_MINUS)
		{
		cntrl_stat/*_new*/=cntrl_stat_old-mess_data[1];
		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_STEP_DOWN)
		{
		static char cntrlStatIsDownCnt;
		cntrl_stat--;

		if((cntrl_stat<=30)||(load_U<USIGN))
			{
			if(++cntrlStatIsDownCnt==250)mess_send(MESS2KB_HNDL,PARAM_CNTRL_IS_DOWN,0,10);
			}
		else 
			{
			cntrlStatIsDownCnt=0;
			}

		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_SET)
		{
		cntrl_stat/*_new*/=mess_data[1];
		}

	else if(mess_data[0]==PARAM_CNTRL_STAT_FAST_REG)
		{
		if(bps_U>u_necc)
			{
			cntrl_hndl_plazma=11;
			if(((bps_U-u_necc)>40)&&(cntrl_stat/*_new*/>0))cntrl_stat/*_new*/-=5;
			else if((cntrl_stat/*_new*/)&&b1Hz_ch)cntrl_stat/*_new*/--;
			}
		else if(bps_U<u_necc)
			{
			cntrl_hndl_plazma=12;	
			if(((u_necc-bps_U)>40)&&(cntrl_stat/*_new*/<1015))cntrl_stat/*_new*/+=5;
			else	if((cntrl_stat/*_new*/<1020)&&b1Hz_ch)cntrl_stat/*_new*/++;
			}
	 	}

	/*gran(&cntrl_stat_new,10,1010);			
	cntrl_stat_old=cntrl_stat_new;
	cntrl_stat=cntrl_stat_new;*/
	}

else if((b1Hz_ch)&&((!bIBAT_SMKLBR)||(bps[8]._cnt>40)))
	{
	cntrl_stat_new=cntrl_stat_old;
	cntrl_hndl_plazma=19;
	if((Ibmax/10)>(2*IZMAX_))
		{
		cntrl_hndl_plazma=20;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
		else	cntrl_stat_new-=10;
		}		
	else if(((Ibmax/10)<(IZMAX_*2))&&(Ibmax>(IZMAX_*15)))
		{
		cntrl_hndl_plazma=21;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
          else	cntrl_stat_new-=3;
		}   
	else if((Ibmax<(IZMAX_*15))&&((Ibmax/10)>IZMAX_))
		{
		cntrl_hndl_plazma=22;
		cntrl_stat_new--;
		}
		
	else if(bps_U<u_necc)
		{
		cntrl_hndl_plazma=23;
		if(bps_U<(u_necc-(UB0-UB20)))
			{
			cntrl_hndl_plazma=24;
			if(Ibmax<0)
				{
				cntrl_hndl_plazma=25;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else cntrl_stat_new+=10;
				}
			else if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=26;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10)/*-10*/))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=27;
				cntrl_stat_new++;
				}					
			}
		else if(bps_U<(u_necc-((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=28;
			if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=29;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10)/*-10*/))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=30;
				cntrl_stat_new++;
				}					
			}	
		else if(bps_U<(u_necc-1))
			{
			cntrl_hndl_plazma=31;
			if(Ibmax<((IZMAX_*10)/*-10*/))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=32;
				cntrl_stat_new++;
				}					
			}					
		}	
	else if((bps_U>u_necc)/*&&(!cntrl_blok)*/)
		{ 	
		cntrl_hndl_plazma=33;
		if(bps_U>(u_necc+(UB0-UB20)))
			{
			cntrl_hndl_plazma=34;
               if(cntrl_stat_blok_cnt)cntrl_stat_new--;
			else	cntrl_stat_new-=10;
			}
		else if(bps_U>(u_necc+((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=35;
               if(cntrl_stat_blok_cnt)cntrl_stat_new--;
			else cntrl_stat_new-=2;
			}	
		else if(bps_U>(u_necc+1))
			{
			cntrl_hndl_plazma=36;
			cntrl_stat_new--;
			}					
		}

	if((hv_vz_stat==hvsOFF)&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF))
		{
///		if(((sk_stat[1]==1)&&(sk_stat_old[1]==0))&&(VZ_KIND==1))cntrl_stat_new=50;
		}

	gran(&cntrl_stat_new,0,1023);			
	cntrl_stat_old=cntrl_stat_new;
	cntrl_stat=cntrl_stat_new;
	
	if(ICA_EN==0)
		{
		if(ica_cntrl_hndl_cnt)
			{
			cntrl_stat = ica_cntrl_hndl;
			cntrl_stat_new = ica_cntrl_hndl;//=10*PWM_START;
			cntrl_stat_old = ica_cntrl_hndl;//=10*PWM_START;
			}
		}
	
	if((ICA_EN==1)||(ICA_EN==2))
		{
		cntrl_stat=cntrl_stat_new+ica_u_necc;
		}			
	}

iiii=0;
for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<30)iiii=1;
     }

if(iiii==0)
	{
	cntrl_stat=600;	
	cntrl_stat_old=600;
	cntrl_stat_new=600;
	cntrl_stat=10*PWM_START;
	cntrl_stat_old=10*PWM_START;
	cntrl_stat_new=10*PWM_START;
	}


if(modbus_register_998)
	{
	cntrl_stat=modbus_register_999;
	}

if(ica_cntrl_hndl_cnt)	ica_cntrl_hndl_cnt--;


gran(&cntrl_stat,0,1023); 
b1Hz_ch=0;
}

//-----------------------------------------------
void kb_init(void)
{
main_kb_cnt=(TBAT*60)-60/*120*/;
}

//-----------------------------------------------
void kb_hndl(void)
{

static signed short ibat[2],ibat_[2];

KB_ALGORITM=3;


if(((++main_kb_cnt>=TBAT*60)&&(TBAT)))
	{
	main_kb_cnt=0;
	
	//kb_start[0]=0;
	//kb_start[1]=0;
	kb_start_ips=0;

	//if( (BAT_IS_ON[0]==bisON) && (bat[0]._Ub>80) && ( (abs(bat[0]._Ib)<IKB) || (bat[0]._av&1) ) ) kb_start[0]=1;
	if( (!ips_bat_av_vzvod)&& ((abs(Ib_ips_termokompensat)<IKB) || (bat_ips._av&1) ) ) kb_start_ips=1;
	if( (net_av) || (num_of_wrks_bps==0) || ( (spc_stat!=spcOFF) && (spc_stat!=spcVZ) ) 
	  ||(((vz1_stat!=vz1sOFF)||(vz2_stat!=vz2sOFF))&&SMART_SPC)
	  ||(sp_ch_stat!=scsOFF) /**/	)  
 
		{
		//kb_start[0]=0;
		//kb_start[1]=0;
		kb_start_ips=0;
		}

	if(/*(kb_start[0]==1)||(kb_start[1]==1)||*/(kb_start_ips==1))
		{
		kb_cnt_1lev=10;
		}
	else kb_cnt_1lev=0;
	}

if(kb_cnt_1lev)
	{
	kb_cnt_1lev--;

	if(kb_cnt_1lev>5)mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_PLUS,30,15);
	else if(kb_cnt_1lev>0) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_MINUS,30,15);


	if(kb_cnt_1lev==5)
		{
/*		ibat[0]=abs(bat[0]._Ib);
		ibat[1]=abs(bat[1]._Ib);   */
		ibat_ips=abs(Ib_ips_termokompensat);
		}
	
	if(kb_cnt_1lev==0)
		{
/*		ibat_[0]=abs(bat[0]._Ib);
		ibat_[1]=abs(bat[1]._Ib); */
		ibat_ips_=abs(Ib_ips_termokompensat);

		kb_cnt_2lev=0;


/*		if(( (ibat[0]+ibat_[0]) < IKB )&& (kb_start[0]==1))
			{
			kb_cnt_2lev=10;  
			}
		else if(bat[0]._Ub>200)
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			}  */
		
/*		if(( (ibat[1]+ibat_[1]) < IKB ) && (kb_start[1]==1))
			{
			kb_cnt_2lev=10;     
			}
		else  if(bat[1]._Ub>200)
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			}  */
		if(( (ibat_ips+ibat_ips_) < IKB ) && (kb_start_ips==1))
			{
			if(KB_ALGORITM==1)
				{
				avar_bat_ips_hndl(1);
				kb_start_ips=0;
				}
			else
				{
				kb_cnt_2lev=10;     
				}
			}
		}	


	}
else if(kb_cnt_2lev)
	{
	kb_cnt_2lev--;

	if(kb_cnt_2lev>5)mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_PLUS,200,15);
	else if(kb_cnt_2lev>0) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_MINUS,200,15);


	if(kb_cnt_2lev==5)
		{
		/*ibat[0]=abs(bat[0]._Ib);
		ibat[1]=abs(bat[1]._Ib); */
		ibat_ips=abs(Ib_ips_termokompensat);
		}
	
	if(kb_cnt_2lev==0)
		{
		/*ibat_[0]=abs(bat[0]._Ib);
		ibat_[1]=abs(bat[1]._Ib); */
		ibat_ips_=abs(Ib_ips_termokompensat);

		kb_full_ver=0;

/*		if(( (ibat[0]+ibat_[0]) < IKB ) && (kb_start[0]==1))
			{
			kb_full_ver=1;  
			}
		else if(bat[0]._Ub>200)			
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			} */

/*		if(( (ibat[1]+ibat_[1]) < IKB ) && (kb_start[1]==1))
			{
			kb_full_ver=1;     
			}
		else	if(bat[1]._Ub>200)		
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			} */

		if(( (ibat_ips+ibat_ips_) < IKB )  && (kb_start_ips==1))
			{
			if(KB_ALGORITM==2)
				{
				avar_bat_ips_hndl(1);
				kb_start_ips=0;
				}
			else
				{
				kb_full_ver=1;     
				}
			}
		}	
	}

else if(kb_full_ver)
	{
	
	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_STEP_DOWN,0,15);

/*	if( abs(bat[0]._Ib) > IKB ) 
		{
		if(kb_start[0]==1)
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			}
		}*/

/*	if( abs(bat[1]._Ib) > IKB ) 
		{
		if(kb_start[1]==1)
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			}
		} */
	if( abs(Ib_ips_termokompensat) > IKB ) 
		{
		if(kb_start_ips==1)
			{
			kb_start_ips=0;
			avar_bat_ips_hndl(0);
			}
		}

	if (/*(kb_start[0]==0) && (kb_start[1]==0) && */(kb_start_ips==0)) 
		{
		kb_full_ver=0;
		}

	if(( (mess_find(MESS2KB_HNDL))	&& (mess_data[0]==PARAM_CNTRL_IS_DOWN) ) || (out_U<(USIGN*10)) )
		{
		kb_full_ver=0;
		/*if((kb_start[0]==1)&&((load_I>(2*IKB)/10))&&(!(bat[0]._av&0x01))) avar_bat_hndl(0,1);
		if((kb_start[1]==1)&&((load_I>(2*IKB)/10))&&(!(bat[1]._av&0x01))) avar_bat_hndl(1,1);*/
		if((kb_start_ips==1)&&((load_I>(2*IKB)/10))&&(!(bat_ips._av&0x01))) avar_bat_ips_hndl(1);
		}
	}

}


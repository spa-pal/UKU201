#include <stm32f10x_conf.h>
#include <string.h>                   /* string and memory functions         */
#include "STM32_Init.h"               /* stm32 initialisation                */
#include <stm32f10x_lib.h>
#include "main.h"
#include "uart1.h"
#include "uart2.h"
#include "modbus.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "memo.h"
#include "control.h"
#include "cmd.h"
#include "full_can.h"
#include "mcp2515.h"
#include "avar_hndl.h"
#include "beep.h"

#define	putchar putchar2
#define can1_out mcp2515_transmit

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
//Состояние первичной сети
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//Состояние батарей
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;
signed short		Ib_ips_termokompensat_temp;

//***********************************************
//Состояние источников
BPS_STAT bps[29];

//***********************************************
//Состояние выхода
signed short bps_U;
signed short out_U;
signed short bps_I;
signed short bps_I_phantom;

//***********************************************
//Состояние нагрузки
signed short load_U;
signed short load_I;

//***********************************************
//Ускоренный заряд
signed short speedChrgCurr;			//максимальный ток ускоренного заряда, отображение из ЕЕПРОМ
signed short speedChrgVolt;			//максимальное напряжение ускоренного заряда, отображение из ЕЕПРОМ
signed short speedChrgTimeInHour; 		//максимальное время ускоренного заряда в часах, отображение из ЕЕПРОМ
signed short speedChrgAvtEn;	    		//Автоматическое включение Ускоренного заряда включено/выключено
signed short speedChrgDU;	    		//Просадка напряжения необходимая для включения ускоренного заряда
signed short speedChIsOn;			//Текущее состояние ускоренного заряда вкл/выкл
signed long  speedChTimeCnt;			//Счетчик времени прямой ускоренного заряда
signed short speedChrgBlckSrc;		//Источник сигнала блокировки, 0-выкл., 1-СК1, 2-СК2
signed short speedChrgBlckLog;		//Логика сигнала блокировки, 1 - блокировка по замкнутому СК, 0 - по разомкнутому
signed short speedChrgBlckStat;		//Сигнал блокировки для выравнивающего и ускоренного заряда.
char  	   	 speedChrgShowCnt;		//Счетчик показа информационного сообщения

//***********************************************
//Новый ускоренный заряд
enum_sp_ch_stat sp_ch_stat=scsOFF,sp_ch_stat_old;
short sp_ch_stat_cnt;
long sp_ch_wrk_cnt;
char speedChargeStartCnt=0;

//***********************************************
//Контроль выходного напряжения
signed short outVoltContrHndlCnt;		//Счетчик, считает в плюс в случае выполнения условия аварии
signed short outVoltContrHndlCnt_;		//Счетчик, считает в плюс в случае отсутствия выполнения условия аварии
char uout_av;

//***********************************************
//Межблоковая связь
signed short cnt_net_drv;
char max_net_slot;
bool bCAN_OFF;

//***********************************************
//Спецфункции
enum_spc_stat spc_stat;
char spc_bat;
char spc_phase;
unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
char bAVZ;
enum_ke_start_stat ke_start_stat;
short cnt_end_ke;
unsigned long ke_date[2];
short __ee_vz_cnt;
short __ee_spc_stat;
short __ee_spc_bat;
short __ee_spc_phase;
char vz_error=0;  		// устанавливается, если выравнивающий заряд заблокирован по вентиляции
char sp_ch_error=0;		// устанавливается, если ускоренный заряд заблокирован по вентиляции
char vz1_error=0;		// устанавливается, если уравнительный заряд заблокирован по вентиляции
char vz2_error=0;		// устанавливается, если формовочный заряд заблокирован по вентиляции

//**********************************************
//Коэффициенты, отображаемые из EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
signed short Kubatm[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Kunet_ext[3];
signed short Ktext[3];
signed short Kuload;
signed short KunetA;
signed short KunetB;
signed short KunetC;
signed short Kubps;
signed short Kuout=2;

signed short MAIN_IST;
signed short UMAX;
signed short UB0;
signed short UB20;
signed short TMAX;
signed short TSIGN;
signed short AV_OFF_AVT;
signed short USIGN;
signed short UMN;
signed short ZV_ON;
signed short IKB;
//signed short KVZ;
signed short UVZ;
signed short IMAX_VZ;
signed short IMAX;
signed short IMIN;
signed short APV_ON;
signed short IZMAX;
signed short U0B;
signed short TZAS;
signed short VZ_HR;
signed short TBAT;
signed short U_AVT;
signed short DU;
signed short PAR;
signed short TBATMAX;
signed short TBATSIGN;
signed short UBM_AV;
signed short RELE_LOG;
signed short TBOXMAX;
signed short TBOXREG;
signed short TBOXVENTMAX;
signed short TLOADDISABLE;
signed short TLOADENABLE;
signed short TBATDISABLE;
signed short TBATENABLE;
signed short TVENTON;
signed short TVENTOFF;
signed short TWARMON;
signed short TWARMOFF;
enum_releventsign RELEVENTSIGN;
signed short TZNPN;
signed short UONPN;
signed short UVNPN;
signed short dUNPN;
enum_npn_out NPN_OUT;
enum_npn_sign NPN_SIGN;
signed short TERMOKOMPENS;
signed short TBOXVENTON; 
signed short TBOXVENTOFF;
signed short TBOXWARMON; 
signed short TBOXWARMOFF;
signed short BAT_TYPE;	//Тип батареи. 0 - обычная свинцовая, 1-литиевая COSLIGHT, 2-литиевая SACRED SUN , 3-литиевая ZTT
signed short DU_LI_BAT;	//Параметр, определяющий напряжение содержания литиевой батареи
signed short FORVARDBPSCHHOUR;	//Периодичностьсмены ведущего источника в часах. Если 0 - функция выключена и ведущий первый источник
signed short NUMBAT;
signed short NUMBAT_TELECORE;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;
signed short NUMAVT;
signed short NUMMAKB;
signed short NUMBYPASS;
signed short NUMBDR;
signed short NUMENMV;
signed short NUMPHASE;
signed short SMART_SPC = 1;
signed short U_OUT_KONTR_MAX;
signed short U_OUT_KONTR_MIN;
signed short U_OUT_KONTR_DELAY;
signed short DOP_RELE_FUNC;
signed short CNTRL_HNDL_TIME;	//Постоянная времени регулирования источников для Телекора
signed short USODERG_LI_BAT;	//Напряжение содержания литиевой батареи
signed short QSODERG_LI_BAT;	//Заряд при котором начинает действовать напряжение содержания литиевой батареи
signed short TVENTMAX;			//Максимальный ресурс вентилятора
signed short ICA_EN;			//Включенность режима выравнивания токов ИПС
signed short ICA_CH;			//Канал связи для выравнивания токов, 0 - MODBUS, 1 - MODBUS-TCP
signed short ICA_MODBUS_ADDRESS;//Адрес ведомого для выравнивания токов по шине MODBUS-RTU
signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	//IP ведомого для выравнивания токов по шине MODBUS-TCP
signed short ICA_MODBUS_TCP_UNIT_ID;	//UNIT ID ведомого для выравнивания токов по шине MODBUS-TCP
signed short PWM_START;			//Начальный шим для ЭЛТЕХа
signed short KB_ALGORITM;		//2-х или 3-х ступеннчатый алгоритм проверки цепи батареи
signed short REG_SPEED;			//скорость регулирования, 1- стандартная, 2,3,4,5- замедленная в 2,3,4,5 раз
enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;
signed short RS485_QWARZ_DIGIT;
signed short UVENTOFF;			//Напряжение (1в) при котором выключится вентиляция после окончания ВЗ или УСКЗ
signed short VZ_KIND;			//Тип выравнивающего заряда, 0 - обычный(исторический, повышение напряжения на время), 
								//1- высоковольтный, с контролем вентиляции и запросами к оператору
signed short SNTP_ENABLE;
signed short SNTP_GMT;

signed short UZ_U;
signed short UZ_IMAX;
signed short UZ_T;

signed short FZ_U1;
signed short FZ_IMAX1;
signed short FZ_T1;
signed short FZ_ISW12;
signed short FZ_U2;
signed short FZ_IMAX2;
signed short FZ_T2;

signed short RELE_SET_MASK[4]={1,2,3,4};

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];
//signed short BAT_TYPE[2];

unsigned short AUSW_MAIN;
unsigned long AUSW_MAIN_NUMBER;
unsigned short AUSW_DAY;
unsigned short AUSW_MONTH;
unsigned short AUSW_YEAR;
unsigned short AUSW_UKU;
unsigned short AUSW_UKU_SUB;
unsigned long AUSW_UKU_NUMBER;
unsigned long	AUSW_BPS1_NUMBER;
unsigned long  AUSW_BPS2_NUMBER;
unsigned short AUSW_RS232;
unsigned short AUSW_PDH;
unsigned short AUSW_SDH;
unsigned short AUSW_ETH;

signed short TMAX_EXT_EN[3];
signed short TMAX_EXT[3];
signed short TMIN_EXT_EN[3];
signed short TMIN_EXT[3];
signed short T_EXT_REL_EN[3];
signed short T_EXT_ZVUK_EN[3];
signed short T_EXT_LCD_EN[3];
signed short T_EXT_RS_EN[3];

signed short SK_SIGN[4];
signed short SK_REL_EN[4];
signed short SK_ZVUK_EN[4];
signed short SK_LCD_EN[4];
signed short SK_RS_EN[4];

enum_avz AVZ;

unsigned short HOUR_AVZ;
unsigned short MIN_AVZ;
unsigned short SEC_AVZ;
unsigned short DATE_AVZ;
unsigned short MONTH_AVZ;
unsigned short YEAR_AVZ;
unsigned short AVZ_TIME;

signed short RELE_VENT_LOGIC;
/*
signed short MODBUS_ADRESS;
signed short MODBUS_BAUDRATE;*/
signed short BAT_LINK;

signed short BAT_C_POINT_1_6;  	//Емкость батареи при разряде 1/6 часа
signed short BAT_C_POINT_1_2;  	//Емкость батареи при разряде 1/2 часа
signed short BAT_C_POINT_1;		//Емкость батареи при разряде 1 час
signed short BAT_C_POINT_3;		//Емкость батареи при разряде 3 часа
signed short BAT_C_POINT_5;		//Емкость батареи при разряде 5 часов
signed short BAT_C_POINT_10;	//Емкость батареи при разряде 10 часов
signed short BAT_C_POINT_20;	//Емкость батареи при разряде 20 часов
signed short BAT_U_END_1_6;  	//Конечное напряжение батареи при разряде 1/6 часа
signed short BAT_U_END_1_2;  	//Конечное напряжение батареи при разряде 1/2 часа
signed short BAT_U_END_1;  		//Конечное напряжение батареи при разряде 1 час
signed short BAT_U_END_3;  		//Конечное напряжение батареи при разряде 3 часа
signed short BAT_U_END_5;  		//Конечное напряжение батареи при разряде 5 часов
signed short BAT_U_END_10;  	//Конечное напряжение батареи при разряде 10 часов
signed short BAT_U_END_20;  	//Конечное напряжение батареи при разряде 20 часов
signed short BAT_C_POINT_NUM_ELEM;	//Количество элементов в батарее
signed short BAT_K_OLD;			//Коэффициент старения батареи

//**********************************************
//Показания АЦП на плате измерения тока батареи
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;
short bIBAT_SMKLBR_cnt;
short ibat_metr_cnt;

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//Отладка
char plazma;
char plazma_tx_cnt;
char plazma_uart1[5];
short plazma_short;


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
char kalibrate_func(short in)
{
char parametr, make;

parametr=(char)((in&0xff00)>>8);
make=(char)(in&0x00ff);

if(parametr==1)		//Температура датчика батареи
	{
	signed short temp_SS;
	temp_SS=lc640_read_int(EE_KTBAT1);
	if(make==0x11)
		{
		temp_SS++;
		}
	else if(make==0x12)
		{
		temp_SS+=10;
		}
	else if(make==0x21)
		{
		temp_SS--;
		}
	else if(make==0x22)
		{
		temp_SS-=10;
		}
	lc640_write_int(EE_KTBAT1,temp_SS);
	}
}
									

//-----------------------------------------------
void net_drv(void)
{ 

max_net_slot=32;
//if(NUMINV) max_net_slot=MINIM_INV_ADRESS+NUMINV;
//gran_char(&max_net_slot,0,MAX_NET_ADRESS);

if(++cnt_net_drv>max_net_slot) 
	{
	cnt_net_drv=-4;
	//LPC_GPIO2->FIODIR|=(1UL<<7);
	//LPC_GPIO2->FIOPIN^=(1UL<<7);
	///if(bCAN_INV)bCAN_INV=0;
	///else bCAN_INV=1;

	} 



if((cnt_net_drv>=0)&&(cnt_net_drv<=32)) // с 1 по 12 посылки адресные
	{
	//cnt_net_drv=2; 
//	if(mess_find_unvol(MESS2NET_DRV))
//		{
//		if(mess_data[0]==PARAM_BPS_NET_OFF)
//			{
			//mess_data[1]=1;
//			if(sub_ind1==cnt_net_drv)
//				{
//				return;
//				}
//			}
//		}
			   
	if(!bCAN_OFF)can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	if(cnt_net_drv<=32)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 /*		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			} */
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;	
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}

else if(cnt_net_drv==-4)
	{
//    if(!bCAN_OFF)can1_out(GETTM_IBATMETER,GETTM_IBATMETER,0,0,0,0,0,0);
//    ibat_metr_cnt++;
	}

else if(cnt_net_drv==-1)
	{
//    if(!bCAN_OFF)can1_out(PUT_BDR,PUT_BDR,bdr_transmit_stat,bdr_transmit_stat,*((char*)((&UMAX))+1),*((char*)(&DU)),*((char*)((&DU))+1),0);
//    plazma_pavlik++;
	}

else if(cnt_net_drv==12)
	{
	if(!bCAN_OFF)can1_out(0xff,0xff,MEM_KF,*((char*)(&UMAX)),*((char*)((&UMAX))+1),*((char*)(&DU)),*((char*)((&DU))+1),0);
	} 
     
else if(cnt_net_drv==13)
	{
	} 

else if(cnt_net_drv==14)
	{                 
	}
	
	
else if(cnt_net_drv==15)
	{
	if(!bCAN_OFF)can1_out(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);
	}


else if(cnt_net_drv==19)
	{
	}
	
/*	
else if((cnt_net_drv>=0)&&(cnt_net_drv<32))
	{
	if(!bCAN_OFF)
		{
		can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
		}
	if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
		{    
		bps[cnt_net_drv]._cnt++;

		}
	else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
	if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
	bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	}*/
}
/*
//-----------------------------------------------
void delay_us(long del)
{
long temp;
temp=5*del;

while (--temp);
return;
}*/

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
if(modbus_timeout_cnt<6)
	{
	modbus_timeout_cnt++;
	if(modbus_timeout_cnt>=6)
		{
		bMODBUS_TIMEOUT=1;
		}
	}
else if (modbus_timeout_cnt>6)
	{
	modbus_timeout_cnt=0;
	bMODBUS_TIMEOUT=0;
	}
} 

/*----------------------------------------------------------------------------
 *        Main: 
 *---------------------------------------------------------------------------*/

int main (void) 
{



stm32_Init();
spi2_config();
//SysTick->LOAD  = 544;  
//                            
//dac_init();
//init_display ();
//plazma1=SD_Reset();
//i2c_init();
//tx1_restart=1;
plazma=1;

lc640_write_int(0x0010,0x64);
adc_init();
can_mcp2515_init();
//beep_init(0x00000005,'A');


while (1) 
	{
	//delay_us(100000); 
	//GPIOB->ODR^=(1<<10)|(1<<11)|(1<<12);
	//GPIOC->ODR^=(1<<6);
	if (b1000Hz) 
		{
		b1000Hz=(bool)0;
		//
		can_mcp2515_hndl();
		}
	if (b100Hz) 
		{
		b100Hz=(bool)0;
		//adc_drv();
		net_drv();
		}
	if (b10Hz) 
		{
		b10Hz=(bool)0;
		adc_drv();
		beep_drv(); 
//		GPIOB->ODR^=(1<<11);
		}   
	if (b5Hz) 
		{
		b5Hz=(bool)0; 
		//can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
		memo_read();
		}
	if (b1Hz) 
		{
		b1Hz=(bool)0;
		plazma_tx_cnt++;
		
		GPIOB->ODR^=(1<<10);

		//putchar1('a');
		//uart_out1 (4,'a','b','c',plazma_tx_cnt,0,0);
		//uart_out2 (4,'d','e','f',plazma_tx_cnt,0,0);
		//printf("MAMA MILA RAMU");
		//printf("plazma = %02d\r\n", plazma++);
		//printf("plazma = %d; %d; %d; %d;\r\n  rx_wr_index1 = %d\r\n", plazma_uart1[0], plazma_uart1[1], plazma_uart1[2], plazma_uart1[3], rx_wr_index1);
		//putchar2('a');
		//plazma_short = lc640_read_int(0x0010);
		//printf("adc_buff_ = %d; %d; %d; %d;\r\n  %d; %d; %d; %d;\r\n %d; %d;", adc_buff_[0], adc_buff_[1], adc_buff_[2], adc_buff_[3], adc_buff_[4],  adc_buff_[5],  adc_buff_[6],  adc_buff_[7],adc_ch, adc_cnt);
		//printf("rtc = %x; %x; %x; %x;\r\n", RTC->CRH, RTC->CRL, RTC->CNTH, RTC->CNTL);
		//printf("cnt = %2d; %2d; %2d; %2d;\r\n", bps[0]._cnt, bps[1]._cnt, bps[2]._cnt, bps[3]._cnt);
		printf("Ktbat[0] = %5d;\r\n", Ktbat[0]);
		//spi2(0x55);
		beep_hndl();
		}
	if(bMODBUS_TIMEOUT)
		{
		bMODBUS_TIMEOUT=0;
		//modbus_plazma++;;
		modbus_in();
		}
	if(bMCP2515_IN)
		{
		bMCP2515_IN=0;
		can_in_an1();
		}
	}
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

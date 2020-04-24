#include <stm32f10x_conf.h>
#include <string.h>                   /* string and memory functions         */
#include "STM32_Init.h"               /* stm32 initialisation                */
#include <stm32f10x_lib.h>
#include "STM32_Reg.h"
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
#include "stm32_rtc.h"

#define	putchar putchar2
#define can1_out mcp2515_transmit

#define SEC_IN_DAY 86400

//***********************************************
//��������
unsigned char t0cnt0, t0cnt1, t0cnt2, t0cnt3, t0cnt4, t0cnt5; 
bool b1000Hz, b100Hz, b50Hz, b1Hz, b10Hz, b5Hz, b2Hz;
bool bFL, bFL2, bFL5;
signed short main_10Hz_cnt;
signed short main_1Hz_cnt;


//***********************************************
//������ �� EEPROM
signed short ICA_MODBUS_ADDRESS;//����� �������� ��� ������������ ����� �� ���� MODBUS-RTU
signed short MODBUS_ADRESS=1;
signed int MODBUS_BAUDRATE=115200;

//***********************************************
//��������� ��������� ����
signed short net_U,net_Ustore,net_Ua,net_Ub, net_Uc, net_Umax;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;	//������� �� �������� ���������� ��������
signed char unet_max_drv_cnt; //������� �� ���������� ���������� ��������
char net_av;		//����������� �������� ���� 0 - �����, 1 - ��������, 2 - ��������

//***********************************************
//��������� �������
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;
signed short		Ib_ips_termokompensat_temp;

//***********************************************
//��������� ����������
BPS_STAT bps[40];

//***********************************************
//��������� ������
signed short bps_U;
signed short out_U;
signed short bps_I;
signed short bps_I_phantom;

//***********************************************
//��������� ��������
signed short load_U;
signed short load_I;

//***********************************************
//���������� �����
signed short speedChrgCurr;			//������������ ��� ����������� ������, ����������� �� ������
signed short speedChrgVolt;			//������������ ���������� ����������� ������, ����������� �� ������
signed short speedChrgTimeInHour; 		//������������ ����� ����������� ������ � �����, ����������� �� ������
signed short speedChrgAvtEn;	    		//�������������� ��������� ����������� ������ ��������/���������
signed short speedChrgDU;	    		//�������� ���������� ����������� ��� ��������� ����������� ������
signed short speedChIsOn;			//������� ��������� ����������� ������ ���/����
signed long  speedChTimeCnt;			//������� ������� ������ ����������� ������
signed short speedChrgBlckSrc;		//�������� ������� ����������, 0-����., 1-��1, 2-��2
signed short speedChrgBlckLog;		//������ ������� ����������, 1 - ���������� �� ���������� ��, 0 - �� ������������
signed short speedChrgBlckStat;		//������ ���������� ��� �������������� � ����������� ������.
char  	   	 speedChrgShowCnt;		//������� ������ ��������������� ���������
signed short SP_CH_VENT_BLOK;
char spch_plazma[2];

//***********************************************
//����� ���������� �����
enum_sp_ch_stat sp_ch_stat=scsOFF,sp_ch_stat_old;
short sp_ch_stat_cnt;
long sp_ch_wrk_cnt;
char speedChargeStartCnt=0;

//***********************************************
//�������� ��������� ����������
signed short outVoltContrHndlCnt;		//�������, ������� � ���� � ������ ���������� ������� ������
signed short outVoltContrHndlCnt_;		//�������, ������� � ���� � ������ ���������� ���������� ������� ������
char uout_av;

//***********************************************
//����������� �����
signed short cnt_net_drv;
char max_net_slot;
bool bCAN_OFF;

//***********************************************
//�����������
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
char vz_error=0;  		// ���������������, ���� ������������� ����� ������������ �� ����������
char sp_ch_error=0;		// ���������������, ���� ���������� ����� ������������ �� ����������
char vz1_error=0;		// ���������������, ���� ������������� ����� ������������ �� ����������
char vz2_error=0;		// ���������������, ���� ����������� ����� ������������ �� ����������

//**********************************************
//������������, ������������ �� EEPROM
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
signed short UMAXN;
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
signed short BAT_TYPE;	//��� �������. 0 - ������� ���������, 1-�������� COSLIGHT, 2-�������� SACRED SUN , 3-�������� ZTT
signed short DU_LI_BAT;	//��������, ������������ ���������� ���������� �������� �������
signed short FORVARDBPSCHHOUR;	//������������������ �������� ��������� � �����. ���� 0 - ������� ��������� � ������� ������ ��������
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
signed short CNTRL_HNDL_TIME;	//���������� ������� ������������� ���������� ��� ��������
signed short USODERG_LI_BAT;	//���������� ���������� �������� �������
signed short QSODERG_LI_BAT;	//����� ��� ������� �������� ����������� ���������� ���������� �������� �������
signed short TVENTMAX;			//������������ ������ �����������
signed short ICA_EN;			//������������ ������ ������������ ����� ���
signed short ICA_CH;			//����� ����� ��� ������������ �����, 0 - MODBUS, 1 - MODBUS-TCP
signed short ICA_MODBUS_ADDRESS;//����� �������� ��� ������������ ����� �� ���� MODBUS-RTU
signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	//IP �������� ��� ������������ ����� �� ���� MODBUS-TCP
signed short ICA_MODBUS_TCP_UNIT_ID;	//UNIT ID �������� ��� ������������ ����� �� ���� MODBUS-TCP
signed short PWM_START;			//��������� ��� ��� ������
signed short KB_ALGORITM;		//2-� ��� 3-� ������������ �������� �������� ���� �������
signed short REG_SPEED;			//�������� �������������, 1- �����������, 2,3,4,5- ����������� � 2,3,4,5 ���
enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;
signed short RS485_QWARZ_DIGIT;
signed short UVENTOFF;			//���������� (1�) ��� ������� ���������� ���������� ����� ��������� �� ��� ����
signed short VZ_KIND;			//��� �������������� ������, 0 - �������(������������, ��������� ���������� �� �����), 
								//1- ��������������, � ��������� ���������� � ��������� � ���������
signed short SNTP_ENABLE;
signed short SNTP_GMT;

signed short RELE1SET;			//��������� ������������ ����1
								//0��� - �������� (1 - ������������ �����������, 0 - ������������ ��������������)
								//1��� - ������ �������� ����, ��������(1 - ���)
								//2��� - ������ �������� ����, ��������(1 - ���)
								//3��� - ������� ������� (1 - ���)
								//4��� - ������ ������� (1 - ���)
								//5��� - ������� ������������ (1 - ���)
								//6��� - ������ ������������ (1 - ���)
								//7��� - ������ ��������� ����������, �������� (1 - ���)
								//8��� - ������ ��������� ����������, �������� (1 - ���)
signed short RELE2SET;			//��������� ������������ ����2, �������� ����� ��� � � ����1
signed short RELE3SET;		   	//��������� ������������ ����3, �������� ����� ��� � � ����1

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

//signed short RELE_SET_MASK[4]={1,2,3,4};

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

signed short BAT_C_POINT_1_6;  	//������� ������� ��� ������� 1/6 ����
signed short BAT_C_POINT_1_2;  	//������� ������� ��� ������� 1/2 ����
signed short BAT_C_POINT_1;		//������� ������� ��� ������� 1 ���
signed short BAT_C_POINT_3;		//������� ������� ��� ������� 3 ����
signed short BAT_C_POINT_5;		//������� ������� ��� ������� 5 �����
signed short BAT_C_POINT_10;	//������� ������� ��� ������� 10 �����
signed short BAT_C_POINT_20;	//������� ������� ��� ������� 20 �����
signed short BAT_U_END_1_6;  	//�������� ���������� ������� ��� ������� 1/6 ����
signed short BAT_U_END_1_2;  	//�������� ���������� ������� ��� ������� 1/2 ����
signed short BAT_U_END_1;  		//�������� ���������� ������� ��� ������� 1 ���
signed short BAT_U_END_3;  		//�������� ���������� ������� ��� ������� 3 ����
signed short BAT_U_END_5;  		//�������� ���������� ������� ��� ������� 5 �����
signed short BAT_U_END_10;  	//�������� ���������� ������� ��� ������� 10 �����
signed short BAT_U_END_20;  	//�������� ���������� ������� ��� ������� 20 �����
signed short BAT_C_POINT_NUM_ELEM;	//���������� ��������� � �������
signed short BAT_K_OLD;			//����������� �������� �������

//**********************************************
//��������� ��� �� ����� ��������� ���� �������
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;
short bIBAT_SMKLBR_cnt;
short ibat_metr_cnt;

//**********************************************
//���������� ��������������
signed short Isumm;
signed short Isumm_;


//**********************************************
//������������� ��������� ��������
short factory_settings_hndl_main_iHz_cnt;

//***********************************************
//���������� ����
char rele_output_stat;
//0 ���� -> "1" ���� 1 ��� ���
//1 ���� -> "1" ���� 2 ��� ���
//2 ���� -> "1" ���� 3 ��� ���
//3 ���� -> "1" ���� HV ��� ���

//***********************************************
//���������� ��������� ����������
unsigned short test_control_register,test_control_register_old;
char rele_output_stat_test_byte=0;
char rele_output_stat_test_mask=0;
char tst_hndl_cnt;
char test_hndl_rele1_cntrl,test_hndl_rele1_cnt;
char test_hndl_rele2_cntrl,test_hndl_rele2_cnt;
char test_hndl_rele3_cntrl,test_hndl_rele3_cnt;
char test_hndl_releHV_cntrl,test_hndl_releHV_cnt;
char test_hndl_bps_number;
char test_hndl_bps_state;
short test_hndl_bps_cnt;


//***********************************************
//���������� ������������
char ledUOUTGOOD;	//������� ��������� "�������� ���������� � �����"
char ledWARNING;  	//������ ��������� "������� � ����� �� ���������"
char ledERROR;		//������� ��������� "������ � ����� �� ���������"
char ledCAN;	   	//������� ��������� "����� �� ��� � �����"


//***********************************************
//***********************************************
//***********************************************
//***********************************************
//�������
char plazma;
char plazma_tx_cnt;
char plazma_uart1[5];
short plazma_short;
char plazma_debug_0,plazma_debug_1;


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
void calendar_hndl(void)
{
long tempL;
tempL=RTC->CNTH;
tempL<<=16;
tempL+=RTC->CNTL;

while(tempL>=SEC_IN_DAY)
	{
	char max_day_of_month = 31;
	
	tempL-=SEC_IN_DAY;

	
	if( ((BKP->DR2)==4)	|| ((BKP->DR2)==6) || ((BKP->DR2)==9) || ((BKP->DR2)==11) )	max_day_of_month = 30;
	if((BKP->DR2)==2)
		{
		if((BKP->DR1)%4) max_day_of_month = 28;
		else max_day_of_month = 29;
		}

	if((BKP->DR3)<max_day_of_month)
		{
		PWR->CR	|= PWR_CR_DBP;
		BKP->DR3++;
		PWR->CR	&= ~PWR_CR_DBP;		
		}
	else 
		{
		PWR->CR	|= PWR_CR_DBP;
		BKP->DR3=1;
		BKP->DR2++;
		if((BKP->DR2)>12)
			{
			(BKP->DR2)=1;
			(BKP->DR1)++;
			}
		PWR->CR	&= ~PWR_CR_DBP;
		}

//	tempL-=SEC_IN_DAY;
/*	PWR->CR	|= PWR_CR_DBP;
	RTC->CNTH=tempL>>16;
	RTC->CNTL=(short)tempL;
	BKP->DR3++;
	PWR->CR	&= ~PWR_CR_DBP;*/
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(tempL>>16);
				RTC->CNTL=(short)tempL;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
	}

}

//-----------------------------------------------
void factory_settings_hndl(void)
{
short tempS;
tempS=(RTC->CNTL)-(BKP->DR4);
if((tempS>30)&&(tempS<60)&&((BKP->DR5)==0))
	{
	PWR->CR	|= PWR_CR_DBP;
	BKP->DR5=1;
	PWR->CR	&= ~PWR_CR_DBP;
	}
else if((tempS>10)&&(tempS<20)&&((BKP->DR5)==1))
	{
	PWR->CR	|= PWR_CR_DBP;
	BKP->DR5=2;
	PWR->CR	&= ~PWR_CR_DBP;
	}
else if((tempS>30)&&(tempS<60)&&((BKP->DR5)==2))
	{
	PWR->CR	|= PWR_CR_DBP;
	BKP->DR5=3;
	PWR->CR	&= ~PWR_CR_DBP;
	lc640_write_int(EE_MODBUS_ADRESS,1);
	lc640_write_int(EE_MODBUS_BAUDRATE,960);
	}
/*else
	{
	PWR->CR	|= PWR_CR_DBP;
	BKP->DR5=0;
	PWR->CR	&= ~PWR_CR_DBP;
	}  */
PWR->CR	|= PWR_CR_DBP;
BKP->DR4=RTC->CNTL;
PWR->CR	&= ~PWR_CR_DBP;

if(factory_settings_hndl_main_iHz_cnt<1000)factory_settings_hndl_main_iHz_cnt++;
if(factory_settings_hndl_main_iHz_cnt==100)
	{
	PWR->CR	|= PWR_CR_DBP;
	BKP->DR5=0;
	PWR->CR	&= ~PWR_CR_DBP;
	}
}	

//-----------------------------------------------
char kalibrate_func(short in)
{
char parametr, make;

parametr=(char)((in&0xff00)>>8);
make=(char)(in&0x00ff);

if(parametr==1)		//����������� ������� �������
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



if((cnt_net_drv>=0)&&(cnt_net_drv<=32)) // � 1 �� 12 ������� ��������
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

	if((GPIOC->IDR&=(1<<1)))bFF=1;
	else bFF=0;
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;
	
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
		if(main_1Hz_cnt<10000) main_1Hz_cnt++;
		}

     hz_out_cnt++;
     if(hz_out_cnt>=500)
	     {	
	     hz_out_cnt=0;
	     net_F=hz_out;
	     hz_out=0;
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
rtc_init();

memo_first_init ();
memo_read();
stm32_Usart1Setup(MODBUS_BAUDRATE*10UL);
lc640_write_int(0x0010,0x64);
adc_init();
can_mcp2515_init();

//calendar_hndl();
factory_settings_hndl();
//beep_init(0x00000005,'A');

//lc640_write_int(EE_NUMPHASE,3);
calendar_hndl();

while (1) 
	{
	//can_rotor[1]++;
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
		//TMAX=80;
		//TSIGN=80;
		
/*		bps[0]._flags_tu=0;
		bps[1]._flags_tu=1;	*/
		//NUMIST=3;
		net_drv();
		}
	if (b10Hz) 
		{
		char i;

		b10Hz=(bool)0;
		adc_drv();
		beep_drv(); 
	   	ext_drv();
		u_necc_hndl();
		cntrl_hndl();
		for(i=0;i<NUMIST;i++)bps_drv(i);
		bps_hndl();
//		calendar_hndl();
		led_hndl();		//���������� ������������, ����������
		led_drv();		//���������� ������������, ����������
	   	rele_hndl(); 	//���������� ���������� ����
		rele_drv(); 	//���������� ���������� ����
		tst_hndl();		//������������ �������� ��������
		mess_hndl();
		unet_drv();
		}   
	if (b5Hz) 
		{
		b5Hz=(bool)0; 
		//can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
		memo_read();
		TZAS=3;
		

		matemat();	//���������� ���� �������

		can1_out(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);

		}
	if (b2Hz) 
		{
		b2Hz=(bool)0;

		avt_klbr_hndl_();	//������� �������������� ���������� ����� � �����

		}

	if (b1Hz) 
		{
		b1Hz=(bool)0;

		num_necc_hndl();
		kb_hndl();



		plazma_tx_cnt++;
		
		//GPIOB->ODR^=(1<<10);
	   	//printf ("\033c");
		//putchar1('a');
		//uart_out1 (4,'a','b','c',plazma_tx_cnt,0,0);
		//uart_out2 (4,'d','e','f',plazma_tx_cnt,0,0);
		//printf("MAMA MILA RAMU");
		//printf("plazma = %02d\r\n", plazma++);
		//printf("plazma = %d; %d; %d; %d;\r\n  rx_wr_index1 = %d\r\n", plazma_uart1[0], plazma_uart1[1], plazma_uart1[2], plazma_uart1[3], rx_wr_index1);
		//putchar2('a');
		//plazma_short = lc640_read_int(0x0010);
		//printf("adc_buff = %d; %d; %d; %d; %d; %d; %d; %d; %d; %d;\r\n", adc_buff_[0], adc_buff_[1], adc_buff_[2], adc_buff_[3], adc_buff_[4],  adc_buff_[5],  adc_buff_[6],  adc_buff_[7],adc_ch, adc_cnt);
		//printf("rtc = %x; %x; %x; %x;\r\n", RTC->CRH, RTC->CRL, RTC->CNTH, RTC->CNTL);
		//printf("cnt = %2d; %2d; %2d; %2d;\r\n", bps[0]._cnt, bps[1]._cnt, bps[2]._cnt, bps[3]._cnt);
		//printf("Ktbat[0] = %5d;\r\n", Ktbat[0]);
		//printf("%5d   %5d   %5d   %5d   %5d   \r\n", plazma_uart1[0], plazma_uart1[1], plazma_uart1[2], plazma_uart1[3], plazma_uart1[4]);
		//spi2(0x55);
		//printf("%5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   %5d   \r\n", adc_buff[0][0], adc_buff[0][1], adc_buff[0][2], adc_buff[0][3], adc_buff[0][4], adc_buff[0][5], adc_buff[0][6], adc_buff[0][7], adc_buff[0][8], adc_buff[0][9], adc_buff[0][10], adc_buff[0][11], adc_buff[0][12], adc_buff[0][13], adc_buff[0][14], adc_buff[0][15]);
		//printf("%3d   %3d   %3d   %3d   %3d    %3d \r\n", bps[0]._cnt, bps[1]._cnt, bps[2]._cnt, can_rotor[1], can_rotor[2], bps[0]._Ti);
		//printf("num_necc= %3d   %d   %d   %3d   %3d    %3d \r\n", num_necc, ibat_metr_buff_[0], ibat_metr_buff_[1], Kibat1[0], Ib_ips_termokompensat, bps[0]._Ti);
		//printf("flags_tu= %2x   %2x   %2x  \r\n", bps[0]._flags_tu, bps[1]._flags_tu, bps[2]._flags_tu);
		//printf("RELESET= %2x   %2x   %2x  \r\n", RELE1SET, RELE2SET, RELE3SET);
		//printf("\r\n bat_temper = %d num_necc= %d  u_necc= %3d  cntrl_stat= %4d  UB0= %3d  UB20= %3d  bps_U= %3d  DU= %3d \r\n",bat[0]._Tb ,num_necc, u_necc, cntrl_stat, UB0, UB20, bps_U, DU);
		//printf("flags_tu= %2X %2X %2X  state= %2X %2X %2X  av= %2X %2X %2X flags_tm= %2X %2X %2X umin_av_cnt= %d %d %d\r\n", bps[0]._flags_tu,bps[1]._flags_tu,bps[2]._flags_tu, bps[0]._state, bps[1]._state, bps[2]._state ,bps[0]._av ,bps[1]._av, bps[2]._av, bps[0]._flags_tm ,bps[1]._flags_tm, bps[2]._flags_tm, bps[0]._umin_av_cnt ,bps[1]._umin_av_cnt, bps[2]._umin_av_cnt);
	   	//printf("%d  %d  %d  %d \r\n", bps[0]._x_, bps[1]._x_, bps[2]._x_, bps[3]._x_);
		//printf("%2d; %4d; %2d; %2d;\r\n", avt_klbr_phase_ui, modbus_register_1022, bps[2]._cnt, bps[3]._cnt);
		//printf("%3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d\r\n", cntrl_hndl_plazma, bps[0]._cnt, bps[1]._cnt, bps[2]._cnt,UMAXN,MODBUS_BAUDRATE,/*MODBUS_ADRESS*/test_hndl_rele2_cntrl,/*plazma_uart1[2]*/test_hndl_rele2_cnt,/* NUMIST*/rele_output_stat_test_byte, /*NUMPHASE*/ rele_output_stat_test_mask);
		printf("net_U= %3d  %3d  %3d  %3d  net_av= %2d u_necc = %2d  releset[0] = %2d \r\n", net_U, net_Ua, net_Ub, net_Uc, net_av, rele_output_stat, RELE1SET);
		printf("%3d; %3d; %3d; %3d;\r\n", net_Umax, UMAXN, unet_max_drv_cnt);
		//RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		//PWR->CR      |= PWR_CR_DBP;
		//BKP->DR1=(BKP->DR1)+1;
		//PWR->CR   &= ~PWR_CR_DBP;
		//printf("%5d %5d %5d %5d %5d %5d %5d %5d %5d\r\n", BKP->DR1, cntrl_stat, modbus_register_995, bps[0]._flags_tu, RTC->CNTH, RTC->CNTL, MODBUS_ADRESS, BKP->DR4, BKP->DR5);
		avg_hndl();

		beep_hndl();
		//calendar_hndl();
		factory_settings_hndl();

		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		speedChargeHndl();
		//averageChargeHndl();
		//vz1_drv();
		//vz2_drv();
		#endif
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
	if(plazma_debug_1)
		{
		plazma_debug_1=0;
		printf("%d;\r\n", plazma_debug_0);
		}
	}
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

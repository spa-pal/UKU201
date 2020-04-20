
#include <stm32f10x_lib.h>
#include <stm32f10x_conf.h>
#include "stm32_Reg.h"
#include "modbus.h"
#include "main.h"
#include "uart1.h"
#include "cmd.h"
#include "control.h"
#include <string.h>
#include <stdio.h>
#include "eeprom_map.h"
#include "rtl.h"
//#include "modbus_tcp.h"
#include "25lc640.h"
//#include "sc16is7xx.h"
//#include "uart0.h"
#include "avar_hndl.h"

#define can1_out mcp2515_transmit

#define MODBUS_RTU_PROT	0
#define MODBUS_TCP_PROT	1

//extern int  mem_copy (void *dp, void *sp, int len);

unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer[30];	//Буфер, куда складывает принимаемые даннные обработчик прерывания по приему УАРТа 
unsigned char modbus_an_buffer[30];    	//Буфер, куда они потом копируются для анализа
unsigned char modbus_rx_buffer_ptr;	//Указатель на текущую позицию принимающего буфера
unsigned char modbus_rx_counter;		//Количество принятых байт, используется при анализе целостности посылки и при расшифровке

short modbus_plazma;				//Отладка
short modbus_plazma1;				//Отладка
short modbus_plazma2;				//Отладка
short modbus_plazma3;				//Отладка
short modbus_plazma_p;				//Отладка
short modbus_plazma_pp;				//Отладка

unsigned short modbus_rx_arg0;		//встроенный в посылку первый аргумент
unsigned short modbus_rx_arg1;		//встроенный в посылку второй аргумент
unsigned short modbus_rx_arg2;		//встроенный в посылку третий аргумент
unsigned short modbus_rx_arg3;		//встроенный в посылку четвертый аргумент

char modbus_tx_buff[100];

char* modbus_tcp_out_ptr;

short modbus_register_offset;
short modbus_register_offset_ui;
short modbus_register_offset_un;
short modbus_register_offset_i;
short modbus_register_offset_t;
short modbus_register_995, modbus_register_996, modbus_register_997;
short modbus_register_998;
short modbus_register_999;
short modbus_register_1000, modbus_register_1001, modbus_register_1002, modbus_register_1003;
short modbus_register_1004, modbus_register_1005, modbus_register_1006, modbus_register_1007;
short modbus_register_1008, modbus_register_1009, modbus_register_1010, modbus_register_1011;
short modbus_register_1012, modbus_register_1013, modbus_register_1014, modbus_register_1015;
short modbus_register_1016, modbus_register_1017, modbus_register_1018, modbus_register_1019;
short modbus_register_1020, modbus_register_1021;
short modbus_register_1022, modbus_register_1023, modbus_register_1024, modbus_register_1025;
short modbus_register_1027, modbus_register_1028, modbus_register_1029, modbus_register_1030;
//char modbus_registers[200];

//static const char foo[] = "I wish I'd read K&R, and other tomes more diligently";



/*modbus_registers[3]=(char)(bps_I%256);
modbus_registers[4]=(char)(net_U/256);					//Рег3   	напряжение сети питания, 1В
modbus_registers[5]=(char)(net_U%256);
modbus_registers[6]=(char)(net_F/256);					//Рег4   	частота сети питания, 0.1Гц
modbus_registers[7]=(char)(net_F%256);
modbus_registers[8]=(char)(net_Ua/256);					//Рег5	напряжение сети питания фаза A, 1В	
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//Рег6	напряжение сети питания фаза B, 1В
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//Рег7	напряжение сети питания фаза C, 1В
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(bat[0]._Ub/256);				//Рег8	напряжение батареи №1, 0.1В
modbus_registers[15]=(char)(bat[0]._Ub%256);
modbus_registers[16]=(char)(bat[0]._Ib/256);				//Рег9   	ток батареи №1, 0.01А
modbus_registers[17]=(char)(bat[0]._Ib%256);
modbus_registers[18]=(char)(bat[0]._Tb/256);				//Рег10	температура батареи №1, 1Гц
modbus_registers[19]=(char)(bat[0]._Tb%256);
modbus_registers[20]=(char)(bat[0]._zar/256);			//Рег11	заряд батареи №1, %
modbus_registers[21]=(char)(bat[0]._zar%256);
modbus_registers[22]=(char)(bat[0]._Ubm/256);			//Рег12	напряжение средней точки батареи №1, 0.1В
modbus_registers[23]=(char)(bat[0]._Ubm%256);
modbus_registers[24]=(char)(bat[0]._dUbm/256);			//Рег13	ошибка средней точки батареи №1, %
modbus_registers[25]=(char)(bat[0]._dUbm%256);
modbus_registers[26]=(char)(BAT_C_REAL[0]/256);			//Рег14	Реальная емкость батареи №1, 0.1А*ч, если 0x5555 то не измерялась
modbus_registers[27]=(char)(BAT_C_REAL[0]%256);
modbus_registers[28]=(char)(bat[1]._Ub/256);				//Рег15	напряжение батареи №1, 0.1В
modbus_registers[29]=(char)(bat[1]._Ub%256);
modbus_registers[30]=(char)(bat[1]._Ib/256);				//Рег16   	ток батареи №1, 0.01А
modbus_registers[31]=(char)(bat[1]._Ib%256);
modbus_registers[32]=(char)(bat[1]._Tb/256);				//Рег17	температура батареи №1, 1Гц
modbus_registers[33]=(char)(bat[1]._Tb%256);
modbus_registers[34]=(char)(bat[1]._zar/256);			//Рег18	заряд батареи №1, %
modbus_registers[35]=(char)(bat[1]._zar%256);
modbus_registers[36]=(char)(bat[1]._Ubm/256);			//Рег19	напряжение средней точки батареи №1, 0.1В
modbus_registers[37]=(char)(bat[1]._Ubm%256);
modbus_registers[38]=(char)(bat[1]._dUbm/256);			//Рег20	ошибка средней точки батареи №1, %
modbus_registers[39]=(char)(bat[1]._dUbm%256);
modbus_registers[40]=(char)(BAT_C_REAL[1]/256);			//Рег21	Реальная емкость батареи №1, 0.1А*ч, если 0x5555 то не измерялась
modbus_registers[41]=(char)(BAT_C_REAL[1]%256);
modbus_registers[42]=(char)(bps[0]._Uii/256);			//Рег22	Выходное напряжение выпрямителя №1, 0.1В
modbus_registers[43]=(char)(bps[0]._Uii%256);
modbus_registers[44]=(char)(bps[0]._Ii/256);				//Рег23	Выходной ток выпрямителя №1, 0.1А
modbus_registers[45]=(char)(bps[0]._Ii%256);
modbus_registers[46]=(char)(bps[0]._Ti/256);				//Рег24	Температура радиатора выпрямителя №1, 1гЦ
modbus_registers[47]=(char)(bps[0]._Ti%256);
modbus_registers[48]=(char)(bps[0]._av/256);				//Рег25	Байт флагов выпрямителя №1, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[49]=(char)(bps[0]._av%256);
modbus_registers[50]=(char)(bps[1]._Uii/256);			//Рег26	Выходное напряжение выпрямителя №2, 0.1В
modbus_registers[51]=(char)(bps[1]._Uii%256);
modbus_registers[52]=(char)(bps[1]._Ii/256);				//Рег27	Выходной ток выпрямителя №2, 0.1А
modbus_registers[53]=(char)(bps[1]._Ii%256);
modbus_registers[54]=(char)(bps[1]._Ti/256);				//Рег28	Температура радиатора выпрямителя №2, 1гЦ
modbus_registers[55]=(char)(bps[1]._Ti%256);
modbus_registers[56]=(char)(bps[1]._av/256);				//Рег29	Байт флагов выпрямителя №2, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[57]=(char)(bps[1]._av%256);
modbus_registers[58]=(char)(bps[2]._Uii/256);			//Рег30	Выходное напряжение выпрямителя №3, 0.1В
modbus_registers[59]=(char)(bps[2]._Uii%256);
modbus_registers[60]=(char)(bps[2]._Ii/256);				//Рег31	Выходной ток выпрямителя №3, 0.1А
modbus_registers[61]=(char)(bps[2]._Ii%256);
modbus_registers[62]=(char)(bps[2]._Ti/256);				//Рег32	Температура радиатора выпрямителя №3, 1гЦ
modbus_registers[63]=(char)(bps[2]._Ti%256);
modbus_registers[64]=(char)(bps[2]._av/256);				//Рег33	Байт флагов выпрямителя №3, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[65]=(char)(bps[2]._av%256);
modbus_registers[66]=(char)(bps[3]._Uii/256);			//Рег34	Выходное напряжение выпрямителя №4, 0.1В
modbus_registers[67]=(char)(bps[3]._Uii%256);
modbus_registers[68]=(char)(bps[3]._Ii/256);				//Рег35	Выходной ток выпрямителя №4, 0.1А
modbus_registers[69]=(char)(bps[3]._Ii%256);
modbus_registers[70]=(char)(bps[3]._Ti/256);				//Рег36	Температура радиатора выпрямителя №4, 1гЦ
modbus_registers[71]=(char)(bps[3]._Ti%256);
modbus_registers[72]=(char)(bps[3]._av/256);				//Рег37	Байт флагов выпрямителя №4, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[73]=(char)(bps[3]._av%256);
modbus_registers[74]=(char)(bps[4]._Uii/256);			//Рег38	Выходное напряжение выпрямителя №5, 0.1В
modbus_registers[75]=(char)(bps[4]._Uii%256);
modbus_registers[76]=(char)(bps[4]._Ii/256);				//Рег39	Выходной ток выпрямителя №5, 0.1А
modbus_registers[77]=(char)(bps[4]._Ii%256);
modbus_registers[78]=(char)(bps[4]._Ti/256);				//Рег40	Температура радиатора выпрямителя №5, 1гЦ
modbus_registers[79]=(char)(bps[4]._Ti%256);
modbus_registers[80]=(char)(bps[4]._av/256);				//Рег41	Байт флагов выпрямителя №5, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[81]=(char)(bps[4]._av%256);
modbus_registers[82]=(char)(bps[5]._Uii/256);			//Рег42	Выходное напряжение выпрямителя №6, 0.1В
modbus_registers[83]=(char)(bps[5]._Uii%256);
modbus_registers[84]=(char)(bps[5]._Ii/256);				//Рег43	Выходной ток выпрямителя №6, 0.1А
modbus_registers[85]=(char)(bps[5]._Ii%256);
modbus_registers[86]=(char)(bps[5]._Ti/256);				//Рег44	Температура радиатора выпрямителя №6, 1гЦ
modbus_registers[87]=(char)(bps[5]._Ti%256);
modbus_registers[88]=(char)(bps[5]._av/256);				//Рег45	Байт флагов выпрямителя №6, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[89]=(char)(bps[5]._av%256);
modbus_registers[90]=(char)(bps[6]._Uii/256);			//Рег46	Выходное напряжение выпрямителя №7, 0.1В
modbus_registers[91]=(char)(bps[6]._Uii%256);
modbus_registers[92]=(char)(bps[6]._Ii/256);				//Рег47	Выходной ток выпрямителя №7, 0.1А
modbus_registers[93]=(char)(bps[6]._Ii%256);
modbus_registers[94]=(char)(bps[6]._Ti/256);				//Рег48	Температура радиатора выпрямителя №7, 1гЦ
modbus_registers[95]=(char)(bps[6]._Ti%256);
modbus_registers[96]=(char)(bps[6]._av/256);				//Рег49	Байт флагов выпрямителя №7, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[97]=(char)(bps[6]._av%256);
modbus_registers[98]=(char)(bps[7]._Uii/256);			//Рег50	Выходное напряжение выпрямителя №8, 0.1В
modbus_registers[99]=(char)(bps[7]._Uii%256);
modbus_registers[100]=(char)(bps[7]._Ii/256);			//Рег51	Выходной ток выпрямителя №8, 0.1А
modbus_registers[101]=(char)(bps[7]._Ii%256);
modbus_registers[102]=(char)(bps[7]._Ti/256);			//Рег52	Температура радиатора выпрямителя №8, 1гЦ
modbus_registers[103]=(char)(bps[7]._Ti%256);
modbus_registers[104]=(char)(bps[7]._av/256);			//Рег53	Байт флагов выпрямителя №8, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[105]=(char)(bps[7]._av%256);
modbus_registers[106]=(char)(bps_U/256);					//Рег54   	напряжение выпрямителей, 0.1В
modbus_registers[107]=(char)(bps_U%256);
tempS=0;
if(speedChIsOn) tempS=1;
modbus_registers[108]=(char)(tempS/256);					//Рег55   	Ускоренный заряд включенность, (1 - вкл, 0 - Выкл)
modbus_registers[109]=(char)(tempS%256);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(char)(tempS/256);					//Рег56   	Выравнивающий заряд включенность, (1 - вкл, 0 - Выкл)
modbus_registers[111]=(char)(tempS%256);
modbus_registers[112]=(char)(uout_av/256);					//Рег57   Контроль выходного напряжения, (0 - норма, 1 - завышено, 2 - занижено)
modbus_registers[113]=(char)(uout_av%256);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(char)(tempS/256);				//Рег201	Внешний датчик температуры №1
modbus_registers[401]=(char)(tempS%256);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(char)(tempS/256);				//Рег202	Внешний датчик температуры №2
modbus_registers[403]=(char)(tempS%256);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(char)(tempS/256);				//Рег203	Внешний датчик температуры №3
modbus_registers[405]=(char)(tempS%256);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(char)(tempS/256);				//Рег204	Внешний датчик температуры №4
modbus_registers[407]=(char)(tempS%256);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(char)(tempS/256);				//Рег211	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[421]=(char)(tempS%256);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(char)(tempS/256);				//Рег212	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[423]=(char)(tempS%256);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(char)(tempS/256);				//Рег213	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[425]=(char)(tempS%256);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(char)(tempS/256);				//Рег214	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[427]=(char)(tempS%256);

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	} */

	//   	};


//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

/*
//-----------------------------------------------
void modbus_zapros_ENMV (void){	 //o_2_s
unsigned short crc_temp;
unsigned char i_cnt;
	modbus_tx_buff[0]=MODBUS_ADRESS;
	modbus_tx_buff[1]=1;
	modbus_tx_buff[2]=0;
	modbus_tx_buff[3]=0;
	modbus_tx_buff[4]=0;
	modbus_tx_buff[5]=64;
	crc_temp=CRC16_2(modbus_tx_buff,6);
	modbus_tx_buff[6]=(char)crc_temp;
	modbus_tx_buff[7]=crc_temp>>8;
	for (i_cnt=0;i_cnt<8;i_cnt++)	putchar_sc16is700(modbus_tx_buff[i_cnt]);
	if(enmv_on<5)enmv_on++;
   	else {for (i_cnt=0;i_cnt<64;i_cnt++) snmp_enmv_data[i_cnt]=0xFF;}
} //o_2_e*/

//-----------------------------------------------
void modbus_in(void)
{
char i;
short crc16_calculated;		//вычисляемая из принятых данных CRC
short crc16_incapsulated;	//встроеннная в посылку CRC
signed short modbus_rx_arg0;		//встроенный в посылку первый аргумент
signed short modbus_rx_arg1;		//встроенный в посылку второй аргумент
//unsigned short modbus_rx_arg2;		//встроенный в посылку третий аргумент
//unsigned short modbus_rx_arg3;		//встроенный в посылку четвертый аргумент
unsigned char modbus_func;			//встроенный в посылку код функции
char i_cnt, j_cnt; //o_2

plazma_uart1[1]++;

memcpy(modbus_an_buffer,modbus_rx_buffer,modbus_rx_buffer_ptr);
modbus_rx_counter=modbus_rx_buffer_ptr;
modbus_rx_buffer_ptr=0;
bMODBUS_TIMEOUT=0;
	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

modbus_plazma1=modbus_rx_counter;
modbus_plazma2=crc16_calculated;
modbus_plazma3=crc16_incapsulated;

modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);

//#define IPS_CURR_AVG_MODBUS_ADRESS	1

if(modbus_an_buffer[0]=='r')
	{
	//pvlk=1;
	if(modbus_an_buffer[1]=='e')
		{
		//pvlk=2;
		if(modbus_an_buffer[2]=='a')
			{
			//pvlk=3;
			if(modbus_an_buffer[3]=='d')
				{
				//pvlk=4;
				if(modbus_an_buffer[6]==crc_87((char*)modbus_an_buffer,6))
					{
					//pvlk=5;
					if(modbus_an_buffer[7]==crc_95((char*)modbus_an_buffer,6))
						{
						//pvlk=6;	

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
							//pvlk++;
							ptr=modbus_an_buffer[4]+(modbus_an_buffer[5]*256U);
							data1=lc640_read_long(ptr);
							data2=lc640_read_long(ptr+4);
							temp_out[0]='r';
							temp_out[1]='e';
							temp_out[2]='a';
							temp_out[3]='d';
							temp_out[4]=*((char*)&ptr);
							temp_out[5]=*(((char*)&ptr)+1);	
							temp_out[6]=*((char*)&data1);
							temp_out[7]=*(((char*)&data1)+1);		
							temp_out[8]=*(((char*)&data1)+2);	
							temp_out[9]=*(((char*)&data1)+3);		
							temp_out[10]=*((char*)&data2);
							temp_out[11]=*(((char*)&data2)+1);		
							temp_out[12]=*(((char*)&data2)+2);	
							temp_out[13]=*(((char*)&data2)+3);	
							temp_out[14]=crc_87(temp_out,14);	
							temp_out[15]=crc_95(temp_out,14);			
							
							temp_out[17]=0;
							for (i=0;i<16;i++)
								{
								//*/putchar_sc16is700(temp_out[i]);
								temp_out[17]^=temp_out[i];
								}
							//*/putchar_sc16is700(16);
							//*/putchar_sc16is700(temp_out[17]^16);
							//*/putchar_sc16is700(0x0a);
							}
						}
					}
				}
			} 
		}	 
	} 

if(modbus_an_buffer[0]=='w')
	{
//	pvlk=1;
	if(modbus_an_buffer[1]=='r')
		{
//		pvlk=2;
		if(modbus_an_buffer[2]=='i')
			{
//			pvlk=3;
			if(modbus_an_buffer[3]=='t')
				{
//				pvlk=4;
				if(modbus_an_buffer[4]=='e')
					{
//					pvlk=5;
					if(modbus_an_buffer[15]==crc_87((char*)modbus_an_buffer,15))
						{
//						pvlk=6;
						if(modbus_an_buffer[16]==crc_95((char*)modbus_an_buffer,15))

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
//							pvlk=7;
							ptr=modbus_an_buffer[5]+(modbus_an_buffer[6]*256U);
							*((char*)&data1)=modbus_an_buffer[7];
							*(((char*)&data1)+1)=modbus_an_buffer[8];
							*(((char*)&data1)+2)=modbus_an_buffer[9];
							*(((char*)&data1)+3)=modbus_an_buffer[10];
							*((char*)&data2)=modbus_an_buffer[11];
							*(((char*)&data2)+1)=modbus_an_buffer[12];
							*(((char*)&data2)+2)=modbus_an_buffer[13];
							*(((char*)&data2)+3)=modbus_an_buffer[14];	
							lc640_write_long(ptr,data1);
							lc640_write_long(ptr+4,data2);
							
							//data1=lc640_read_long(ptr);
							//data2=lc640_read_long(ptr+4);
							temp_out[0]='w';
							temp_out[1]='r';
							temp_out[2]='i';
							temp_out[3]='t';
							temp_out[4]='e';
							temp_out[5]=*((char*)&ptr);
							temp_out[6]=*(((char*)&ptr)+1);	
						
							temp_out[7]=crc_87(temp_out,7);	
							temp_out[8]=crc_95(temp_out,7);			
							
							temp_out[10]=0;
							for (i=0;i<9;i++)
								{
								//*/putchar_sc16is700(temp_out[i]);
								temp_out[10]^=temp_out[i];
								}
							//*/putchar_sc16is700(9);
							//*/putchar_sc16is700(temp_out[10]^9);
							//*/putchar_sc16is700(0x0a);
							}
						}
					}
				}
		   	}
		}
	}

if(crc16_calculated==crc16_incapsulated)
	{
	
	
 	if(modbus_an_buffer[0]==MODBUS_ADRESS)
		{
		//modbus_modbus_adress_eq++;
		if(modbus_func==3)		//чтение произвольного кол-ва регистров хранения
			{
			//modbus_plazma++;
			
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		if(modbus_func==4)		//чтение произвольного кол-ва регистров	входов
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			//modbus_modbus4f_cnt++;
			plazma_uart1[3]++;
			}

		else if(modbus_func==6) 	//запись регистров хранения
			{
			if(modbus_rx_arg0==11)		//Установка времени, год
				{
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR1=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 11 writed   \r\n");
				}
			if(modbus_rx_arg0==12)		//Установка времени, месяц 
				{
				gran(&modbus_rx_arg1,1,12);
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR2=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 12 writed   \r\n");
				}
			if(modbus_rx_arg0==13)		//Установка времени, день 
				{
				short temp_t;
				if(((BKP->DR2)==1)||((BKP->DR2)==3)||((BKP->DR2)==5)||((BKP->DR2)==7)||((BKP->DR2)==8)||((BKP->DR2)==10)||((BKP->DR2)==12))temp_t=31;
				else if(((BKP->DR2)==4)||((BKP->DR2)==6)||((BKP->DR2)==9)||((BKP->DR2)==11))temp_t=30;
				else if(((BKP->DR2)==2)&&((BKP->DR1)%4==0)) temp_t=29;
				else temp_t=28;

				gran(&modbus_rx_arg1,1,temp_t);
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR3=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				//LPC_RTC->DOM=(uint16_t)modbus_rx_arg1;
				printf("Reg 13 writed   \r\n");
				}
			if(modbus_rx_arg0==14)		//Установка времени, час
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,23);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=modbus_rx_arg1;			
				temp_M=((temp_L)%3600)/60;		
				temp_S=((temp_L)%3600)%60;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
  				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 14 writed   \r\n");
				printf("%d",temp_L);
				}
			if(modbus_rx_arg0==15)		//Установка времени, минуты 
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,59);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=((temp_L)/3600);			
				temp_M=modbus_rx_arg1;		
				temp_S=((temp_L)%3600)%60;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
  				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 15 writed   \r\n");
				printf("%d",temp_L);
				}
			if(modbus_rx_arg0==16)		//Установка времени, секунды 
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,59);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=((temp_L)/3600);			
				temp_M=((temp_L)%3600)/60;		
				temp_S=modbus_rx_arg1;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 16 writed   \r\n");
				printf("%d",temp_L);

				}
			if(modbus_rx_arg0==20)		//ток стабилизации для режима стабилизации тока
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=18))
				lc640_write_int(EE_NUMIST,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==21)		//ток стабилизации для режима стабилизации тока
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_PAR,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==22)		//ток стабилизации для режима стабилизации тока
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_ZV_ON,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==23)		//ток стабилизации для режима стабилизации тока
				{
				///*/if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				///*/lc640_write_int(EE_TERMOKOMP,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==24)		//ток стабилизации для режима стабилизации тока
				{
				///*/if(/*(modbus_rx_arg1>=0)||*/(modbus_rx_arg1<=20))
				///*/lc640_write_int(EE_UBM_AV,modbus_rx_arg1);  
				}


			if(modbus_rx_arg0==30)		//напряжение стабилизации для режима стабилизации напряжения
				{
				if((modbus_rx_arg1>0)&&(modbus_rx_arg1<5))modbus_rx_arg1=0;
				else if(modbus_rx_arg1>=60)TBAT=60;
				else TBAT=modbus_rx_arg1;
				lc640_write_int(EE_TBAT,TBAT);

				main_kb_cnt=(TBAT*60)-20;
	     		}
			if(modbus_rx_arg0==31)		//
				{
				 lc640_write_int(EE_UMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==32)		//
				{
				 lc640_write_int(EE_DU,UB20-modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==33)		//
				{
				 lc640_write_int(EE_UB0,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==34)		//
				{
				 lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==35)		//
				{
				 lc640_write_int(EE_USIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==36)		//
				{
				 lc640_write_int(EE_UMN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==37)		//
				{
				 lc640_write_int(EE_U0B,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==38)		//
				{
				 lc640_write_int(EE_IKB,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==39)		//
				{
				 lc640_write_int(EE_IZMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==40)		//
				{
				 lc640_write_int(EE_IMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==41)		//
				{
				 lc640_write_int(EE_IMIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==42)		//
				{
				 lc640_write_int(EE_UVZ,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==43)		//
				{
				 if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=3))lc640_write_int(EE_TZAS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==44)		//
				{
				lc640_write_int(EE_TMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==45)		//
				{
				lc640_write_int(EE_TSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==46)		//
				{
				lc640_write_int(EE_TBATMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==47)		//
				{
				lc640_write_int(EE_TBATSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==48)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_CURR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==49)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_VOLT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==50)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_TIME,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==51)		//
				{
				 lc640_write_int(EE_U_OUT_KONTR_MAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==52)		//
				{
				 lc640_write_int(EE_U_OUT_KONTR_MIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==53)		//
				{
				 if((modbus_rx_arg1>=5)&&(modbus_rx_arg1<=100))lc640_write_int(EE_U_OUT_KONTR_DELAY,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==54)		//
				{
				 lc640_write_int(EE_UB0,modbus_rx_arg1);
				 lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==55)		//Максимальное (аварийное) напряжение питающей сети, 1В
				{
				lc640_write_int(EE_UMAXN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==70)		//
				{
				gran(&modbus_rx_arg1,1,254);
				lc640_write_int(EE_MODBUS_ADRESS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==71)		//
				{
				if(	(modbus_rx_arg1==240)||(modbus_rx_arg1==480)||
					(modbus_rx_arg1==960)||(modbus_rx_arg1==1920)||
					(modbus_rx_arg1==3840)||(modbus_rx_arg1==5670)||
					(modbus_rx_arg1==11520)||(modbus_rx_arg1==23040)
					) lc640_write_int(EE_MODBUS_BAUDRATE,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==72)		//Рег72	 Автоматический ускоренный заряд, вкл(1) выкл(0)
				{
				gran(&modbus_rx_arg1,0,1);
				lc640_write_int(EE_SPEED_CHRG_AVT_EN,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==73)		//Рег73	 Автоматический ускоренный заряд дельта (напряжение просадки включения), 1В
				{
				gran(&modbus_rx_arg1,0,20);
				lc640_write_int(EE_SPEED_CHRG_D_U,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==74)		//Рег74	 Источник блокировки ускоренного заряда, выкл(0) СК1(1) СК2(2)
				{
				gran(&modbus_rx_arg1,0,2);
				lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==75)		//Рег75	 Сигнал блокировки ускоренного заряда, разомкнуто(0) замкнуто(1)
				{
				printf("Reg 75 writed   \r\n");
				gran(&modbus_rx_arg1,0,2);
				lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==76)		//Рег76	 Блокирование ускоренного заряда вентиляцией, вкл(1) выкл(0) 
				{
				printf("Reg 76 writed   \r\n");
				gran(&modbus_rx_arg1,0,1);
				lc640_write_int(EE_SP_CH_VENT_BLOK,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==982)		// Включение-выключение ускоренного заряда
				{
				speedChargeStartStop();
				}

			if(modbus_rx_arg0==994)		//
				{
/*				  RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            // enable clock for Power interface
  RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
  PWR->CR      |= PWR_CR_DBP;                                   // enable access to RTC, BDC registers


 
//  RTC->CRL   &= ~(1<<3);                                        // reset Registers Synchronized Flag
//  while ((RTC->CRL & (1<<3)) == 0);                             // wait until registers are synchronized

  RTC->CRL  |=  RTC_CRL_CNF;                                    // set configuration mode
  RTC->PRLH  = ((__RTC_PERIOD*__RTCCLK/1000-1)>>16) & 0x00FF;   // set prescaler load register high
  RTC->PRLL  = ((__RTC_PERIOD*__RTCCLK/1000-1)    ) & 0xFFFF;   // set prescaler load register low
//RTC->PRLL  = 5;
	//BKP->DR5=5;
  RTC->CNTH  = ((__RTC_CNT)>>16) & 0xFFFF;                      // set counter high
  RTC->CNTL  = ((__RTC_CNT)    ) & 0xFFFF;                      // set counter low
  RTC->ALRH  = ((__RTC_ALR)>>16) & 0xFFFF;                      // set alarm high
  RTC->ALRL  = ((__RTC_ALR)    ) & 0xFFFF;                      // set alarm low
  if (__RTC_INTERRUPTS) {                                       // RTC interrupts used
    RTC->CRH = __RTC_CRH;                                       // enable RTC interrupts
    NVIC->ISER[0]  = (1 << (RTC_IRQChannel & 0x1F));            // enable interrupt
  }
  RTC->CRL  &= ~RTC_CRL_CNF;                                    // reset configuration mode
  while ((RTC->CRL & RTC_CRL_RTOFF) == 0);                      // wait until write is finished

  PWR->CR   &= ~PWR_CR_DBP; */    
	     		}
			if(modbus_rx_arg0==995)		//
				{
				modbus_register_995=modbus_rx_arg1;
	     		}

			if(modbus_rx_arg0==996)		//
				{
				if(modbus_rx_arg1==1)
					{
					printf("Zero current for IPS#%d executed   \r\n", modbus_register_998);
					can1_out(modbus_register_998-1,modbus_register_998-1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
					//modbus_register_998=modbus_rx_arg1;
					//if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}


			if(modbus_rx_arg0==997)		//
				{
				if(modbus_rx_arg1==1)
					{
					can1_out(modbus_register_998-1,modbus_register_998-1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
					//modbus_register_998=modbus_rx_arg1;
					//if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}

			if(modbus_rx_arg0==998)		//Запись номера калибруемого источника 
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=NUMIST))
					{
					modbus_register_998=modbus_rx_arg1;
					if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}

			if(modbus_rx_arg0==999)		//
				{
				//if(modbus_rx_arg1!=0)
					{
					modbus_register_999=modbus_rx_arg1;
					gran(&modbus_register_999,0,1023);
					}
	     		}

			if(modbus_rx_arg0==1000)		//
				{
				//if(modbus_rx_arg1==1)
					{
					modbus_register_1000=modbus_rx_arg1;
					}
	     		}
			if(modbus_rx_arg0==1001)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=500L;
				tempL/=(signed long)adc_buff_[5];
				Kuout=(signed short)tempL;
				lc640_write_int(EE_KUOUT,Kuout);

	     		}
			if(modbus_rx_arg0==1002)		//
				{
					{
					modbus_register_1002=modbus_rx_arg1;
					}

	     		}
			if(modbus_rx_arg0==1003)		//
				{

	     		}
			if(modbus_rx_arg0==1005)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[0];
				KunetA=(signed short)tempL;
				lc640_write_int(EE_KUNETA,KunetA);
	     		}

			if(modbus_rx_arg0==1006)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1006 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[1];
				KunetB=(signed short)tempL;
				lc640_write_int(EE_KUNETB,KunetB);
	     		}

			if(modbus_rx_arg0==1007)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1007 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[2];
				KunetC=(signed short)tempL;
				lc640_write_int(EE_KUNETC,KunetC);
	     		}
 			if(modbus_rx_arg0==1008)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1008 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[2];
				KunetC=(signed short)tempL;
				lc640_write_int(EE_KUNETC,KunetC);
	     		}
			if(modbus_rx_arg0==1009)		//
				{
				printf("Reg 1009 writed   \r\n");
					{
					short tempS;
					signed long tempL,tempL1;
				/*	tempL=(long)modbus_rx_arg1;
					tempL*=1000L;
					Kuout=(short)(tempL/(long)adc_buff_virt_0);	*/

					tempL=(signed long)modbus_rx_arg1;
					//if(modbus_rx_arg1<0)tempL=(signed long)(-modbus_rx_arg1);
					tempL*=2000L;
					//tempL/=((signed long)ibat_metr_buff_[0]-(signed long)ibat_metr_buff_[1]);
					tempL1=(signed long)(ibat_metr_buff_[0]-ibat_metr_buff_[1]);
					//tempL1=-500L;
					tempL/=tempL1;
					
					Kibat1[0]=(signed short)tempL;
					//if(modbus_rx_arg1<0)Kibat1[0]=(signed short)(-tempL);
					lc640_write_int(EE_KI1BAT1,Kibat1[0]);
					}
	     		}

			if(modbus_rx_arg0==1010)		//
				{
				printf("Reg 1010 writed   \r\n");
					{
					short tempS;
					long tempL;
				/*	tempL=(long)modbus_rx_arg1;
					tempL*=1000L;
					Kuout=(short)(tempL/(long)adc_buff_virt_0);	*/

					tempL=(long)modbus_rx_arg1;
					tempL+=273;
					tempL*=20000L;
					tempL/=(signed long)adc_buff_[3];
					Ktbat[0]=(signed short)tempL;
					lc640_write_int(EE_KTBAT1,Ktbat[0]);
					}
	     		}

			if(modbus_rx_arg0==1011)		//
				{
				printf("Reg 1011 writed   \r\n");

	     		}
						
			if(modbus_rx_arg0==1022)		//
				{
				printf("Reg 1022 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_ui=1;
					avt_klbr_num_ui=modbus_register_998;
					avt_klbr_phase_ui=1;
					avt_klbr_necc_value_ui=modbus_rx_arg1;
					avt_klbr_err_cnt_ui=300;
					modbus_register_1022=modbus_rx_arg1;
					modbus_register_offset_ui=22;
					avt_klbr_start_ui();
					}
				else
					{
					avt_klbr_mode_ui=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1022=0;
					} 
	     		}
			if(modbus_rx_arg0==1023)		//
				{
				printf("Reg 1023 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_un=1;
					avt_klbr_num_un=modbus_register_998;
					avt_klbr_phase_un=1;
					avt_klbr_necc_value_un=modbus_rx_arg1;
					avt_klbr_err_cnt_un=300;
					modbus_register_1023=modbus_rx_arg1;
					modbus_register_offset_un=23;
					avt_klbr_start_un();
					}
				else
					{
					avt_klbr_mode_un=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1023=0;
					} 
	     		}
			if(modbus_rx_arg0==1024)		//
				{
				printf("Reg 1024 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_i=1;
					avt_klbr_num_i=modbus_register_998;
					avt_klbr_phase_i=1;
					avt_klbr_necc_value_i=modbus_rx_arg1;
					avt_klbr_err_cnt_i=300;
					modbus_register_1024=modbus_rx_arg1;
					modbus_register_offset_i=24;
					avt_klbr_start_i();
					}
				else
					{
					avt_klbr_mode_i=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1024=0;
					} 
	     		}
			if(modbus_rx_arg0==1025)		//
				{
				printf("Reg 1025 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_t=1;
					avt_klbr_num_t=modbus_register_998;
					avt_klbr_phase_t=1;
					avt_klbr_necc_value_t=modbus_rx_arg1;
					avt_klbr_err_cnt_t=300;
					modbus_register_1025=modbus_rx_arg1;
					modbus_register_offset_t=25;
					avt_klbr_start_t();
					}
				else
					{
					avt_klbr_mode_t=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1025=0;
					} 
	     		}
			if(modbus_rx_arg0==1027)		//
				{
				printf("Reg 1027 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode=akmUI;
					avt_klbr_num=2;
					avt_klbr_phase=1;
					avt_klbr_necc_value=modbus_rx_arg1;
					avt_klbr_err_cnt=300;
					modbus_register_1027=modbus_rx_arg1;
					modbus_register_offset=27;
					}
				else
					{
					avt_klbr_mode=akmOFF;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1027=0;
					} 
	     		}

			if(modbus_rx_arg0==1070)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=500L;
				tempL/=(signed long)adc_buff_[4];
				Kubps=(signed short)tempL;
				lc640_write_int(EE_KUBPS,Kubps);

	     		}
			if(modbus_rx_arg0==19)		//вкл/выкл источника напр.
				{
	/*			if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsPS)
						{
						work_stat=wsPS;
						time_proc=0;
						time_proc_remain=T_PROC_PS;
						restart_on_PS();
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIN);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsPS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					} */
				}
			if(modbus_rx_arg0==20)		//вкл/выкл источника тока
				{
/*				if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsGS)
						{
						work_stat=wsGS;
						time_proc=0;
						time_proc_remain=T_PROC_GS;
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIT);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsGS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					}*/
				}
			//modbus_hold_register_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0);

			if(modbus_rx_arg0==100)		//Передача шима для управления от ведущего ИПС
				{
				//plazma1000[2]=modbus_rx_arg1;
				if(modbus_rx_arg1&0x4000)
					{
					short tempSSSS;
					
					tempSSSS=modbus_rx_arg1&0x3fff;
					//plazma1000[3]=tempSSSS;
					if((tempSSSS>0)&&(tempSSSS<5))tempSSSS=0;
					else if(tempSSSS>=60)tempSSSS=60;
				//	else tempSSSS=modbus_rx_arg1;
				///*/	if(TBAT!=tempSSSS)lc640_write_int(EE_TBAT,tempSSSS);

				///*/	main_kb_cnt=(tempSSSS*60)-20;
					}
				///*/else ica_cntrl_hndl=modbus_rx_arg1;
				///*/ica_cntrl_hndl_cnt=200;

				//plazma1000[1]++;
				}
			if(modbus_rx_arg0==101)		//Значение тока в ведомом ИПСе (прочитанное мастером в ведомом и переданное ведущему)
				{
				///*/ica_your_current==modbus_rx_arg1;
				///*/ica_cntrl_hndl_cnt=200;
				//plazma1000[2]++;
				}
			if(modbus_rx_arg0==300)		//Калибровка измеряемых аналоговых величин
				{
				kalibrate_func(modbus_rx_arg1);
				}

			printf("Register %d writed   \r\n",modbus_rx_arg0);

			memcpy(modbus_tx_buff,modbus_rx_buffer,8);
	
			for (i=0;i<(8);i++)
				{
				putchar1(modbus_tx_buff[i]);
				}
			}
		} 
	else if(modbus_an_buffer[0]==ICA_MODBUS_ADDRESS)
		{
//		ica_plazma[3]++;
		if(modbus_func==4)		//чтение произвольного кол-ва регистров	входов
			{
//			ica_plazma[2]++;
			if(modbus_an_buffer[2]==2)
				{
				///*/ica_your_current=(((unsigned short)modbus_an_buffer[3])*((unsigned short)256))+((unsigned short)modbus_an_buffer[4]);
				}
			}
		}
	
	}


}
/*
//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[50];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//Рег1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//Рег2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//Рег3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//Рег4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//Рег5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//Рег6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//Рег7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//Рег8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//Рег9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//Рег10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//Рег11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//Рег12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//Рег13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//Рег14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//Рег15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//Рег16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//Рег17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//Рег18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//Рег19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//Рег20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//Рег21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//Рег22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//Рег23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//Рег24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//Рег25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//Рег26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//Рег27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//Рег28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//Рег29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//Рег30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//Рег31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//Рег32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//Рег33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//Рег34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//Рег35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//Рег36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//Рег37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//Рег38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//Рег39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//Рег40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//Рег41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//Рег42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//Рег43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//Рег44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//Рег45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//Рег46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//Рег47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//Рег48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//Рег49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//Рег50
modbus_registers[99]=(char)(load_I%256);

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*//-----------------------------------------------
void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity)
{
char modbus_registers[100];
//char modbus_tx_buff[100];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//Рег1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//Рег2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//Рег3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//Рег4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//Рег5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//Рег6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//Рег7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//Рег8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//Рег9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//Рег10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//Рег11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//Рег12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//Рег13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//Рег14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//Рег15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//Рег16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//Рег17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//Рег18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//Рег19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//Рег20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//Рег21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//Рег22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//Рег23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//Рег24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//Рег25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//Рег26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//Рег27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//Рег28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//Рег29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//Рег30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//Рег31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//Рег32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//Рег33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//Рег34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//Рег35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//Рег36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//Рег37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//Рег38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//Рег39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//Рег40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//Рег41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//Рег42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//Рег43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//Рег44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//Рег45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//Рег46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//Рег47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//Рег48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//Рег49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//Рег50
modbus_registers[99]=(char)(load_I%256);




modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
//modbus_tx_buff[2]=(char)(reg_adr/256);
//modbus_tx_buff[3]=(char)(reg_adr%256);
modbus_tx_buff[2]=(char)(reg_quantity*2);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

//int2lcdyx(reg_quantity,0,10,0);

for (i=0;i<15;i++)
	{
	putchar0(modbus_tx_buff[i]);
	} 
for (i=0;i<15;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*
//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//Рег11  Время, год
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//Рег12  Время, месяц
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//Рег13  Время, день месяца
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//Рег14  Время, час
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//Рег15  Время, минуты
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//Рег16  Время, секунды
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[38]=(char)(NUMIST/256);				//Рег20  Количество выпрямителей в структуре
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);					//Рег21  Параллельная работа выпрямителей вкл./выкл.
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);					//Рег22  Звуковая аварийная сигнализация вкл./выкл.
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[46]=(char)(UBM_AV/256);				//Рег24  Аварийный уровень отклонения напряжения средней точки батареи, %
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[58]=(char)(TBAT/256);					//Рег30  Период проверки цепи батареи, минут.
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);					//Рег31  Максимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);				//Рег32  Минимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);					//Рег33  Напряжение содержания батареи при 0гЦ, 0.1В
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);					//Рег34  Напряжение содержания батареи при 20гЦ, 0.1В
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);					//Рег35  Минимальное (сигнальное) напряжение батареи, 1В
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);					//Рег36  Минимальное (аварийное) напряжение питающей сети, 1В
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);					//Рег37  Рабочее напряжение при невведенных батареях, 0.1В
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);					//Рег38  Ток контроля наличия батареи, 0.1а
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);					//Рег39  Ток заряда батареи максимальный, 0.1А
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);					//Рег40  Ток переключения на большее кол-во выпрямителей, 0.1А
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);					//Рег41  Ток переключения на меньшее кол-во выпрямителей, 0.1А
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);					//Рег42  Напряжение выравнивающего заряда, 0.1В
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);					//Рег43  Время задержки включения выпрямителей, сек
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);					//Рег44  Температура выпрямителей аварийная, 1гЦ
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);					//Рег45  Температура выпрямителей сигнальная, 1гЦ
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);				//Рег46  Температура батареи аварийная, 1гЦ
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);				//Рег47  Температура батареи сигнальная, 1гЦ
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(speedChrgCurr/256);					//Рег48  Ток ускоренного заряда, 0.1А
modbus_registers[95]=(char)(speedChrgCurr%256);
modbus_registers[96]=(char)(speedChrgVolt/256);				//Рег49	 Напряжение ускоренного заряда, 0.1В 
modbus_registers[97]=(char)(speedChrgVolt%256);
modbus_registers[98]=(char)(speedChrgTimeInHour/256);				//Рег50	 Время ускоренного заряда, 1ч
modbus_registers[99]=(char)(speedChrgTimeInHour%256);


modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);

mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}
*/

//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[2500];
signed short tempSS;
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;
long temp_time;

modbus_registers[20]=(char)((BKP->DR1)>>8);					//Рег11  Время, год
modbus_registers[21]=(char)((BKP->DR1));
modbus_registers[22]=(char)((BKP->DR2)>>8);					//Рег12  Время, месяц
modbus_registers[23]=(char)((BKP->DR2));
modbus_registers[24]=(char)((BKP->DR3)>>8);					//Рег13  Время, день месяца
modbus_registers[25]=(char)((BKP->DR3));
temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
modbus_registers[26]=(char)((temp_time)/3600)>>8;			//Рег14  Время, час
modbus_registers[27]=(char)((temp_time)/3600);
modbus_registers[28]=(char)(((temp_time)%3600)/60)>>8;		//Рег15  Время, минуты
modbus_registers[29]=(char)(((temp_time)%3600)/60);
modbus_registers[30]=(char)(((temp_time)%3600)%60)>>8;		//Рег16  Время, секунды
modbus_registers[31]=(char)(((temp_time)%3600)%60);
modbus_registers[38]=(char)(NUMIST>>8);				//Рег20  Количество выпрямителей в структуре
modbus_registers[39]=(char)(NUMIST);
///*/

//PAR=123;

modbus_registers[40]=(char)(PAR>>8);					//Рег21  Параллельная работа выпрямителей вкл./выкл.
modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//Рег22  Звуковая аварийная сигнализация вкл./выкл.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);					//Рег24  Аварийный уровень отклонения напряжения средней точки батареи, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//Рег30  Период проверки цепи батареи, минут.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//Рег31  Максимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[61]=(char)(UMAX);
modbus_registers[62]=(char)((UB20-DU)>>8);				//Рег32  Минимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[63]=(char)((UB20-DU));
modbus_registers[64]=(char)(UB0>>8);					//Рег33  Напряжение содержания батареи при 0гЦ, 0.1В
modbus_registers[65]=(char)(UB0);
modbus_registers[66]=(char)(UB20>>8);					//Рег34  Напряжение содержания батареи при 20гЦ, 0.1В
modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//Рег35  Минимальное (сигнальное) напряжение батареи, 1В
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//Рег36  Минимальное (аварийное) напряжение питающей сети, 1В
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//Рег37  Рабочее напряжение при невведенных батареях, 0.1В
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//Рег38  Ток контроля наличия батареи, 0.1а
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//Рег39  Ток заряда батареи максимальный, 0.1А
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//Рег40  Ток переключения на большее кол-во выпрямителей, 0.1А
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//Рег41  Ток переключения на меньшее кол-во выпрямителей, 0.1А
modbus_registers[81]=(char)(IMIN);
modbus_registers[82]=(char)(UVZ>>8);					//Рег42  Напряжение выравнивающего заряда, 0.1В
modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//Рег43  Время задержки включения выпрямителей, сек
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//Рег44  Температура выпрямителей аварийная, 1гЦ
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//Рег45  Температура выпрямителей сигнальная, 1гЦ
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//Рег46  Температура батареи аварийная, 1гЦ
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//Рег47  Температура батареи сигнальная, 1гЦ
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);			//Рег48  Ток ускоренного заряда, 0.1А
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);			//Рег49	 Напряжение ускоренного заряда, 0.1В 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);	//Рег50	 Время ускоренного заряда, 1ч
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);		//Рег51	 Контроль выходного напряжения, Umax, 0.1В
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);		//Рег52	 Контроль выходного напряжения, Umin, 0.1В
modbus_registers[103]=(char)(U_OUT_KONTR_MIN);
modbus_registers[104]=(char)(U_OUT_KONTR_DELAY>>8);		//Рег53	 Контроль выходного напряжения, Tзадержки, 1сек.
modbus_registers[105]=(char)(U_OUT_KONTR_DELAY);
modbus_registers[106]=(char)(UB0>>8);					//Рег54	 Установка выходного напряжения для ИПС без батареи(СГЕП-ГАЗПРОМ)
modbus_registers[107]=(char)(UB0);
modbus_registers[108]=(char)(UMAXN>>8);					//Рег55  Максимальное (аварийное) напряжение питающей сети, 1В
modbus_registers[109]=(char)(UMAXN);
modbus_registers[138]=(char)(MODBUS_ADRESS>>8);			//Рег70	 MODBUS ADRESS
modbus_registers[139]=(char)(MODBUS_ADRESS);
modbus_registers[140]=(char)(MODBUS_BAUDRATE>>8);		//Рег71	 MODBUS BAUDRATE
modbus_registers[141]=(char)(MODBUS_BAUDRATE);
modbus_registers[142]=(char)(speedChrgAvtEn>>8);		//Рег72	 Автоматический ускоренный заряд, вкл(1) выкл(0)
modbus_registers[143]=(char)(speedChrgAvtEn);
modbus_registers[144]=(char)(speedChrgDU>>8);			//Рег73	 Автоматический ускоренный заряд дельта (напряжение просадки включения), 1В
modbus_registers[145]=(char)(speedChrgDU);
modbus_registers[146]=(char)(speedChrgBlckSrc>>8);		//Рег74	 Источник блокировки ускоренного заряда, выкл(0) СК1(1) СК2(2)
modbus_registers[147]=(char)(speedChrgBlckSrc);
modbus_registers[148]=(char)(speedChrgBlckLog>>8);		//Рег75	 Сигнал блокировки ускоренного заряда, разомкнуто(0) замкнуто(1) 
modbus_registers[149]=(char)(speedChrgBlckLog);
modbus_registers[150]=(char)(SP_CH_VENT_BLOK>>8);		//Рег76	 Блокирование ускоренного заряда вентиляцией, вкл(1) выкл(0) 
modbus_registers[151]=(char)(SP_CH_VENT_BLOK);
/*

	ptrs[0]=	" Iуск.зар.        !А";
	ptrs[1]=	" Uуск.зар.        @В";
	ptrs[2]=	" Tуск.зар.        #ч";
	ptrs[3]=	" Автоматический     ";
	ptrs[4]=	" ускор.заряд.      $";
	ptrs[5]=	" dUуск.зар.       %В";
	ptrs[6]=	" Блокирование      ^";
	ptrs[7]=	" Сигнал блокирования";
	ptrs[8]=	"                   &";
	ptrs[9]=	" Блокирование       ";
	ptrs[10]=	" вентиляцией       (";
	ptrs[11]=	" Выход              ";

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	"  УСКОРЕННЫЙ ЗАРЯД  ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(speedChrgCurr,'!',1);
	int2lcd(speedChrgVolt,'@',1);
	int2lcd(speedChrgTimeInHour,'#',0);
	if(speedChrgAvtEn==1)sub_bgnd("ВКЛ.",'$',-3);   
    else sub_bgnd("ВЫКЛ.",'$',-4); 
	int2lcd(speedChrgDU,'%',0);
	if(speedChrgBlckSrc==1)sub_bgnd("СК1",'^',-2);
	else if(speedChrgBlckSrc==2)sub_bgnd("СК2",'^',-2);   
    else if(speedChrgBlckSrc==0)sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("НЕКОРР..",'^',-6); 
	if(speedChrgBlckLog==0)sub_bgnd("РАЗОМКН.",'&',-7);
	else if(speedChrgBlckLog==1) sub_bgnd("ЗАМКН.",'&',-5);  
    else sub_bgnd("НЕКОРР.",'&',-6); 	  
	if(SP_CH_VENT_BLOK==1)sub_bgnd("ВКЛ",'(',-2);
    else if(SP_CH_VENT_BLOK==0)sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("НЕКОРР..",'(',-6);*/

//modbus_register_1000 = 2;
//modbus_register_1001 = 7890;
//modbus_register_1002 = 8765;hv_vz_up_cnt
tempSS=0;
if((sp_ch_stat==scsSTEP1)||(sp_ch_stat==scsWRK))tempSS=1;
modbus_registers[1962]=(char)(tempSS>>8);										//Рег982	 Включенность ускоренного заряда вкл(1) выкл(0) 
modbus_registers[1963]=(char)(tempSS);

tempSS=(short)(hv_vz_up_cnt/60L);
modbus_registers[1964]=(char)(tempSS>>8);										//Рег983	 Время работы ускоренного заряда в минутах  
modbus_registers[1965]=(char)(tempSS);


modbus_registers[1994]=(char)(modbus_register_998>>8);							//Рег998	 Выбор калибруемого источника
modbus_registers[1995]=(char)(modbus_register_998);
modbus_registers[1996]=(char)(modbus_register_999>>8);							//Рег999	 Установка выходного ШИМа при калибровке
modbus_registers[1997]=(char)(modbus_register_999);
modbus_registers[1998]=(char)(modbus_register_1000>>8);							//Рег54	 Установка выходного напряжения для ИПС без батареи(СГЕП-ГАЗПРОМ)
modbus_registers[1999]=(char)(modbus_register_1000);
modbus_registers[2000]=(char)(modbus_register_1001>>8);							//Рег54	 Установка выходного напряжения для ИПС без батареи(СГЕП-ГАЗПРОМ)
modbus_registers[2001]=(char)(modbus_register_1001);
modbus_registers[2002]=(char)(modbus_register_1002>>8);							//Рег54	 Установка выходного напряжения для ИПС без батареи(СГЕП-ГАЗПРОМ)
modbus_registers[2003]=(char)(modbus_register_1002);
modbus_registers[2004]=(char)(modbus_register_1003>>8);							//Рег54	 Установка выходного напряжения для ИПС без батареи(СГЕП-ГАЗПРОМ)
modbus_registers[2005]=(char)(modbus_register_1003);

modbus_registers[2042]=(char)(modbus_register_1022>>8);							//Рег1022	 Калибровка Uист. БПС
modbus_registers[2043]=(char)(modbus_register_1022);
modbus_registers[2044]=(char)(modbus_register_1023>>8);							//Рег1023	 Калибровка Uист.шин. БПС
modbus_registers[2045]=(char)(modbus_register_1023);
modbus_registers[2046]=(char)(modbus_register_1024>>8);							//Рег1024	 Калибровка Iист. БПС
modbus_registers[2047]=(char)(modbus_register_1024);
modbus_registers[2048]=(char)(modbus_register_1025>>8);							//Рег1025	 Калибровка Tрад.ист. БПС
modbus_registers[2049]=(char)(modbus_register_1025);


if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	memcpy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;



	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	//*/mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
///*/	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}

//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[2200];
//char modbus_tx_buff[200];
unsigned short crc_temp;
char i;
short tempS;
/*
plazma_uart1[2]++;

out_U=plazma_short;
bps_I=5678;

net_U=223;
bps[0]._Uii=281;
bps[0]._Ii=123;


bps_I=5678;
*/
//out_U=1234;
//bps_U=3456;

modbus_registers[0]=(signed char)(out_U>>8);					//Рег1   	напряжение выходной шины, 0.1В
modbus_registers[1]=(signed char)(out_U);
modbus_registers[2]=(signed char)(bps_I>>8);					//Рег2   	ток выпрямителей, 0.1А
modbus_registers[3]=(signed char)(bps_I);



modbus_registers[4]=(signed char)(net_U>>8);					//Рег3   	напряжение сети питания, 1В
modbus_registers[5]=(signed char)(net_U);
modbus_registers[6]=(signed char)(net_F>>8);					//Рег4   	частота сети питания, 0.1Гц
modbus_registers[7]=(signed char)(net_F);
modbus_registers[8]=(signed char)(net_Ua>>8);					//Рег5	напряжение сети питания фаза A, 1В	
modbus_registers[9]=(signed char)(net_Ua);		 	
modbus_registers[10]=(signed char)(net_Ub>>8);				//Рег6	напряжение сети питания фаза B, 1В
modbus_registers[11]=(signed char)(net_Ub);
modbus_registers[12]=(signed char)(net_Uc>>8);				//Рег7	напряжение сети питания фаза C, 1В
modbus_registers[13]=(signed char)(net_Uc);
modbus_registers[14]=(signed char)(bat[0]._Ub>>8);				//Рег8	напряжение батареи №1, 0.1В
modbus_registers[15]=(signed char)(bat[0]._Ub);
modbus_registers[16]=(signed char)(Ib_ips_termokompensat/*bat[0]._Ib*/>>8);				//Рег9   	ток батареи №1, 0.01А
modbus_registers[17]=(signed char)(Ib_ips_termokompensat/*bat[0]._Ib*/);

t_ext[0]=bat[0]._Tb;
modbus_registers[18]=(signed char)(t_ext[0]>>8);				//Рег10	температура батареи №1, 1Гц
modbus_registers[19]=(signed char)(t_ext[0]);
/*
modbus_registers[18]=(signed char)(bat[0]._Tb>>8);				//Рег10	температура батареи №1, 1Гц
modbus_registers[19]=(signed char)(bat[0]._Tb);
#endif*/
#ifdef UKU_ZVU
modbus_registers[20]=(signed char)(((short)(bat_hndl_zvu_Q/10000L))>>8);			//Рег11	заряд батареи №1, %
modbus_registers[21]=(signed char)(((short)(bat_hndl_zvu_Q/10000L)));
#else
modbus_registers[20]=(signed char)(bat[0]._zar>>8);			//Рег11	заряд батареи №1, %
modbus_registers[21]=(signed char)(bat[0]._zar);
#endif
modbus_registers[22]=(signed char)(bat[0]._Ubm>>8);			//Рег12	напряжение средней точки батареи №1, 0.1В
modbus_registers[23]=(signed char)(bat[0]._Ubm);
modbus_registers[24]=(signed char)(bat[0]._dUbm>>8);			//Рег13	ошибка средней точки батареи №1, %
modbus_registers[25]=(signed char)(bat[0]._dUbm);
modbus_registers[26]=(signed char)(BAT_C_REAL[0]>>8);			//Рег14	Реальная емкость батареи №1, 0.1А*ч, если 0x5555 то не измерялась
modbus_registers[27]=(signed char)(BAT_C_REAL[0]);
modbus_registers[28]=(signed char)(bat[1]._Ub>>8);				//Рег15	напряжение батареи №1, 0.1В
modbus_registers[29]=(signed char)(bat[1]._Ub);
modbus_registers[30]=(signed char)(bat[1]._Ib>>8);				//Рег16   	ток батареи №1, 0.01А
modbus_registers[31]=(signed char)(bat[1]._Ib);
modbus_registers[32]=(signed char)(bat[1]._Tb>>8);				//Рег17	температура батареи №1, 1Гц
modbus_registers[33]=(signed char)(bat[1]._Tb);
modbus_registers[34]=(signed char)(bat[1]._zar>>8);				//Рег18	заряд батареи №1, %
modbus_registers[35]=(signed char)(bat[1]._zar);
modbus_registers[36]=(signed char)(bat[1]._Ubm>>8);				//Рег19	напряжение средней точки батареи №1, 0.1В
modbus_registers[37]=(signed char)(bat[1]._Ubm);
modbus_registers[38]=(signed char)(bat[1]._dUbm>>8);			//Рег20	ошибка средней точки батареи №1, %
modbus_registers[39]=(signed char)(bat[1]._dUbm);
modbus_registers[40]=(signed char)(BAT_C_REAL[1]>>8);			//Рег21	Реальная емкость батареи №1, 0.1А*ч, если 0x5555 то не измерялась
modbus_registers[41]=(signed char)(BAT_C_REAL[1]);
modbus_registers[42]=(signed char)(bps[0]._Uii>>8);				//Рег22	Выходное напряжение выпрямителя №1, 0.1В
modbus_registers[43]=(signed char)(bps[0]._Uii);
modbus_registers[44]=(signed char)(bps[0]._Uin>>8);				//Рег23	Напряжение шины выпрямителя №1, 0.1В
modbus_registers[45]=(signed char)(bps[0]._Uin);
modbus_registers[46]=(signed char)(bps[0]._Ii>>8);				//Рег24	Выходной ток выпрямителя №1, 0.1А
modbus_registers[47]=(signed char)(bps[0]._Ii);
modbus_registers[48]=(signed char)(bps[0]._Ti>>8);				//Рег25	Температура радиатора выпрямителя №1, 1гЦ
modbus_registers[49]=(signed char)(bps[0]._Ti);
modbus_registers[50]=(signed char)(bps[0]._av>>8);				//Рег26	Байт флагов выпрямителя №1, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[51]=(signed char)(bps[0]._av);
modbus_registers[52]=(signed char)(bps[1]._Uii>>8);				//Рег27	Выходное напряжение выпрямителя №2, 0.1В
modbus_registers[53]=(signed char)(bps[1]._Uii);
modbus_registers[54]=(signed char)(bps[1]._Uin>>8);				//Рег28	Напряжение шины выпрямителя №2, 0.1В
modbus_registers[55]=(signed char)(bps[1]._Uin);
modbus_registers[56]=(signed char)(bps[1]._Ii>>8);				//Рег29	Выходной ток выпрямителя №2, 0.1А
modbus_registers[57]=(signed char)(bps[1]._Ii);
modbus_registers[58]=(signed char)(bps[1]._Ti>>8);				//Рег30	Температура радиатора выпрямителя №2, 1гЦ
modbus_registers[59]=(signed char)(bps[1]._Ti);
modbus_registers[60]=(signed char)(bps[1]._av>>8);				//Рег31	Байт флагов выпрямителя №2, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[61]=(signed char)(bps[1]._av);
modbus_registers[62]=(signed char)(bps[2]._Uii>>8);				//Рег32	Выходное напряжение выпрямителя №3, 0.1В
modbus_registers[63]=(signed char)(bps[2]._Uii);
modbus_registers[64]=(signed char)(bps[2]._Uin>>8);				//Рег33	Напряжение шины выпрямителя №3, 0.1В
modbus_registers[65]=(signed char)(bps[2]._Uin);
modbus_registers[66]=(signed char)(bps[2]._Ii>>8);				//Рег34	Выходной ток выпрямителя №3, 0.1А
modbus_registers[67]=(signed char)(bps[2]._Ii);
modbus_registers[68]=(signed char)(bps[2]._Ti>>8);				//Рег35	Температура радиатора выпрямителя №3, 1гЦ
modbus_registers[69]=(signed char)(bps[2]._Ti);
modbus_registers[70]=(signed char)(bps[2]._av>>8);				//Рег36	Байт флагов выпрямителя №3, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[71]=(signed char)(bps[2]._av);
modbus_registers[72]=(signed char)(bps[3]._Uii>>8);				//Рег37	Выходное напряжение выпрямителя №4, 0.1В
modbus_registers[73]=(signed char)(bps[3]._Uii);
modbus_registers[74]=(signed char)(bps[3]._Uin>>8);				//Рег38	Напряжение шины выпрямителя №4, 0.1В
modbus_registers[75]=(signed char)(bps[3]._Uin);
modbus_registers[76]=(signed char)(bps[3]._Ii>>8);				//Рег39	Выходной ток выпрямителя №4, 0.1А
modbus_registers[77]=(signed char)(bps[3]._Ii);
modbus_registers[78]=(signed char)(bps[3]._Ti>>8);				//Рег40	Температура радиатора выпрямителя №4, 1гЦ
modbus_registers[79]=(signed char)(bps[3]._Ti);
modbus_registers[80]=(signed char)(bps[3]._av>>8);				//Рег41	Байт флагов выпрямителя №4, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[81]=(signed char)(bps[3]._av);
modbus_registers[82]=(signed char)(bps[4]._Uii>>8);				//Рег42	Выходное напряжение выпрямителя №5, 0.1В
modbus_registers[83]=(signed char)(bps[4]._Uii);
modbus_registers[84]=(signed char)(bps[4]._Uin>>8);				//Рег43	Напряжение шины выпрямителя №5, 0.1В
modbus_registers[85]=(signed char)(bps[4]._Uin);
modbus_registers[86]=(signed char)(bps[4]._Ii>>8);				//Рег44	Выходной ток выпрямителя №5, 0.1А
modbus_registers[87]=(signed char)(bps[4]._Ii);
modbus_registers[88]=(signed char)(bps[4]._Ti>>8);				//Рег45	Температура радиатора выпрямителя №5, 1гЦ
modbus_registers[89]=(signed char)(bps[4]._Ti);
modbus_registers[90]=(signed char)(bps[4]._av>>8);				//Рег46	Байт флагов выпрямителя №5, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[91]=(signed char)(bps[4]._av);
modbus_registers[92]=(signed char)(bps[5]._Uii>>8);				//Рег47	Выходное напряжение выпрямителя №6, 0.1В
modbus_registers[93]=(signed char)(bps[5]._Uii);
modbus_registers[94]=(signed char)(bps[5]._Uin>>8);				//Рег48	Напряжение шины выпрямителя №6, 0.1В
modbus_registers[95]=(signed char)(bps[5]._Uin);
modbus_registers[96]=(signed char)(bps[5]._Ii>>8);				//Рег49	Выходной ток выпрямителя №6, 0.1А
modbus_registers[97]=(signed char)(bps[5]._Ii);
modbus_registers[98]=(signed char)(bps[5]._Ti>>8);				//Рег50	Температура радиатора выпрямителя №6, 1гЦ
modbus_registers[99]=(signed char)(bps[5]._Ti);
modbus_registers[100]=(signed char)(bps[5]._av>>8);				//Рег51	Байт флагов выпрямителя №6, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[101]=(signed char)(bps[5]._av);
modbus_registers[102]=(signed char)(bps[6]._Uii>>8);			//Рег52	Выходное напряжение выпрямителя №7, 0.1В
modbus_registers[103]=(signed char)(bps[6]._Uii);
modbus_registers[104]=(signed char)(bps[6]._Uin>>8);			//Рег53	Напряжение шины выпрямителя №7, 0.1В
modbus_registers[105]=(signed char)(bps[6]._Uin);
modbus_registers[106]=(signed char)(bps[6]._Ii>>8);				//Рег54	Выходной ток выпрямителя №7, 0.1А
modbus_registers[107]=(signed char)(bps[6]._Ii);
modbus_registers[108]=(signed char)(bps[6]._Ti>>8);				//Рег55	Температура радиатора выпрямителя №7, 1гЦ
modbus_registers[109]=(signed char)(bps[6]._Ti);
modbus_registers[110]=(signed char)(bps[6]._av>>8);				//Рег56	Байт флагов выпрямителя №7, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[111]=(signed char)(bps[6]._av);
modbus_registers[112]=(signed char)(bps[7]._Uii>>8);			//Рег57	Выходное напряжение выпрямителя №8, 0.1В
modbus_registers[113]=(signed char)(bps[7]._Uii);
modbus_registers[114]=(signed char)(bps[7]._Uin>>8);			//Рег58	Напряжение шины выпрямителя №8, 0.1В
modbus_registers[115]=(signed char)(bps[7]._Uin);
modbus_registers[116]=(signed char)(bps[7]._Ii>>8);				//Рег59	Выходной ток выпрямителя №8, 0.1А
modbus_registers[117]=(signed char)(bps[7]._Ii);
modbus_registers[118]=(signed char)(bps[7]._Ti>>8);				//Рег60	Температура радиатора выпрямителя №8, 1гЦ
modbus_registers[119]=(signed char)(bps[7]._Ti);
modbus_registers[120]=(signed char)(bps[7]._av>>8);				//Рег61	Байт флагов выпрямителя №8, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
modbus_registers[121]=(signed char)(bps[7]._av);
modbus_registers[122]=(signed char)(bps[0]._cnt>>8);			//Рег62	Счетчик связи выпрямителя №1
modbus_registers[123]=(signed char)(bps[0]._cnt);
modbus_registers[124]=(signed char)(bps[1]._cnt>>8);			//Рег63	Счетчик связи выпрямителя №2
modbus_registers[125]=(signed char)(bps[1]._cnt);
modbus_registers[126]=(signed char)(bps[2]._cnt>>8);			//Рег64	Счетчик связи выпрямителя №3
modbus_registers[127]=(signed char)(bps[2]._cnt);
modbus_registers[128]=(signed char)(bps[3]._cnt>>8);			//Рег65	Счетчик связи выпрямителя №4
modbus_registers[129]=(signed char)(bps[3]._cnt);
modbus_registers[130]=(signed char)(bps[4]._cnt>>8);			//Рег66	Счетчик связи выпрямителя №5
modbus_registers[131]=(signed char)(bps[4]._cnt);
modbus_registers[132]=(signed char)(bps[5]._cnt>>8);			//Рег67	Счетчик связи выпрямителя №6
modbus_registers[133]=(signed char)(bps[5]._cnt);
modbus_registers[134]=(signed char)(bps[6]._cnt>>8);			//Рег68	Счетчик связи выпрямителя №7
modbus_registers[135]=(signed char)(bps[6]._cnt);
modbus_registers[136]=(signed char)(bps[7]._cnt>>8);			//Рег69	Счетчик связи выпрямителя №8
modbus_registers[137]=(signed char)(bps[7]._cnt);


modbus_registers[138]=(signed char)(bps_U>>8);					//Рег70   	напряжение выпрямителей, 0.1В
modbus_registers[139]=(signed char)(bps_U);
tempS=0;
if((speedChIsOn)||(sp_ch_stat==scsWRK)) tempS=1;
modbus_registers[140]=(signed char)(tempS>>8);					//Рег71   	Ускоренный заряд включенность, (1 - вкл, 0 - Выкл)
modbus_registers[141]=(signed char)(tempS);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[142]=(signed char)(tempS>>8);					//Рег72   	Выравнивающий заряд включенность, (1 - вкл, 0 - Выкл)
modbus_registers[143]=(signed char)(tempS);
modbus_registers[144]=(signed char)(uout_av>>8);				//Рег73   Контроль выходного напряжения, (0 - норма, 1 - завышено, 2 - занижено)
modbus_registers[145]=(signed char)(uout_av);

tempS=0;													 	//Рег74	Регистр флагов состояния системы
if(bat_ips._av)			tempS|=(1<<0);						 	// Бит 0	Авария батареи
if(avar_stat&0x0001)   	tempS|=(1<<1);						 	//	Бит 1	Авария питающей сети 
if(avar_stat&(1<<(3+0)))tempS|=(1<<2);						 	//	Бит 2	Авария выпрямителя №1
if(avar_stat&(1<<(3+1)))tempS|=(1<<3);						 	//	Бит 3	Авария выпрямителя №2
if(avar_stat&(1<<(3+2)))tempS|=(1<<4);						 	//	Бит 4	Авария выпрямителя №3
if(avar_stat&(1<<(3+3)))tempS|=(1<<5);						 	//	Бит 4	Авария выпрямителя №4
if(avar_stat&(1<<(3+4)))tempS|=(1<<6);						 	//	Бит 4	Авария выпрямителя №5
if(avar_stat&(1<<(3+5)))tempS|=(1<<7);						 	//	Бит 4	Авария выпрямителя №6
if(avar_stat&(1<<(3+6)))tempS|=(1<<48);						 	//	Бит 4	Авария выпрямителя №7
modbus_registers[146]=(signed char)(tempS>>8);
modbus_registers[147]=(signed char)(tempS);




///*/
/*
tempS=cntrl_stat_old;
if(	(main_kb_cnt==(TBAT*60)-21) || (main_kb_cnt==(TBAT*60)-20) || (main_kb_cnt==(TBAT*60)-19)) tempS=((short)TBAT)|0x4000;
//tempS=0x800f;
modbus_registers[198]=(signed char)(tempS>>8);				//???100	????????? ???
modbus_registers[199]=(signed char)(tempS);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(signed char)(tempS>>8);				//Рег201	Внешний датчик температуры №1
modbus_registers[401]=(signed char)(tempS);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(signed char)(tempS>>8);				//Рег202	Внешний датчик температуры №2
modbus_registers[403]=(signed char)(tempS);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(signed char)(tempS>>8);				//Рег203	Внешний датчик температуры №3
modbus_registers[405]=(signed char)(tempS);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(signed char)(tempS>>8);				//Рег204	Внешний датчик температуры №4
modbus_registers[407]=(signed char)(tempS);   */

///*/
/*
tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(signed char)(tempS>>8);				//Рег211	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[421]=(signed char)(tempS);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(signed char)(tempS>>8);				//Рег212	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[423]=(signed char)(tempS);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(signed char)(tempS>>8);				//Рег213	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[425]=(signed char)(tempS);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(signed char)(tempS>>8);				//Рег214	Состояние  сухого контакта №1, (нулевой бит - физическое состояние, 1 - замкнут, 0 - разомкнут, первый бит - аварийность, 1 - авария, 0 - норма)
modbus_registers[427]=(signed char)(tempS);
*/
//modbus_registers[


if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	memcpy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

//	plazma_uart1[3]=reg_quantity;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	//*/mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
///*/	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}


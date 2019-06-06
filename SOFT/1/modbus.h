
extern unsigned char modbus_buf[20];
extern short modbus_crc16;
extern char modbus_timeout_cnt;
extern char bMODBUS_TIMEOUT;
extern unsigned char modbus_rx_buffer[30];	//Буфер, куда складывает принимаемые даннные обработчик прерывания по приему УАРТа
extern unsigned char modbus_an_buffer[30];	//Буфер, куда они потом копируются для анализа
extern unsigned char modbus_rx_buffer_ptr;	//Указатель на текущую позицию принимающего буфера
extern unsigned char modbus_rx_counter;		//Количество принятых байт, используется при анализе целостности посылки и при расшифровке

extern short modbus_plazma;				//Отладка
extern short modbus_plazma1;				//Отладка
extern short modbus_plazma2;				//Отладка
extern short modbus_plazma3;				//Отладка

extern unsigned short modbus_rx_arg0;		//встроенный в посылку первый аргумент
extern unsigned short modbus_rx_arg1;		//встроенный в посылку второй аргумент
extern unsigned short modbus_rx_arg2;		//встроенный в посылку третий аргумент
extern unsigned short modbus_rx_arg3;		//встроенный в посылку четвертый аргумент

extern char modbus_tx_buff[100];

//extern char modbus_registers[200];
//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len);
//-----------------------------------------------
//void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity);
//-----------------------------------------------
//void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr);
//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);
//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);
//-----------------------------------------------
//void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr);




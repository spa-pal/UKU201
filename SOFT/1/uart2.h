
#define RX_BUFFER_SIZE2 512
#define TX_BUFFER_SIZE2 512

//***********************************************
//сюпр
extern char bRXIN2;
extern char UIB2[100];
extern char flag2;
extern char rx_buffer2[RX_BUFFER_SIZE2];
extern char tx_buffer2[TX_BUFFER_SIZE2];
extern unsigned short rx_wr_index2,rx_rd_index2,rx_counter2;
extern unsigned short tx_wr_index2,tx_rd_index2,tx_counter2;
extern char rx_buffer_overflow2;
extern char tx2_restart;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void putchar2(char c);
void uart_out2 (char num,char data0,char data1,char data2,char data3,char data4,char data5);

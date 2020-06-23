/**************************************************************************
DO NOT CHANGE ANYTHING BELOW
***************************************************************************/ 

// Define CAN SFR address bases 
#define CAN_REG_BASE                    (0xE0000000)
#define ACCEPTANCE_FILTER_RAM_BASE      (CAN_REG_BASE + 0x00038000)
#define ACCEPTANCE_FILTER_REGISTER_BASE (CAN_REG_BASE + 0x0003C000)
#define CENTRAL_CAN_REGISTER_BASE       (CAN_REG_BASE + 0x00040000)              

// Common CAN bit rates
#define   CANBitrate125k_12MHz          0x001C001D
#define   CANBitrate125k_60MHz          0x001C001D
#define   CANBitrate250k_12MHz          0x001C000E
#define   CANBitrate250k_60MHz          0x009C000E
#define   CANBitrate62k_60MHz          	0x009C003b
#define 	BITRATE62_5K25MHZ			0x009c0018
#define 	BITRATE125K25MHZ			0x00940018

/**************************************************************************
USER DEFINABLE PARAMETERS
***************************************************************************/ 

// Maximum number of CAN interfaces supported by this driver (1 to 4)
// So far values 3 and 4 were not tested
// The example code in "main" uses CAN ports 1 and 2 (MAX_CANPORTS 2)
#define MAX_CANPORTS 2

// Maximum number of total FullCAN Filters for ALL CAN interfaces
#define MAX_FILTERS 2

extern short can_error_cntr;
// Type definition to hold a FullCAN message
// Compatible to FullCAN Mode Stored Messages in LPC User Manual


// Counts number of filters (CAN message objects) used so far
//extern short volatile gCANFilter = 0;

extern char ptr_can1_tx_wr,ptr_can1_tx_rd;
extern long can1_info[8];
extern long can1_id[8];
extern long can1_data[8];
extern long can1_datb[8];
																							 
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;

extern long can2_info[8];
extern long can2_id[8];
extern long can2_data[8];
extern long can2_datb[8];

extern unsigned short rotor_can[6];
// FullCAN Message List
//extern FULLCAN_MSG volatile gFullCANList[MAX_FILTERS];

extern char bR;
extern char RXBUFF[40],TXBUFF[40];
extern char bIN,bIN2;
extern char bd_dumm[25];
extern char bd[25];
extern char TX_len;
//char bOUT;
extern char RXBUFF2[40],TXBUFF2[40];
extern char can_tx_cnt;
extern char can_tx_cnt2;
//extern char bOUT_FREE=1;
//extern char bOUT_FREE2=1;
extern char rotor_rotor_rotor[2];
extern char can_tx_cnt;

extern const char Table87[];
extern const char Table95[];

extern char can_debug_plazma[2][10];
extern char bOUT_FREE;
extern char can_rotor[10];
extern char plazma_can;
extern char plazma_can1,plazma_can2,plazma_can3,plazma_can4;



typedef struct
{
  unsigned int Dat1; // Bits  0..10: CAN Message ID
                     // Bits 13..15: CAN interface number (1..4)
                     // Bits 16..19: DLC - Data Length Counter
                     // Bits 24..25: Semaphore bits
  unsigned int DatA; // CAN Message Data Bytes 0-3
  unsigned int DatB; // CAN Message Data Bytes 4-7
} FULLCAN_MSG; 
extern short volatile gCANFilter;
extern FULLCAN_MSG volatile gFullCANList[MAX_FILTERS];
extern char can_reset_cnt;



char CRC1_in(void);
char CRC2_in(void);
char CRC1_out(void);
char CRC2_out(void);
void can1_out_adr(char* ptr,char num);
__irq void can_isr_err (void);
void can1_out(char data0,char data1,char data2,char data3,char data4,char data5,char data6,char data7);
void can_adr_hndl(void);
void can_in_an(void);
void can_in_an2(void);
__irq void can_isr_rx (void); 
__irq void can_isr_tx (void); 
short can1_init ( unsigned int can_btr);
short can2_init ( unsigned int can_btr);
short FullCAN_SetFilter (
  unsigned short can_port, // CAN interface number
  unsigned int CANID // 11-bit CAN ID
  );

void CAN_IRQHandler(void);
void CAN_ISR_Rx1( void );

extern char can_debug_plazma[2][10];
extern char ccc_plazma[20];


void can_in_an1(void);



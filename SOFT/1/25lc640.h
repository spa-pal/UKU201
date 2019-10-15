/* SPI_SR - bit definitions. */
#define RXNE    0x01
#define TXE     0x02
#define BSY     0x80

#define LC640_CS 20
#define CS_ON       GPIOC->ODR&=~(1<<10);
#define CS_OFF      GPIOC->ODR|=(1<<10);

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
char spi2(char in);
void spi2_config(void);
void spi2_unconfig(void);
void lc640_wren(void);
char lc640_rdsr(void);
int lc640_read(int ADR);
int lc640_read_int(int ADR);
long lc640_read_long(int ADR);
void lc640_read_long_ptr(int ADR,char* out_ptr);
void lc640_read_str(int ADR, char* ram_ptr, char num);
char lc640_write(int ADR,char in);
char lc640_write_int(short ADR,short in);
char lc640_write_long(int ADR,long in);
char lc640_write_long_ptr(int ADR,char* in);

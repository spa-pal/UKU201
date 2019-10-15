

//***********************************************
//Аварии
extern unsigned avar_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
extern unsigned avar_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;
//Структура переменных
//1бит  - питающая сеть
//2бита - батареи
//12бит - БПСы
//5бит  - инверторы
//4бита - внешние датчики температуры
//4бита - внешние сухие контакты

//Аварии
extern unsigned rki_avar1_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
extern unsigned rki_avar1_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
extern unsigned rki_avar1_stat_old;
extern unsigned rki_avar1_stat_new, rki_avar1_stat_offed;

extern unsigned rki_avarI1_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
extern unsigned rki_avarI1_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
extern unsigned rki_avarI1_stat_old;
extern unsigned rki_avarI1_stat_new, rki_avarI1_stat_offed;

extern unsigned rki_avarI2_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
extern unsigned rki_avarI2_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
extern unsigned rki_avarI2_stat_old;
extern unsigned rki_avarI2_stat_new, rki_avarI2_stat_offed;

void avar_hndl(void);
void avar_unet_hndl(char in);
void avar_uout_hndl(char in);
void reload_hndl(void);
void avar_bps_hndl(char bps, char v, char in);
void avar_bat_hndl(char bat, char in);
void avar_bat_as_hndl(char b, char in);
void ke_mem_hndl(char b,unsigned short in);
void ke_zvu_mem_hndl(char b,unsigned short in,unsigned short in1);
void vz_mem_hndl(unsigned short in);
void wrk_mem_hndl(char b);
void avar_bat_ips_hndl(char in);




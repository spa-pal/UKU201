
//***********************************************
//АЦП
extern unsigned short adc_buff[10][16];
extern unsigned short adc_buff_[10];
extern char adc_ch, adc_cnt;
extern short adc_buff_virt_0;

//-----------------------------------------------
//Автокалибровка
typedef enum {akmOFF=0, akmUI,  akmI,  akmT,  akmUN} avt_klbr_mode_enum;
extern avt_klbr_mode_enum avt_klbr_mode;
extern char avt_klbr_num;
extern char avt_klbr_phase;
extern char avt_klbr_main_cnt;
extern short avt_klbr_real_value;
extern short avt_klbr_necc_value;
extern short avt_klbr_err_cnt;
extern signed short avt_klbr_err;
extern char avt_klbr_err_sign;
extern char avt_klbr_err_sign_old;

extern char avt_klbr_main_cnt_;
extern char avt_klbr_mode_ui, avt_klbr_mode_un, avt_klbr_mode_i, avt_klbr_mode_t;
extern short avt_klbr_real_value_ui, avt_klbr_real_value_un, avt_klbr_real_value_i, avt_klbr_real_value_t; 
extern short avt_klbr_necc_value_ui, avt_klbr_necc_value_un, avt_klbr_necc_value_i, avt_klbr_necc_value_t; 
extern char avt_klbr_num_ui, avt_klbr_num_un, avt_klbr_num_i, avt_klbr_num_t;
extern short avt_klbr_err_cnt_ui, avt_klbr_err_cnt_un, avt_klbr_err_cnt_i, avt_klbr_err_cnt_t;
extern signed avt_klbr_err_ui, avt_klbr_err_un, avt_klbr_err_i, avt_klbr_err_t;
extern char avt_klbr_err_sign_ui, avt_klbr_err_sign_un, avt_klbr_err_sign_i, avt_klbr_err_sign_t;
extern char avt_klbr_err_sign_old_ui, avt_klbr_err_sign_old_un, avt_klbr_err_sign_old_i, avt_klbr_err_sign_old_t;
extern char avt_klbr_phase_ui, avt_klbr_phase_un, avt_klbr_phase_i, avt_klbr_phase_t;
extern char avt_klbr_cmd_ui, avt_klbr_cmd_un, avt_klbr_cmd_i, avt_klbr_cmd_t;

//**********************************************
//Работа с БПСами
extern char num_of_wrks_bps;
extern char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
extern char bps_hndl_2sec_cnt;
extern unsigned short bps_on_mask,bps_off_mask;
extern char num_necc_up,num_necc_down;
extern unsigned char sh_cnt0,b1Hz_sh;
extern signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
extern signed short main_cnt_5Hz;
extern signed short num_necc;
extern signed short num_necc_Imax;
extern signed short num_necc_Imin;
extern signed short cnt_num_necc;
extern signed short mat_temper;

//***********************************************
//Управление ШИМом
extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;
extern unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
extern unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
extern unsigned char	ch_cnt1,b1_30Hz_ch;
extern unsigned char	ch_cnt2,b1_10Hz_ch;
extern unsigned short IZMAX_;
extern unsigned short IZMAX_70;
extern unsigned short IZMAX_130;
extern unsigned short Ubpsmax;
extern unsigned short cntrl_stat_blck_cnt;
extern short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];
extern char cntrl_hndl_plazma;

//***********************************************
//Блокировка ИПС
extern signed short ipsBlckSrc;
extern signed short ipsBlckLog;
extern signed short ipsBlckStat;

//***********************************************
//Уравнительный заряд
typedef enum {vz1sOFF=0, vz1sSTEP1=1, vz1sSTEP2=2, vz1sSTEP3=3, vz1sWRK=10, vz1sERR1, vz1sERR2, vz1sERR3, vz1sERR4, vz1sFINE, vz1sSTOP}enum_vz1_stat;
extern enum_vz1_stat vz1_stat, vz1_stat_old;
extern short vz1_stat_cnt;
extern long vz1_wrk_cnt;
extern long vz1_up_cnt;
extern char volt_region;

//***********************************************
//Формовочный заряд
typedef enum {vz2sOFF=0, vz2sSTEP1=1, vz2sSTEP2=2, vz2sSTEP3=3, vz2sWRK1=10, vz2sWRK2=11, vz2sERR1, vz2sERR2, vz2sERR3, vz2sERR4, vz2sERR5, vz2sERR6, vz2sFINE, vz2sSTOP}enum_vz2_stat;
extern enum_vz2_stat vz2_stat, vz2_stat_old;
extern short vz2_stat_cnt;
extern long vz2_wrk_cnt;
extern long vz2_up_cnt;
extern signed short vz2_stat_ph2_cnt;

//***********************************************
//Высоковольтный выравнивающий заряд
typedef enum {hvsOFF,hvsSTEP1,hvsSTEP2,hvsSTEP3,hvsSTEP4,hvsWRK,hvsERR1,hvsERR2,hvsERR3,hvsERR4} enum_hv_vz_stat;
extern enum_hv_vz_stat hv_vz_stat,hv_vz_stat_old;
extern short hv_vz_stat_cnt;
extern long hv_vz_wrk_cnt;
extern long hv_vz_up_cnt;

//-----------------------------------------------
//Выравнивание токов ИПС
extern char ica_plazma[10];
extern char ica_timer_cnt;
extern signed short ica_my_current;
extern signed short ica_your_current;
extern signed short ica_u_necc;
extern signed short ica_cntrl_hndl;
extern signed short ica_cntrl_hndl_cnt;
extern unsigned char tcp_soc_avg;
extern unsigned char tcp_connect_stat;

//***********************************************
//Ротация ведущего источника
extern char numOfForvardBps,numOfForvardBps_old;
extern char numOfForvardBps_minCnt;
extern short numOfForvardBps_hourCnt;

//***********************************************
//Состояние внешних датчиков
//signed short tout[4];
extern char tout_max_cnt[4],tout_min_cnt[4];
typedef enum {tNORM,tMAX,tMIN}enum_tout_stat;
extern enum_tout_stat tout_stat[4];
extern signed short t_ext[3];
extern char ND_EXT[3];
extern signed char sk_cnt[4],sk_av_cnt[4];
typedef enum  {ssOFF,ssON} enum_sk_stat;
extern enum_sk_stat sk_stat[4],sk_stat_old[4];
typedef enum  {sasOFF,sasON} enum_sk_av_stat;
extern enum_sk_av_stat sk_av_stat[4],sk_av_stat_old[4];
extern signed short t_box,t_box_warm,t_box_vent;
extern char TELECORE2017_EXT_VENT_PWM,TELECORE2017_INT_VENT_PWM;


//**********************************************
//Контроль наличия батарей
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 		kb_full_ver;
extern char /*kb_start[2],*/kb_start_ips;
extern signed short ibat_ips,ibat_ips_;
extern char ips_bat_av_vzvod;
extern char ips_bat_av_stat;

//-----------------------------------------------
//Контроль выходного напряжения
extern signed short outVoltContrHndlCntUp;		//Счетчик, считает в плюс в случае превышения
extern signed short outVoltContrHndlCntDn;		//Счетчик, считает в плюс в случае занижения
extern char uout_av;							//Авария по выходному напряжению 0 - норма, 1 - занижено, 2 - завышено


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void adc_drv(void);
void adc_init(void);



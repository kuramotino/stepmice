/*
 * PL_sensor.h
 *
 *  Created on: Sep 28, 2022
 *      Author: Ryu
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_

extern uint16_t g_sensor_on[4];
extern uint16_t g_sensor_off[4];
extern uint16_t g_sensor_now[4];
extern uint16_t g_sensor_nowBu[4];
extern uint16_t g_sensor_nowHennka[4];
extern float M_g_sensor_nowHennka[2];
extern uint16_t M_g_sensor_pre[2];
extern uint16_t g_sensor_pre[5][4];
extern float SENSOR_GAIN;
extern float M_GAIN;
extern float CENTER_R;
extern float CENTER_L;
extern const float ABS_CENTER_R;
extern const float ABS_CENTER_L;
extern float CENTER_F_R;
extern float CENTER_F_L;
extern float Rsla_CENTER_F_R;
extern float Rsla_CENTER_F_L;
extern float Lsla_CENTER_F_R;
extern float Lsla_CENTER_F_L;
extern float R_Oo_CENTER_F_R;
extern float R_Oo_CENTER_F_L;
extern float L_Oo_CENTER_F_R;
extern float L_Oo_CENTER_F_L;
extern float R_Ko_CENTER_F_R;
extern float R_Ko_CENTER_F_L;
extern float L_Ko_CENTER_F_R;
extern float L_Ko_CENTER_F_L;
extern float ABS_CENTER_F_R;
extern float ABS_CENTER_F_L;
extern float FRONT_GAIN;
extern float THRESHOLD_R;
extern float THRESHOLD_L;
extern float THRESHOLD_F_R;
extern float THRESHOLD_F_L;
extern float THRESHOLD_F_R_2;
extern float THRESHOLD_F_L_2;
extern float THRESHOLD_DIFE_R;
extern float THRESHOLD_DIFE_L;
extern int g_WallControlStatus[2];
extern int kabe_cut_count[2];
extern int kabe_cut_counter[2];
extern float g_V_batt;
extern float g_V_batt_Remit;
extern int kabe_inf[3];//0が左壁、1が前壁、2が右壁
extern int sotokabe_inf[3];//0が左壁、1が前壁、2が右壁
extern float batBu[150];
extern int batCount;
extern float batAVE;
extern int isBatTest;
extern float MinbatAVE;
extern int isSafe;
extern int sensacount;//センサのカウント
extern float Side_L_ADtoX[4];//距離変換関数の係数
extern float Side_R_ADtoX[4];//距離変換関数の係数
extern float Front_L_ADtoX[6];//距離変換関数の係数
extern float Front_R_ADtoX[4];//距離変換関数の係数
extern float dif_log_F[100];//前補正量のログ配列
extern float dif_log_B[100];//後補正量のログ配列
extern int dif_F_count;//前ログのカウント
extern int dif_B_count;//後ログのカウント

void pl_callback_getSensor(void);
void pl_interupt_getSensor(void);
void pl_print(void);
float calWallConrol(void);
int Hidarite(void);
float TestBatt(void);
void TestBattAVE(void);
void Senser(void);//周囲の壁情報を取得
float Front_Wall_Offset(void);//前壁との距離から補正量を返す関数
float Side_Wall_Offset(void);//横壁との距離から補正量を返す関数
void SetOffsetLog(int TfFb,float dif,int isLog);//前か後か、補正量、出力モードか否か
void Front_Offset_Stop(int setnum,int* isstop_offset);//前オフセットをやめる関数,次の行動パターン,オフセットをやめさせたか
int KabeGire_Check(void);//壁切れをチェックする関数
int M_KabeGire_Check(int TrFl);//最短の壁切れをチェックする関数,TrFl=0両壁あり、1,右壁あり、-1,左壁あり,-2両壁なし


#endif /* INC_PL_SENSOR_H_ */

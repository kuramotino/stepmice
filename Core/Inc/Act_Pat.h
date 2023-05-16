/*
 * Act_Pat.h
 *
 *  Created on: 2022/11/23
 *      Author: Ryu
 */

#ifndef INC_ACT_PAT_H_
#define INC_ACT_PAT_H_

extern void Straight(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int busenkaivec,int posset);
extern void Go_Offset(float offset,int isKabe,int isDist,int isFront,int setnum);
extern void Turn_Offset(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int isMaeKabe,int isKabeGire,int setnum);//大回りなど各種ターンのオフセット
extern void Left_Slalom(float wa,float wv_start,float wv_max,float wv_end,float wx);
extern void Left_Oo_Ko(float wa,float wv_start,float wv_max,float wv_end,float wx,int Oo_Ko);
extern void Right_Slalom(float wa,float wv_start,float wv_max,float wv_end,float wx);
extern void Right_Oo_Ko(float wa,float wv_start,float wv_max,float wv_end,float wx,int Oo_Ko);
extern void Left_Senkai();
extern void Right_Senkai();
extern void U_turn();
extern void Return_Stop();
extern void StopCheck();
extern int StartButton();
extern int StopButton();
extern void MotorStart();
extern void MotorStop();
extern void Show_sensor_log();
extern void Stop_Delay_Start(int isStop);

#endif /* INC_ACT_PAT_H_ */

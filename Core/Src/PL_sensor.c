/*
 * PL_sensor.c
 *
 *  Created on: Sep 28, 2022
 *      Author: Ryu
 */
#include "main.h"
#include "adc.h"
#include "stdio.h"
#include "stm32l4xx_it.h"
#include "wait_ms.h"
#include "daikei.h"
#include "PL_sensor.h"

uint16_t g_ADCBuffer[5];
char AD_step;
uint16_t g_sensor_on[4];
uint16_t g_sensor_off[4];
uint16_t g_sensor_now[4];
uint16_t g_sensor_nowBu[4];
uint16_t g_sensor_nowHennka[4];
float M_g_sensor_nowHennka[2];
uint16_t M_g_sensor_pre[2];
uint16_t g_sensor_pre[5][4];
float g_V_batt;
float g_V_batt_Remit=11.1;
float SENSOR_GAIN;
float M_GAIN;
float CENTER_R;
float CENTER_L;
const float ABS_CENTER_R=335.0;
const float ABS_CENTER_L=280.0;
float CENTER_F_R;
float CENTER_F_L;
float Rsla_CENTER_F_R;
float Rsla_CENTER_F_L;
float Lsla_CENTER_F_R;
float Lsla_CENTER_F_L;
float R_Oo_CENTER_F_R;
float R_Oo_CENTER_F_L;
float L_Oo_CENTER_F_R;
float L_Oo_CENTER_F_L;
float R_Ko_CENTER_F_R;
float R_Ko_CENTER_F_L;
float L_Ko_CENTER_F_R;
float L_Ko_CENTER_F_L;
float ABS_CENTER_F_R;
float ABS_CENTER_F_L;
float FRONT_GAIN;
float THRESHOLD_R;
float THRESHOLD_L;
float THRESHOLD_F_R;
float THRESHOLD_F_L;
float THRESHOLD_F_R_2;
float THRESHOLD_F_L_2;
float THRESHOLD_DIFE_R;
float THRESHOLD_DIFE_L;
int g_WallControlStatus[2];
int kabe_cut_count[2];
int kabe_cut_counter[2];
int kabe_inf[3];//0が左壁、1が前壁、2が右壁
int sotokabe_inf[3];//0が左壁、1が前壁、2が右壁

float batBu[150];
int batCount=0;
float batAVE;
int isBatTest=1;
float MinbatAVE=9.3;
int isSafe=0;
int sensacount=0;//センサのカウント
float Side_L_ADtoX[4]={237.48,-1.7282,0.0051,-0.000006};//距離変換関数の係数
float Side_R_ADtoX[4]={190.7,-1.0232,0.0023,-0.000002};//距離変換関数の係数
float Front_L_ADtoX[6]={260.76,-2.3844,0.0128,-0.00004,0.00000005,-0.00000000003};//距離変換関数の係数
float Front_R_ADtoX[4]={209.59,-1.1957,0.0031,-0.000003};//距離変換関数の係数
float dif_log_F[100];//前補正量のログ配列
float dif_log_B[100];//後補正量のログ配列
int dif_F_count=0;//前ログのカウント
int dif_B_count=0;//後ログのカウント
/*******************************************************************/
/*callback用関数(pl_callback_getSensor)*/
/*******************************************************************/
/*DMAがスタートしたら実行するコード*//*******************************************************************/
void pl_callback_getSensor(void) {
	uint16_t V_battAD;
	int j;
	HAL_ADC_Stop_DMA(&hadc1);
	switch(AD_step) {
	    case 0:
	    	g_sensor_off[0] = g_ADCBuffer[1];
	    	g_sensor_off[1] = g_ADCBuffer[2];
	    	g_sensor_off[2] = g_ADCBuffer[3];
	    	g_sensor_off[3] = g_ADCBuffer[4];
		    HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
		    for(j = 0; j <= 500; j++) {
		    }
		    break;
		case 1:
			g_sensor_on[0] = g_ADCBuffer[1];
			g_sensor_on[1] = g_ADCBuffer[2];
			g_sensor_off[2] = g_ADCBuffer[3];
			g_sensor_off[3] = g_ADCBuffer[4];
			HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_SET);
			for(j = 0; j <= 500; j++) {
			}
			break;
		case 2:
			g_sensor_off[0] = g_ADCBuffer[1];
			g_sensor_off[1] = g_ADCBuffer[2];
			g_sensor_on[2] = g_ADCBuffer[3];
			g_sensor_on[3] = g_ADCBuffer[4];
			HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
			for(j = 0; j <= 10; j++) {
			}
			break;
	}



	V_battAD= g_ADCBuffer[0];
	g_V_batt = 3.3 * (float) V_battAD / 1023 * (100.0 + 47.0) / 47.0;
	if(g_V_batt<g_V_batt_Remit)
	{
		//isStart=0;
	}
	AD_step++;



	if(AD_step!= 3) {
		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
	} else{
		AD_step= 0;
	}
}
/*******************************************************************/
/*割り込み用動作関数(センサー取得)(interupt_calSensor)*//*******************************************************************/
/*センサーの情報を取得する割り込み関数．*//*******************************************************************/
void pl_interupt_getSensor(void){
	HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));

	    g_sensor_now[0]=g_sensor_on[0]-g_sensor_off[0];
		g_sensor_now[1]=g_sensor_on[1]-g_sensor_off[1];
		g_sensor_now[2]=g_sensor_on[2]-g_sensor_off[2];
		g_sensor_now[3]=g_sensor_on[3]-g_sensor_off[3];

		M_g_sensor_nowHennka[0]=(float)g_sensor_now[1]-(float)M_g_sensor_pre[0];//left
		M_g_sensor_nowHennka[1]=(float)g_sensor_now[2]-(float)M_g_sensor_pre[1];//right
		M_g_sensor_pre[0]=g_sensor_now[1];//left
		M_g_sensor_pre[1]=g_sensor_now[2];//right

		if(sensacount==5)
		{
			sensacount=0;
			g_sensor_pre[sensacount][0]=g_sensor_now[0];
		    g_sensor_pre[sensacount][1]=g_sensor_now[1];
			g_sensor_pre[sensacount][2]=g_sensor_now[2];
			g_sensor_pre[sensacount][3]=g_sensor_now[3];

			sensacount++;
		}
		else
		{
			g_sensor_pre[sensacount][0]=g_sensor_now[0];
			g_sensor_pre[sensacount][1]=g_sensor_now[1];
			g_sensor_pre[sensacount][2]=g_sensor_now[2];
			g_sensor_pre[sensacount][3]=g_sensor_now[3];

			sensacount++;
		}

		for(int i=0;i<4;i++)
		{
			uint16_t ave=0;
			int avecount=5;
			for(int j=0;j<5;j++)
			{
				if(g_sensor_pre[j][i]==0 && avecount!=0)
				{
					avecount--;
				}
				ave+=g_sensor_pre[j][i];
			}
			ave=ave/(uint16_t)avecount;
			g_sensor_now[i]=ave;
		}

		g_sensor_nowHennka[0]=g_sensor_nowBu[0]-g_sensor_now[0];
		g_sensor_nowHennka[1]=g_sensor_nowBu[1]-g_sensor_now[1];
		g_sensor_nowHennka[2]=g_sensor_nowBu[2]-g_sensor_now[2];
		g_sensor_nowHennka[3]=g_sensor_nowBu[3]-g_sensor_now[3];

		g_sensor_nowBu[0]=g_sensor_now[0];
		g_sensor_nowBu[1]=g_sensor_now[1];
		g_sensor_nowBu[2]=g_sensor_now[2];
		g_sensor_nowBu[3]=g_sensor_now[3];
}

void pl_print(void)
{
	//printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n\r", g_sensor_off[0],g_sensor_off[1],g_sensor_off[2],g_sensor_off[3]);
	//printf("BATT=%f\n\r",g_V_batt);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d\n\r", g_sensor_now[0],g_sensor_now[1],g_sensor_now[2],g_sensor_now[3]);
	//printf("dif_SEN2=%f,dif_SEN3=%f\n\r", (float)M_g_sensor_nowHennka[0],(float)M_g_sensor_nowHennka[1]);
}

float calWallConrol(void)
{
	//bitの決定
	if(g_WallControlStatus[0]==1)
	{
		if(g_sensor_now[1]<THRESHOLD_L || g_sensor_nowHennka[1]>THRESHOLD_DIFE_L)
		{
			g_WallControlStatus[0]=0;
		}
	}
	else
	{
		if(g_sensor_now[1]>THRESHOLD_L && g_sensor_nowHennka[1]<THRESHOLD_DIFE_L)
		{
			kabe_cut_counter[0]++;
		}
		else
		{
			kabe_cut_counter[0]=0;
		}

		if(kabe_cut_counter[0]>kabe_cut_count[0])
		{
			g_WallControlStatus[0]=1;
			kabe_cut_counter[0]=0;
		}
	}
	if(g_WallControlStatus[1]==1)
	{
		if(g_sensor_now[2]<THRESHOLD_R || g_sensor_nowHennka[2]>THRESHOLD_DIFE_R)
		{
			g_WallControlStatus[1]=0;
		}
	}
	else
	{
		if(g_sensor_now[2]>THRESHOLD_R && g_sensor_nowHennka[2]<THRESHOLD_DIFE_R)
		{
			kabe_cut_counter[1]++;
		}
		else
		{
			kabe_cut_counter[1]=0;
		}

		if(kabe_cut_counter[1]>kabe_cut_count[1])
		{
			g_WallControlStatus[1]=1;
			kabe_cut_counter[1]=0;
		}
	}


	float PID_wall;

	if(g_WallControlStatus[0]==0)
	{
		if(g_WallControlStatus[1]==0)
		{
			//1両壁なし
			PID_wall=0.0;
		}
		else
		{
			//2左壁なし
			PID_wall=SENSOR_GAIN*v/500*-2*(float)(g_sensor_now[2]-CENTER_R);
			if(mode==4 || mode==6)//1最短走行なら
			{
				PID_wall=M_GAIN*-2*(float)(g_sensor_now[2]-CENTER_R);
			}
		}
	}
	else
	{
		if(g_WallControlStatus[1]==0)
		{
			//3右壁なし
			PID_wall=SENSOR_GAIN*v/500*2*(float)(g_sensor_now[1]-CENTER_L);
			if(mode==4 || mode==6)//1最短走行なら
			{
				PID_wall=M_GAIN*2*(float)(g_sensor_now[1]-CENTER_L);
			}
		}
		else
		{
			//4両壁あり
			PID_wall=SENSOR_GAIN*v/500*((float)(g_sensor_now[1]-CENTER_L)-(float)(g_sensor_now[2]-CENTER_R));
			if(mode==4 || mode==6)//1最短走行なら
			{
				PID_wall=M_GAIN*((float)(g_sensor_now[1]-CENTER_L)-(float)(g_sensor_now[2]-CENTER_R));
			}
		}
	}

	return PID_wall;
}

int Hidarite(void)
{
	//1壁情報の更新
	if(g_sensor_now[1]>=THRESHOLD_L)
	{
			kabe_inf[0]=1;
	}
	else
	{
		    kabe_inf[0]=0;
	}
	if(g_sensor_now[0]>=THRESHOLD_F_L && g_sensor_now[3]>=THRESHOLD_F_R)
	{
			kabe_inf[1]=1;
	}
	else
	{
			kabe_inf[1]=0;
	}
	if(g_sensor_now[2]>=THRESHOLD_R)
	{
			kabe_inf[2]=1;
	}
	else
	{
			kabe_inf[2]=0;
	}


	int num;
	if(g_sensor_now[1]<THRESHOLD_L)
	{
		//1左壁なし
		num=0;
	}
	else if(g_sensor_now[0]<THRESHOLD_F_L && g_sensor_now[3]<THRESHOLD_F_R)
	{
		//2左壁ありかつ前壁なし
		num=1;
	}
	else if(g_sensor_now[2]<THRESHOLD_R)
	{
		//3左壁ありかつ前壁ありかつ右壁なし
		num=2;
	}
	else
	{
		//4左壁ありかつ前壁ありかつ右壁あり
		num=3;
	}
	return num;
}

float batf;
uint16_t bat;
float TestBatt(void)
{

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	bat = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	batf= 3.3* (float) (bat) / 1023.0* (100.0+ 22.0) / 22.0;

	/*batBu[batCount]=batf;
	batCount++;
	if(batCount==140)
	{
		isBatTest=0;
	}*/
	return batf;

	//batfの平均9.7が11.7V
	//batfの平均10.5が12.6V
}

void TestBattAVE(void)
{
	int batcount=150;
	for(int i=0;i<150;i++)
	{
		if(batBu[i]==0)
		{
			batcount--;
		}
		batAVE+=batBu[i];
	}
	batAVE/=batcount;
	if(batAVE<MinbatAVE)
	{
		isSafe=1;
		printf("BATT=%f WARNIG!!\n\r",batAVE);
	}
	else
	{
		printf("BATT=%f\n\r",batAVE);
	}
}

void Senser(void)//周囲の壁情報を取得
{
	//1壁情報の更新
	if(g_sensor_now[1]>=THRESHOLD_L)
	{
			kabe_inf[0]=1;
	}
	else
	{
			kabe_inf[0]=0;
	}
	if(g_sensor_now[0]>=THRESHOLD_F_L && g_sensor_now[3]>=THRESHOLD_F_R)
	{
			kabe_inf[1]=1;
	}
	else if((g_sensor_now[0]>=THRESHOLD_F_L || g_sensor_now[3]>=THRESHOLD_F_R) && isNoKabeAte==1)
	{
			kabe_inf[1]=1;
	}
	else
	{
			kabe_inf[1]=0;
	}
	if(g_sensor_now[2]>=THRESHOLD_R)
	{
			kabe_inf[2]=1;
	}
	else
	{
			kabe_inf[2]=0;
	}
}

float Front_Wall_Offset(void)//前壁との距離から補正量を返す関数
{
	if(kabe_inf[1]==1)
	{
		//1左前壁センサと右前壁センサのad値を計算
		float dif_l=CENTER_F_L-ABS_CENTER_F_L;
		float dif_r=CENTER_F_R-ABS_CENTER_F_R;
		float adl=(float)g_sensor_now[0]+dif_l;
		float adr=(float)g_sensor_now[3]+dif_r;

		//2前壁との距離を計算
		float l_len=Front_L_ADtoX[0]+Front_L_ADtoX[1]*adl+Front_L_ADtoX[2]*adl*adl+Front_L_ADtoX[3]*adl*adl*adl+Front_L_ADtoX[4]*adl*adl*adl*adl+Front_L_ADtoX[5]*adl*adl*adl*adl*adl;
		float r_len=Front_R_ADtoX[0]+Front_R_ADtoX[1]*adr+Front_R_ADtoX[2]*adr*adr+Front_R_ADtoX[3]*adr*adr*adr;

		//3真ん中にいるときの前壁との距離を計算
		float cl=CENTER_F_L;
		float cr=CENTER_F_R;
		float center_l_len=Front_L_ADtoX[0]+Front_L_ADtoX[1]*cl+Front_L_ADtoX[2]*cl*cl+Front_L_ADtoX[3]*cl*cl*cl+Front_L_ADtoX[4]*cl*cl*cl*cl+Front_L_ADtoX[5]*cl*cl*cl*cl*cl;
		float center_r_len=Front_R_ADtoX[0]+Front_R_ADtoX[1]*cr+Front_R_ADtoX[2]*cr*cr+Front_R_ADtoX[3]*cr*cr*cr;

		//4真ん中からのずれ距離を計算
		float dif_l_len=l_len-center_l_len;
		float dif_r_len=r_len-center_r_len;

		//5補正量を平均
		float dif_len=(dif_l_len+dif_r_len)/2;

		//6補正量を足した値がマイナスなら0にする
		if(kabe_inf[0]==1 && kabe_inf[2]==0)
		{
			if(dif_len+r_front_offset-front_offset<0)
			{
				dif_len=-1*(r_front_offset-front_offset);
			}
		}
		else if(kabe_inf[0]==0 && kabe_inf[2]==1)
		{
			if(dif_len+l_front_offset-front_offset<0)
			{
				dif_len=-1*(l_front_offset-front_offset);
			}
		}
		else
		{
			dif_len=0.0;
		}

		//7補正量を返す
		return dif_len;
	}
	return 0;
}

float Side_Wall_Offset(void)//横壁との距離から補正量を返す関数
{
	if(kabe_inf[0]==1 && kabe_inf[2]==0)
	{
		//1左壁ありの時
		float dif=CENTER_L-ABS_CENTER_L;
		float adc=(float)g_sensor_now[1]+dif;

		//2左壁との距離を計算
		float len=Side_L_ADtoX[0]+Side_L_ADtoX[1]*adc+Side_L_ADtoX[2]*adc*adc+Side_L_ADtoX[3]*adc*adc*adc;

		//3真ん中にいるときの左壁との距離を計算
		float center_len=Side_L_ADtoX[0]+Side_L_ADtoX[1]*CENTER_L+Side_L_ADtoX[2]*CENTER_L*CENTER_L+Side_L_ADtoX[3]*CENTER_L*CENTER_L*CENTER_L;

		//4真ん中からのずれ距離を計算
		float dif_len=center_len-len;

		//5補正量を足した値がマイナスなら0にする
		if(dif_len+r_back_offset<0)
		{
			dif_len=-1*r_back_offset;
		}

		//6補正量が大きすぎたら調整
		if(dif_len>max_offset)
		{
			dif_len=max_offset;
		}

		//7補正後距離を返す
		return dif_len;
	}
	else if(kabe_inf[2]==1 && kabe_inf[0]==0)
	{
		//1右壁ありの時
		float dif=CENTER_R-ABS_CENTER_R;
		float adc=(float)g_sensor_now[2]+dif;

		//2右壁との距離を計算
		float len=Side_R_ADtoX[0]+Side_R_ADtoX[1]*adc+Side_R_ADtoX[2]*adc*adc+Side_R_ADtoX[3]*adc*adc*adc;

		//3真ん中にいるときの右壁との距離を計算
		float center_len=Side_R_ADtoX[0]+Side_R_ADtoX[1]*CENTER_R+Side_R_ADtoX[2]*CENTER_R*CENTER_R+Side_R_ADtoX[3]*CENTER_R*CENTER_R*CENTER_R;

		//4真ん中からのずれ距離を計算
		float dif_len=center_len-len;

		//5補正量を足した値がマイナスなら0にする
		if(dif_len+l_back_offset<0)
		{
			dif_len=-1*l_back_offset;
		}

		//6補正量が大きすぎたら調整
		if(dif_len>max_offset)
		{
			dif_len=max_offset;
		}

		//7補正後距離を返す
		return dif_len;
	}
	return 0;
}

//aオフセットのログを格納する関数
void SetOffsetLog(int TfFb,float dif,int isLog)//前か後か、補正量、出力モードか否か
{
	if(isLog==0)
	{
		if(TfFb==1)
		{
			dif_log_F[dif_F_count]=dif;
			if(dif_F_count!=99)
			{
				dif_F_count++;
			}
		}
		else
		{
			dif_log_B[dif_B_count]=dif;
			if(dif_B_count!=99)
			{
				dif_B_count++;
			}
		}
	}
	else
	{
		for(int i=0;i<100;i++)
		{
			printf("front_offset=%f,back_offset=%f\n\r", dif_log_F[i],dif_log_B[i]);
		}
	}
}

void Front_Offset_Stop(int setnum,int* isstop_offset)//前オフセットをやめる関数,次の行動パターン、オフセットをやめさせたか
{
	if(kabe_inf[1]==1 && setnum==0)//1左スラローム
	{
		if(Lsla_CENTER_F_R<(float)g_sensor_now[3] && Lsla_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
	else if(kabe_inf[1]==1 && setnum==2)//2右スラローム
	{
		if(Rsla_CENTER_F_R<(float)g_sensor_now[3] && Rsla_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
	else if(kabe_inf[1]==1 && setnum==-4)//3左大回り
	{
		if(L_Oo_CENTER_F_R<(float)g_sensor_now[3] && L_Oo_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
	else if(kabe_inf[1]==1 && setnum==-6)//4右大回り
	{
		if(R_Oo_CENTER_F_R<(float)g_sensor_now[3] && R_Oo_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
	else if(kabe_inf[1]==1 && setnum==-5)//5左小回り
	{
		if(L_Ko_CENTER_F_R<(float)g_sensor_now[3] && L_Ko_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
	else if(kabe_inf[1]==1 && setnum==-7)//6右小回り
	{
		if(R_Ko_CENTER_F_R<(float)g_sensor_now[3] && R_Ko_CENTER_F_L<(float)g_sensor_now[0])
		{
			kasokuflag=0;
			*isstop_offset=1;
		}
	}
}

int KabeGire_Check(void)//壁切れをチェックする関数
{
	if(g_sensor_now[1]<THRESHOLD_L || g_sensor_now[2]<THRESHOLD_R)//左か右で壁切れが起こったら
	{
		kasokuflag=0;
		return 1;
	}
	return 0;
}

int M_KabeGire_Check(int TrFl)//最短の壁切れをチェックする関数,TrFl=0両壁あり、1,右壁あり、-1,左壁あり,-2両壁なし
{
	if(TrFl==0)//0両壁あり
	{
		if(g_sensor_now[1]<THRESHOLD_L || g_sensor_now[2]<THRESHOLD_R)//左か右で壁切れが起こったら
		{
			kasokuflag=0;
			return 1;
		}
	}
	else if(TrFl==1)//1右壁あり
	{
		if(g_sensor_now[2]<THRESHOLD_R)//右で壁切れが起こったら
		{
			kasokuflag=0;
			return 1;
		}
	}
	else if(TrFl==-1)//2左壁あり
	{
		if(g_sensor_now[1]<THRESHOLD_L)//左で壁切れが起こったら
		{
			kasokuflag=0;
			return 1;
		}
	}
	else//両壁なし
	{
		return 0;
	}
	return 0;
}









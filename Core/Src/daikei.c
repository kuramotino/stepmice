/*
 * daikei.c
 *
 *  Created on: Oct 8, 2022
 *      Author: Ryu
 */

#include "main.h"
#include "stdio.h"
#include "math.h"
#include "PL_sensor.h"
#include "daikei.h"
#include "mapping.h"

int kasokuflag=0;//台形加速のフラッグ
int kabeflag=0;//壁制御のフラッグ
int mode=0;//モード
int isStart=0;
int isStop=0;
int isReturn=0;
uint16_t g_motorCount_l=10000;
uint16_t g_motorCount_r=10000;
float dx=0.0f;//位置の変位
float dt=0.001;//刻み時間
float a;//加速度
float v_start;//初速度
float v;//現在の速度
float v_max;//最大速度
float v_end;//終端速度
float x;//目標距離
float xde;//減速距離
float Taiya=51.115;//タイヤ径
float bu_Taiya=51.115;//代入用のタイヤ径
float M_1500_Taiya=48.1;//最短1.5mのタイヤ径

float const_ba=3000.0f;//定速区間の加速度
float const_bv_start=500.0f;//定速区間の初速度
float const_bv_max=500.0f;//定速区間の最高速度
float const_bv_end=500.0f;//低速区間の終端速度
const float front_offset=20.0;//前オフセット
float l_front_offset=25.0;//左前オフセット
float r_front_offset=30.0;//右前オフセット
float const_bx=180.0-front_offset;//定速区間の代入用目標距離
float r_back_offset=15.0;//右の後オフセット
float l_back_offset=20.0;//左の後オフセット
float bu_front_offset=0.0;//代入用前オフセット
float bu_back_offset=0.0;//代入用後オフセット
float max_offset=10.0;//オフセットの最大調整量
float add_offset=1.0;//足りないときに加える前オフセット

float de_ba=3000.0f;//代入用の?��?速度の設?��?
float de_bv_start=500.0f;//代入用初�??��度の設?��?
float de_bv_max=500.0f;//代入用?��?高�??��?��??��設?��?
float de_bv_end=100.0f;//代入用終端速度の設?��?
float de_bx=90.0f;//代入用目標距離の設?��?

float ac_ba=3000.0f;//代入用の?��?速度の設?��?
float ac_bv_start=100.0f;//代入用初�??��度の設?��?
float ac_bv_max=500.0f;//代入用?��?高�??��?��??��設?��?
float ac_bv_end=500.0f;//代入用終端速度の設?��?
float ac_bx=90.0f;//代入用目標距離の設?��?

float gv=0.0;//重心速度
float bu_gv=0.0;//代入用重心速度
float W=84.0;//トレッド幅(+で左回転)
float wa=2850.0;//角加速度
float wv_start=114.0;//初角速度
float wv_max=456.0;//最高角速度
float wv_end=114.0;//終端角速度
float wx=90.0;//回転角度
int senkaivec;//旋回の向き(1で左回転、-1で右回転)
int isSenkai=0;//旋回するかどうか
int cwR;//右モーターの回転の向き(0が反時計回り)
int cwL;//左モーターの回転の向き(0が反時計回り)

float s_gv=500;//スラロームの重心速度
float r_s_wa=12000;//スラロームの角加速度
float r_s_wv_start=0.0;//スラロームの初角速度
float r_s_wv_max=480.0;//スラロームの最高角速度
float r_s_wv_end=0.0;//スラロームの終端角速度
float r_s_wx=92;//スラロームの回転角度

float l_s_wa=12000;//スラロームの角加速度
float l_s_wv_start=0.0;//スラロームの初角速度
float l_s_wv_max=480.0;//スラロームの最高角速度
float l_s_wv_end=0.0;//スラロームの終端角速度
float l_s_wx=92;//スラロームの回転角度


float M_ba=3000.0f;//最短の加速度
float M_bv_start=500.0f;//最短の初速度
float M_bv_max=1500.0f;//最短の最高速度
float bu_M_bv_max=1500.0f;//最短の最高速度1
float bu_M_bv_max_2=1300;//最短の最高速度2
float bu_M_bv_max_3=1000;//最短の最高速度3
float bu_M_bv_max_4=800;//最短の最高速度4
float M_bv_end=500.0f;//最短の終端速度
float M_bx=90.0;//最短の代入用目標距離

float kabegire_x=70;//壁切れの後に進む距離
float M_kabegire_x=70.0;//壁切れの後に進む距離
float M_not_kabegire_x=10.0;//壁切れが起きるまで進む距離

float M_OOmawari_Conect_v=700.0;//大回りの接続速度
float M_Komawari_Conect_v=800.0;//小回りの接続速度
float M_start_conect_v;//直進の接続初速度
float M_end_conect_v;//直進の接続終端速度

float Oo_gv=700;//大回りの重心速度
float r_Oo_wa=17000;//右大回りの角加速度
float r_Oo_wv_start=0.0;//右大回りの初角速度
float r_Oo_wv_max=480.0;//右大回りの最高角速度
float r_Oo_wv_end=0.0;//右大回りの終端角速度
float r_Oo_wx=168;//右大回りの回転角度
float r_Oo_front_offset=60.0;//右大回り前オフセット
float r_Oo_back_offset=71.0;//右大回り後オフセット

float l_Oo_wa=17000;//左大回りの角加速度
float l_Oo_wv_start=0.0;//左大回りの初角速度
float l_Oo_wv_max=480.0;//左大回りの最高角速度
float l_Oo_wv_end=0.0;//左大回りの終端角速度
float l_Oo_wx=168;//左大回りの回転角度
float l_Oo_front_offset=60.0;//左大回り前オフセット
float l_Oo_back_offset=66.0;//左大回り後オフセット

float Ko_gv=800;//右小回りの重心速度
float r_Ko_wa=12000;//右小回りの角加速度
float r_Ko_wv_start=0.0;//右小回りの初角速度
float r_Ko_wv_max=380.0;//右小回りの最高角速度
float r_Ko_wv_end=0.0;//右小回りの終端角速度
float r_Ko_wx=90;//右小回りの回転角度
float r_Ko_front_offset=30.0;//右小回り前オフセット
float r_Ko_back_offset=18.0;//右小回り後オフセット

float l_Ko_wa=12000;//左小回りの角加速度
float l_Ko_wv_start=0.0;//左小回りの初角速度
float l_Ko_wv_max=380.0;//左小回りの最高角速度
float l_Ko_wv_end=0.0;//左小回りの終端角速度
float l_Ko_wx=92;//左小回りの回転角度
float l_Ko_front_offset=30.0;//左小回り前オフセット
float l_Ko_back_offset=20.0;//左小回り後オフセット

float Conect_start_v_Log[20];
float Conect_end_v_Log[20];
int isNoKabeAte=0;//壁当てをしないか

//1daikeikasokuを初期化、加速度、初速度(角速度)、最高速度、終端速度、目標距離(角度)、壁制御するか、旋回するかどうか、旋回の向き(0なら後進)
void ResetKasoku(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int buisSenkai,int busenkaivec)
{
	if(buisSenkai==1)
	{
		DegToRad(&bua, &buv_start, &buv_max, &buv_end, &bux);
	}
	kasokuflag=1;
	kabeflag=isKabe;
	isSenkai=buisSenkai;
	senkaivec=busenkaivec;
	g_motorCount_l=10000;
	g_motorCount_r=10000;

	dx=0.0;
	a=bua;
	v_start=buv_start;
	v=buv_start;
	v_max=buv_max;
	v_end=buv_end;
	x=bux;
	xde=(v_max*v_max-v_end*v_end)/(2*a);
}

void Check_SRAROOMorSENKAI(int isSraroom)//2旋回かスラロームかどうかで重心速度を変える変数　1がスラローム、0が超信地旋回,-1が大回り、-2が小回り
{
	if(isSraroom==1)
	{
		gv=s_gv;
	}
	else if(isSraroom==0)
	{
		gv=bu_gv;
	}
	else if(isSraroom==-1)
	{
		gv=Oo_gv;
	}
	else if(isSraroom==-2)
	{
		gv=Ko_gv;
	}
}



//TIM6割り込み
void kasoku(void)
{

	if(kasokuflag==1)
	{
		if(v<v_max&&x-dx>xde)
		{
			dx+=v*dt;
			v+=a*dt;
		}
		else if(x-dx>xde)
		{
			dx+=v*dt;
		}
		else if(v>v_end)
		{
			dx+=v*dt;
			v-=a*dt;
		}
		else
		{
			kasokuflag=0;
		}


		float countR;
		float countL;
		float buvR;
		float buvL;
		if(isSenkai==0)
		{
			//左と右の速度の決定
			buvR=v;
			buvL=v;

			//壁kabeの処理
			if(kabeflag==1)
			{
				float PID_wall=calWallConrol();
				buvL+=PID_wall;
				buvR-=PID_wall;
			}

			//vによって正転逆転の決定
			if(buvR<0.0)
			{
				cwR=1;
			}
			else
			{
				cwR=0;
			}

			if(buvL<0.0)
			{
				cwL=0;
			}
			else
			{
				cwL=1;
			}

			//backかどうか
			if(senkaivec==0)
			{
				cwR=1;
				cwL=0;
			}
		}
		else
		{
			//左と右の速度の決定
			buvR=gv+W*senkaivec*v/2;
			buvL=gv-W*senkaivec*v/2;

			//vによって正転逆転の決定
			if(buvR<0.0)
			{
				cwR=1;
			}
			else
			{
				cwR=0;
			}

			if(buvL<0.0)
			{
				cwL=0;
			}
			else
			{
				cwL=1;
			}
		}

		if((fabs(buvR)>0.0) && (10000000*0.9*3.14*51/(180*2*fabs(buvR)) < UINT16_MAX) && (fabs(buvL)>0.0) && (10000000*0.9*3.14*51/(180*2*fabs(buvL)) < UINT16_MAX))
		{
			countR=10000000*0.9*3.14*Taiya/(180*2*fabs(buvR));
			countL=10000000*0.9*3.14*Taiya/(180*2*fabs(buvL));
		}
		else
		{
			countR=UINT16_MAX-1;
			countL=UINT16_MAX-1;
		}
		g_motorCount_l=(uint16_t)countL;
		g_motorCount_r=(uint16_t)countR;

	}
}

void DegToRad(float* bua, float* buv_start, float* buv_max, float* buv_end, float* bux)//度数からラジアンに変換する関数
{
	*bua=*bua/90.0*1.57;
	*buv_start=*buv_start/90.0*1.57;
	*buv_max=*buv_max/90.0*1.57;
	*buv_end=*buv_end/90.0*1.57;
	*bux=*bux/90.0*1.57;
}

void Conect_v_cal(int passcount)//直進の接続速度を計算
{
	if(passcount!=254)
	{
		if((pass[passcount]>0) && (pass[passcount+1]==-6 || pass[passcount+1]==-4))//1現在が直進、次が大回り
		{
			M_end_conect_v=M_OOmawari_Conect_v-500;
		}
		else if((pass[passcount]>0) && (pass[passcount+1]==-7 || pass[passcount+1]==-5))//2現在が直進、次が小回り
		{
			M_end_conect_v=M_Komawari_Conect_v-500;
		}
		else
		{
			M_end_conect_v=0.0;
		}
	}

	if(passcount!=0)
	{
		if((pass[passcount]>0) && (pass[passcount-1]==-6 || pass[passcount-1]==-4))//1現在が直進、一個前が大回り
		{
			M_start_conect_v=M_OOmawari_Conect_v-500;
		}
		else if((pass[passcount]>0) && (pass[passcount-1]==-7 || pass[passcount-1]==-5))//2現在が直進、一個前が小回り
		{
			M_start_conect_v=M_Komawari_Conect_v-500;
		}
		else
		{
			M_start_conect_v=0.0;
		}
	}
}

void Conect_v_Login(int passcount)
{
	if(passcount<20)
	{
		Conect_start_v_Log[passcount]=M_start_conect_v;
		Conect_end_v_Log[passcount]=M_end_conect_v;
	}
}

void Show_Conect_Log()
{
	for(int i=0;i<20;i++)
	{
		printf("start_conect_v=%f end_conect_v=%f\n\r",Conect_start_v_Log[i],Conect_end_v_Log[i]);
	}
}

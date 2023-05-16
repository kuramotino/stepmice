/*
 * Act_Pat.c
 *
 *  Created on: 2022/11/23
 *      Author: Ryu
 */

#include "Act_Pat.h"
#include "main.h"
#include "tim.h"
#include "mapping.h"
#include "PL_sensor.h"
#include "daikei.h"
#include "stm32l4xx_it.h"

void StopCheck()
{
	//stopボタンが押されたか
	if(StopButton()==1)
	{
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		isStop=1;
		kasokuflag=0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(500);
		//Load_Row_Column();//壁情報をロード
		mode=0;
	}
}

//1daikeikasokuを初期化、加速度、初速度(角速度)、最高速度、終端速度、目標距離(角度)、壁制御するか(-1なら壁切れを見る)、旋回の向き(0なら後進)、位置を更新するか
void Straight(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int busenkaivec,int posset)
{
	int isKabeGire=0;
	float pre_v=0.0;

	if(isKabe==-1)
	{
		ResetKasoku(bua,buv_start,buv_max,buv_end,bux,1,0,busenkaivec);
	}
	else
	{
		ResetKasoku(bua,buv_start,buv_max,buv_end,bux,isKabe,0,busenkaivec);
	}

	if(posset==1)
	{
		PosDecide();
	}

	while(kasokuflag==1)
	{
		StopCheck();
		if(isKabe==-1 && (kabe_inf[0]==1 && kabe_inf[2]==1))
		{
			isKabeGire = KabeGire_Check();//壁が切れたら走行を中断
			pre_v=v;
		}
	}

	if(isKabeGire==1)
	{
		ResetKasoku(bua,pre_v,pre_v,buv_end,kabegire_x,1,0,1);//1壁が切れたら一定距離前進
		while(kasokuflag==1)
		{
			StopCheck();
		}
	}
}

//aオフセット量、壁制御するか,歩数マップの更新を行うか、前壁制御するか,次の行動
void Go_Offset(float offset,int isKabe,int isDist,int isFront,int setnum)
{
	ResetKasoku(const_ba,const_bv_start,const_bv_max,const_bv_end,offset,isKabe,0,1);//frontoffset分直進
	int isstop_offset=0;

	if(isDist)
	{
		Re_Decide_Dist();//歩数マップ�?�更新
	}
	while(kasokuflag==1)
	{
		StopCheck();
		if(isFront)
		{
			Front_Offset_Stop(setnum,&isstop_offset);
		}
	}

	if(isstop_offset==0 && isFront==1)//1前オフセットが足りていないとき
	{
		ResetKasoku(const_ba,const_bv_start,const_bv_max,const_bv_end,add_offset,isKabe,0,1);//add_offset分直進
		while(kasokuflag==1)
		{
			StopCheck();
			if(isFront)
			{
				Front_Offset_Stop(setnum,&isstop_offset);
			}
		}
	}
}
//1daikeikasokuを初期化、加速度、初速度(角速度)、最高速度、終端速度、目標距離(角度)、壁制御するか、前壁制御するか、壁切れを見るか、ターンの種類
void Turn_Offset(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int isMaeKabe,int isKabeGire,int setnum)//大回りなど各種ターンのオフセット
{
	float pre_v=0.0;
	int isDoKabeGire=0;
	int isDoMaeKabe=0;
	int isKG=0;//0壁が切れたか
	int isstop_offset=0;

	Senser();//壁情報を取�?

	if(kabe_inf[0]==1 && kabe_inf[2]==1 && isKabeGire)
	{
		isDoKabeGire=1;
	}
	else if(kabe_inf[1]==1 && isMaeKabe)
	{
		isDoMaeKabe=1;
	}

	ResetKasoku(bua,buv_start,buv_max,buv_end,bux,isKabe,0,1);

	while(kasokuflag==1)
	{
		StopCheck();
		if(isDoKabeGire==1)
		{
			isKG=KabeGire_Check();
			pre_v=v;
		}

		if(isDoMaeKabe==1)
		{
			Front_Offset_Stop(setnum,&isstop_offset);
		}
	}

	if(isKG==1)//2壁が切れたら一定距離進む
	{
		ResetKasoku(bua,pre_v,pre_v,buv_end,bux-20,isKabe,0,1);
		while(kasokuflag==1)
		{
			StopCheck();
		}
	}

}

//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度
void Left_Slalom(float wa,float wv_start,float wv_max,float wv_end,float wx)
{
	Check_SRAROOMorSENKAI(1);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,1);
	VecDecide(2);
	PosDecide();
	while(kasokuflag==1)
	{
		StopCheck();
	}
}
//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度,大回り-1か小回り-2か
void Left_Oo_Ko(float wa,float wv_start,float wv_max,float wv_end,float wx,int Oo_Ko)
{
	Check_SRAROOMorSENKAI(Oo_Ko);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,1);
	VecDecide(2);
	PosDecide();
	while(kasokuflag==1)
	{
		StopCheck();
	}
}
//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度
void Right_Slalom(float wa,float wv_start,float wv_max,float wv_end,float wx)
{
	Check_SRAROOMorSENKAI(1);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,-1);
	VecDecide(1);
	PosDecide();
	while(kasokuflag==1)
	{
		StopCheck();
	}
}
//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度,大回り-1か小回り-2か
void Right_Oo_Ko(float wa,float wv_start,float wv_max,float wv_end,float wx,int Oo_Ko)
{
	Check_SRAROOMorSENKAI(Oo_Ko);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,-1);
	VecDecide(1);
	PosDecide();
	while(kasokuflag==1)
	{
		StopCheck();
	}
}
//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度
void Left_Senkai()
{
	Check_SRAROOMorSENKAI(0);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,1);
	VecDecide(2);
	while(kasokuflag==1)
	{
		StopCheck();
	}

}
//1daikeikasokuを初期化、角加速度、角速度、最高角速度、終端角速度、目標角度
void Right_Senkai()
{
	Check_SRAROOMorSENKAI(0);
	ResetKasoku(wa,wv_start,wv_max,wv_end,wx,0,1,-1);
	VecDecide(1);
	while(kasokuflag==1)
	{
		StopCheck();
	}
}

void U_turn()
{
	int isFront=kabe_inf[1];
	int isSide=1;
	if(kabe_inf[0]==0 && kabe_inf[2]==0)
	{
		isSide=0;
	}

	//90mm直進
	Straight(de_ba,de_bv_start,de_bv_max,de_bv_end,de_bx-front_offset,1,1,0);
	Stop_Delay_Start(1);
	Stop_Delay_Start(0);

	if(kabe_inf[2]==1)
	{
		//90度左回転
		Left_Senkai();
	}
	else
	{
		//90度右回転
		Right_Senkai();
	}

	Stop_Delay_Start(1);
	Stop_Delay_Start(0);

	if(isSide==1 && isFront==1)
	{
		//60mm後進
		Straight(ac_ba/2.0,ac_bv_start/4.0,ac_bv_max/4.0,de_bv_end/4.0,100,0,0,0);
		Stop_Delay_Start(1);

		//53.5mm前進
		Stop_Delay_Start(0);
		Straight(ac_ba,ac_bv_start,ac_bv_max,de_bv_end,55.5,0,1,0);
		Stop_Delay_Start(1);
	}

	Stop_Delay_Start(0);

	if(kabe_inf[2]==1)
	{
		//90度左回転
		Left_Senkai();
	}
	else
	{
		//90度右回転
		Right_Senkai();
	}
	PosDecide();
	Stop_Delay_Start(1);

	if(isFront==1)
	{
		//60mm後進
		Stop_Delay_Start(0);
		Straight(ac_ba/2.0,ac_bv_start/4.0,ac_bv_max/4.0,de_bv_end/4.0,100,0,0,0);
		Stop_Delay_Start(1);

		//115mm前進
		Stop_Delay_Start(0);
		Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);
	}
	else
	{
		//90mm前進
		Stop_Delay_Start(0);
		Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,de_bx,1,1,0);
	}

}

void Return_Stop()
{
	//90mm直進
	Straight(de_ba,de_bv_start,de_bv_max,de_bv_end,de_bx-front_offset,1,1,0);
	Stop_Delay_Start(1);
	Stop_Delay_Start(0);

	//90度左回転
	Left_Senkai();
	Stop_Delay_Start(1);
	Stop_Delay_Start(0);

	//60mm後進
	Straight(ac_ba/2.0,ac_bv_start/4.0,ac_bv_max/4.0,de_bv_end/4.0,100,0,0,0);
	Stop_Delay_Start(1);

	//53.5mm前進
	Stop_Delay_Start(0);
	Straight(ac_ba,ac_bv_start,ac_bv_max,de_bv_end,55.5,0,1,0);
	Stop_Delay_Start(1);


	Stop_Delay_Start(0);

	//90度左回転
	Left_Senkai();
	Stop_Delay_Start(1);

	//60mm後進
	Stop_Delay_Start(0);
	Straight(ac_ba/2.0,ac_bv_start/4.0,ac_bv_max/4.0,de_bv_end/4.0,100,0,0,0);
	Stop_Delay_Start(1);

	HAL_Delay(2500);//2.5s停止
}

int StartButton()
{
	//startボタンが押されたか
	if(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)
	{
		return 1;
	}
	return 0;
}


int StopButton()
{
	//stopボタンが押されたか
	if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)
	{
		return 1;
	}
	return 0;
}

void MotorStart()
{
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Slalom_test()
{
	for(int i=0;i<3;i++)
		  		{
		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,front_offset,0,1,0);//frontoffset分直進
		  			Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);//右スラローム
		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,r_back_offset,0,1,0);//backoffset分直進

		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,front_offset,0,1,0);//frontoffset分直進
		  			Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);//右スラローム
		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,r_back_offset,0,1,0);//backoffset分直進

		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,front_offset,0,1,0);//frontoffset分直進
		  			Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);//左スラローム
		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,l_back_offset,0,1,0);//backoffset分直進

		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,front_offset,0,1,0);//frontoffset分直進
		  			Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);//左スラローム
		  			Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,l_back_offset,0,1,0);//backoffset分直進
		  		}

		  		Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,180,0,1,1);//1 180mm直進
}

void MotorStop()
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
}

void Show_sensor_log()
{
	if(flag_log==1)
	{
		float t=0.0;
		for(int i=0;i<2500;i++)
		{
			printf("%d %d %d %d %d\n\r", t,log_sensor_1[i],log_sensor_2[i],log_sensor_3[i],log_sensor_4[i]);
			t+=0.003;
		}
	}
}

void Stop_Delay_Start(int isStop)//一時的にモータを止めたり動かす関数
{
	if(isStop==1)
	{
		//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		HAL_Delay(100);
		HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
	else
	{
		//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		MotorStart();
	}
}




/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "wait_ms.h"
#include "lcd_cmd.h"
#include "PL_sensor.h"
#include "daikei.h"
#include "stm32l4xx_it.h"
#include "mapping.h"
#include "Act_Pat.h"
#include "PL_sound.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  pl_timer_init();


  //壁kabeSeigyoの設�?
  SENSOR_GAIN=0.25;
  CENTER_R=145;//中心の時の右AD値
  CENTER_L=280;//中心の時の左AD値
  CENTER_F_R=158.0;
  CENTER_F_L=180.0;
  Rsla_CENTER_F_R=150.0;//右スラロームの前オフセット前進時の前壁AD値
  Rsla_CENTER_F_L=190.0;//右スラロームの前オフセット前進時の前壁AD値
  Lsla_CENTER_F_R=140.0;//左スラロームの前オフセット前進時の前壁AD値
  Lsla_CENTER_F_L=180.0;//左スラロームの前オフセット前進時の前壁AD値
  R_Oo_CENTER_F_R=50;//右大回りの前オフセット前進時の前壁AD値
  R_Oo_CENTER_F_L=90;//
  L_Oo_CENTER_F_R=70;//左大回りの前オフセット前進時の前壁AD値
  L_Oo_CENTER_F_L=72;//
  R_Ko_CENTER_F_R=30;//右小回りの前オフセット前進時の前壁AD値
  R_Ko_CENTER_F_L=72;//
  L_Ko_CENTER_F_R=40;//左小回りの前オフセット前進時の前壁AD値
  L_Ko_CENTER_F_L=48;//
  FRONT_GAIN=0.5;
  M_GAIN=0.30;
  THRESHOLD_R=100.0;
  THRESHOLD_L=150.0;
  THRESHOLD_F_R=50.0;
  THRESHOLD_F_L=50.0;
  THRESHOLD_F_R_2=70.0;
  THRESHOLD_F_L_2=70.0;
  THRESHOLD_DIFE_R=70.0;
  THRESHOLD_DIFE_L=70.0;
  kabe_cut_count[0]=1;
  kabe_cut_count[1]=1;
  kabe_cut_counter[0]=kabe_cut_count[0]+1;
  kabe_cut_counter[1]=kabe_cut_count[1]+1;


  Pos[0]=0;//位置の初期化x
  Pos[1]=1;//位置の初期化y
  MiceVec=1;//向きの初期�?
  PreMiceVec=1;//前回の機体�?�向きの初期�?
  Column[0][0]=1;//初期位置は�?ず左壁がある

  senkaivec=1;//左回転にする(0だと後進してしまうため)

  mode=0;//モード選択
  int now_mode=1;//現在のモード
  int setnum=0;

  int passcount=0;//passのカウント

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_init();//LCDの初期設定
  HAL_TIM_Base_Start_IT(&htim1);//motor
  HAL_TIM_PWM_MspInit(&htim1);//motor
  HAL_TIM_Base_Start_IT(&htim2);//motor
  HAL_TIM_PWM_MspInit(&htim2);//motor
  HAL_TIM_Base_Start_IT(&htim15);//speaker
  HAL_TIM_PWM_MspInit(&htim15);//speaker



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(mode)
	  {
	  	  case 0://mode選択

	  		  //startボタンがおされたら動作開始
	  		  if(isStart==0 && isStop==0 && StartButton()==1)
	  		  {
	  			HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_SET);
	  			sound(500,1000);
	  			HAL_GPIO_WritePin(INTERFACELED_GPIO_Port,INTERFACELED_Pin,GPIO_PIN_RESET);
	  			mode=now_mode;
	  		  }

	  		  //MEIRO情報を出力&スタートストップフラグをリセット
	  		  if(isStart==1 && isStop==1 && StopButton()==1)
	  		  {
	  			  sound(350,300);
	  			  ShowMap();
	  			  ShowAdatiMap();
	  			  Show_M_AdatiMap();
	  			  TestBattAVE();
	  			  Show_sensor_log();
	  			  Show_Pass();
	  			  //SetOffsetLog(0,0,1);
	  			  Show_Conect_Log();
	  			  isStart=0;
	  			  isStop=0;
	  			  isGoal=0;
	  		  }

	  		  //stopボタンを押してモード切替
	  		  if(isStart==0 && isStop==0 && StopButton()==1)
	  		  {
	  			  if(now_mode<20)
	  			  {
	  				  sound(250,200);
	  				  now_mode++;
	  			  }
	  			  else
	  			  {
	  				  sound(250,200);
	  				  now_mode=1;
	  			  }
	  		  }

	  		  //now_modeによってLCDの表示を変更
	  		  if(now_mode==1 && isStart==0 && isStop==0)
	  		  {
	  			  float batt = TestBatt();
	  			  char strBuffer[17] = {0};
	  			  sprintf(strBuffer, "B=%04f", batt+2.0);
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts(strBuffer);
	  		  }
	  		  else if(isStart==1 && isStop==1)
	  		  {
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts("END");
	  		  }
	  		  else if(now_mode==2)
	  		  {
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts("L slalome");
	  		  }
	  		  else if(now_mode==3)
	  		  {
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts("R slalome");
	  		  }
	  		  else if(now_mode==4)
	  		  {
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts("Saitan mode");
	  		  }
	  		else if(now_mode==5)
	  		{
	  			  lcd_clear();
	  			  lcd_pos(10, 1);
	  			  lcd_puts("Sensor mode");
	  		}
	  		else if(now_mode==6)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("1500 test mode");
	  		}
	  		else if(now_mode==7)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("L_Oo mode");
	  		}
	  		else if(now_mode==8)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("R_Oo mode");
	  		}
	  		else if(now_mode==9)
	  		{
	  			 lcd_clear();
	  			 lcd_pos(10, 1);
	  			 lcd_puts("L_Ko mode");
	  		}
	  		else if(now_mode==10)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("R_Ko mode");
	  		}
	  		else if(now_mode==11)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("Kabegire mode");
	  		}
	  		else if(now_mode==12)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("L_Maekabe mode");
	  		}
	  		else if(now_mode==13)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("R_Maekabe mode");
	  		}
	  		else if(now_mode==14)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("Yokokabe mode");
	  		}
	  		else if(now_mode==15)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("1300 mode");
	  		}
	  		else if(now_mode==16)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("1000 mode");
	  		}
	  		else if(now_mode==17)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("800 mode");
	  		}
	  		else if(now_mode==18)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("1500 mode");
	  		}
	  		else if(now_mode==19)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("no OO_1000 mode");
	  		}
	  		else if(now_mode==20)
	  		{
	  			lcd_clear();
	  			lcd_pos(10, 1);
	  			lcd_puts("no KabeAte mode");
	  		}

	  		  break;

	  	  case 1://normalモード

	  		  if(isStart==0)//初めに115mm前進
	  		  {
	  			Pos[0]=0;//位置の初期化x
	  			Pos[1]=1;//位置の初期化y
	  			MiceVec=1;//向きの初期�?
	  			PreMiceVec=1;//前回の機体�?�向きの初期�?
	  			Init_Row_Column();
	  			MotorStart();
	  			Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);
	  			isStart=1;
	  			Init_M_Row_Column();//最短経路用壁配列の初期化
	  		  }

	  		  Senser();//壁情報を取�?
	  		  MapDecide();//壁情報と位置�?報と向き�?報から周囲の壁を決�?
	  		  GoalCheck();//ゴールしたかどうかを判定

	  		  //1区画の中心に来たときスラロームオフセットの補正量を求める
	  		  bu_front_offset=0.0;
	  		  bu_back_offset=0.0;
	  		  //bu_front_offset=Front_Wall_Offset();
	  		  bu_back_offset=Side_Wall_Offset();


	  		  if(isGoal==0)
	  		  {
	  			  //20mm直進
	  			  Go_Offset(front_offset,1,1,0,setnum);//frontoffset分直進
	  			//a歩数マップ�?�更新
	  		  }
	  		  else if(isReturn==0)
	  		  {
	  			  //goalしたら再出発する
	  			  //180mm直進
	  			  Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,180,-1,1,1);
	  			  Senser();//壁情報を取�?
	  			  MapDecide();//壁情報と位置�?報と向き�?報から周囲の壁を決�?
	  			  U_turn();//Uターン
	  			  isReturn=1;
	  			  isGoal=0;
	  			  //Save_Row_Column();//壁情報をセーブ

	  			  break;
	  		  }
	  		  else
	  		  {
	  			  //start地点に戻ったら最短開始
	  			  Return_Stop();
	  			  sound(350,500);
	  			  isStart=0;
	  			  isGoal=0;
	  			  isReturn=0;
	  			  mode=4;
	  			  break;
	  		  }

	  		  setnum=Adati();//足立法で次の行動を決�?
	  		  if(setnum==0)
	  		  {
	  			  //0左に曲がる
	  			  SetOffsetLog(1,bu_front_offset,0);
	  			  SetOffsetLog(0,bu_back_offset,0);
	  			  Go_Offset(l_front_offset-front_offset+bu_front_offset,0,0,1,setnum);//front_offset直進
	  			  Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);
	  			  Go_Offset(l_back_offset+bu_back_offset,0,0,0,setnum);//backoffset直進
	  		  }
	  		  else if(setnum==1)
	  		  {
	  			  //1 180mm直進
	  			  Straight(const_ba,const_bv_start,const_bv_max,const_bv_end,const_bx,-1,1,1);
	  		  }
	  		  else if(setnum==2)
	  		  {
	  			  //2右に曲がる
	  			  SetOffsetLog(1,bu_front_offset,0);
	  			  SetOffsetLog(0,bu_back_offset,0);
	  			  Go_Offset(r_front_offset-front_offset+bu_front_offset,0,0,1,setnum);//frontoffset分直進
	  			  Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);
	  			  Go_Offset(r_back_offset+bu_back_offset,0,0,0,setnum);//backoffset分直進
	  		  }
	  		  else if(setnum==3)
	  		  {
	  			  //3Uターン
	  			  U_turn();
	  		  }

	  		  break;

	  	  case 2://Lスラロームモード

	  		if(isStart==0)//初めに295mm前進
	  		{
	  			 MotorStart();
	  			 //Straight(ac_ba,ac_bv_start,1500,de_bv_end,295,1,1,1);
	  			 Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,295,1,1,1);
	  			 isStart=1;
	  		}

	  		for(int i=0;i<1;i++)
	  		{
	  			Go_Offset(front_offset,1,1,0,setnum);//frontoffset分直進
	  			Go_Offset(l_front_offset-front_offset,0,0,0,setnum);//1左右オフセット前進
	  			Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);//左スラローム
	  			Go_Offset(l_back_offset,0,0,0,setnum);//backoffset分直進

	  		}

	  		Straight(de_ba,de_bv_start,de_bv_max,de_bv_end,180,0,1,1);//1 180mm直進*/

	  		MotorStop();
	  		isStop=1;
	  		mode=0;

	  		  break;

	  	  case 3://Rスラロームモード
	  		if(isStart==0)//初めに295mm前進
	  		{
	  			 MotorStart();
	  			 Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,295,1,1,1);
	  			 isStart=1;
	  		}

	  		for(int i=0;i<1;i++)
	  		{
	  			 Go_Offset(front_offset,1,1,0,setnum);//frontoffset分直進
	  			 Go_Offset(r_front_offset-front_offset,0,0,0,setnum);//1左右オフセット前進
	  			 Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);//右スラローム
	  			 Go_Offset(r_back_offset,0,0,0,setnum);//backoffset分直進

	  		}

	  		Straight(de_ba,de_bv_start,de_bv_max,de_bv_end,180,0,1,1);//1 180mm直進*/

	  		MotorStop();
	  		isStop=1;
	  		mode=0;

	  		  break;

	  	  case 4://saitan走行mode

	  		if(isStart==0)//初めに115mm前進
	  		{
	  			passcount=0;
	  			Taiya=M_1500_Taiya;
	  			M_Pass();
	  			MotorStart();
	  			Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);
	  			isStart=1;
	  			pass[0]=-100;
	  			//M_bv_max=bu_M_bv_max;
	  		}

	  		if(pass[passcount]>=0)//1直進
	  		{
	  			if(pass[passcount]==0)//passの最後尾のとき
	  			{
	  				//goalしたとき止める
	  				//180mm直進
	  				Taiya=bu_Taiya;
	  				isStop=1;
	  				//Straight(de_ba,de_bv_start,de_bv_max,de_bv_end,180,1,1,0);
	  				Taiya=bu_Taiya;
	  				MotorStop();
	  				mode=0;
	  				break;
	  			}
	  			else
	  			{
	  				//1一定距離前進
	  				Conect_v_cal(passcount);//直進の接続速度を計算
	  				Conect_v_Login(passcount);//接続速度のログ格納
	  				float add_v=(M_bv_max - 800)/14*pass[passcount];
	  				if(add_v>=700)
	  				{
	  					add_v=700;
	  				}
	  				if(add_v<=100)
	  				{
	  					add_v=0;
	  				}
	  				ResetKasoku(M_ba,M_bv_start+M_start_conect_v,800 + add_v,M_bv_end+M_end_conect_v,M_bx*pass[passcount],1,0,1);

	  				while(kasokuflag==1)
	  				{
	  					StopCheck();
	  				}

	  			}
	  		}
	  		else if(pass[passcount]==-2)//2左スラローム
	  		{
	  			Senser();//壁情報を取�?
	  			bu_back_offset=0.0;
	  			bu_back_offset=Side_Wall_Offset();
	  			setnum=0;

	  			Go_Offset(l_front_offset,1,0,1,setnum);//front_offset直進
	  			Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);
	  			Go_Offset(l_back_offset+bu_back_offset,0,0,0,setnum);//backoffset直進
	  		}
	  		else if(pass[passcount]==-3)//3右スラローム
	  		{
	  			Senser();//壁情報を取�?
	  			bu_back_offset=0.0;
	  			bu_back_offset=Side_Wall_Offset();
	  			setnum=2;

	  			Go_Offset(r_front_offset,1,0,1,setnum);//frontoffset分直進
	  			Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);
	  			Go_Offset(r_back_offset+bu_back_offset,0,0,0,setnum);//backoffset分直進
	  		}
	  		else if(pass[passcount]==-4)//4左大回り
	  		{
	  			//sound(500,300);//debug

	  			//Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,l_Oo_front_offset,1,1,0);//frontoffset分直進、壁が切れたら60-28進む
	  			Turn_Offset(const_ba,Oo_gv,Oo_gv,Oo_gv,l_Oo_front_offset,1,0,1,-4);
	  			Left_Oo_Ko(l_Oo_wa,l_Oo_wv_start,l_Oo_wv_max,l_Oo_wv_end,l_Oo_wx,-1);
	  			Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,l_Oo_back_offset,1,1,0);//backoffset分直進
	  		}
	  		else if(pass[passcount]==-6)//5右大回り
	  		{
	  			//sound(500,300);//debug

	  			//Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,r_Oo_front_offset,1,1,0);//frontoffset分直進、壁が切れたら60-28進む
	  			Turn_Offset(const_ba,Oo_gv,Oo_gv,Oo_gv,r_Oo_front_offset,1,0,1,-6);
	  			Right_Oo_Ko(r_Oo_wa,r_Oo_wv_start,r_Oo_wv_max,r_Oo_wv_end,r_Oo_wx,-1);
	  			Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,r_Oo_back_offset,1,1,0);//backoffset分直進
	  		}
	  		else if(pass[passcount]==-5)//6左小回り
	  		{
	  			 //sound(500,300);//debug

	  			 //Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,l_Ko_front_offset,1,1,0);//frontoffset分直進、壁が切れたら30-28進む
	  			 Turn_Offset(const_ba,Ko_gv,Ko_gv,Ko_gv,l_Ko_front_offset,1,0,1,-5);
	  			 Left_Oo_Ko(l_Ko_wa,l_Ko_wv_start,l_Ko_wv_max,l_Ko_wv_end,l_Ko_wx,-2);
	  			 Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,l_Ko_back_offset,1,1,0);//backoffset分直進
	  		}
	  		else if(pass[passcount]==-7)//7右小回り
	  		{
	  			 //sound(500,300);//debug

	  			 //Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,r_Ko_front_offset,1,1,0);//frontoffset分直進、壁が切れたら40-28進む
	  			 Turn_Offset(const_ba,Ko_gv,Ko_gv,Ko_gv,r_Ko_front_offset,1,0,1,-7);
	  			 Right_Oo_Ko(r_Ko_wa,r_Ko_wv_start,r_Ko_wv_max,r_Ko_wv_end,r_Ko_wx,-2);
	  			 Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,r_Ko_back_offset,1,1,0);//backoffset分直進
	  		}

	  		passcount++;

	  		  break;

	  	  case 5:
	  		flag_log=0;//sensor_logをとるかを決める
	  		while(1)
	  		{
	  			  pl_print();
	  			  HAL_Delay(100);
	  			  //StopCheck();
	  		}
	  		  break;

	  	  case 6://1500のタイヤ径調整
	  		if(isStart==0)//初めに2635mm前進
	  		{
	  			 MotorStart();
	  			 Taiya=M_1500_Taiya;
	  			 Straight(ac_ba,ac_bv_start,1500,de_bv_end,2635,1,1,1);
	  			 isStart=1;
	  		}
	  			Taiya=bu_Taiya;
	  			MotorStop();
	  			isStop=1;
	  			mode=0;
	  		  break;

	  	  case 7://7左大回りモード
	  		if(isStart==0)//初めに205mm前進
	  		{
	  			 MotorStart();
	  			 Taiya=M_1500_Taiya;
	  			 Straight(ac_ba,ac_bv_start,Oo_gv,Oo_gv,205,1,1,1);
	  			 isStart=1;
	  		}

	  		Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,l_Oo_front_offset,1,1,0);//frontoffset分直進
	  		Left_Oo_Ko(l_Oo_wa,l_Oo_wv_start,l_Oo_wv_max,l_Oo_wv_end,l_Oo_wx,-1);
	  		Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,l_Oo_back_offset,1,1,0);//backoffset分直進

	  		Straight(ac_ba,Oo_gv,Oo_gv,de_bv_end,90,0,1,0);//90前進

	  		Taiya=bu_Taiya;
	  		MotorStop();
	  		isStop=1;
	  		mode=0;
	  		  break;

	  	  case 8://8右大回りモード
	  		if(isStart==0)//初めに205mm前進
	  		{
	  			MotorStart();
	  			Taiya=M_1500_Taiya;
	  			Straight(ac_ba,ac_bv_start,Oo_gv,Oo_gv,205,1,1,1);
	  			isStart=1;
	  		}

	  		Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,r_Oo_front_offset,1,1,0);//frontoffset分直進
	  		Right_Oo_Ko(r_Oo_wa,r_Oo_wv_start,r_Oo_wv_max,r_Oo_wv_end,r_Oo_wx,-1);
	  		Straight(const_ba,Oo_gv,Oo_gv,Oo_gv,r_Oo_back_offset,1,1,0);//backoffset分直進

	  		Straight(ac_ba,Oo_gv,Oo_gv,de_bv_end,90,0,1,0);//90前進

	  		Taiya=bu_Taiya;
	  		MotorStop();
	  		isStop=1;
	  		mode=0;
	  		  break;

	  	  case 9://9左小回りモード
	  		if(isStart==0)//初めに205mm前進
	  		{
	  			MotorStart();
	  			Taiya=M_1500_Taiya;
	  			Straight(ac_ba,ac_bv_start,500,500,115,1,1,1);
	  			isStart=1;
	  		}
	  		Straight(ac_ba,500,800,800,90,1,1,0);
	  		Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,l_Ko_front_offset,1,1,0);//frontoffset分直進
	  		Left_Oo_Ko(l_Ko_wa,l_Ko_wv_start,l_Ko_wv_max,l_Ko_wv_end,l_Ko_wx,-2);
	  		Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,l_Ko_back_offset,1,1,0);//backoffset分直進

	  		Straight(ac_ba,Ko_gv,Ko_gv,de_bv_end,90,0,1,0);//90前進

	  		Taiya=bu_Taiya;
	  		MotorStop();
	  		isStop=1;
	  		mode=0;
	  		  break;

	  	  case 10://10右小回りモード
	  		if(isStart==0)//初めに205mm前進
	  		{
	  			MotorStart();
	  			Taiya=M_1500_Taiya;
	  			Straight(ac_ba,ac_bv_start,500,500,115,1,1,1);
	  			isStart=1;
	  		}
	  		Straight(ac_ba,500,800,800,90,1,1,0);
	  		Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,r_Ko_front_offset,1,1,0);//frontoffset分直進
	  		Right_Oo_Ko(r_Ko_wa,r_Ko_wv_start,r_Ko_wv_max,r_Ko_wv_end,r_Ko_wx,-2);
	  		Straight(const_ba,Ko_gv,Ko_gv,Ko_gv,r_Ko_back_offset,1,1,0);//backoffset分直進

	  		Straight(ac_ba,Ko_gv,Ko_gv,de_bv_end,90,0,1,0);//90前進

	  		Taiya=bu_Taiya;
	  		MotorStop();
	  		isStop=1;
	  		mode=0;
	  		  break;

	  	  case 11://11壁切れの調整モード
	  		  //180mm直進
	  		if(isStart==0)
	  		{
	  			 MotorStart();
	  			 isStart=1;
	  		}
	  		  Straight(const_ba,ac_bv_start,const_bv_max,const_bv_max,115,1,1,1);
	  		  Senser();
	  		  Straight(const_ba,const_bv_max,const_bv_max,de_bv_end,180,-1,1,1);
	  		MotorStop();
	  			  		isStop=1;
	  			  		mode=0;
	  		  break;

	  	  case 12://12左前壁制御の調整モード
	  		if(isStart==0)
	  			  		{
	  			  			 MotorStart();
	  			  			 isStart=1;
	  			  		}
	  		Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);//115前進
	  		//20mm直進
	  		//Go_Offset(front_offset,1,1,0,0);//frontoffset分直進
	  		Go_Offset(l_front_offset,0,0,1,0);//front_offset直進
	  		Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);//左スラローム
	  		MotorStop();
	  			  		isStop=1;
	  			  		mode=0;
	  		  break;

	  	  case 13://13右前壁制御の調整モード
	  		if(isStart==0)
	  			  		{
	  			  			 MotorStart();
	  			  			 isStart=1;
	  			  		}
	  		Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);//115前進
	  		//20mm直進
	  		//Go_Offset(front_offset,1,1,0,2);//frontoffset分直進
	  		Go_Offset(r_front_offset,0,0,1,2);//frontoffset分直進
	  		Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);//右スラローム
	  		MotorStop();
	  			  			  		isStop=1;
	  			  			  		mode=0;
	  		  break;

	  	  case 14://14横壁制御の調整モード
	  		if(isStart==0)
	  			  		{
	  			  			 MotorStart();
	  			  			 isStart=1;
	  			  		}
	  		Straight(ac_ba,ac_bv_start,ac_bv_max,ac_bv_end,115,1,1,0);//115前進
	  		Senser();//壁情報を取�?
	  		//1区画の中心に来たときスラロームオフセットの補正量を求める
	  		bu_back_offset=0.0;
	  		bu_back_offset=Side_Wall_Offset();

	  		if(kabe_inf[0]==1 && kabe_inf[2]==0)
	  		{
	  			Go_Offset(r_front_offset,1,1,0,2);//frontoffset分直進
	  			Right_Slalom(r_s_wa,r_s_wv_start,r_s_wv_max,r_s_wv_end,r_s_wx);//右スラローム
	  			Go_Offset(r_back_offset+bu_back_offset,0,0,0,2);//backoffset分直進
	  		}
	  		else if(kabe_inf[0]==0 && kabe_inf[2]==1)
	  		{
	  			Go_Offset(l_front_offset,1,1,0,1);//frontoffset分直進
	  			Left_Slalom(l_s_wa,l_s_wv_start,l_s_wv_max,l_s_wv_end,l_s_wx);//左スラローム
	  			Go_Offset(l_back_offset+bu_back_offset,0,0,0,0);//backoffset直進
	  		}
	  		MotorStop();
	  			  		isStop=1;
	  			  		mode=0;
	  		  break;

	  	  case 15://1300の最短
	  		  M_bv_max=bu_M_bv_max_2;
	  		  mode=4;
	  		  break;

	  	  case 16://1000の最短
	  		  M_bv_max=bu_M_bv_max_3;
	  		  mode=4;
	  		  break;

	  	  case 17://800の最短
	  		  M_bv_max=bu_M_bv_max_4;
	  		  mode=4;
	  		  break;

	  	  case 18://1500の最短
	  		  M_bv_max=bu_M_bv_max;
	  		  mode=4;
	  		  break;

	  	  case 19://1000大回りなし
	  		  M_bv_max=bu_M_bv_max_3;
	  		  mode=4;
	  		  break;

	  	  case 20://20壁当てなし
	  		  isNoKabeAte=1;
	  		  THRESHOLD_F_R=THRESHOLD_F_R_2;
	  		  THRESHOLD_F_L=THRESHOLD_F_L_2;
	  		  mode=1;
	  		  break;

	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 3;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

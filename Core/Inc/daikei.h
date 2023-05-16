/*
 * daikei.h
 *
 *  Created on: Oct 8, 2022
 *      Author: Ryu
 */

#ifndef INC_DAIKEI_H_
#define INC_DAIKEI_H_

extern int kasokuflag;//台形加速のフラッグ
extern int kabeflag;//壁制御のフラッグ
extern int isStart;
extern int isStop;
extern int isReturn;
extern int isMotorStop;//モータをストップさせるか
extern uint16_t g_motorCount_l;
extern uint16_t g_motorCount_r;
extern float dx;//位置の変位
extern float a;//加速度
extern float v_start;//初速度
extern float v;//現在の速度
extern float v_max;//最大速度
extern float v_end;//終端速度
extern float x;//目標距離
extern float xde;//減速距離
extern float Taiya;//タイヤ径
extern float bu_Taiya;//代入用のタイヤ径
extern float M_1500_Taiya;//最短1.5mのタイヤ径
extern int testcode;

extern float const_ba;//定速区間の加速度
extern float const_bv_start;//定速区間の初速度
extern float const_bv_max;//定速区間の最高速度
extern float const_bv_end;//低速区間の終端速度
extern const float front_offset;//前オフセット
extern float l_front_offset;//左前オフセット
extern float r_front_offset;//右前オフセット
extern float const_bx;//定速区間の代入用目標距離
extern float r_back_offset;//右の後オフセット
extern float l_back_offset;//左の後オフセット
extern float bu_front_offset;//代入用前オフセット
extern float bu_back_offset;//代入用後オフセット
extern float max_offset;//オフセットの最大調整量
extern float add_offset;//足りないときに加える前オフセット

extern float de_ba;//代入用の?��?速度の設?��?
extern float de_bv_start;//代入用初�??��度の設?��?
extern float de_bv_max;//代入用?��?高�??��?��??��設?��?
extern float de_bv_end;//代入用終端速度の設?��?
extern float de_bx;//代入用目標距離の設?��?

extern float ac_ba;//代入用の?��?速度の設?��?
extern float ac_bv_start;//代入用初�??��度の設?��?
extern float ac_bv_max;//代入用?��?高�??��?��??��設?��?
extern float ac_bv_end;//代入用終端速度の設?��?
extern float ac_bx;//代入用目標距離の設?��?

extern float gv;//重心速度
extern float bu_gv;//代入用重心速度
extern float W;//トレッド幅(+で左回転)
extern float wa;//角加速度
extern float wv_start;//初角速度
extern float wv_max;//最高角速度
extern float wv_end;//終端角速度
extern float wx;//回転角度
extern int senkaivec;//旋回の向き(1で左回転、-1で右回転)
extern int isSenkai;//旋回するかどうか
extern int cwR;//右モーターの回転の向き(0が反時計回り)
extern int cwL;//左モーターの回転の向き(0が反時計回り)

extern float s_gv;//スラロームの重心速度
extern float r_s_wa;//スラロームの角加速度
extern float r_s_wv_start;//スラロームの初角速度
extern float r_s_wv_max;//スラロームの最高角速度
extern float r_s_wv_end;//スラロームの終端角速度
extern float r_s_wx;//スラロームの回転角度

extern float l_s_wa;//スラロームの角加速度
extern float l_s_wv_start;//スラロームの初角速度
extern float l_s_wv_max;//スラロームの最高角速度
extern float l_s_wv_end;//スラロームの終端角速度
extern float l_s_wx;//スラロームの回転角度

extern float M_ba;//最短の加速度
extern float M_bv_start;//最短の初速度
extern float M_bv_max;//最短の最高速度
extern float bu_M_bv_max;//最短の最高速度1
extern float bu_M_bv_max_2;//最短の最高速度2
extern float bu_M_bv_max_3;//最短の最高速度3
extern float bu_M_bv_max_4;//最短の最高速度4
extern float M_bv_end;//最短の終端速度
extern float M_bx;//最短の代入用目標距離

extern float kabegire_x;//壁切れの後に進む距離
extern float M_kabegire_x;//壁切れの後に進む距離
extern float M_not_kabegire_x;//壁切れが起きるまで進む距離

extern float M_OOmawari_Conect_v;//大回りの接続速度
extern float M_Komawari_Conect_v;//小回りの接続速度
extern float M_start_conect_v;//直進の接続初速度
extern float M_end_conect_v;//直進の接続終端速度

extern float Oo_gv;//大回りの重心速度
extern float r_Oo_wa;//右大回りの角加速度
extern float r_Oo_wv_start;//右大回りの初角速度
extern float r_Oo_wv_max;//右大回りの最高角速度
extern float r_Oo_wv_end;//右大回りの終端角速度
extern float r_Oo_wx;//右大回りの回転角度
extern float r_Oo_front_offset;//右大回り前オフセット
extern float r_Oo_back_offset;//右大回り後オフセット

extern float l_Oo_wa;//左大回りの角加速度
extern float l_Oo_wv_start;//左大回りの初角速度
extern float l_Oo_wv_max;//左大回りの最高角速度
extern float l_Oo_wv_end;//左大回りの終端角速度
extern float l_Oo_wx;//左大回りの回転角度
extern float l_Oo_front_offset;//左大回り前オフセット
extern float l_Oo_back_offset;//左大回り後オフセット

extern float Ko_gv;//右小回りの重心速度
extern float r_Ko_wa;//右小回りの角加速度
extern float r_Ko_wv_start;//右小回りの初角速度
extern float r_Ko_wv_max;//右小回りの最高角速度
extern float r_Ko_wv_end;//右小回りの終端角速度
extern float r_Ko_wx;//右小回りの回転角度
extern float r_Ko_front_offset;//右小回り前オフセット
extern float r_Ko_back_offset;//右小回り後オフセット

extern float l_Ko_wa;//左小回りの角加速度
extern float l_Ko_wv_start;//左小回りの初角速度
extern float l_Ko_wv_max;//左小回りの最高角速度
extern float l_Ko_wv_end;//左小回りの終端角速度
extern float l_Ko_wx;//左小回りの回転角度
extern float l_Ko_front_offset;//左小回り前オフセット
extern float l_Ko_back_offset;//左小回り後オフセット

extern float Conect_start_v_Log[20];
extern float Conect_end_v_Log[20];
extern int isNoKabeAte;//壁当てをしないか

extern int mode;//モード

void ResetKasoku(float bua, float buv_start, float buv_max, float buv_end, float bux,int isKabe,int buisSenkai,int busenkaivec);
void Check_SRAROOMorSENKAI(int isSraroom);//2旋回かスラロームかどうかで重心速度を変える変数　1がスラローム、0が超信地旋回,-1が大回り、-2が小回り
void kasoku(void);
void DegToRad(float* bua, float* buv_start, float* buv_max, float* buv_end, float* bux);//度数からラジアンに変換する関数
void Conect_v_cal(int passcount);//直進の接続速度を計算
void Conect_v_Login(int passcount);//接続速度のログを格納する関数
void Show_Conect_Log();//接続速度を表示

#endif /* INC_DAIKEI_H_ */

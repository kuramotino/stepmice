/*
 * mapping.h
 *
 *  Created on: 2022/11/05
 *      Author: Ryu
 */

#ifndef INC_MAPPING_H_
#define INC_MAPPING_H_

#define MAX_QUEUE_NUM 50

extern int Pos[2];//0がx、1がy
extern int BuPosX[100];//PosXのバッファ
extern int BuPosY[100];//PosYのバッファ
extern int Row[15][16];//横列で左の添え字が下から何列目か、右の添え字が左から何番目か
extern int Column[15][16];//縦列で左の添え字が左から何列目か、右の添え字が下から何番目か
extern int MiceVec;//機体の向き
extern int PreMiceVec;//前回の機体の向き(1が上、2が右、3が下、4が左)
extern int MapSize;//マップのサイズ
extern int Dist_map[16][16];//歩数マップ
extern int goal_x[4];//ゴール座標X
extern int goal_y[4];//ゴール座標Y
extern int isGoal;//ゴールしたかどうか
extern int M_Row[15];//横列で左の添え字が下から何列目か、ビットが左から何番目か
extern int M_Column[15];//横列で左の添え字が左から何列目か、ビットが下から何番目か
//extern int bu_M_Row[15];//予備の横列で左の添え字が下から何列目か、ビットが左から何番目か
//extern int bu_M_Column[15];//予備の横列で左の添え字が左から何列目か、ビットが下から何番目か
extern int pass[255];//最短経路のpass
extern int OOmawari_R[4];//右大回り
extern int OOmawari_L[4];//左大回り
extern int Komawari_R[4];//右小回り
extern int Komawari_L[4];//左小回り

//pos_stack構造体
typedef struct POS_STACK
{
	int x;
	int y;
}POS;

//stack構造体
typedef struct STACK
{
	int head;//データの最前列
	int tail;//データの最後尾
	POS pos[MAX_QUEUE_NUM];//stackされているデータ
}STACK_T;

extern void VecDecide(int lotvec);//向き決め関数(1入力で右回転、2入力で左回転)
extern void PosDecide(void);//位置決め関数
extern void MapDecide(void);//マップ情報を更新する関数
extern void Init_Row_Column(void);//壁情報を初期化する関数
extern void ShowMap(void);//マップを表示する関数
extern void Init_Dist(void);//歩数マップの初期化を行う関数
extern void Re_Decide_Dist(void);//改良歩数マップの更新を行う関数
extern int Adati(void);//足立法アルゴリズムに則り移動
extern void ShowAdatiMap(void);//歩数マップの表示
extern void GoalCheck(void);//ゴールしたかどうかを判定する関数
void initStack(STACK_T *stack);//stackの初期化
void pushStack_walk(STACK_T *stack,POS input);//stackのpush
POS popStack_walk(STACK_T *stack);//stackのpop
extern void Init_M_Row_Column(void);//最短経路用壁配列の初期化
extern void M_Decide_Dist(void);//最短歩数マップの更新を行う関数
extern void M_Pass(void);//passに最短経路を記録する関数
extern void Comp_Pass(int *tar_pass,int pass_size,int setpassnum);//passを圧縮する関数
void St_Comp_Pass(void);//直線の圧縮
extern void Show_M_AdatiMap(void);//最短歩数マップの表示
extern void Show_Pass(void);//パスを表示
extern int M_Adati(void);//足立法アルゴリズムに則り移動
extern void Save_Row_Column(void);//壁情報をセーブ
extern void Load_Row_Column(void);//壁情報をロード


#endif /* INC_MAPPING_H_ */

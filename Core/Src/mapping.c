/*
 * mapping.c
 *
 *  Created on: 2022/11/05
 *      Author: Ryu
 */

#include "mapping.h"
#include "main.h"
#include "stdio.h"
#include "math.h"
#include "PL_sensor.h"
#include "daikei.h"
#include "PL_sound.h"


int Pos[2];//0がx、1がy
int BuPosX[100];//PosXのバッファ
int BuPosY[100];//PosYのバッファ
int BuPosCount=0;
int Row[15][16];//横列で左の添え字が下から何列目か、右の添え字が左から何番目か
int Column[15][16];//縦列で左の添え字が左から何列目か、右の添え字が下から何番目か
int MiceVec;//機体の向き(1が上、2が右、3が下、4が左)
int PreMiceVec;//前回の機体の向き(1が上、2が右、3が下、4が左)
int MapSize=16;//マップのサイズ
int Dist_map[16][16];//歩数マップ
int goal_x[4]={3,3,4,4};//ゴール座標X(左が最小値)
int goal_y[4]={0,1,0,1};//ゴール座標Y(左が最小値)
int isGoal=0;//ゴールしたかどうか
int M_Row[15];//横列で左の添え字が下から何列目か、ビットが左から何番目か
int M_Column[15];//横列で左の添え字が左から何列目か、ビットが下から何番目か
//int bu_M_Row[15];//予備の横列で左の添え字が下から何列目か、ビットが左から何番目か
//int bu_M_Column[15];//予備の横列で左の添え字が左から何列目か、ビットが下から何番目か
int pass[255];//最短経路のpass
int OOmawari_R[4]={1,-3,-3,1};//右大回り
int OOmawari_L[4]={1,-2,-2,1};//左大回り
int Komawari_R[4]={1,-3,1};//右小回り
int Komawari_L[4]={1,-2,1};//左小回り

void VecDecide(int lotvec)//向き決め関数(1入力で右回転、2入力で左回転)
{
	switch(MiceVec)
		{
		case 1:
			if(lotvec==1)
			{
				//1右回転
				MiceVec=2;
			}
			else
			{
				//2左回転
				MiceVec=4;
			}
			break;

		case 2:
			if(lotvec==1)
			{
				//1右回転
				MiceVec=3;
			}
			else
			{
				//2左回転
				MiceVec=1;
			}
			break;

		case 3:
			if(lotvec==1)
			{
				//1右回転
				MiceVec=4;
			}
			else
			{
				//2左回転
				MiceVec=2;
			}
			break;
		case 4:
			if(lotvec==1)
			{
				//1右回転
				MiceVec=1;
			}
			else
			{
				//2左回転
				MiceVec=3;
			}
			break;
		}
}

void PosDecide(void)//位置決め関数
{
	switch(MiceVec)
	{

		case 1:
			if(PreMiceVec==3)
			{

			}
			else if(PreMiceVec==4)
			{
				Pos[0]--;
				Pos[1]++;
			}
			else
			{
				Pos[1]++;
			}
			PreMiceVec=MiceVec;
		    break;

		case 2:
			if(PreMiceVec==4)
			{

			}
			else if(PreMiceVec==3)
			{
				Pos[0]++;
				Pos[1]--;
			}
			else
			{
				Pos[0]++;
			}
			PreMiceVec=MiceVec;
			break;

		case 3:
			if(PreMiceVec==1 || PreMiceVec==2)
			{

		    }
			else if(PreMiceVec==4)
			{
				Pos[0]--;
			}
			else
			{
				Pos[1]--;
			}
			PreMiceVec=MiceVec;
			break;

		case 4:
			if(PreMiceVec==1 || PreMiceVec==2)
			{

			}
			else if(PreMiceVec==3)
			{
				Pos[1]--;
			}
			else
			{
				Pos[0]--;
			}
			PreMiceVec=MiceVec;
			break;
	}
	//BuPosX[BuPosCount]=Pos[0];
	//BuPosY[BuPosCount]=Pos[1];
	//BuPosCount++;
}

void Init_Row_Column(void)//壁情報を初期化する関数
{
	for(int i=0;i<15;i++)
	{
		for(int j=0;j<16;j++)
		{
			Row[i][j]=0;
			Column[i][j]=0;
		}
	}
	Column[0][0]=1;
}

void MapDecide(void)//マップ情報を更新する関数
{
	for(int i=0;i<3;i++)
	{
		sotokabe_inf[i]=0;
	}
	//Xが端にいるか
	if(Pos[0]==0)
	{
		if(MiceVec==1)
		{
			//1左が外壁
			sotokabe_inf[0]=1;
		}
		else if(MiceVec==3)
		{
			//3右が外壁
			sotokabe_inf[2]=1;
		}
		else if(MiceVec==4)
		{
			//4前が外壁
			sotokabe_inf[1]=1;
		}
	}
	else if(Pos[0]==MapSize-1)
	{
		if(MiceVec==1)
		{
			//1右が外壁
			sotokabe_inf[2]=1;
		}
		else if(MiceVec==2)
		{
			//2前が外壁
			sotokabe_inf[1]=1;
		}
		else if(MiceVec==3)
		{
			//3左が外壁
			sotokabe_inf[0]=1;
		}
	}

	//yが端にいるか
	if(Pos[1]==0)
	{
		if(MiceVec==2)
		{
			//2右が外壁
			sotokabe_inf[2]=1;
		}
		else if(MiceVec==3)
		{
			//3前が外壁
			sotokabe_inf[1]=1;
		}
		else if(MiceVec==4)
		{
			//4左が外壁
			sotokabe_inf[0]=1;
		}
	}
	else if(Pos[1]==MapSize-1)
	{
		if(MiceVec==1)
		{
			//1前が外壁
			sotokabe_inf[1]=1;
		}
		else if(MiceVec==2)
		{
			//2左が外壁
			sotokabe_inf[0]=1;
		}
		else if(MiceVec==4)
		{
			//4右が外壁
			sotokabe_inf[2]=1;
		}
	}

	//map情報の更新
	if(MiceVec==1)
	{
		if(kabe_inf[0]==1&&sotokabe_inf[0]!=1)
		{
			Column[Pos[0]-1][Pos[1]]=1;
		}
		if(kabe_inf[1]==1&&sotokabe_inf[1]!=1)
		{
			Row[Pos[1]][Pos[0]]=1;
		}
		if(kabe_inf[2]==1&&sotokabe_inf[2]!=1)
		{
			Column[Pos[0]][Pos[1]]=1;
		}

		if(kabe_inf[0]==0&&sotokabe_inf[0]!=1)
		{
			M_Column[Pos[0]-1]=M_Column[Pos[0]-1] & (~(1<<Pos[1]));
		}
		if(kabe_inf[1]==0&&sotokabe_inf[1]!=1)
		{
			M_Row[Pos[1]]=M_Row[Pos[1]] & (~(1<<Pos[0]));
		}
		if(kabe_inf[2]==0&&sotokabe_inf[2]!=1)
		{
			M_Column[Pos[0]]=M_Column[Pos[0]] & (~(1<<Pos[1]));
		}
	}
	else if(MiceVec==2)
	{
		if(kabe_inf[0]==1&&sotokabe_inf[0]!=1)
		{
			Row[Pos[1]][Pos[0]]=1;
		}
		if(kabe_inf[1]==1&&sotokabe_inf[1]!=1)
		{
			Column[Pos[0]][Pos[1]]=1;
		}
		if(kabe_inf[2]==1&&sotokabe_inf[2]!=1)
		{
			Row[Pos[1]-1][Pos[0]]=1;
		}

		if(kabe_inf[0]==0&&sotokabe_inf[0]!=1)
		{
			M_Row[Pos[1]]=M_Row[Pos[1]] & (~(1<<Pos[0]));
		}
		if(kabe_inf[1]==0&&sotokabe_inf[1]!=1)
		{
			M_Column[Pos[0]]=M_Column[Pos[0]] & (~(1<<Pos[1]));
		}
		if(kabe_inf[2]==0&&sotokabe_inf[2]!=1)
		{
			M_Row[Pos[1]-1]=M_Row[Pos[1]-1] & (~(1<<Pos[0]));
		}
	}
	else if(MiceVec==3)
	{
		if(kabe_inf[0]==1&&sotokabe_inf[0]!=1)
		{
			Column[Pos[0]][Pos[1]-1]=1;
		}
		if(kabe_inf[1]==1&&sotokabe_inf[1]!=1)
		{
			Row[Pos[1]-2][Pos[0]]=1;
		}
		if(kabe_inf[2]==1&&sotokabe_inf[2]!=1)
		{
			Column[Pos[0]-1][Pos[1]-1]=1;
		}

		if(kabe_inf[0]==0&&sotokabe_inf[0]!=1)
		{
			M_Column[Pos[0]]=M_Column[Pos[0]] & (~(1<<(Pos[1]-1)));
		}
		if(kabe_inf[1]==0&&sotokabe_inf[1]!=1)
		{
			M_Row[Pos[1]-2]=M_Row[Pos[1]-2] & (~(1<<Pos[0]));
		}
		if(kabe_inf[2]==0&&sotokabe_inf[2]!=1)
		{
			M_Column[Pos[0]-1]=M_Column[Pos[0]-1] & (~(1<<(Pos[1]-1)));
		}
	}
	else if(MiceVec==4)
	{
		if(kabe_inf[0]==1&&sotokabe_inf[0]!=1)
		{
			Row[Pos[1]-1][Pos[0]-1]=1;
		}
		if(kabe_inf[1]==1&&sotokabe_inf[1]!=1)
		{
			Column[Pos[0]-2][Pos[1]]=1;
		}
		if(kabe_inf[2]==1&&sotokabe_inf[2]!=1)
		{
			Row[Pos[1]][Pos[0]-1]=1;
		}

		if(kabe_inf[0]==0&&sotokabe_inf[0]!=1)
		{
			M_Row[Pos[1]-1]=M_Row[Pos[1]-1] & (~(1<<(Pos[0]-1)));
		}
		if(kabe_inf[1]==0&&sotokabe_inf[1]!=1)
		{
			M_Column[Pos[0]-2]=M_Column[Pos[0]-2] & (~(1<<Pos[1]));
		}
		if(kabe_inf[2]==0&&sotokabe_inf[2]!=1)
		{
			M_Row[Pos[1]]=M_Row[Pos[1]] & (~(1<<(Pos[0]-1)));
		}
	}
}

void ShowMap(void)//マップを表示する関数
{
	printf("\n\r");
	int i,j;
	for(i=2*MapSize+1;i>0;i--)
	{
		if(i==2*MapSize+1 || i==1)
		{
			for(int f=0;f<MapSize-1;f++)
			{
				printf("+---");
			}
			printf("+---+\n\r");
		}
		else if(i%2==0)
		{
			//columnの出力
			printf("|   ");
			for(j=0;j<MapSize-1;j++)
			{
				if(Column[j][i/2-1]==1)
				{
					printf("|   ");
				}
				else
				{
					printf("    ");
				}
			}
			printf("|   \n\r");
		}
		else
		{
			//Rowの出力
			printf("+");
			for(j=0;j<MapSize;j++)
			{
				if(Row[(i-1)/2-1][j]==1)
				{
					printf("---+");
				}
				else
				{
					printf("   +");
				}
			}
			printf("   +\n\r");
		}
	}
}

void Init_Dist(void)//歩数マップの初期化を行う関数
{
	for(int x=0;x<16;x++)
	{
		for(int y=0;y<16;y++)
		{
			Dist_map[x][y]=255;
		}
	}
}



void initStack(STACK_T *stack)//stackの初期化
{
	stack->head=0;
	stack->tail=0;
}

void pushStack_walk(STACK_T *stack, POS input)//stackのpush
{
	//1データをデータの最後尾の一つ後ろに格納
	stack->pos[stack->tail]=input;
	//2データの最後尾を一つ後ろに移動
	stack->tail=stack->tail+1;
	//3巡回シフト
	if(stack->tail==MAX_QUEUE_NUM)
	{
		stack->tail=0;
	}
	//4スタックが満杯なら何もせずに関数終了
	if(stack->tail==stack->head)
	{
		return;
	}
}

POS popStack_walk(STACK_T *stack)//stackのpop
{
	POS ret={0,0};

	//1スタックが空なら何もせずに終了
	if(stack->tail==stack->head)
	{
		ret.x=65535;
		return ret;
	}
	//2データの最前列からデータを取得
	ret=stack->pos[stack->head];
	//3データの最前列を一つ前にずらす
	stack->head=stack->head+1;
	//4巡回シフト
	if(stack->head==MAX_QUEUE_NUM)
	{
		stack->head=0;
	}
	//5取得したデータを返却
	return ret;
}

void Re_Decide_Dist(void)//改良歩数マップの更新を行う関数
{
	Init_Dist();

	STACK_T stack;
	POS bupos;
	initStack(&stack);//1スタックを初期化

	if(isReturn==0)
	{
		for(int i=0;i<4;i++)
		{
			Dist_map[goal_x[i]][goal_y[i]]=0;//goal座標を0にする
			bupos.x=goal_x[i];
			bupos.y=goal_y[i];
			pushStack_walk(&stack,bupos);//goal座標をスタックにpushする
		}
	}
	else
	{
		Dist_map[0][0]=0;//start座標を0にする
		bupos.x=0;
		bupos.y=0;
		pushStack_walk(&stack,bupos);//start座標をスタックにpushする
	}

	int count=0;

	while(1)
	{
		bupos=popStack_walk(&stack);//スタックから座標をpopする
		int x=bupos.x;
		int y=bupos.y;

		if(x==65535)//stackが空なのに取り出そうとしたらbreak;
		{
			break;
		}

		if(Dist_map[x][y]==count+1)//popした座標の歩数が次のcountなら現在のカウントを++
		{
			count++;
		}

		if(count==254)
		{
			break;
		}

		//Popされた座標の周辺状況の確認
		if(Dist_map[x][y]==count)
		{
			if(x!=MapSize-1 && Dist_map[x+1][y]==255 && Column[x][y]==0)//1右端でなく、探索済みでなく、右壁なしのとき
			{
				Dist_map[x+1][y]=count+1;
				bupos.x=x+1;
				bupos.y=y;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(x!=0 && Dist_map[x-1][y]==255 && Column[x-1][y]==0)//2左端でなく、探索済みでなく、左壁なしのとき
			{
				Dist_map[x-1][y]=count+1;
				bupos.x=x-1;
				bupos.y=y;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(y!=MapSize-1 && Dist_map[x][y+1]==255 && Row[y][x]==0)//3上端でなく、探索済みでなく、上壁なしのとき
			{
				Dist_map[x][y+1]=count+1;
				bupos.x=x;
				bupos.y=y+1;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(y!=0 && Dist_map[x][y-1]==255 && Row[y-1][x]==0)//3下端でなく、探索済みでなく、下壁なしのとき
			{
				Dist_map[x][y-1]=count+1;
				bupos.x=x;
				bupos.y=y-1;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
		}
	}
}



int Adati(void)//足立法アルゴリズムに則り移動
{
	int num;
	int x=Pos[0];
	int y=Pos[1];
	int front_dist=255;
	int back_dist=255;
	int right_dist=255;
	int left_dist=255;
	int min=255;//0最小値


	//1ある位置(x,y)にいるときの前後左右の歩数を取得
	if(MiceVec==1)
	{
		if(y!=MapSize-1)
		{
			front_dist=Dist_map[x][y+1];
		}
		if(y!=0)
		{
			back_dist=Dist_map[x][y-1];
		}
		if(x!=MapSize-1)
		{
			right_dist=Dist_map[x+1][y];
		}
		if(x!=0)
		{
			left_dist=Dist_map[x-1][y];
		}
	}
	else if(MiceVec==2)
	{
		if(y!=MapSize-1)
		{
			left_dist=Dist_map[x][y+1];
		}
		if(y!=0)
		{
			right_dist=Dist_map[x][y-1];
		}
		if(x!=MapSize-1)
		{
			front_dist=Dist_map[x+1][y];
		}
		if(x!=0)
		{
			back_dist=Dist_map[x-1][y];
		}
	}
	else if(MiceVec==3)
	{
		if(y!=MapSize)
		{
			back_dist=Dist_map[x][y];
		}
		if(y!=1)
		{
			front_dist=Dist_map[x][y-2];
		}
		if(x!=MapSize-1)
		{
			left_dist=Dist_map[x+1][y-1];
		}
		if(x!=0)
		{
			right_dist=Dist_map[x-1][y-1];
		}
	}
	else
	{
		if(y!=MapSize-1)
		{
			right_dist=Dist_map[x-1][y+1];
		}
		if(y!=0)
		{
			left_dist=Dist_map[x-1][y-1];
		}
		if(x!=MapSize)
		{
			back_dist=Dist_map[x][y];
		}
		if(x!=1)
		{
			front_dist=Dist_map[x-2][y];
		}
	}

	//2最小の歩数を決定

	//3前→右→左→後
	int sort[4];
	sort[0]=front_dist;
	sort[1]=right_dist;
	sort[2]=left_dist;
	sort[3]=back_dist;

	//4壁に阻まれた歩数を除外
	for(int i=0;i<4;i++)
	{
		switch(i)
		{
			case 0:
			if(kabe_inf[1]!=0)
			{
				sort[i]=255;
			}
			break;

			case 1:
		    if(kabe_inf[2]!=0)
		    {
			    sort[i]=255;
			}
		    break;

			case 2:
		    if(kabe_inf[0]!=0)
			{
				sort[i]=255;
			}
			break;

			case 3:

			break;
         }
	}

	//5最小値を決定
	for(int i=0;i<4;i++)
	{
		if(sort[i]<min)
		{
			min=sort[i];
			num=i;
		}
	}

	//6setnumに変更
	if(num==0)
	{
		num=1;
	}
	else if(num==1)
	{
		num=2;
	}
	else if(num==2)
	{
		num=0;
	}
	else
	{
		num=3;
	}


	return num;
}

void ShowAdatiMap(void)//歩数マップの表示
{
	printf("\n\r");

		int i,j;
		for(i=2*MapSize+1;i>0;i--)
		{
			if(i==2*MapSize+1 || i==1)
			{
				for(int f=0;f<MapSize-1;f++)
				{
					printf("+---");
				}
				printf("+---+\n\r");
			}
			else if(i%2==0)
			{
				//columnの出力
				printf("|%3d",Dist_map[0][i/2-1]);
				for(j=0;j<MapSize-1;j++)
				{
					if(Column[j][i/2-1]==1)
					{
						if(j==0)
						{
							printf("|");
						}
						else
						{
							printf("%3d|",Dist_map[j][i/2-1]);
						}

					}
					else
					{
						if(j==0)
						{
							printf(" ");
						}
						else
						{
							printf("%3d ",Dist_map[j][i/2-1]);
						}

					}
				}
				if(Column[MapSize-1][i/2-1]==1)
				{
					printf("%3d|\n\r",Dist_map[MapSize-1][i/2-1]);
				}
				else
				{
					printf("%3d|\n\r",Dist_map[MapSize-1][i/2-1]);
				}
			}
			else
			{
				//Rowの出力
				printf("+");
				for(j=0;j<MapSize;j++)
				{
					if(Row[(i-1)/2-1][j]==1)
					{
						printf("---+");
					}
					else
					{
						printf("   +");
					}
				}
				printf("   +\n\r");
			}
		}
}

void GoalCheck(void)//ゴールしたかどうかを判定する関数
{
	if(isReturn==0)
	{
		for(int i=0;i<4;i++)
		{
			if(MiceVec==1 || MiceVec==2)
			{
				if(Pos[0]==goal_x[i] && Pos[1]==goal_y[i])
				{
					isGoal=1;
				}
			}
			else if(MiceVec==3)
			{
				if(Pos[0]==goal_x[i] && Pos[1]==goal_y[i]+1)
				{
					isGoal=1;
				}
			}
			else
			{
				if(Pos[0]==goal_x[i]+1 && Pos[1]==goal_y[i])
				{
					isGoal=1;
				}
			}
		}
	}
	else
	{
		if(Pos[0]==0 && Pos[1]==1)
		{
			isGoal=1;
		}
	}
}

void Init_M_Row_Column(void)//最短経路用壁配列の初期化
{
	for(int i=0;i<15;i++)
	{
		M_Row[i]=0b1111111111111111;
		M_Column[i]=0b1111111111111111;
	}
	M_Row[0]=M_Row[0] & (0b1111111111111110);
	int gx=goal_x[0];
	int gy=goal_y[0];
	M_Column[gx]=M_Column[gx] & (~(1<<gy));
	M_Column[gx]=M_Column[gx] & (~(1<<(gy+1)));
	M_Row[gy]=M_Row[gy] & (~(1<<gx));
	M_Row[gy]=M_Row[gy] & (~(1<<(gx+1)));
}

void M_Decide_Dist(void)//最短歩数マップの更新を行う関数
{
	Init_Dist();

	STACK_T stack;
	POS bupos;
	initStack(&stack);//1スタックを初期化

	for(int i=0;i<4;i++)
	{
		Dist_map[goal_x[i]][goal_y[i]]=0;//goal座標を0にする
		bupos.x=goal_x[i];
		bupos.y=goal_y[i];
		pushStack_walk(&stack,bupos);//goal座標をスタックにpushする
	}

	int count=0;

	while(1)
	{
		bupos=popStack_walk(&stack);//スタックから座標をpopする
		int x=bupos.x;
		int y=bupos.y;

		if(x==65535)//stackが空なのに取り出そうとしたらbreak;
		{
			break;
		}

		if(Dist_map[x][y]==count+1)//popした座標の歩数が次のcountなら現在のカウントを++
		{
			count++;
		}

		if(count==254)
		{
			break;
		}

		//Popされた座標の周辺状況の確認
		if(Dist_map[x][y]==count)
		{
			if(x!=MapSize-1 && Dist_map[x+1][y]==255 && ((M_Column[x] & (1<<y)) != (1<<y)))//1右端でなく、探索済みでなく、右壁なしのとき
			{
				Dist_map[x+1][y]=count+1;
				bupos.x=x+1;
				bupos.y=y;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(x!=0 && Dist_map[x-1][y]==255 && ((M_Column[x-1] & (1<<y)) != (1<<y)))//2左端でなく、探索済みでなく、左壁なしのとき
			{
				Dist_map[x-1][y]=count+1;
				bupos.x=x-1;
				bupos.y=y;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(y!=MapSize-1 && Dist_map[x][y+1]==255 && ((M_Row[y] & (1<<x)) != (1<<x)))//3上端でなく、探索済みでなく、上壁なしのとき
			{
				Dist_map[x][y+1]=count+1;
				bupos.x=x;
				bupos.y=y+1;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
			if(y!=0 && Dist_map[x][y-1]==255 && ((M_Row[y-1] & (1<<x)) != (1<<x)))//3下端でなく、探索済みでなく、下壁なしのとき
			{
				Dist_map[x][y-1]=count+1;
				bupos.x=x;
				bupos.y=y-1;
				pushStack_walk(&stack,bupos);//代入した座標をスタックにpushする
			}
		}
	}
}

void M_Pass(void)//passに最短経路を記録する関数
{
	M_Decide_Dist();
	MiceVec=1;
	PreMiceVec=1;
	Pos[0]=0;
	Pos[1]=1;
	isGoal=0;
	int n=0;

	for(int i=0;i<255;i++)//passを初期化
	{
		pass[i]=0;
	}

	sound(300,200);
	while(1)
	{
		int setnum=M_Adati();

		if(setnum==0)
		{
			n++;
			pass[n]=-2;//Leftスラローム
			VecDecide(2);
			PosDecide();
		}
		else if(setnum==1)
		{
			n++;
			pass[n]=1;//180mm直進
			n++;
			pass[n]=1;
			PosDecide();
		}
		else if(setnum==2)
		{
			n++;
			pass[n]=-3;//Rightスラローム
			VecDecide(1);
			PosDecide();
		}
		else
		{
			n=252;
		}

		GoalCheck();
		if(isGoal==1 || n==252)
		{
			n++;
			pass[n]=1;//180mm直進
			n++;
			pass[n]=1;
			break;
		}
	}

	if(mode!=19)
	{
		//passの圧縮大回り
		Comp_Pass(OOmawari_R,4,-6);
		Comp_Pass(OOmawari_L,4,-4);
		//passの圧縮小回り
		Comp_Pass(Komawari_R,3,-7);
		Comp_Pass(Komawari_L,3,-5);
	}
	St_Comp_Pass();//直線の圧縮
	sound(500,300);
}

void Comp_Pass(int *tar_pass,int pass_size,int setpassnum)//passを圧縮する関数
{
	int size=pass_size;
	for(int i=0;i<=255-size;i++)//1pass配列を全走査
	{
		int isEq=1;
		for(int j=0;j<size;j++)//2pass配列とtar配列を比較
		{
			if(pass[i+j]!=*(tar_pass+j))
			{
				isEq=0;
			}
		}

		if(isEq==1)
		{
			pass[i]=setpassnum;
			for(int k=i+1;k<=255-size;k++)//3pass配列を圧縮
			{
				pass[k]=pass[size+k-1];
			}
		}
	}
}

void St_Comp_Pass(void)//直線の圧縮
{
	int bu_pass[100];
	int n=0;
	int count=1;
	int isSt=0;

	while(1)
	{
		if(pass[count]<=0)
		{
			isSt=0;
			n++;
			bu_pass[n]=pass[count];
			count++;
		}
		else if(pass[count]>0)
		{
			if(isSt==0)
			{
				isSt=1;
				n++;
				count++;
				bu_pass[n]=1;
			}
			else
			{
				count++;
				bu_pass[n]+=1;
			}
		}


		if(n==99)
		{
			break;
		}
	}

	for(int i=0;i<255;i++)
	{
		pass[i]=0;
	}
	for(int i=0;i<100;i++)
	{
		pass[i]=bu_pass[i];
	}
}

void Show_M_AdatiMap(void)//最短歩数マップの表示
{
	printf("\n\r");

		int i,j;
		for(i=2*MapSize+1;i>0;i--)
		{
			if(i==2*MapSize+1 || i==1)
			{
				for(int f=0;f<MapSize-1;f++)
				{
					printf("+---");
				}
				printf("+---+\n\r");
			}
			else if(i%2==0)
			{
				//columnの出力
				printf("|%3d",Dist_map[0][i/2-1]);
				for(j=0;j<MapSize-1;j++)
				{
					if((M_Column[j] & (1<<(i/2-1))) == (1<<(i/2-1)))
					{
						if(j==0)
						{
							printf("|");
						}
						else
						{
							printf("%3d|",Dist_map[j][i/2-1]);
						}

					}
					else
					{
						if(j==0)
						{
							printf(" ");
						}
						else
						{
							printf("%3d ",Dist_map[j][i/2-1]);
						}

					}
				}
				if((M_Column[MapSize-1] & (1<<(i/2-1)))== (1<<(i/2-1)))
				{
					printf("%3d|\n\r",Dist_map[MapSize-1][i/2-1]);
				}
				else
				{
					printf("%3d|\n\r",Dist_map[MapSize-1][i/2-1]);
				}
			}
			else
			{
				//Rowの出力
				printf("+");
				for(j=0;j<MapSize;j++)
				{
					if((M_Row[(i-1)/2-1] & (1<<j))== (1<<j))
					{
						printf("---+");
					}
					else
					{
						printf("   +");
					}
				}
				printf("   +\n\r");
			}
		}
}

void Show_Pass(void)//パスを表示
{
	printf("\n\r");
	for(int i=0;i<255;i++)
	{
		printf("%d\n\r",pass[i]);
	}
}

int M_Adati(void)//足立法アルゴリズムに則り移動
{
	int num;
	int x=Pos[0];
	int y=Pos[1];
	int front_dist=255;
	int back_dist=255;
	int right_dist=255;
	int left_dist=255;
	int min=255;//0最小値

	for(int i=0;i<3;i++)
	{
		kabe_inf[i]=0;
	}

	//1ある位置(x,y)にいるときの前後左右の歩数と壁情報を取得
	if(MiceVec==1)
	{
		if(y!=MapSize-1)
		{
			front_dist=Dist_map[x][y+1];
		}
		if(y!=0)
		{
			back_dist=Dist_map[x][y-1];
		}
		if(x!=MapSize-1)
		{
			right_dist=Dist_map[x+1][y];
		}
		if(x!=0)
		{
			left_dist=Dist_map[x-1][y];
		}

		if((1<<Pos[1])==(M_Column[Pos[0]-1] & (1<<Pos[1])))
		{
			kabe_inf[0]=1;
		}
		if((1<<Pos[0])==(M_Row[Pos[1]] & (1<<Pos[0])))
		{
			kabe_inf[1]=1;
		}
		if((1<<Pos[1])==(M_Column[Pos[0]] & (1<<Pos[1])))
		{
			kabe_inf[2]=1;
		}
	}
	else if(MiceVec==2)
	{
		if(y!=MapSize-1)
		{
			left_dist=Dist_map[x][y+1];
		}
		if(y!=0)
		{
			right_dist=Dist_map[x][y-1];
		}
		if(x!=MapSize-1)
		{
			front_dist=Dist_map[x+1][y];
		}
		if(x!=0)
		{
			back_dist=Dist_map[x-1][y];
		}

		if((1<<Pos[0])==(M_Row[Pos[1]] & (1<<Pos[0])))
		{
			kabe_inf[0]=1;
		}
		if((1<<Pos[1])==(M_Column[Pos[0]] & (1<<Pos[1])))
		{
			kabe_inf[1]=1;
		}
		if((1<<Pos[0])==(M_Row[Pos[1]-1] & (1<<Pos[0])))
		{
			kabe_inf[2]=1;
		}
	}
	else if(MiceVec==3)
	{
		if(y!=MapSize)
		{
			back_dist=Dist_map[x][y];
		}
		if(y!=1)
		{
			front_dist=Dist_map[x][y-2];
		}
		if(x!=MapSize-1)
		{
			left_dist=Dist_map[x+1][y-1];
		}
		if(x!=0)
		{
			right_dist=Dist_map[x-1][y-1];
		}

		if((1<<(Pos[1]-1))==(M_Column[Pos[0]] & (1<<(Pos[1]-1))))
		{
			kabe_inf[0]=1;
		}
		if((1<<Pos[0])==(M_Row[Pos[1]-2] & (1<<Pos[0])))
		{
			kabe_inf[1]=1;
		}
		if((1<<(Pos[1]-1))==(M_Column[Pos[0]-1] & (1<<(Pos[1]-1))))
		{
			kabe_inf[2]=1;
		}
	}
	else
	{
		if(y!=MapSize-1)
		{
			right_dist=Dist_map[x-1][y+1];
		}
		if(y!=0)
		{
			left_dist=Dist_map[x-1][y-1];
		}
		if(x!=MapSize)
		{
			back_dist=Dist_map[x][y];
		}
		if(x!=1)
		{
			front_dist=Dist_map[x-2][y];
		}

		if((1<<(Pos[0]-1))==(M_Row[Pos[1]-1] & (1<<(Pos[0]-1))))
		{
			kabe_inf[0]=1;
		}
		if((1<<Pos[1])==(M_Column[Pos[0]-2] & (1<<Pos[1])))
		{
			kabe_inf[1]=1;
		}
		if((1<<(Pos[0]-1))==(M_Row[Pos[1]] & (1<<(Pos[0]-1))))
		{
			kabe_inf[2]=1;
		}
	}

	//2最小の歩数を決定

	//3前→右→左→後
	int sort[4];
	sort[0]=front_dist;
	sort[1]=right_dist;
	sort[2]=left_dist;
	sort[3]=back_dist;

	//4壁に阻まれた歩数を除外
	for(int i=0;i<4;i++)
	{
		switch(i)
		{
			case 0:
			if(kabe_inf[1]!=0)
			{
				sort[i]=255;
			}
			break;

			case 1:
		    if(kabe_inf[2]!=0)
		    {
			    sort[i]=255;
			}
		    break;

			case 2:
		    if(kabe_inf[0]!=0)
			{
				sort[i]=255;
			}
			break;

			case 3:

			break;
         }
	}

	//5最小値を決定
	for(int i=0;i<4;i++)
	{
		if(sort[i]<min)
		{
			min=sort[i];
			num=i;
		}
	}

	//6setnumに変更
	if(num==0)
	{
		num=1;
	}
	else if(num==1)
	{
		num=2;
	}
	else if(num==2)
	{
		num=0;
	}
	else
	{
		num=3;
	}


	return num;
}

void Save_Row_Column(void)//壁情報をセーブ
{
	/*for(int i=0;i<15;i++)
	{
		//bu_M_Row[i]=M_Row[i];
		//bu_M_Column[i]=M_Column[i];
	}*/
}

void Load_Row_Column(void)//壁情報をロード
{
	/*for(int i=0;i<15;i++)
	{
		//M_Row[i]=bu_M_Row[i];
		//M_Column[i]=bu_M_Column[i];
	}*/
}









/*
 * mode.h
 *
 *  Created on: 2020/07/28
 *      Author: junmo
 */

#ifndef INC_NEZUTAKA_H_
#define INC_NEZUTAKA_H_
#include"main.h"

//　機体パラメータ
#define TIRE_DIAMETER   52.0					//　タイヤの直径　52mm
#define TIRE_CIRCUIT    (PI * TIRE_DIAMETER)	//　タイヤの円周 163.363mm
#define TREAD_WIDTH     87.0					//　トレッド幅　87.0mm(再計測)
#define TREAD_CIRCUIT   (TREAD_WIDTH * PI)		//360度旋回時にタイヤが動く距離　87*3.14mm 273.18mm
#define	STEP_DEGREE  (1.8 / 2.0)				//　ステッピングモータ1-2相励ステップ角（度/step) 0.9°
#define	STEP_LENGTH	(TIRE_CIRCUIT * STEP_DEGREE / 360.0)	//1ステップで進む距離　0.408mm
//　迷路のパラメータ
#define	BLOCK_LENGTH  180.0					//1区画 180mm
//　定数定義
#define	PI            3.14159265358979		//円周率
//　最大速度
#define MAX_SPEED 1000

#define NEZUTAKA_LENGTH 135


#define RS_WALL 0
#define LS_WALL 1
#define RF_WALL 2
#define LF_WALL 3
#define RS_FREE 4
#define LS_FREE 5
#define RF_FREE 6
#define LF_FREE 7
#define RS_threshold 8
#define LS_threshold 9
#define RF_threshold 10
#define LF_threshold 11

#define MODE_NAME0 "MUSIC"
#define MODE_NAME1 "SENSOR DEBUG"
#define MODE_NAME2 "SENSOR CALIBRATION"
#define MODE_NAME3 "1BLOCK RUN"
#define MODE_NAME4 "TURN R 90°"
#define MODE_NAME5 "TURN U"
#define MODE_NAME6 "SLALOM R"
#define MODE_NAME7 "MAP INIT"
#define MODE_NAME8 "PRINT MAP"
#define MODE_NAME9 "TANSAKU MAP"
#define MODE_NAME10 "SAITAN MAP"
#define MODE_NAME11 "NON"
#define MODE_NAME12 "ADACHI"
#define MODE_NAME13 "HIDARITE"
#define MODE_NAME14 "SPEED TEST"
#define MODE_NAME15 "GURUGURU"



void nezutaka(void);
void MENU(int16_t *mode);
void mode0(void);//music
void mode1(void);//sensordebug
void mode2(void);//init wall value
void mode3(void);//1block run
void mode4(void);//turn 90°
void mode5(void);//turn 180° and sirituke
void mode6(void);//wallprint
void mode7(void);//hidarite
void mode8(void);//kakutyou
void mode9(void);//adachi
void mode10(void);//saitan
void mode11(void);
void mode12(void);
void mode13(void);
void mode14(void);
void mode15(void);



#endif /* INC_NEZUTAKA_H_ */

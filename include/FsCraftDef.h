#pragma once


#include <string>
class Corner {
public:
	//
	double WeldingSpeed = 15;		        //焊接速度
	double MaxRotorSpeed = 0.0;             //最大转速
	double WeldingCrt_Spd = 220;	        //焊接电流
	double WeldingVtg_Strth = 23;           //焊接电压
	int	VtgUniCorrection = 0;				//电压一元修正值
	double Front_Lengh = 0.0;				//拐角前距离
	double Back_Lengh = 0.0;				//拐角后距离
	//
	int Weave_Enable = 1;                   //启用摆焊
	int Shape = 0;							// 摆弧形状: 0 正弦, 1 圆弧, 2 8字形, 3 三角
	double LeftWidth = 4;					//左振幅
	double RightWidth = 4;					//右振幅

	double Freq = 0.0;					    //摆弧频率 Hz 
	double L_StayTime = 0.0;				//左停留时间
	double R_StayTime = 0.0;			    //右停留时间
	double L_Angle = 0.0;				    //左摆角度
	double R_Angle = 0.0;				    //右摆角度
	int StayMode = 0;					    //停留方式: 0 停止摆动继续向前, 1 完全静止
	//

};










class ReArc {
public :

	int ReArc_Enable = 1;					//启用再起弧
	int ReArcCount = 0;						//再起弧次数
	double ReArcTime = 0.0;					//再起弧时间
	double ReArcSnagTime = 0.0;				//再起弧抽丝时间
	//

	int ScrubArc_Enable = 1;				//启用刮擦起弧	
	double ScrubArcCrt = 0.0;				//刮擦电流
	double ScrubArcVtg = 0.0;				//刮擦电压
	double ScrubArcVtgCorrect = 0.0;		//刮擦电压修正值
	int ScrubArcCount = 0;					//刮擦起弧次数
	double ScrubArcLengh = 0.0;				//刮擦距离
	double ScrubArcSpeed = 0.0;				//刮擦速度
	double SnagTime = 0.0;					//抽丝时间
	//

	int Weave_Enable = 1;                   //启用摆焊
	int Shape = 0;							//摆弧形状: 0 正弦, 1 圆弧, 2 8字形, 3 三角
	double LeftWidth = 4;					//左振幅
	double RightWidth = 4;					//右振幅

	double Freq = 0.0;					    //摆弧频率 Hz 
	double L_StayTime = 0.0;				//左停留时间
	double R_StayTime = 0.0;			    //右停留时间
	double L_Angle = 0.0;				    //左摆角度
	double R_Angle = 0.0;				    //右摆角度
	int StayMode = 0;					    //停留方式: 0 停止摆动继续向前, 1 完全静止
    //
};


//摆焊参数
class Weave {
public:
	int Id = 0;
	int Master = 0;				            //摆弧基准？周期
	int Shape = 3;				            //摆弧形状: 0 正弦, 1 圆弧, 2 8字形, 3 三角
	double Length = 1.47;			
	double Width = 8;					    //振幅
	double Radius = 0;					    //摆弧半径
	double Bias = 0;					    //摆弧偏差
	int Dwell_type = 1;					    //等待类型: 0 摆动停止, 1 机器人停止
	double Dwell_left = 0.5;			    //左侧等待
	double Dwell_center = 0.1;			    //中心等待
	double Dwell_right = 0.5;			    //右侧等待
	double Angle_plane = 0;					//仰角类型: 0 V型, 1 三角
	double Angle_tilt = 0;					//倾斜角度
	double Angle_lead = 0;
	double Angle_Ltype_top = 30;			//上仰角度
	double Angle_Ltype_btm = 30;			//下仰角度
	double Phase = 0;					    //波形反相: 0 正常, 1 反向   
	double Offset = 0;					
	double Invert = 0;
	double IsWeave_3D = 1;				    //3D摆弧
	double LeftWidth = 4;					//左振幅
	double RightWidth = 4;					//右振幅
	double LengthLE_value = 0.68;

	double Freq = 0.0;					    //摆弧频率 Hz
};



//电弧跟踪参数
class Track {
public:
	int Id = 0;
	int Lr_enable = 1;						//上下跟踪使能	
	int Ud_enable = 1;						//左右跟踪使能
	int Signal_Type = 0;					//信号源 0焊机
	int TrackerPar_Id = 0;					//跟踪器ID
	int RefCurrentType = 0;					//
	int InvalidSampleCount = 3;				//无效采样计数
	double AdjustDelayTime = 0.5;			//采样延迟时间
	double SegCorrectMaxProportion = 0.0;   //分段纠偏最大比例
	double SegCorrectCycles = 0.0;          //分段纠偏周期数

	double Lr_gain = 0.28;					//左右跟踪增益
	double Lr_offset = 0;					//左右基准偏差
	double Lr_maxSingleCompensation = 3;	//左右单次最大补偿量
	double Lr_minCompensation = 0;			//左右最小补偿量
	double Lr_maxCompensation = 200;		//左右最大补偿量
	int Lr_startTrackCount = 5;				//左右开始跟踪计数
	double Lr_MaxCorrectAngle = 0.0;        //左右最大纠偏角度

	double Ud_gain = -0.25;					//上下跟踪增益
	double Ud_offset = 0;					//上下基准偏差
	double Ud_refCurrent = 0;					
	double Ud_constRefCurrent = 0;			//	
	double Ud_maxSingleCompensation = 3;	//上下单次最大补偿量
	double Ud_minCompensation = 0;			//上下最小补偿量
	double Ud_maxCompensation = 200;		//上下最大补偿量
	int Ud_refSampleCount = 0;				//上下引用采样计数
	int Ud_startTrackCount = 5;				//上下开始跟踪计数
	double Ud_sampleFreq = 5;				//上下采样频率
	double Ud_MaxCorrectAngle = 0.0;        //上下最大纠偏角度


};

//焊接参数
class Arc_WeldingParaItem {
public:
	int Id = -1;
	int CurrentMode = 0;		            //电流模式: 0 电流值
	int VoltageMode = 1;		            //电压模式: 0 分别, 1 一元化
	int CoolingMode = 0;		            //冷却模式
	int Inductance = 0;
	int BaseCurrentRatio = 0;
	double ReburnCorrectionTime = 0;

	//起弧参数
	int ArcOnWorkMode = 0;			        //起弧模式 0直流 1脉冲 2Job模式 3断续焊 4分别模式
	int JobChannelNum = 0;
	double ArcOnCrt_Spd = 220;		        //起弧电流
	double ArcOnVtg_Strth = 23;		        //起弧电压
	double ArcOnTime = 0.5;			        //起弧时间
	double ArcOnVtg_Correction = 0.0;       //起弧电压修正值
	double ArcOnBlowTime = 0.5;             //引气时间
	double SlowUpTime = 0.0;                //缓升时间

	//焊接参数
	int WeldingWorkMode = 0;		        //工作模式： 0直流 1脉冲 2Job模式 3断续焊 4分别模式
	double WeldingCrt_Spd = 220;		    //焊接电流
	double WeldingVtg_Strth = 23;		    //焊接电压
	double WeldingSpeed = 15;		        //焊接速度

	//收弧参数
	int ArcOffWorkMode = 0;				    //起弧模式 0直流 1脉冲 2Job模式 3断续焊 4分别模式
	double ArcOffCrt_Spd = 200;				//收弧电流
	double ArcOffVtg_Strth = 25;			//收弧电压
	double ArcOffTime = 0.5;				//收弧时间
	double UnstickCurrent = 30;				//防粘丝电流
	double UnstickVoltage = 25;				//防粘丝电压
	double UnstickTimeOut = 0;				//防粘丝时间
	double ArcOffVtg_Correction = 0.0;      //收弧电压修正值
	double ArcOffBlowTime = 0.5;            //收气时间
	double SlowDownTime = 0.0;              //缓降时间



	int	 WelderParamType = 0;				//焊机参数类型: 0 一元模式, 1 分别模式
	int	VtgUniCorrection = 0;				//电压一元修正值
	double ArcOnTimeoutTime = 0.0;			//起弧超时时间
	double ArcInterruptReconnectTime = 0.0;	//断弧重连时间

	double SearchSpeedLimit = 0.0;          //寻位限速
	double MaxRotorSpeed = 0.0;             //最大转速
	double LowpassFilter = 0.0;             //低通滤波
	//
	//
	//
	//
	//
	//
	//
	//
	//
		//
	//
	//
	//
	//
	//



};




















//class Arc_WeldingParaItem {
//public:
//	int Id;
//	int CurrentMode;
//	int VoltageMode;
//	int CoolingMode;
//	int Inductance;
//	int BaseCurrentRatio;
//	double ReburnCorrectionTime;
//	int ArcOnWorkMode;
//	int JobChannelNum;
//	double ArcOnCrt_Spd;
//	double ArcOnVtg_Strth;
//	double ArcOnTime;
//	int WeldingWorkMode;
//	double WeldingCrt_Spd;
//	double WeldingVtg_Strth;
//	double WeldingSpeed;
//	int ArcOffWorkMode;
//	double ArcOffCrt_Spd;
//	double ArcOffVtg_Strth;
//	double ArcOffTime;
//	double UnstickCurrent;
//	double UnstickVoltage;
//	double UnstickTimeOut;
//	std::string Annotation;
//};






//class Weave
//{
//public:
//	 int id = 0;
//	 int master = 0;
//	 int shape = 3;
//	 double length = 1.47;
//	 double width = 8;
//	 double radius = 0;
//	 double bias = 0;
//	 int dwell_type = 1;
//	 double dwell_left = 0.5;
//	 double dwell_center = 0.1;
//	 double dwell_right = 0.5;
//	 double angle_plane = 0;
//	 double angle_tilt = 0;
//	 double angle_lead = 0;
//	 double angle_Ltype_top = 30;
//	 double angle_Ltype_btm = 30;
//	 double phase = 0;
//	 double offset = 0;
//	 double invert = 0;
//	 double isWeave_3D = 1;
//	 double leftWidth = 4;
//	 double rightWidth = 4;
//	 double lengthLE_value = 0.68;
//
//};

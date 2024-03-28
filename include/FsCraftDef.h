#pragma once


#include <string>
class Corner {
public:
	//
	double WeldingSpeed = 15;		        //�����ٶ�
	double MaxRotorSpeed = 0.0;             //���ת��
	double WeldingCrt_Spd = 220;	        //���ӵ���
	double WeldingVtg_Strth = 23;           //���ӵ�ѹ
	int	VtgUniCorrection = 0;				//��ѹһԪ����ֵ
	double Front_Lengh = 0.0;				//�ս�ǰ����
	double Back_Lengh = 0.0;				//�սǺ����
	//
	int Weave_Enable = 1;                   //���ðں�
	int Shape = 0;							// �ڻ���״: 0 ����, 1 Բ��, 2 8����, 3 ����
	double LeftWidth = 4;					//�����
	double RightWidth = 4;					//�����

	double Freq = 0.0;					    //�ڻ�Ƶ�� Hz 
	double L_StayTime = 0.0;				//��ͣ��ʱ��
	double R_StayTime = 0.0;			    //��ͣ��ʱ��
	double L_Angle = 0.0;				    //��ڽǶ�
	double R_Angle = 0.0;				    //�ҰڽǶ�
	int StayMode = 0;					    //ͣ����ʽ: 0 ֹͣ�ڶ�������ǰ, 1 ��ȫ��ֹ
	//

};










class ReArc {
public :

	int ReArc_Enable = 1;					//��������
	int ReArcCount = 0;						//���𻡴���
	double ReArcTime = 0.0;					//����ʱ��
	double ReArcSnagTime = 0.0;				//���𻡳�˿ʱ��
	//

	int ScrubArc_Enable = 1;				//���ùβ���	
	double ScrubArcCrt = 0.0;				//�β�����
	double ScrubArcVtg = 0.0;				//�β���ѹ
	double ScrubArcVtgCorrect = 0.0;		//�β���ѹ����ֵ
	int ScrubArcCount = 0;					//�β��𻡴���
	double ScrubArcLengh = 0.0;				//�β�����
	double ScrubArcSpeed = 0.0;				//�β��ٶ�
	double SnagTime = 0.0;					//��˿ʱ��
	//

	int Weave_Enable = 1;                   //���ðں�
	int Shape = 0;							//�ڻ���״: 0 ����, 1 Բ��, 2 8����, 3 ����
	double LeftWidth = 4;					//�����
	double RightWidth = 4;					//�����

	double Freq = 0.0;					    //�ڻ�Ƶ�� Hz 
	double L_StayTime = 0.0;				//��ͣ��ʱ��
	double R_StayTime = 0.0;			    //��ͣ��ʱ��
	double L_Angle = 0.0;				    //��ڽǶ�
	double R_Angle = 0.0;				    //�ҰڽǶ�
	int StayMode = 0;					    //ͣ����ʽ: 0 ֹͣ�ڶ�������ǰ, 1 ��ȫ��ֹ
    //
};


//�ں�����
class Weave {
public:
	int Id = 0;
	int Master = 0;				            //�ڻ���׼������
	int Shape = 3;				            //�ڻ���״: 0 ����, 1 Բ��, 2 8����, 3 ����
	double Length = 1.47;			
	double Width = 8;					    //���
	double Radius = 0;					    //�ڻ��뾶
	double Bias = 0;					    //�ڻ�ƫ��
	int Dwell_type = 1;					    //�ȴ�����: 0 �ڶ�ֹͣ, 1 ������ֹͣ
	double Dwell_left = 0.5;			    //���ȴ�
	double Dwell_center = 0.1;			    //���ĵȴ�
	double Dwell_right = 0.5;			    //�Ҳ�ȴ�
	double Angle_plane = 0;					//��������: 0 V��, 1 ����
	double Angle_tilt = 0;					//��б�Ƕ�
	double Angle_lead = 0;
	double Angle_Ltype_top = 30;			//�����Ƕ�
	double Angle_Ltype_btm = 30;			//�����Ƕ�
	double Phase = 0;					    //���η���: 0 ����, 1 ����   
	double Offset = 0;					
	double Invert = 0;
	double IsWeave_3D = 1;				    //3D�ڻ�
	double LeftWidth = 4;					//�����
	double RightWidth = 4;					//�����
	double LengthLE_value = 0.68;

	double Freq = 0.0;					    //�ڻ�Ƶ�� Hz
};



//�绡���ٲ���
class Track {
public:
	int Id = 0;
	int Lr_enable = 1;						//���¸���ʹ��	
	int Ud_enable = 1;						//���Ҹ���ʹ��
	int Signal_Type = 0;					//�ź�Դ 0����
	int TrackerPar_Id = 0;					//������ID
	int RefCurrentType = 0;					//
	int InvalidSampleCount = 3;				//��Ч��������
	double AdjustDelayTime = 0.5;			//�����ӳ�ʱ��
	double SegCorrectMaxProportion = 0.0;   //�ֶξ�ƫ������
	double SegCorrectCycles = 0.0;          //�ֶξ�ƫ������

	double Lr_gain = 0.28;					//���Ҹ�������
	double Lr_offset = 0;					//���һ�׼ƫ��
	double Lr_maxSingleCompensation = 3;	//���ҵ�����󲹳���
	double Lr_minCompensation = 0;			//������С������
	double Lr_maxCompensation = 200;		//������󲹳���
	int Lr_startTrackCount = 5;				//���ҿ�ʼ���ټ���
	double Lr_MaxCorrectAngle = 0.0;        //��������ƫ�Ƕ�

	double Ud_gain = -0.25;					//���¸�������
	double Ud_offset = 0;					//���»�׼ƫ��
	double Ud_refCurrent = 0;					
	double Ud_constRefCurrent = 0;			//	
	double Ud_maxSingleCompensation = 3;	//���µ�����󲹳���
	double Ud_minCompensation = 0;			//������С������
	double Ud_maxCompensation = 200;		//������󲹳���
	int Ud_refSampleCount = 0;				//�������ò�������
	int Ud_startTrackCount = 5;				//���¿�ʼ���ټ���
	double Ud_sampleFreq = 5;				//���²���Ƶ��
	double Ud_MaxCorrectAngle = 0.0;        //��������ƫ�Ƕ�


};

//���Ӳ���
class Arc_WeldingParaItem {
public:
	int Id = -1;
	int CurrentMode = 0;		            //����ģʽ: 0 ����ֵ
	int VoltageMode = 1;		            //��ѹģʽ: 0 �ֱ�, 1 һԪ��
	int CoolingMode = 0;		            //��ȴģʽ
	int Inductance = 0;
	int BaseCurrentRatio = 0;
	double ReburnCorrectionTime = 0;

	//�𻡲���
	int ArcOnWorkMode = 0;			        //��ģʽ 0ֱ�� 1���� 2Jobģʽ 3������ 4�ֱ�ģʽ
	int JobChannelNum = 0;
	double ArcOnCrt_Spd = 220;		        //�𻡵���
	double ArcOnVtg_Strth = 23;		        //�𻡵�ѹ
	double ArcOnTime = 0.5;			        //��ʱ��
	double ArcOnVtg_Correction = 0.0;       //�𻡵�ѹ����ֵ
	double ArcOnBlowTime = 0.5;             //����ʱ��
	double SlowUpTime = 0.0;                //����ʱ��

	//���Ӳ���
	int WeldingWorkMode = 0;		        //����ģʽ�� 0ֱ�� 1���� 2Jobģʽ 3������ 4�ֱ�ģʽ
	double WeldingCrt_Spd = 220;		    //���ӵ���
	double WeldingVtg_Strth = 23;		    //���ӵ�ѹ
	double WeldingSpeed = 15;		        //�����ٶ�

	//�ջ�����
	int ArcOffWorkMode = 0;				    //��ģʽ 0ֱ�� 1���� 2Jobģʽ 3������ 4�ֱ�ģʽ
	double ArcOffCrt_Spd = 200;				//�ջ�����
	double ArcOffVtg_Strth = 25;			//�ջ���ѹ
	double ArcOffTime = 0.5;				//�ջ�ʱ��
	double UnstickCurrent = 30;				//��ճ˿����
	double UnstickVoltage = 25;				//��ճ˿��ѹ
	double UnstickTimeOut = 0;				//��ճ˿ʱ��
	double ArcOffVtg_Correction = 0.0;      //�ջ���ѹ����ֵ
	double ArcOffBlowTime = 0.5;            //����ʱ��
	double SlowDownTime = 0.0;              //����ʱ��



	int	 WelderParamType = 0;				//������������: 0 һԪģʽ, 1 �ֱ�ģʽ
	int	VtgUniCorrection = 0;				//��ѹһԪ����ֵ
	double ArcOnTimeoutTime = 0.0;			//�𻡳�ʱʱ��
	double ArcInterruptReconnectTime = 0.0;	//�ϻ�����ʱ��

	double SearchSpeedLimit = 0.0;          //Ѱλ����
	double MaxRotorSpeed = 0.0;             //���ת��
	double LowpassFilter = 0.0;             //��ͨ�˲�
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

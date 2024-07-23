#pragma warning(disable : 4996)
#pragma once

#include<vector>
#include<eigen3/Eigen/Dense>
#include<chrono>
#include<thread>
#include<fstream>

#include "zmotion.h"
#include "zauxdll2.h"

#include"FsCraftDef.h"
#include "DiscreteTrajectory.h"

#ifndef M_PI
constexpr static double M_PI = 3.14159265358979323846;
#endif

class ZauxAxis {
	static std::vector<int> AxisIdxList;
	ZauxAxis();
	// ��ȡ�Ѿ����ص����ƿ��е������
	//uint8_t load_uploaded_config();
	// ���������
	//uint8_t download_config();
	// ��ȡ����λ��
	//std::vector<float> get_axis_param(const std::vector<int>& axisList);

private:
	// ������
	int atype;
	// ���嵱��
	float units;
	// �˶�ѧ����
	float speed, accel, decel;
};

class ZauxRobot {
public:
	enum class ConnType{INIT = 0, FK = 1, IK = 2};

	// ���������
	ZMC_HANDLE handle_ = NULL;
	// ��������ؽ���Ĺ���ģʽ
	ConnType connType = ConnType::INIT;

	// �ؽ���
	std::vector<int> jointAxisIdx_ = { 0,1,2,3,4,5 };
	// ���λ����
	std::vector<int> ikPosAxisIdx_ = { 9,10,11 };
	// TCP ��̬��
	std::vector<int> tcpAngleAxisIdx_ = { 12,13,14 };
	// TCP λ����
	std::vector<int> tcpPosAxisIdx_ = { 18, 19, 20 };
	// ������
	std::vector<int> appAxisIdx_ = { 6,7,8 };
	// ͹����
	std::vector<int> camAxisIdx_ = { 36,37,38 };
	// �ڶ���
	std::vector<int> swingAxisIdx_ = { 40,41,42 };
	// �岹ʸ����
	std::vector<int> connpathAxisIdx_ = { 39 };
	// ������
	std::vector<int> transAxisIdx_ = { 18,19,20,21,22,23,24,25,26 };

	// �ڶ���־λ����
	size_t swingFlagIdx = 1000;

	// ���嵱��
	float axisUnits = 1000;


public:
	ZauxRobot() {}
	ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx
	);
	/**
	* @brief ͨ���������ӿ�����
	*/
	int32 connect_eth(char *ip_addr);
	int32 connect_pci(uint32 cardNum);
	int32 lazy_connect();
	int32 connect(std::string addr);

	/**
	* @brief ��¼ basic ���򵽿�����
	* @param basPath    basic ����·��
	* @param mode       ��¼ģʽ��
	         0          ��¼�� RAM
			 1          ��¼�� ROM
	*/
	int32 load_basic_pragma(const char *basPath, uint32_t mode = 0);
	//uint8_t load_basic_project(const char *basPath, uint32_t mode = 0);

	//uint8_t enable_zaux_log(int logMode);

	/**
	* @brief �趨���ƿ����
	* @param handle    ���ƿ����
	*/
	int32 set_handle(ZMC_HANDLE handle);
	/**
	* @brief �趨���
	* @param jointAxisIdx       �ؽ���
	* @param ikPosAxisIdx       ���λ����
	* @param tcpAngleAxisIdx    ��������ϵ�� TCP �Ƕ���: �����̬��
	* @param tcpPosAxisIdx      ��������ϵ�� TCP λ����: ��Ӹ������� TCP λ��
	* @param appAxisIdx         ������
	* @param camAxisIdx         ͹����
	* @param swingAxisIdx       �ڶ���
	* @param connpathAxisIdx    �岹ʸ����
	*/
	int32 set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx);

	/**
	* @brief �Ͽ�����
	*/
	int32 disconnect();

	/**
	* @brief �л�������ģʽ��ÿ��ִ�йؽ��˶�ʱ��Ҫ��һ�Σ�ͬ�����㹤������仯
	*/
	int32 forward_kinematics(int delay = 1e3);

	/**
	* @brief �л������ģʽ��ÿ��ִ�й�������ϵ�˶�ʱ��Ҫ��һ�Σ�ͬ������ؽ�����仯
	*/
	int32 inverse_kinematics(int delay = 1e3);

	int32 wait_idle(int axisIdx);

	/**
	* @brief  ��ȡ�������
	* @param       axisList     ��Ҫ��ȡ����������б�
	* @param       paramName    ��������
	* @param[out]  paramList    ���صĲ����б�
	*/
	int32 get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList);

	/**
	* @brief ���ö������
	* @param       axisList     ��Ҫ��ȡ����������б�
	* @param       paramName    ��������
	* @param       paramList    ��������
	* @param       principal    ��������: -1 ��������; >0 ����������
	*/
	int32 set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal = -1);

	/**
	* @brief ����table���ݵ�����
	*/
	int32 save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	int32 update_swing_table(const Weave& waveCfg);
	int32 swing_on(float vel, const Weave& waveCfg, const std::vector<float>& toolDir = std::vector<float>());
	int32 swing_off(float displacement = 0.0);

	/**
	* @brief ��㶯
	* @param       idx     �˶����
	* @param       type    �˶�����
					  0      �˶�����
					  ��1    �ؽ���/�����˶�
					  ��2    ��������ϵ��/�����˶�
					  ��3    ��������ϵ��/�����˶�
	* @param       dir     �˶�����
	*/
	int32 jog_moving(int idx, int type, int dir);

	/**
	* @brief movePtp
	* @param       axis          �ؽ��˶����
	* @param       relEndMove    ����˶�����
	* @param       speedRatio    �ٶȱ���
	*/
	int32 move_ptp(const std::vector<float>& relEndMove, float speedRatio = 1.0);
	int32 move_ptp_abs(const std::vector<float>& endMove, float speedRatio = 1.0);

	/**
	* @brief moveJ
	* @param       axis          �ؽ��˶����
	* @param       relEndMove    ����˶�����
	* @param       speedRatio    �ٶȱ���
	*/
	int32 moveJ(const std::vector<float>& relEndMove, float speedRatio = 1.0);
	int32 moveJ(const std::vector<int>& axis, const std::vector<float>& relEndMove, float speedRatio = 1.0);

	/**
	* @brief moveJ_abs
	* @param      axis          �ؽ��˶����
	* @param      endMove       �����˶�λ��
	* @param      speedRatio    �ٶȱ���
	*/
	int32 moveJ_abs(const std::vector<float>& endMove, float speedRatio = 1.0);
	int32 moveJ_abs(const std::vector<int>& axis, const std::vector<float>& endMove, float speedRatio = 1.0);

	int32 moveL(const std::vector<int>& axis, const std::vector<float>& relMove);
	int32 moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, std::vector<float>& midPoint, std::vector<float>& endPoint,
				  int imode = 0);

	// ��������
	int32 wlder_on(float current, float voltage);
	int32 wlder_off();

	int32 execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory);
	int32 execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory);

	int32 swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	int32 swing_tri(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	int32 swing_sin(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	int32 swing_tri();

	int32 arc_tracking_config(const Track& trackCfg);

	int32 handle_zaux_error(int32 errCode);
	/**
	* @brief ��λ������ֹͣ
	*/
	int32 emergency_stop();
	/**
	* @brief ��λ��������ͣ
	*/
	int32 emergency_pause();
	/**
	* @brief ��λ�������ָ�
	*/
	int32 emergency_resume();

};


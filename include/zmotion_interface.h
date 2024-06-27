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
	std::vector<int> ikPosAxisIdx_ = { 7,8,9 };
	// TCP ��̬��
	std::vector<int> tcpAngleAxisIdx_ = { 10,11,12 };
	// TCP λ����
	std::vector<int> tcpPosAxisIdx_ = { 20, 21, 22 };
	// ������
	std::vector<int> appAxisIdx_ = { 6,30,31 };
	// ͹����
	std::vector<int> camAxisIdx_ = { 23, 24, 25 };
	// �ڶ���
	std::vector<int> swingAxisIdx_ = { 26,27,28 };
	// �岹ʸ����
	std::vector<int> connpathAxisIdx_ = { 15 };

	// �ڶ���־λ����
	size_t swingFlagIdx = 1000;

	// ���嵱��
	float axisUnits = 1000;


public:
	ZauxRobot();
	ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx
	);
	/**
	* @brief ͨ���������ӿ�����
	*/
	uint8_t connect_eth(char *ip_addr);
	uint8_t connect_pci(uint32 cardNum);
	uint8_t lazy_connect();

	/**
	* @brief ��¼ basic ���򵽿�����
	* @param basPath    basic ����·��
	* @param mode       ��¼ģʽ��
	         0          ��¼�� RAM
			 1          ��¼�� ROM
	*/
	uint8_t load_basic_pragma(const char *basPath, uint32_t mode = 0);
	//uint8_t load_basic_project(const char *basPath, uint32_t mode = 0);

	//uint8_t enable_zaux_log(int logMode);

	/**
	* @brief �趨���ƿ����
	* @param handle    ���ƿ����
	*/
	uint8_t set_handle(ZMC_HANDLE handle);
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
	uint8_t set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx);

	/**
	* @brief �Ͽ�����
	*/
	uint8_t disconnect();

	/**
	* @brief �л�������ģʽ��ÿ��ִ�йؽ��˶�ʱ��Ҫ��һ�Σ�ͬ�����㹤������仯
	*/
	uint8_t forward_kinematics();

	/**
	* @brief �л������ģʽ��ÿ��ִ�й�������ϵ�˶�ʱ��Ҫ��һ�Σ�ͬ������ؽ�����仯
	*/
	uint8_t inverse_kinematics();

	uint8_t wait_idle(int axisIdx);

	/**
	* @brief  ��ȡ����ĵ�ǰ�ͻ������˶�������λ��
	* @param       axisList     ��Ҫ��ȡ����������б�
	* @param       paramName    ��������
	* @param[out]  paramList    ���صĲ����б�
	*/
	uint8_t get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList);

	/**
	* @brief ���ö������
	* @param       axisList     ��Ҫ��ȡ����������б�
	* @param       paramName    ��������
	* @param       paramList    ��������
	* @param       principal    ��������: -1 ��������; >0 ����������
	*/
	uint8_t set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal = -1);

	/**
	* @brief ����table���ݵ�����
	*/
	uint8_t save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	uint8_t update_swing_table(const Weave& waveCfg);
	uint8_t swing_on(float vel, const Weave& waveCfg, const std::vector<float>& moveDir = std::vector<float>());
	uint8_t swing_off(float displacement = 0.0);

	uint8_t moveL(const std::vector<int>& axis, const std::vector<float>& relMove);
	uint8_t moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, const std::vector<float>& midPoint, const std::vector<float>& endPoint,
				  int imode = 0);

	// ��������
	uint8_t wlder_on(float current, float voltage);
	uint8_t wlder_off();

	uint8_t execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory);
	uint8_t execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory);

	uint8_t swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	uint8_t swing_tri(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	uint8_t swing_sin(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);
	uint8_t swing_tri();

	uint8_t arc_tracking_config(const Track& trackCfg);

	/**
	* @brief ��λ������ֹͣ
	*/
	uint8_t emergency_stop();
	/**
	* @brief ��λ��������ͣ
	*/
	uint8_t emergency_pause();
	/**
	* @brief ��λ�������ָ�
	*/
	uint8_t emergency_resume();

};


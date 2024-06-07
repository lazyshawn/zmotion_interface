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

	// �ں�����
	//Weave waveCfg;
	// �����켣����
	//DiscreteTrajectory<float> discreteTrajectory;

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

	uint8_t set_handle(ZMC_HANDLE handle);

	uint8_t set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx);
	/**
	* @brief ֱ�߰ڶ�
	*/
	uint8_t disconnect();

	/**
	* @brief ����ʾ����
	*/
	uint8_t trigger_scope();

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
	* @param       principal    ��������: -1 ��������; >0 ����������
	*/
	uint8_t set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal = -1);

	/**
	* @brief ����table���ݵ�����
	*/
	uint8_t save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	uint8_t swing_on(float vel, const Weave& waveCfg);
	uint8_t swing_off(float displacement = 0.0);
	uint8_t moveJ(const std::vector<float>& jntDPos);
	uint8_t moveJ_single();
	uint8_t moveL(const std::vector<float>& moveCmd);
	uint8_t moveL_single();
	uint8_t moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, const std::vector<float>& midPoint, const std::vector<float>& endPoint,
				  int imode = 0);

	// ��������
	uint8_t wlder_on(float current, float voltage);
	uint8_t wlder_off();

	/**
	* @brief ���Ӱڶ���ֱ���˶�
	* @param moveCmd    λ��ָ�ǰ����Ԫ��Ϊ TCP ��λ�ƣ�����Ԫ��Ϊ������λ��
	* @param upper      �˶�ƽ����Ϸ���
	*/
	uint8_t swingL(const std::vector<float>& moveCmd, const Weave& waveCfg);
	uint8_t swingL_(const std::vector<float>& moveCmd, const Weave& waveCfg);
	uint8_t swingLAbs(const std::vector<float>& moveCmd, const Weave& waveCfg);

	/**
	* @brief ���Ӱڶ���Բ���˶�
	* @param traj    Բ���˶����м����յ�
	* @param end     Բ���յ�
	* @param via     Բ���м��
	*/
	uint8_t swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig, const Weave& waveCfg);
	uint8_t swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig, const Weave& waveCfg);

	uint8_t execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory);
	uint8_t execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory);

	uint8_t swing_tri();
	uint8_t swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg);

	uint8_t arc_tracking_config(const Track& trackCfg);

	uint8_t emergency_stop();

};


Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end);

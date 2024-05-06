#pragma once

#include<vector>
#include<eigen3/Eigen/Dense>
#include<chrono>
#include<thread>

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
	uint8_t download_config();

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
	// �����
	std::vector<int> ikAxisIdx_ = { 7,8,9,10,11,12 };
	// ������
	std::vector<int> appAxisIdx_ = { 6 };
	// ������
	std::vector<int> toolAxisIdx_ = { 20, 21, 22, 6 };
	// ͹���˶���
	std::vector<int> camAxisIdx_ = { 23, 24, 25 };
	// �����������ͬ���˶�����Ϊ͹�ָ���Ĳο���
	//std::vector<int> virtualAxisIdx = { 26, 27, 28, 29, 30, 31 };

	// ��ͣ��ʱ��
	int leftHoldT = 0;
	// ��ͣ��ʱ��
	int rightHoldT = 0;
	// ����ͣ��ʱ��
	int midHoldT = 0;

	// �ڶ���־λ����
	size_t swingFlagIdx = 1000;
	// �ں�����
	Weave waveCfg;

	DiscreteTrajectory<float, 7> discreteTrajectory;

public:
	ZauxRobot();
	/**
	* @brief ͨ���������ӿ�����
	*/
	uint8_t connect(char *ip_addr);

	/**
	* @brief ��¼ basic ���򵽿�����
	* @param basPath    basic ����·��
	* @param mode       ��¼ģʽ��
	         0          ��¼�� RAM
			 1          ��¼�� ROM
	*/
	uint8_t load_basic_pragma(const char *basPath, uint32_t mode = 0);

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

	uint8_t swing_on();
	uint8_t swing_off();
	uint8_t moveJ(const std::vector<float>& jntDPos);
	uint8_t moveJ_single();
	uint8_t moveL(const std::vector<float>& moveCmd);
	uint8_t moveL_single();
	uint8_t moveC(const std::vector<float>& endConfig, const std::vector<float>& midConfig);

	// ��������
	uint8_t wlder_on(float current, float voltage);
	uint8_t wlder_off();

	/**
	* @brief ���Ӱڶ���ֱ���˶�
	* @param moveCmd    λ��ָ�ǰ����Ԫ��Ϊ TCP ��λ�ƣ�����Ԫ��Ϊ������λ��
	* @param upper      �˶�ƽ����Ϸ���
	*/
	uint8_t swingL(const std::vector<float>& moveCmd, Eigen::Vector3f upper);
	uint8_t swingL(const std::vector<float>& moveCmd);
	uint8_t swingL_(const std::vector<float>& moveCmd);
	uint8_t swingLAbs(const std::vector<float>& moveCmd);

	/**
	* @brief ���Ӱڶ���Բ���˶�
	* @param traj    Բ���˶����м����յ�
	* @param end     Բ���յ�
	* @param via     Բ���м��
	*/
	uint8_t swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig);
	uint8_t zswingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig);
	uint8_t swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig);

	uint8_t test();

	uint8_t execute_discrete_trajectory_abs();

	uint8_t execute_discrete_trajectory();
};

Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end);

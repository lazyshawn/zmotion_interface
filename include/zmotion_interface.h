#pragma once

#include<vector>
#include <eigen3/Eigen/Dense>

#include "zmotion.h"
#include "zauxdll2.h"

#ifndef M_PI
constexpr static double M_PI = 3.14159265358979323846;
#endif

class ZauxRobot {
public:
	enum class ConnType{INIT = 0, FK = 1, IK = 2};

	// ���������
	ZMC_HANDLE handle = NULL;
	// ��������ؽ���Ĺ���ģʽ
	ConnType connType = ConnType::INIT;
	// �ؽ���
	std::vector<int> jointAxisIdx = { 0,1,2,3,4,5 };
	// ������
	std::vector<int> toolAxisIdx = { 6,7,8,9,10,11 };
	// ͹���˶���
	std::vector<int> camAxisIdx = { 20, 21, 22, 23, 24, 25 };
	// �����������ͬ���˶�����Ϊ͹�ָ���Ĳο���
	std::vector<int> virtualAxisIdx = { 26, 27, 28, 29, 30, 31 };

	// ��ͣ��ʱ��
	int leftHoldT = 0;
	// ��ͣ��ʱ��
	int rightHoldT = 0;
	// ����ͣ��ʱ��
	int midHoldT = 0;

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

	uint8_t moveJ();
	uint8_t moveJ_single();
	uint8_t moveL();
	uint8_t moveL_single();
	uint8_t moveC();

	/**
	* @brief ���Ӱڶ���ֱ���˶�
	* @param displ    ֱ��λ��
	* @param upper    �˶�ƽ����Ϸ���
	*/
	uint8_t swingL(Eigen::Vector3f displ, Eigen::Vector3f upper);

	/**
	* @brief ���Ӱڶ���Բ���˶�
	* @param traj    Բ���˶����м����յ�
	*/
	uint8_t swingC();
};

Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end);

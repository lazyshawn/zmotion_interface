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
	// 读取已经下载到控制卡中的轴参数
	//uint8_t load_uploaded_config();
	// 加载轴参数
	uint8_t download_config();

private:
	// 轴类型
	int atype;
	// 脉冲当量
	float units;
	// 运动学参数
	float speed, accel, decel;
};

class ZauxRobot {
public:
	enum class ConnType{INIT = 0, FK = 1, IK = 2};

	// 控制器句柄
	ZMC_HANDLE handle_ = NULL;
	// 工具轴与关节轴的关联模式
	ConnType connType = ConnType::INIT;
	// 关节轴
	std::vector<int> jointAxisIdx_ = { 0,1,2,3,4,5 };
	// 逆解轴
	std::vector<int> ikAxisIdx_ = { 7,8,9,10,11,12 };
	// 附加轴
	std::vector<int> appAxisIdx_ = { 6 };
	// 工具轴
	std::vector<int> toolAxisIdx_ = { 20, 21, 22, 6 };
	// 凸轮运动轴
	std::vector<int> camAxisIdx_ = { 23, 24, 25 };
	// 和主轴进行相同的运动，作为凸轮跟随的参考轴
	//std::vector<int> virtualAxisIdx = { 26, 27, 28, 29, 30, 31 };

	// 左停留时间
	int leftHoldT = 0;
	// 右停留时间
	int rightHoldT = 0;
	// 回中停留时间
	int midHoldT = 0;

	// 摆动标志位索引
	size_t swingFlagIdx = 1000;
	// 摆焊参数
	Weave waveCfg;

	DiscreteTrajectory<float, 7> discreteTrajectory;

public:
	ZauxRobot();
	/**
	* @brief 通过网口连接控制器
	*/
	uint8_t connect(char *ip_addr);

	/**
	* @brief 烧录 basic 程序到控制器
	* @param basPath    basic 程序路径
	* @param mode       烧录模式；
	         0          烧录到 RAM
			 1          烧录到 ROM
	*/
	uint8_t load_basic_pragma(const char *basPath, uint32_t mode = 0);

	/**
	* @brief 直线摆动
	*/
	uint8_t disconnect();

	/**
	* @brief 触发示波器
	*/
	uint8_t trigger_scope();

	/**
	* @brief 切换到正解模式：每次执行关节运动时需要绑定一次，同步计算工具坐标变化
	*/
	uint8_t forward_kinematics();

	/**
	* @brief 切换到逆解模式：每次执行工具坐标系运动时需要绑定一次，同步计算关节坐标变化
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

	// 焊机控制
	uint8_t wlder_on(float current, float voltage);
	uint8_t wlder_off();

	/**
	* @brief 叠加摆动的直线运动
	* @param moveCmd    位移指令，前三个元素为 TCP 点位移，后续元素为附加轴位移
	* @param upper      运动平面的上方向
	*/
	uint8_t swingL(const std::vector<float>& moveCmd, Eigen::Vector3f upper);
	uint8_t swingL(const std::vector<float>& moveCmd);
	uint8_t swingL_(const std::vector<float>& moveCmd);
	uint8_t swingLAbs(const std::vector<float>& moveCmd);

	/**
	* @brief 叠加摆动的圆弧运动
	* @param traj    圆弧运动的中间点和终点
	* @param end     圆弧终点
	* @param via     圆弧中间点
	*/
	uint8_t swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig);
	uint8_t zswingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig);
	uint8_t swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig);

	uint8_t test();

	uint8_t execute_discrete_trajectory_abs();

	uint8_t execute_discrete_trajectory();
};

Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end);

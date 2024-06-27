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
	// 读取已经下载到控制卡中的轴参数
	//uint8_t load_uploaded_config();
	// 加载轴参数
	//uint8_t download_config();
	// 获取缓冲位置
	//std::vector<float> get_axis_param(const std::vector<int>& axisList);

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
	// 逆解位置轴
	std::vector<int> ikPosAxisIdx_ = { 7,8,9 };
	// TCP 姿态轴
	std::vector<int> tcpAngleAxisIdx_ = { 10,11,12 };
	// TCP 位置轴
	std::vector<int> tcpPosAxisIdx_ = { 20, 21, 22 };
	// 附加轴
	std::vector<int> appAxisIdx_ = { 6,30,31 };
	// 凸轮轴
	std::vector<int> camAxisIdx_ = { 23, 24, 25 };
	// 摆动轴
	std::vector<int> swingAxisIdx_ = { 26,27,28 };
	// 插补矢量轴
	std::vector<int> connpathAxisIdx_ = { 15 };

	// 摆动标志位索引
	size_t swingFlagIdx = 1000;

	// 脉冲当量
	float axisUnits = 1000;


public:
	ZauxRobot();
	ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx
	);
	/**
	* @brief 通过网口连接控制器
	*/
	uint8_t connect_eth(char *ip_addr);
	uint8_t connect_pci(uint32 cardNum);
	uint8_t lazy_connect();

	/**
	* @brief 烧录 basic 程序到控制器
	* @param basPath    basic 程序路径
	* @param mode       烧录模式；
	         0          烧录到 RAM
			 1          烧录到 ROM
	*/
	uint8_t load_basic_pragma(const char *basPath, uint32_t mode = 0);
	//uint8_t load_basic_project(const char *basPath, uint32_t mode = 0);

	//uint8_t enable_zaux_log(int logMode);

	/**
	* @brief 设定控制卡句柄
	* @param handle    控制卡句柄
	*/
	uint8_t set_handle(ZMC_HANDLE handle);
	/**
	* @brief 设定轴号
	* @param jointAxisIdx       关节轴
	* @param ikPosAxisIdx       逆解位置轴
	* @param tcpAngleAxisIdx    世界坐标系下 TCP 角度轴: 逆解姿态轴
	* @param tcpPosAxisIdx      世界坐标系下 TCP 位置轴: 添加附加轴后的 TCP 位置
	* @param appAxisIdx         附加轴
	* @param camAxisIdx         凸轮轴
	* @param swingAxisIdx       摆动轴
	* @param connpathAxisIdx    插补矢量轴
	*/
	uint8_t set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx);

	/**
	* @brief 断开连接
	*/
	uint8_t disconnect();

	/**
	* @brief 切换到正解模式：每次执行关节运动时需要绑定一次，同步计算工具坐标变化
	*/
	uint8_t forward_kinematics();

	/**
	* @brief 切换到逆解模式：每次执行工具坐标系运动时需要绑定一次，同步计算关节坐标变化
	*/
	uint8_t inverse_kinematics();

	uint8_t wait_idle(int axisIdx);

	/**
	* @brief  读取多轴的当前和缓冲中运动的最终位置
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param[out]  paramList    返回的参数列表
	*/
	uint8_t get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList);

	/**
	* @brief 设置多轴参数
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param       paramList    参数数组
	* @param       principal    主轴索引: -1 立即设置; >0 缓冲中设置
	*/
	uint8_t set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal = -1);

	/**
	* @brief 保存table数据到本地
	*/
	uint8_t save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	uint8_t update_swing_table(const Weave& waveCfg);
	uint8_t swing_on(float vel, const Weave& waveCfg, const std::vector<float>& moveDir = std::vector<float>());
	uint8_t swing_off(float displacement = 0.0);

	uint8_t moveL(const std::vector<int>& axis, const std::vector<float>& relMove);
	uint8_t moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, const std::vector<float>& midPoint, const std::vector<float>& endPoint,
				  int imode = 0);

	// 焊机控制
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
	* @brief 上位机紧急停止
	*/
	uint8_t emergency_stop();
	/**
	* @brief 上位机紧急暂停
	*/
	uint8_t emergency_pause();
	/**
	* @brief 上位机紧急恢复
	*/
	uint8_t emergency_resume();

};


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
	std::vector<int> ikPosAxisIdx_ = { 9,10,11 };
	// TCP 姿态轴
	std::vector<int> tcpAngleAxisIdx_ = { 12,13,14 };
	// TCP 位置轴
	std::vector<int> tcpPosAxisIdx_ = { 18, 19, 20 };
	// 附加轴
	std::vector<int> appAxisIdx_ = { 6,7,8 };
	// 凸轮轴
	std::vector<int> camAxisIdx_ = { 36,37,38 };
	// 摆动轴
	std::vector<int> swingAxisIdx_ = { 40,41,42 };
	// 插补矢量轴
	std::vector<int> connpathAxisIdx_ = { 39 };
	// 过渡轴
	std::vector<int> transAxisIdx_ = { 18,19,20,21,22,23,24,25,26 };

	// 摆动标志位索引
	size_t swingFlagIdx = 1000;

	// 脉冲当量
	float axisUnits = 1000;


public:
	ZauxRobot() {}
	ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx
	);
	/**
	* @brief 通过网口连接控制器
	*/
	int32 connect_eth(char *ip_addr);
	int32 connect_pci(uint32 cardNum);
	int32 lazy_connect();
	int32 connect(std::string addr);

	/**
	* @brief 烧录 basic 程序到控制器
	* @param basPath    basic 程序路径
	* @param mode       烧录模式；
	         0          烧录到 RAM
			 1          烧录到 ROM
	*/
	int32 load_basic_pragma(const char *basPath, uint32_t mode = 0);
	//uint8_t load_basic_project(const char *basPath, uint32_t mode = 0);

	//uint8_t enable_zaux_log(int logMode);

	/**
	* @brief 设定控制卡句柄
	* @param handle    控制卡句柄
	*/
	int32 set_handle(ZMC_HANDLE handle);
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
	int32 set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
		const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& swingAxisIdx,
		const std::vector<int>& connpathAxisIdx);

	/**
	* @brief 断开连接
	*/
	int32 disconnect();

	/**
	* @brief 切换到正解模式：每次执行关节运动时需要绑定一次，同步计算工具坐标变化
	*/
	int32 forward_kinematics(int delay = 1e3);

	/**
	* @brief 切换到逆解模式：每次执行工具坐标系运动时需要绑定一次，同步计算关节坐标变化
	*/
	int32 inverse_kinematics(int delay = 1e3);

	int32 wait_idle(int axisIdx);

	/**
	* @brief  读取多轴参数
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param[out]  paramList    返回的参数列表
	*/
	int32 get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList);

	/**
	* @brief 设置多轴参数
	* @param       axisList     需要获取参数的轴号列表
	* @param       paramName    参数名称
	* @param       paramList    参数数组
	* @param       principal    主轴索引: -1 立即设置; >0 缓冲中设置
	*/
	int32 set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal = -1);

	/**
	* @brief 保存table数据到本地
	*/
	int32 save_table(size_t startIdx, size_t num = 1, const std::string& path = "./tableData.txt");

	int32 update_swing_table(const Weave& waveCfg);
	int32 swing_on(float vel, const Weave& waveCfg, const std::vector<float>& toolDir = std::vector<float>());
	int32 swing_off(float displacement = 0.0);

	/**
	* @brief 轴点动
	* @param       idx     运动轴号
	* @param       type    运动类型
					  0      运动结束
					  ±1    关节正/负向运动
					  ±2    世界坐标系正/负向运动
					  ±3    工具坐标系正/负向运动
	* @param       dir     运动方向
	*/
	int32 jog_moving(int idx, int type, int dir);

	/**
	* @brief movePtp
	* @param       axis          关节运动轴号
	* @param       relEndMove    相对运动距离
	* @param       speedRatio    速度比率
	*/
	int32 move_ptp(const std::vector<float>& relEndMove, float speedRatio = 1.0);
	int32 move_ptp_abs(const std::vector<float>& endMove, float speedRatio = 1.0);

	/**
	* @brief moveJ
	* @param       axis          关节运动轴号
	* @param       relEndMove    相对运动距离
	* @param       speedRatio    速度比率
	*/
	int32 moveJ(const std::vector<float>& relEndMove, float speedRatio = 1.0);
	int32 moveJ(const std::vector<int>& axis, const std::vector<float>& relEndMove, float speedRatio = 1.0);

	/**
	* @brief moveJ_abs
	* @param      axis          关节运动轴号
	* @param      endMove       绝对运动位置
	* @param      speedRatio    速度比率
	*/
	int32 moveJ_abs(const std::vector<float>& endMove, float speedRatio = 1.0);
	int32 moveJ_abs(const std::vector<int>& axis, const std::vector<float>& endMove, float speedRatio = 1.0);

	int32 moveL(const std::vector<int>& axis, const std::vector<float>& relMove);
	int32 moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, std::vector<float>& midPoint, std::vector<float>& endPoint,
				  int imode = 0);

	// 焊机控制
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
	* @brief 上位机紧急停止
	*/
	int32 emergency_stop();
	/**
	* @brief 上位机紧急暂停
	*/
	int32 emergency_pause();
	/**
	* @brief 上位机紧急恢复
	*/
	int32 emergency_resume();

};


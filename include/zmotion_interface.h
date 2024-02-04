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
	// 控制器句柄
	ZMC_HANDLE handle = NULL;
	std::vector<int> jointAxisIdx = { 0,1,2,3,4,5 };
	std::vector<int> toolAxisIdx = { 6,7,8,9,10,11 };
	// 凸轮运动轴
	std::vector<int> camAxisIdx = { 20, 21, 22, 23, 24, 25 };
	// 和主轴进行相同的运动，作为凸轮跟随的参考轴
	std::vector<int> virtualAxisIdx = { 26, 27, 28, 29, 30, 31 };
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

	uint8_t moveJ();
	uint8_t moveJ_single();
	uint8_t moveL();
	uint8_t moveL_single();
	uint8_t moveC();

	/**
	* @brief 叠加正弦摆动的直线运动
	* @param displ    直线位移
	* @param upper    运动平面的上方向
	*/
	uint8_t sin_swingL(Eigen::Vector3f displ, Eigen::Vector3f upper);
};


#include<chrono>
#include<thread>
#include<iostream>

#include"zmotion_interface.h"

// 轴号占用列表
std::vector<int> ZauxAxis::AxisIdxList = std::vector<int>(100, 0);

ZauxAxis::ZauxAxis() {
}

ZauxRobot::ZauxRobot() {
}

uint8_t ZauxRobot::connect(char *ip_addr) {
	printf("Connecting to: %s\n", ip_addr);
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle_)) {
		printf("Connect controller failed!\nPress <Enter> exist!\n");
		handle_ = NULL;
		getchar();
		return 1;
	}
	return 0;
}

uint8_t ZauxRobot::load_basic_pragma(const char *basPath, uint32_t mode) {
	// 加载 bas 程序
	if (ZAux_BasDown(handle_, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return 1;
	}
	// 等待 bas 程序下载
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::disconnect() {
	//关闭连接 
	if (ZAux_Close(handle_) > 0) {
		printf("Error: # ZauxRobot::disconnect()!\n");
		return 1;
	}
	printf("connection closed!\n");
	handle_ = NULL;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::trigger_scope() {
	ZAux_Trigger(handle_);
	return 0;
}

uint8_t ZauxRobot::forward_kinematics() {
	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 0 && kinematics < 2) {
		return 1;
	}
	// 等待末端运动结束，工具轴+附加轴
	wait_idle(20);
	//if (connType == ConnType::FK) {
	//	return 1;
	//}
	int ret = 0;
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connreframe()
	//ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	ret = ZAux_Direct_Connreframe(handle_, baseAxis.size(), baseAxis.data(), 93, 100, connreAxis.size(), connreAxis.data());
	ret = ZAux_Direct_Connreframe(handle_, ikAxisIdx_.size(), ikAxisIdx_.data(), 6, 0, jointAxisIdx_.size(), jointAxisIdx_.data());

	connType = ConnType::FK;
	ZAux_Direct_SetUserVar(handle_, "kinematic", 1);
	return 0;
}

uint8_t ZauxRobot::inverse_kinematics() {
	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 1) {
		return 1;
	}
	// 等待关节运动结束，关节轴+附加轴
	wait_idle(jointAxisIdx_[0]);
	//if (connType == ConnType::IK) {
	//	return 1;
	//}
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connframe()
	//ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, toolAxisIdx.size(), toolAxisIdx.data());
	int ret = 0;
	ret = ZAux_Direct_Connframe(handle_, jointAxisIdx_.size(), jointAxisIdx_.data(), 6, 0, ikAxisIdx_.size(), ikAxisIdx_.data());
	ret = ZAux_Direct_Connframe(handle_, connreAxis.size(), connreAxis.data(), 93, 100, baseAxis.size(), baseAxis.data());
	connType = ConnType::IK;
	ret = ZAux_Direct_SetUserVar(handle_, "kinematic", 2);

	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	wait_idle(toolAxisIdx_[0]);
	sprintf(cmdbuff, "VECTOR_MOVED(%d)=0", toolAxisIdx_[0]);
	ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	return 0;
}

uint8_t ZauxRobot::wait_idle(int axisIdx) {
	float IDLE;
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		ZAux_Direct_GetParam(handle_, "IDLE", axisIdx, &IDLE);
		if (IDLE < 0)break;
	}
	return 0;
}

uint8_t ZauxRobot::moveJ(const std::vector<float>& jntDPos) {
	forward_kinematics();

	std::vector<float> cmd(jntDPos.begin(), jntDPos.begin() + jointAxisIdx_.size() + appAxisIdx_.size());
	std::vector<int> axis = jointAxisIdx_;
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	ZAux_Direct_MoveAbs(handle_, axis.size(), axis.data(), cmd.data());
	return 0;
}

uint8_t ZauxRobot::moveJ_single() {
	return 0;
}

uint8_t ZauxRobot::moveL(const std::vector<float>& moveCmd) {
	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	int axisList[] = { 20, 21, 22, 6 };
	float dist[] = { moveCmd[0], moveCmd[1], moveCmd[2], moveCmd[3] };
	ZAux_Direct_Move(handle_, 3, axisList, dist);
	return 0;
}

uint8_t ZauxRobot::moveL_single() {
	return 0;
}

uint8_t ZauxRobot::moveC(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
	// 切换到逆解模式, 关联逆解
	inverse_kinematics();
	//ZAux_Direct_MSphericalAbs(handle_, 6, toolAxisIdx_.data(), 800, 400, 300, 900, 200, 600, 0, 20, 20, 50);
	ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	return 0;
}

uint8_t ZauxRobot::swing_on() {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	// 句柄号
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴号 + 附加轴轴号
	std::vector<int> toolAxisIdx = { toolAxisIdx_[0],toolAxisIdx_[1],toolAxisIdx_[2],toolAxisIdx_[3] };
	// TCP 姿态轴号
	std::vector<int> poseAxisIdx = { ikAxisIdx_[3], ikAxisIdx_[4], ikAxisIdx_[5] };
	// 凸轮轴轴号
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2] };

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 10.0;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// 机器人停止
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// 摆动停留时间
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}

	// 四分之一摆动周期占用的 table 个数
	size_t numQuarter = numInterp * (1000 / freq) / (1000 / freq + 2 * swingHoldTime) / 4;
	// 右停留时间占用的 table 个数
	size_t numRightHold = (numInterp - 4 * numQuarter) / 2;
	std::vector<float> sinTableValue(numInterp);
	// 缓冲中写入凸轮表
	for (size_t i = 0; i < 4 * numQuarter; ++i) {
		// 插值点对应的偏移幅值
		float tmp = std::sin(2 * M_PI * i / (4 * numQuarter - 1));
		if (i < numQuarter) {
			sinTableValue[i] = tmp;
		}
		else if (i < 3 * numQuarter) {
			sinTableValue[numRightHold + i] = tmp;
		}
		else {
			sinTableValue[numInterp - 4 * numQuarter + i] = tmp;
		}
	}
	for (size_t i = 0; i < numRightHold; ++i) {
		sinTableValue[numQuarter + i] = 1;
	}
	for (size_t i = 0; i < numInterp - 4 * numQuarter - numRightHold; ++i) {
		sinTableValue[3 * numQuarter + numRightHold + i] = -1;
	}
	ZAux_Direct_SetTable(handle, sinTableBeg, numInterp, sinTableValue.data());

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	// 周期长度
	float dist = vel * (1/freq + 2*swingHoldTime/1000);
	// 焊枪方向
	Eigen::Vector3f zDir(0, 0, 1);

	float vectorBuffered2 = 0.0;
	ret = ZAux_Direct_GetVariablef(handle_, "VECTOR_BUFFERED2(20)", &vectorBuffered2);

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, 主轴, 矢量距离, 周期长度, 左右摆幅, 开始Table, 结束Table
		4, toolAxisIdx_[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1
	);
	std::cout << cmdbuff  << std::endl;

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	float setTableValue[1] = { 1 };
	ZAux_Direct_SetTable(handle_, 1000, 1, setTableValue);
	return ret;
}

uint8_t ZauxRobot::swing_off() {
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];

	float vectorBuffered2 = 0.0;
	ZAux_Direct_GetVariablef(handle_, "VECTOR_BUFFERED2(20)", &vectorBuffered2);

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nconn_swing(%d,%d,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, 主轴, 矢量距离
		-1, toolAxisIdx_[0], vectorBuffered2
	);
	std::cout << cmdbuff << std::endl;

	//调用命令执行函数
	int ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	wait_idle(20);
	float setTableValue[1] = { -1 };
	ZAux_Direct_SetTable(handle_, 1000, 1, setTableValue);

	return ret;
}

uint8_t ZauxRobot::wlder_on(float current, float voltage) {
	// 开气
	//ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 2, 5, 1);

	uint16_t dec = 0;
	uint8_t low = 0, high = 0;
	// 设置电流
	dec = std::floor(current * 65535 / 550);
	low = dec & 0x00FF;
	high = (dec >> 8) & 0x00FF;
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 5, 5, low);
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 6, 5, high);

	// 设置电压
	dec = std::floor(voltage * 65535 / 60);
	low = dec & 0x00FF;
	high = (dec >> 8) & 0x00FF;
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 7, 5, low);
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 8, 5, high);

	// 焊接模式(直流一元) & 开始焊接
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 1, 5, 7);

	return 0;
}

uint8_t ZauxRobot::wlder_off() {
	// 关气
	//ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 2, 5, 0);

	// 结束焊接
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 1, 5, 6);
	return 0;
}

uint8_t ZauxRobot::swingL(const std::vector<float>& moveCmd) {
	// 句柄号
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴号 + 附加轴轴号
	std::vector<int> toolAxisIdx = { toolAxisIdx_[0],toolAxisIdx_[1],toolAxisIdx_[2],toolAxisIdx_[3] };
	// TCP 姿态轴号
	std::vector<int> poseAxisIdx = { ikAxisIdx_[3], ikAxisIdx_[4], ikAxisIdx_[5] };
	// 凸轮轴轴号
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2] };
	// 跟随矢量轴轴号
	int connpathAxisIdx = 15;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 10.0;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// 机器人停止
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// 摆动停留时间
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}

	// *** 计算摆焊参数 ***************************************
	// 读取缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, poseAxisIdx[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// 读取脉冲当量
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// TCP 点位移
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// 直线轨迹的朝向
	Eigen::Vector3f displDir = displ.normalized();
	// 偏移方向
	Eigen::Vector3f offDir = zDir.cross(displ).normalized();
	// 波形反向
	offDir *= waveCfg.Phase > 0 ? -1 : 1;
	// 总位移(mm)
	float dist = displ.norm();
	// 总时间(s)
	float totalTime = dist / vel;
	// 周期数
	size_t numPeriod = std::ceil(1000 * totalTime / (1000 / freq + 2 * swingHoldTime));

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle, connpathAxisIdx, (char*)"TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], connpathAxisIdx);

	// *** 设置凸轮虚拟轴 *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 等待摆动标志位置位
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], (char*)"TABLE", swingFlagIdx, 1, 0);
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg + numInterp - 1, ampl * units * offDir[i], dist / numPeriod, connpathAxisIdx, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// 设置附加轴不参与速度计算
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	}

	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", connpathAxisIdx, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴位置清零
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", camAxisIdx[i], 0);
	}

	// 偏移方向和深度方向写入 Table
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], 1004 + i, offDir[i]);
		ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], 1007 + i, zDir[i]);
	}

	// 四分之一摆动周期占用的 table 个数
	size_t numQuarter = numInterp * (1000 / freq) / (1000 / freq + 2 * swingHoldTime) / 4;
	// 右停留时间占用的 table 个数
	size_t numRightHold = (numInterp - 4 * numQuarter) / 2;
	// 缓冲中写入凸轮表
	for (size_t i = 0; i < 4 * numQuarter; ++i) {
		// 插值点对应的偏移幅值
		float tmp = std::sin(2 * M_PI * i / (4 * numQuarter - 1));
		if (i < numQuarter) {
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + i, tmp);
		}
		else if (i < 3 * numQuarter) {
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numRightHold + i, tmp);
		}
		else {
			size_t tmpIdx = sinTableBeg + numInterp - 4*numQuarter + i;
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp - 4*numQuarter + i, tmp);
		}
	}
	for (size_t i = 0; i < numRightHold; ++i) {
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numQuarter + i, 1);
	}
	for (size_t i = 0; i < numInterp - 4 * numQuarter - numRightHold; ++i) {
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + 3*numQuarter + numRightHold + i, -1);
	}

	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// 开始运动指令
	std::vector<float> cmd = { moveCmd[0] / numPeriod / 4, moveCmd[1] / numPeriod / 4, moveCmd[2] / numPeriod / 4, moveCmd[3] / numPeriod / 4 };

	// 1/4 周期
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	if (robotHoldTime > 0)
		ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], robotHoldTime);
	
	for (size_t i = 0; i < cmd.size(); ++i) cmd[i] *= 2;
	for (size_t i = 0; i < numPeriod - 1; ++i) {
		// 开始采集电流电压
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);
		ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
		if (robotHoldTime > 0)
			ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], robotHoldTime);
		ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
		if (robotHoldTime > 0)
			ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], robotHoldTime);
	}

	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	if (robotHoldTime > 0)
		ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], robotHoldTime);

	for (size_t i = 0; i < cmd.size(); ++i) cmd[i] /= 2;
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], connpathAxisIdx, 0);

	// 附加轴恢复计算插补速度
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}

	return 0;
}
uint8_t ZauxRobot::swingL_(const std::vector<float>& moveCmd) {
	// 句柄号
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴号 + 附加轴轴号
	std::vector<int> toolAxisIdx = { toolAxisIdx_[0],toolAxisIdx_[1],toolAxisIdx_[2],toolAxisIdx_[3] };
	// TCP 姿态轴号
	std::vector<int> poseAxisIdx = { ikAxisIdx_[3], ikAxisIdx_[4], ikAxisIdx_[5] };
	// 凸轮轴轴号
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_ [1], camAxisIdx_[2] };
	// 跟随矢量轴轴号
	int connpathAxisIdx = 15;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 10.0;
	// 停留时间
	float holdTime = (waveCfg.Dwell_left+ waveCfg.Dwell_right) / 2;

	// *** 计算摆焊参数 ***************************************
	// 读取缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, poseAxisIdx[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// 读取脉冲当量
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// TCP 点位移
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// 直线轨迹的朝向
	Eigen::Vector3f displDir = displ.normalized();
	// 偏移方向
	Eigen::Vector3f offDir = zDir.cross(displ).normalized();
	// 波形反向
	offDir *= waveCfg.Phase > 0 ? -1 : 1;
	// 总位移
	float dist = displ.norm();
	// 周期数
	size_t numPeriod = std::ceil(dist / vel * freq * units) / units;

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle, connpathAxisIdx, (char*)"TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], connpathAxisIdx);

	// *** 设置凸轮虚拟轴 *************************************
	// INT(SQR(dx*dx + dy * dy + dz * dz) * UNITS(20)) / UNITS(20)
	// 等待摆动标志位置位
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], (char*)"TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg + numInterp - 1, ampl * units * offDir[i], dist / numPeriod, connpathAxisIdx, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// 设置附加轴不参与速度计算
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	}

	// 移动到最右侧
	std::vector<float> cmd = moveCmd;
	for (size_t i = 0; i < 3; ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 4 + ampl * offDir[i];
	}
	cmd[3] = moveCmd[3] / numPeriod / 4;
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", connpathAxisIdx, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴位置清零
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", camAxisIdx[i], 0);
	}

	// 偏移方向和深度方向写入 Table
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], 1004 + i, offDir[i]);
		ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], 1007 + i, zDir[i]);
	}

	// 缓冲中写入凸轮表
	for (size_t i = 0; i < numInterp; ++i) {
		// 插值点对应的偏移幅值
		float tmp = std::sin(M_PI/2 + 2 * M_PI * i / (numInterp - 1));
		// 左侧摆宽
		// tmp *= i > numInterp / 2 ? (waveCfg.LeftWidth / waveCfg.RightWidth) : 1;
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + i, tmp);
	}

	
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// 开始运动指令
	//std::vector<float> cmd(moveCmd.begin(), moveCmd.begin() + toolAxisIdx.size());
	// ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	for (size_t i = 0; i < moveCmd.size(); ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 2;
	}
	for (size_t i = 0; i < numPeriod - 1; ++i) {
		// 开始采集电流电压
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);
		ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
		if (holdTime > 0) {
			ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], holdTime);
		}
		ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
		if (holdTime > 0) {
			ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], holdTime);
		}
	}
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	if (holdTime > 0) {
		ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], holdTime);
	}

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], connpathAxisIdx, 0);

	// 运动到摆动结束位置
	for (size_t i = 0; i < 3; ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 4 + ampl * offDir[i];
	}
	cmd[3] = moveCmd[3] / numPeriod / 4;
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// 附加轴恢复计算插补速度
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}


	return 0;
}
uint8_t ZauxRobot::swingLAbs(const std::vector<float>& moveCmd) {
	float curEndMove[4] = { 0,0,0,0 };
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 20 + i, &curEndMove[i]);
	}
	swingL({ moveCmd[0] - curEndMove[0], moveCmd[1] - curEndMove[1], moveCmd[2] - curEndMove[2], moveCmd[3] - curEndMove[3] });
	return 0;
}

uint8_t ZauxRobot::swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
#pragma region swing_config
	// 句柄号
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴号 + 附加轴轴号
	std::vector<int> toolAxisIdx = { toolAxisIdx_[0],toolAxisIdx_[1],toolAxisIdx_[2],toolAxisIdx_[3] };
	// TCP 姿态轴号
	std::vector<int> poseAxisIdx = { ikAxisIdx_[3], ikAxisIdx_[4], ikAxisIdx_[5] };
	// 凸轮轴轴号
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2] };
	// 跟随矢量轴轴号
	int connpathAxisIdx = 15;

	
	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	// *** 获取的摆焊参数 *************************************
// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 10.0;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// 机器人停止
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// 摆动停留时间
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterpInPeriod = 20;
	
	Eigen::Vector3f begAngle, detAngle{ 0,0,0 };

	// *** 计算摆焊参数 ***************************************
	// 读取当前缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, 10 + i, &zEuler[i]);
	}
	begAngle = zEuler;
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// 读取脉冲当量
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// 读取圆弧起点，当前缓冲中的最终位置
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, toolAxisIdx[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// 计算圆心坐标
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// 半径方向
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// 处理半径过大的情况
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// 圆弧运动平面的法线方向
	Eigen::Vector3f normal = op1.cross(op3);
	// 圆心角角度
	float theta = std::acos(op1.dot(op3));
	// 修正圆心角和正法向
	// 2,3 在 1 的两侧
	if (n12.dot(n13) < 0) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q12 > q13
	else if (q12 > q13) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q13 > q12
	else if (q13 > q12) {
		normal = n12;
	}
	normal.normalize();

	// 初始切线方向
	//Eigen::Vector3f tanDirBase = (normal.cross(op1)).normalized();
	// 初始摆动方向
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	// 初始深度方向
	Eigen::Vector3f depthDirBase = (normal.cross(op1).cross(zDir)).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// 总距离
	float dist = std::floor((begPnt - center).norm() * theta * units) / units;
	// 周期数
	size_t numPeriod = std::ceil(dist / vel * freq);
	// 整条路径上的插值点
	size_t numInterp = numPeriod * numInterpInPeriod;

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle, 15, "TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], 15);

	// *** 设置凸轮虚拟轴 *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	// 等待摆动标志位置位
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg + numInterp * i, sinTableBeg + numInterp * (i + 1) - 1, ampl * 1000, dist, 15, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);
#pragma endregion swing_config

	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴位置清零
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", camAxisIdx[i], 0);
	}

	int iBuffLen = 0;
	while (iBuffLen < 1000) {
		int tmpBuffLen = 9999;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// 最小的缓冲数
		for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
			int tmp = 9999;
			ZAux_Direct_GetRemain_LineBuffer(handle, toolAxisIdx[0], &tmp);
			tmpBuffLen = std::min(tmpBuffLen, tmp);
		}
		iBuffLen = tmpBuffLen;
	}

	// 缓冲中写入凸轮表
	// 插值点对应的圆心角
	float dq = theta / (numInterp - 1), curQ = 0.0;
	Eigen::Vector3f offDir(0, 0, 0), depthDir(0, 0, 0);
	// 记录凸轮表数据的数组
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			// 偏移方向
			// Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			offDir = (Eigen::AngleAxisf(curQ, normal) * offDirBase).normalized();
			// 切线方向
			//Eigen::Vector3f tanDir = normal.cross(offDir);
			// 偏移相位角
			float off = sin(float(j) / (numInterpInPeriod) * 2 * M_PI);

			// 记录凸轮表
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// 更新当前角度
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 3 - 1, 0);
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// 开始运动指令
	//ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	dq = theta / numPeriod / 2;
	curQ = 0.0;
	op1 = begPnt - center;
	if (endConfig.size() > 5) {
		for (size_t i = 0; i < 3; ++i) {
			detAngle[i] = (endConfig[3 + i] - begAngle[i]) / numPeriod / 2;
		}
	}
	int toolAxisPoseIdx[6] = { toolAxisIdx[0], toolAxisIdx[1], toolAxisIdx[2], poseAxisIdx[0], poseAxisIdx[1], poseAxisIdx[2] };
	Eigen::Vector3f curAngle = begAngle;
	Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
	Eigen::Matrix3f trans;

	// 1/4 周期
	curQ += dq/4;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle/4;
	curQ += dq/4;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle/4;
	// 周期开始标志
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);
	trans = Eigen::AngleAxisf(curQ, normal).matrix();
	// 偏移方向
	offDir = (trans * offDirBase).normalized();
	// 深度方向
	depthDir = (trans * depthDirBase).normalized();
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1004 + i, offDir[i]);
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1007 + i, depthDir[i]);
	}

	for (size_t i = 0; i < numPeriod-1; ++i) {
		curQ += dq / 2;
		curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle += detAngle / 2;
		curQ += dq / 2;
		curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle += detAngle / 2;

		// 周期开始标志
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);

		//ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
		//std::cout << "ret = " << ret << std::endl;
		//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);
		trans = Eigen::AngleAxisf(curQ, normal).matrix();
		// 偏移方向
		offDir = (trans * offDirBase).normalized();
		// 深度方向
		depthDir = (trans * depthDirBase).normalized();
		for (size_t i = 0; i < 3; ++i) {
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1004 + i, offDir[i]);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1007 + i, depthDir[i]);
		}

		curQ += dq / 2;
		curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle += detAngle / 2;
		curQ += dq / 2;
		curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle += detAngle / 2;
		//ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
		//std::cout << "ret = " << ret << std::endl;
		//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);
	}

	// 1/2 周期
	curQ += dq / 2;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 2;
	curQ += dq / 2;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle;
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);

	// 1/4 周期
	curQ += dq / 4;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 4;
	curQ += dq / 4;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 4;
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], 15, 0);
	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	return 0;
}

uint8_t ZauxRobot::zswingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 10.0;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = holdType > 0 ? (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2 : 0;

	//// 切换到逆解模式, 关联逆解
	//inverse_kinematics();

	// *** 计算摆焊参数 ***************************************
	// 读取当前缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 10 + i, &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// 读取脉冲当量
	float units = 0.0;
	ZAux_Direct_GetUnits(handle_, toolAxisIdx_[0], &units);

	// 读取圆弧起点，当前缓冲中的最终位置
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx_[i], &begPnt[i]);
		begPnt[i] = std::floor(begPnt[i] * units) / units;
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// 计算圆心坐标
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// 半径方向
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// 处理半径过大的情况
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// 圆弧运动平面的法线方向
	Eigen::Vector3f normal = op1.cross(op3);
	// 圆心角角度
	float theta = std::acos(op1.dot(op3));
	// 修正圆心角和正法向
	// 2,3 在 1 的两侧
	if (n12.dot(n13) < 0) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q12 > q13
	else if (q12 > q13) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q13 > q12
	else if (q13 > q12) {
		normal = n12;
	}
	normal.normalize();

	//// 初始摆动方向
	//Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	//std::cout << "zDir       = " << zDir.transpose() << std::endl;
	//std::cout << "normal     = " << normal.transpose() << std::endl;
	//std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	//// 总距离
	//int distUnits = std::floor((begPnt - center).norm() * theta * units);
	//float dist = distUnits / units;
	//// 周期数
	//size_t numPeriod = std::ceil(dist / vel * freq);
	//
	//// 缓冲中写入凸轮表
	//float dq = theta / numPeriod / 2, curQ = 0.0;
	//op1 = begPnt - center;

	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	size_t numPeriod = 10;
	float curQ = 0.0, dq = theta / numPeriod / 2;
	// 记录凸轮表数据的数组
	for (size_t i = 0; i < numPeriod; ++i) {
		// 当前周期的运动
		Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
		curQ += dq;
		curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curQ += dq;
		curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		int ret = ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		//ZAux_Direct_MoveDelay(handle_, toolAxisIdx_[0], 1000);
		std::cout << ret << ", pos = " << curEnd.transpose() << std::endl;
	}

	return 0;
}

uint8_t ZauxRobot::swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
#pragma region swing_config
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = waveCfg.Width / 2;
	// 焊接速度
	float vel = 5.0;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterpInPeriod = 20;

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	Eigen::Vector3f begAngle, detAngle{ 0,0,0 };

	// *** 计算摆焊参数 ***************************************
	// 读取当前缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 10 + i, &zEuler[i]);
	}
	begAngle = zEuler;
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// 读取脉冲当量
	float units = 0.0;
	ZAux_Direct_GetUnits(handle_, toolAxisIdx_[0], &units);

	// 读取圆弧起点，当前缓冲中的最终位置
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx_[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// 计算圆心坐标
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// 半径方向
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// 处理半径过大的情况
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// 圆弧运动平面的法线方向
	Eigen::Vector3f normal = op1.cross(op3);
	// 圆心角角度
	float theta = std::acos(op1.dot(op3));
	// 修正圆心角和正法向
	// 2,3 在 1 的两侧
	if (n12.dot(n13) < 0) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q12 > q13
	else if (q12 > q13) {
		normal = -n13;
		theta = 2 * M_PI - theta;
	}
	// q13 > q12
	else if (q13 > q12) {
		normal = n12;
	}
	normal.normalize();

	// 初始摆动方向
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// 总距离
	float dist = std::floor((begPnt - center).norm() * theta * units) / units;
	// 周期数
	size_t numPeriod = std::ceil(dist / vel * freq);
	// 整条路径上的插值点
	size_t numInterp = numPeriod * numInterpInPeriod;

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle_, 15, "TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle_, 1, toolAxisIdx_[0], 15);

	// *** 设置凸轮虚拟轴 *************************************
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle_, toolAxisIdx_[i], camAxisIdx_[i]);
	}
	// 等待摆动标志位置位
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveWait(handle_, camAxisIdx_[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle_, camAxisIdx_[i], sinTableBeg + numInterp * i, sinTableBeg + numInterp * (i + 1) - 1, ampl * 1000, dist, 15, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetSpeed(handle_, toolAxisIdx_[i], vel);
	}
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);
#pragma endregion swing_config

	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// 凸轮轴位置清零
		ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", camAxisIdx_[i], 0);
	}

	// 缓冲中写入凸轮表
	// 插值点对应的圆心角
	float dq = theta / (numInterp - 1), curQ = 0.0;
	// 记录凸轮表数据的数组
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			// 偏移方向
			// Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			Eigen::Vector3f offDir = (Eigen::AngleAxisf(curQ, normal) * offDirBase).normalized();
			// 切线方向
			//Eigen::Vector3f tanDir = normal.cross(offDir);
			// 偏移相位角
			float off = sin(float(j) / (numInterpInPeriod - 1) * 2 * M_PI);

			// 记录凸轮表
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// 更新当前角度
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 3 - 1, 0);
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);

	// 开始运动指令
	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	dq = theta / numPeriod / 2;
	curQ = 0.0;
	op1 = begPnt - center;
	if (endConfig.size() > 5) {
		for (size_t i = 0; i < 3; ++i) {
			detAngle[i] = (endConfig[3 + i] - begAngle[i]) / numPeriod / 2;
		}
	}
	int toolAxisIdx[6] = { 20,21,22,10,11,12 };
	Eigen::Vector3f curAngle = begAngle;
	for (size_t i = 0; i < numPeriod; ++i) {
		Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
		curQ += dq;
		curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle = detAngle;
		curQ += dq;
		curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curAngle = detAngle;
		//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		ZAux_Direct_MSphericalAbs(handle_, 6, toolAxisIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
		//std::cout << "ret = " << ret << std::endl;
		ZAux_Direct_MoveDelay(handle_, toolAxisIdx_[0], 100);
	}

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], camAxisIdx_[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], 15, 0);
	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, -1);

	return 0;
}

uint8_t circle_swing() {
	// 圆弧起点、过渡点、终点
	Eigen::Vector3d begPnt, viaPnt, endPnt;
	return 0;
}

Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end) {
	Eigen::Vector3f a = beg - mid, b = end - mid;
	if (a.cross(b).squaredNorm() < 1e-12) {
		float inf = (std::numeric_limits<float>::max)();
		return { inf, inf, inf };
	}

	return (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + mid;
}

uint8_t ZauxRobot::test() {
	int ret = 0;
	//ret = discreteTrajectory.set_starting_point({ 0, 0, 0, 45, 0, 90, 0 });
	//ret = discreteTrajectory.add_line({ 0, 100, 0, 135, 0, 90, 10 });
	//ret = discreteTrajectory.add_line({ -100, 100, 0, -135, 0, 90, 0 });

	//discreteTrajectory.set_starting_point({ 1342.7040, - 200, 176.6830, 158.6850, - 19.9750, - 131.2030, 0});
	//discreteTrajectory.add_line({ 1339.7890, 400, 176.6830, - 158.6850, - 19.9750, 131.2030, 0 });
	//discreteTrajectory.add_line({ 1747.3510, 400, 176.6830, - 158.6850, 19.9750, 48.7960, 0 });

	//discreteTrajectory.set_starting_point({ 1015.2012, - 100, 139.4826, 158.7217, - 24.1698, - 136.4338, 0});
	//discreteTrajectory.add_line({ 1013.0574, 100, 139.5904, -154.5673, -19.7173, 125.3549, 0 });
	//discreteTrajectory.add_line({ 1200, 102.7630, 139.2665, -153.2616, 17.8371, 58.6995, 0 });

	//discreteTrajectory.corner_transition(20, 20);

	Eigen::Vector3f begEuler(-158.6850, -19.9750, 131.2030), endEuler(-158.6850, 19.9750, 48.7960);
	get_zyx_euler_distance(begEuler, endEuler);


	//std::list<Eigen::Matrix<float, 7, 1>>::iterator pntIte = discreteTrajectory.nodePoint.begin();
	//while (++pntIte != discreteTrajectory.nodePoint.end()) {
	//	int axis[6] = { 6,7,8,9,10,11 };
	//	float dist[6] = { 0,0,0,0,0,0 };

	//	// 缓冲终点处的欧拉角与空间位置
	//	Eigen::Matrix<float, 6, 1> begPnt;
	//	Eigen::Vector3f zEuler(0, 0, 0), zPos;
	//	for (size_t i = 0; i < 3; ++i) {
	//		ZAux_Direct_GetEndMoveBuffer(handle_, 9 + i, &begPnt[i+3]);
	//		ZAux_Direct_GetEndMoveBuffer(handle_, 6 + i, &begPnt[i]);
	//	}
	//	// 当前目标点
	//	Eigen::Matrix<float, 7, 1> endPnt = *pntIte;

	//	for (size_t i = 0; i < 6; ++i) {
	//		dist[i] = endPnt[i] - begPnt[i];
	//	}
	//	
	//	inverse_kinematics();
	//	ZAux_Direct_MoveAbs(handle_, 3, axis, dist);
	//}

	//discreteTrajectory.test_Rxyz({0,0,0,  179.747, -24.7807, -127.451 });
	return 0;
}

uint8_t ZauxRobot::execute_discrete_trajectory_abs() {
	// 控制器句柄
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴轴号
	std::vector<int> tcpPosAxis = { 20, 21, 22 };
	// TCP 欧拉角轴轴号
	std::vector<int> tcpAngleAxis = { 10, 11, 12 };
	// 附加轴轴号
	std::vector<int> appendAxis = { 6 };

	std::vector<int> axis = tcpPosAxis;
	axis.insert(axis.end(), tcpAngleAxis.begin(), tcpAngleAxis.end());
	axis.insert(axis.end(), appendAxis.begin(), appendAxis.end());

	std::list<Eigen::Matrix<float, 7, 1>>::iterator nodePointIte = discreteTrajectory.nodePoint.begin();
	nodePointIte++;
	std::list<Eigen::Matrix<float, 7, 1>>::iterator midPointIte = discreteTrajectory.midPoint.begin();
	std::list<Eigen::Matrix<float, 7, 1>>::iterator infoIte = discreteTrajectory.trajInfo.begin();

	size_t i = 0;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer(axis.size()), relEndMove(axis.size());
		for (size_t i = 0; i < endBuffer.size(); ++i) {
			// 缓冲终点位置
			ZAux_Direct_GetEndMoveBuffer(handle, axis[i], &endBuffer[i]);
			// 终点到起点的相对运动
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动
		Eigen::Vector3f begEuler(endBuffer[3], endBuffer[4], endBuffer[5]), endEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}

		// 圆弧运动
		if ((*infoIte)(3) > 0) {
			// 圆弧中间点
			Eigen::Matrix<float, 7, 1> midPoint = *midPointIte;
			std::vector<float> relMidMove(axis.size());
			for (size_t i = 0; i < endBuffer.size(); ++i) {
				relMidMove[i] = midPoint[i] - endBuffer[i];
			}
			endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
			relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				relMidMove[3 + i] = relEuler[i];
			}

			// 圆弧模式
			int imode = 0;
			// 生成命令
			char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

			strcpy(cmdbuff, "BASE(");
			for (size_t i = 0; i < axis.size() - 1; i++) {
				sprintf(tempbuff, "%d,", axis[i]);
				strcat(cmdbuff, tempbuff);
			}
			sprintf(tempbuff, "%d)", axis.back());
			strcat(cmdbuff, tempbuff);
			strcat(cmdbuff, "\n");

			sprintf(tempbuff, "MSPHERICAL(%f,%f,%f,%f,%f,%f,%d", relEndMove[0], relEndMove[1], relEndMove[2], relMidMove[0], relMidMove[1], relMidMove[2], imode);
			strcat(cmdbuff, tempbuff);
			for (size_t i = 3; i < 7; ++i) {
				sprintf(tempbuff, ",%f", relEndMove[i]);
				strcat(cmdbuff, tempbuff);
			}
			strcat(cmdbuff, ")");
			std::cout << cmdbuff << std::endl;
			//调用命令执行函数
			return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		}
		// 直线运动
		else {
			// 绝对运动
			//ZAux_Direct_MoveAbs(handle, axis.size(), axis.data(), (*nodePointIte).data());
			// 相对运动
			ZAux_Direct_Move(handle, axis.size(), axis.data(), relEndMove.data());
		}

		nodePointIte++;
		midPointIte++;
		infoIte++;
	}

	return 0;
}

uint8_t ZauxRobot::execute_discrete_trajectory() {
	// 控制器句柄
	ZMC_HANDLE handle = handle_;
	// TCP 位置轴轴号
	std::vector<int> tcpPosAxis = { 20, 21, 22 };
	// TCP 欧拉角轴轴号
	std::vector<int> tcpAngleAxis = { 10, 11, 12 };
	// 附加轴轴号
	std::vector<int> appendAxis = { 6 };

	std::vector<int> axis = tcpPosAxis;
	axis.insert(axis.end(), tcpAngleAxis.begin(), tcpAngleAxis.end());
	axis.insert(axis.end(), appendAxis.begin(), appendAxis.end());

	std::list<Eigen::Matrix<float, 7, 1>>::iterator nodePointIte = discreteTrajectory.nodePoint.begin();
	std::list<Eigen::Matrix<float, 7, 1>>::iterator midPointIte = discreteTrajectory.midPoint.begin();
	std::list<Eigen::Matrix<float, 7, 1>>::iterator infoIte = discreteTrajectory.trajInfo.begin();

	while (infoIte != discreteTrajectory.trajInfo.end()) {
		Eigen::Matrix<float, 7, 1> endBuffer = *(nodePointIte++);

		std::vector<float> relEndMove(axis.size());
		for (size_t i = 0; i < endBuffer.size(); ++i) {
			// 终点到起点的相对运动
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动
		Eigen::Vector3f begEuler(endBuffer[3], endBuffer[4], endBuffer[5]), endEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}

		// 圆弧运动
		if ((*infoIte)(3) > 0) {
			// 圆弧中间点
			Eigen::Matrix<float, 7, 1> midPoint = *midPointIte;
			std::vector<float> relMidMove(axis.size());
			for (size_t i = 0; i < endBuffer.size(); ++i) {
				relMidMove[i] = midPoint[i] - endBuffer[i];
			}
			endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
			relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				relMidMove[3 + i] = relEuler[i];
			}

			// 圆弧模式
			int imode = 0;
			// 生成命令
			char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

			strcpy(cmdbuff, "BASE(");
			for (size_t i = 0; i < axis.size() - 1; i++) {
				sprintf(tempbuff, "%d,", axis[i]);
				strcat(cmdbuff, tempbuff);
			}
			sprintf(tempbuff, "%d)", axis.back());
			strcat(cmdbuff, tempbuff);
			strcat(cmdbuff, "\n");

			sprintf(tempbuff, "MSPHERICAL(%f,%f,%f,%f,%f,%f,%d", relEndMove[0], relEndMove[1], relEndMove[2], relMidMove[0], relMidMove[1], relMidMove[2], imode);
			strcat(cmdbuff, tempbuff);
			for (size_t i = 3; i < 7; ++i) {
				sprintf(tempbuff, ",%f", relEndMove[i]);
				strcat(cmdbuff, tempbuff);
			}
			strcat(cmdbuff, ")");
			std::cout << cmdbuff << std::endl;
			//调用命令执行函数
			return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		}
		// 直线运动
		else {
			// 绝对运动
			//ZAux_Direct_MoveAbs(handle, axis.size(), axis.data(), (*nodePointIte).data());
			// 相对运动
			ZAux_Direct_Move(handle, axis.size(), axis.data(), relEndMove.data());
		}

		midPointIte++;
		infoIte++;
	}

	return 0;
}


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

ZauxRobot::ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
				     const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx,
					 const std::vector<int>& swingAxisIdx,const std::vector<int>& connpathAxisIdx) {

	set_axis(jointAxisIdx, ikPosAxisIdx, tcpAngleAxisIdx, tcpPosAxisIdx, appAxisIdx, camAxisIdx, connpathAxisIdx, swingAxisIdx);
}


uint8_t ZauxRobot::set_handle(ZMC_HANDLE handle) {
	handle_ = handle;

	return 0;
}


uint8_t ZauxRobot::set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
	                        const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx,
	                        const std::vector<int>& swingAxisIdx, const std::vector<int>& connpathAxisIdx) {
	jointAxisIdx_ = jointAxisIdx;
	ikPosAxisIdx_ = ikPosAxisIdx;
	tcpAngleAxisIdx_ = tcpAngleAxisIdx;
	tcpPosAxisIdx_ = tcpPosAxisIdx;
	appAxisIdx_ = appAxisIdx;
	camAxisIdx_ = camAxisIdx;
	swingAxisIdx_ = swingAxisIdx;
	connpathAxisIdx_ = connpathAxisIdx;

	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx_[0], &axisUnits);
	return 0;
}


uint8_t ZauxRobot::connect_eth(char *ip_addr) {
	printf("Connecting to: %s... ", ip_addr);
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle_)) {
		printf("Failed!\n");
		handle_ = NULL;
		//getchar();
		return 1;
	}

	printf("Succeed\n"); 

	ZAux_SetTraceFile(4, "sdf");
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx_[0], &axisUnits);
	return 0;
}


uint8_t ZauxRobot::connect_pci(uint32 cardNum) {
	printf("Connecting to pci: %d...", cardNum);
	int ret = ZAux_OpenPci(cardNum, &handle_);

	if (ERR_SUCCESS != ret) {
		printf("Failed! Retrun %d.\n", ret);
		handle_ = NULL;
		//getchar();
		return 1;
	}

	printf("Succeed\n");
	ZAux_SetTraceFile(4, "sdf");
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx_[0], &axisUnits);

	return 0;
}


uint8_t ZauxRobot::lazy_connect() {
	if (this->connect_eth((char *)"127.0.0.1") == 0) {
		return 0;
	}
	else if (this->connect_eth((char *)"192.168.1.14") == 0) {
		return 0;
	}
	else if (this->connect_pci(0) == 0) {
		return 0;
	}
	// 连接失败
	return 1;
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


//uint8_t ZauxRobot::load_basic_project(const char *basPath, uint32_t mode) {
//	return 1;
//}


uint8_t ZauxRobot::disconnect() {
	//关闭连接 
	if (ZAux_Close(handle_) > 0) {
		printf("Error: # ZauxRobot::disconnect()!\n");
		return 1;
	}
	printf("connection closed!\n");
	handle_ = NULL;
	//std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}


uint8_t ZauxRobot::forward_kinematics() {
	// 逆解轴 = 逆解位置 + TCP 姿态
	std::vector<int> ikAxisIdx = ikPosAxisIdx_;
	ikAxisIdx.insert(ikAxisIdx.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());

	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, (char *)"kinematic", &kinematics);
	if (kinematics > 0 && kinematics < 2) {
		return 1;
	}
	// 等待末端运动结束，工具轴+附加轴
	wait_idle(tcpPosAxisIdx_[0]);
	//if (connType == ConnType::FK) {
	//	return 1;
	//}
	int ret = 0;
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connreframe()
	//ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	ret = ZAux_Direct_Connreframe(handle_, baseAxis.size(), baseAxis.data(), 93, 100, connreAxis.size(), connreAxis.data());
	ret = ZAux_Direct_Connreframe(handle_, ikAxisIdx.size(), ikAxisIdx.data(), 6, 0, jointAxisIdx_.size(), jointAxisIdx_.data());

	connType = ConnType::FK;
	ZAux_Direct_SetUserVar(handle_, (char *)"kinematic", 1);
	return 0;
}


uint8_t ZauxRobot::inverse_kinematics() {
	// 逆解轴 = 逆解位置 + TCP 姿态
	std::vector<int> ikAxisIdx = ikPosAxisIdx_;
	ikAxisIdx.insert(ikAxisIdx.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());

	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, (char *)"kinematic", &kinematics);
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
	ret = ZAux_Direct_Connframe(handle_, jointAxisIdx_.size(), jointAxisIdx_.data(), 6, 0, ikAxisIdx.size(), ikAxisIdx.data());
	ret = ZAux_Direct_Connframe(handle_, connreAxis.size(), connreAxis.data(), 93, 100, baseAxis.size(), baseAxis.data());
	connType = ConnType::IK;
	ret = ZAux_Direct_SetUserVar(handle_, (char *)"kinematic", 2);

	return 0;
}


uint8_t ZauxRobot::wait_idle(int axisIdx) {
	float IDLE;
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		ZAux_Direct_GetParam(handle_, (char *) "IDLE", axisIdx, &IDLE);
		if (IDLE < 0)break;
	}
	return 0;
}


uint8_t ZauxRobot::get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	if (axisList.size() < 1) {
		return 1;
	}
	paramList = std::vector<float>();
	paramList.reserve(axisList.size());

	//生成命令
	sprintf(cmdbuff, "?%s(%d)", paramName, axisList[0]);
	for (size_t i = 1; i < axisList.size(); ++i) {
		sprintf(tempbuff, ",%s(%d)", paramName, axisList[i]);
		strcat(cmdbuff, tempbuff);
	}

	int32 iresult = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	// 判断返回状态
	if (ERR_OK != iresult) {
		return iresult;
	}
	if (0 == strlen(cmdbuffAck)) {
		return ERR_NOACK;
	}

	// 解析返回值
	std::stringstream ackStr(cmdbuffAck);
	std::string word;
	// Extract word from the stream
	while (ackStr >> word) {
		paramList.push_back(std::stof(word));
	}

	return 0;
}


uint8_t ZauxRobot::set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	if (axisList.size() < 1 || paramList.size() < axisList.size()) {
		return 1;
	}

	// 立即设置
	if (principal < 0) {
		sprintf(cmdbuff, "%s(%d)=%f", paramName, axisList[0], paramList[0]);
		for (size_t i = 1; i < axisList.size(); ++i) {
			sprintf(tempbuff, "\n%s(%d)=%f", paramName, axisList[i], paramList[i]);
			strcat(cmdbuff, tempbuff);
		}
	}
	// 缓冲中设置
	else {
		sprintf(cmdbuff, "MOVE_PARA(%s,%d,%f) axis(%d)", paramName, axisList[0], paramList[0], principal);
		for (size_t i = 1; i < axisList.size(); ++i) {
			sprintf(tempbuff, "\nMOVE_PARA(%s,%d,%f) axis(%d)", paramName, axisList[i], paramList[i], principal);
			strcat(cmdbuff, tempbuff);
		}
	}
	std::cout << cmdbuff << std::endl;
	return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
}


uint8_t ZauxRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relEndMove) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(relEndMove.size(), axis.size());
	if (num < 1) {
		return 1;
	}

	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)", axis[num-1]);
	strcat(cmdbuff, tempbuff);
	strcat(cmdbuff, "\n");

	strcat(cmdbuff, "MOVESP(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%f,", relEndMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", relEndMove[num - 1]);
	strcat(cmdbuff, tempbuff);

	std::cout << cmdbuff << std::endl;
	//调用命令执行函数
	return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
}


uint8_t ZauxRobot::moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, const std::vector<float>& midPoint,
						 const std::vector<float>& endPoint, int imode) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(begPoint.size(), axis.size());

	std::vector<float> relEndMove(num);
	for (size_t i = 0; i < num; ++i) {
		relEndMove[i] = endPoint[i] - begPoint[i];
	}

	// 圆弧中间点
	std::vector<float> relMidMove(num);
	for (size_t i = 0; i < num; ++i) {
		relMidMove[i] = midPoint[i] - begPoint[i];
	}
	// 相对角度: beg -> mid -> end
	Eigen::Vector3f begEuler = Eigen::Vector3f(begPoint[3], begPoint[4], begPoint[5]);
	Eigen::Vector3f endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
	Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
	begEuler = endEuler;
	endEuler = Eigen::Vector3f(endPoint[3], endPoint[4], endPoint[5]);
	relEuler += get_zyx_euler_distance(begEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		relMidMove[3 + i] = relEuler[i];
	}

	// 生成命令
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%d,", axis[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)", axis[num - 1]);
	strcat(cmdbuff, tempbuff);
	strcat(cmdbuff, "\n");

	sprintf(tempbuff, "MSPHERICALSP(%f,%f,%f,%f,%f,%f,%d", relEndMove[0], relEndMove[1], relEndMove[2], relMidMove[0], relMidMove[1], relMidMove[2], imode);
	strcat(cmdbuff, tempbuff);
	for (size_t i = 3; i < num; ++i) {
		sprintf(tempbuff, ",%f", relEndMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	strcat(cmdbuff, ")");
	//std::cout << cmdbuff << std::endl;
	//调用命令执行函数
	return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
}


uint8_t ZauxRobot::save_table(size_t startIdx, size_t num, const std::string& path) {
	if (num == 0) {
		return 0;
	}

	// 单次最大读取数量
	size_t maxNum = 1000;
	// 读取缓冲
	std::vector<float> tableData(maxNum, 0);
	// 读取次数
	size_t times = std::floor(num / maxNum);
	// 输出文件
	std::ofstream out(path, std::ios::trunc);

	for (size_t i = 0; i < times; ++i) {
		// 读取数据
		ZAux_Direct_GetTable(handle_, startIdx + maxNum * i, maxNum, tableData.data());
		// 保存到文件
		for (const auto& data : tableData) {
			out << data << std::endl;
		}
	}
	
	// 剩余数据长度
	size_t remain = num - maxNum * times;
	// 读取剩余数据
	ZAux_Direct_GetTable(handle_, startIdx + maxNum * times, remain, tableData.data());
	for (size_t i = 0; i < remain; ++i) {
		out << tableData[i] << std::endl;
	}

	return 0;
}


uint8_t ZauxRobot::update_swing_table(const Weave& waveCfg) {
	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;
	
	std::vector<float> waveGenerator(numInterp, 0);

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
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
		// 周期时间(ms)
		float totalTime = 1000 / freq + 2 * swingHoldTime;
		// 四分之一摆动周期占用的 table 个数
		size_t numQuarter = numInterp * (1000 / freq) / totalTime / 4;
		// 右停留时间占用的 table 个数
		size_t numRightHold = numInterp * waveCfg.Dwell_right / totalTime;

		// 缓冲中写入凸轮表
		for (size_t i = 0; i < 4 * numQuarter; ++i) {
			// 插值点对应的偏移幅值
			float tmp = std::sin(2 * M_PI * i / (4 * numQuarter - 1));
			if (i < numQuarter) {
				waveGenerator[i] = tmp;
			}
			else if (i < 3 * numQuarter) {
				waveGenerator[numRightHold + i] = tmp;
			}
			else {
				waveGenerator[numInterp - 4 * numQuarter + i] = tmp;
			}
		}
		for (size_t i = 0; i < numRightHold; ++i) {
			waveGenerator[numQuarter + i] = 1;
		}
		for (size_t i = 0; i < numInterp - 4 * numQuarter - numRightHold; ++i) {
			waveGenerator[3 * numQuarter + numRightHold + i] = -1;
		}
	}

	// 自定义摆焊函数
	if (holdType > 0) {
		sinTableBeg = 2000;
		//for (size_t i = 0; i < numInterp; ++i) {
		//	ZAux_Direct_MoveTable(handle_, swingAxisIdx_[0], sinTableBeg + i, waveGenerator[i]);
		//}
	}
	// 摆动停止，焊接不停止
	else {
		sinTableBeg = 2100;
		for (size_t i = 0; i < numInterp; ++i) {
			ZAux_Direct_MoveTable(handle_, swingAxisIdx_[0], sinTableBeg + i, waveGenerator[i]);
		}
	}

	return 0;
}


uint8_t ZauxRobot::swing_on(float vel, const Weave& waveCfg, const std::vector<float>& moveDir) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// *** 获取的摆焊参数 *************************************
	// 摆动频率
	float freq = waveCfg.Freq;
	// 摆动振幅
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// 停止模式
	int holdType = waveCfg.Dwell_type;
	// 机器人停留时间, 摆动停留时间 (仅一个生效)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;

	//// 正弦摆Table数组
	//std::vector<float> sinTableValue(numInterp);
	//// 机器人停止
	//if (holdType > 0) {
	//	robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	//}
	//// 摆动停留时间
	//else {
	//	swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	//	// 周期时间(ms)
	//	float totalTime = 1000 / freq + 2 * swingHoldTime;
	//	// 四分之一摆动周期占用的 table 个数
	//	size_t numQuarter = numInterp * (1000 / freq) / totalTime / 4;
	//	// 右停留时间占用的 table 个数
	//	size_t numRightHold = numInterp * waveCfg.Dwell_right / totalTime;
	//
	//	// 缓冲中写入凸轮表
	//	for (size_t i = 0; i < 4 * numQuarter; ++i) {
	//		// 插值点对应的偏移幅值
	//		float tmp = std::sin(2 * M_PI * i / (4 * numQuarter - 1));
	//		if (i < numQuarter) {
	//			sinTableValue[i] = tmp;
	//		}
	//		else if (i < 3 * numQuarter) {
	//			sinTableValue[numRightHold + i] = tmp;
	//		}
	//		else {
	//			sinTableValue[numInterp - 4 * numQuarter + i] = tmp;
	//		}
	//	}
	//	for (size_t i = 0; i < numRightHold; ++i) {
	//		sinTableValue[numQuarter + i] = 1;
	//	}
	//	for (size_t i = 0; i < numInterp - 4 * numQuarter - numRightHold; ++i) {
	//		sinTableValue[3 * numQuarter + numRightHold + i] = -1;
	//	}
	//}
	//
	// 机器人停止
	//if (holdType > 0) {
	//	sinTableBeg = 2000;
	//}
	//else {
	//	sinTableBeg = 2100;
	//	for (size_t i = 0; i < 100; ++i) {
	//		ZAux_Direct_MoveTable(handle_, swingAxisIdx_[0], sinTableBeg + i, sinTableValue[i]);
	//	}
	//}

	sinTableBeg = holdType > 0 ? 2000 : 2100;

	// 周期长度
	float dist = vel * (1/freq + 2*swingHoldTime/1000);

	// 读取缓冲最终位置处的欧拉角(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, tcpAngleAxisIdx_[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// 缓冲最终位置的工具 Z 方向
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

	// 设置摆角
	if (moveDir.size() > 0) {
		Eigen::Vector3f tanDir(moveDir[0], moveDir[1], moveDir[2]);
		tanDir.normalize();
		zDir = (Eigen::AngleAxisf(waveCfg.Angle_Ltype_top * M_PI / 180, tanDir) * zDir).eval();
	}

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", swingAxisIdx_[0]);
	float vectorBuffered2 = 0.0;
	ret = ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, 主轴, 矢量距离, 周期长度, 左右摆幅, 开始Table, 结束Table
		5, swingAxisIdx_[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1,
		zDir[0], zDir[1], zDir[2]
	);
	std::cout << cmdbuff  << std::endl;

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	return ret;
}


uint8_t ZauxRobot::swing_off(float displacement) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", swingAxisIdx_[0]);
	float vectorBuffered2 = 0.0;
	ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);
	vectorBuffered2 += displacement;

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, 主轴, 矢量距离
		-1, swingAxisIdx_[0], vectorBuffered2
	);
	std::cout << cmdbuff << std::endl;

	//调用命令执行函数
	int ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

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


uint8_t ZauxRobot::execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory) {
	std::vector<int> axis = tcpPosAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// 只计算TCP位置速度
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, axis[i], i < 3);
	}

	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++), relEndMove(axis.size(), 0);

		for (size_t i = 0; i < num; ++i) {
			// 终点到起点的相对运动
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动: beg -> mid -> end
		Eigen::Vector3f begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		begEuler = endEuler;
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		relEuler += get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
		begEuler = endEuler;

		// 轨迹速度
		float vel = (*infoIte)[7];
		// 设置速度
		//ZAux_Direct_MovePara(handle_, axis[0], (char*)"SPEED", axis[0], vel);
		ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);

		// 圆弧运动
		if ((*infoIte)[3] > 0) {
			this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
		}
		// 直线运动
		else {
			// 相对运动
			this->moveL(axis, relEndMove);
			//ZAux_Direct_Move(handle_, axis.size(), axis.data(), relEndMove.data());
		}

		// 轨迹数+1
		ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);

		midPointIte++;
		infoIte++;
	}

	return 0;
}


uint8_t ZauxRobot::execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory) {
	std::vector<int> axis = tcpPosAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// 只计算TCP位置速度
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], i < 3);
	}

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++);

		std::vector<float> relEndMove(axis.size(), 0);
		// 终点到起点的相对运动
		for (size_t i = 0; i < num; ++i) {
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动
		begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
		begEuler = endEuler;

		// 轨迹速度
		float vel = (*infoIte)[7];
		// 设置速度
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"SPEED", axis[0], vel);

		// 圆弧运动
		if ((*infoIte)[3] > 0) {
			// 圆弧中间点
			std::vector<float> midPoint = *midPointIte;
			std::vector<float> relMidMove(axis.size());
			for (size_t i = 0; i < endBuffer.size(); ++i) {
				relMidMove[i] = midPoint[i] - endBuffer[i];
			}
			// 中间点角度相对值
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
			sprintf(tempbuff, "%d)", axis[num - 1]);
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
			return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
		}
		// 直线运动
		else {
			// 相对运动
			int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), relEndMove.data());
		}

		midPointIte++;
		infoIte++;
	}
	
	return 0;
}

uint8_t ZauxRobot::swing_tri(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	std::vector<int> axis = swingAxisIdx_, camAxis = camAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	axis.insert(axis.end(), camAxisIdx_.begin(), camAxisIdx_.end());

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// 只计算TCP位置速度
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, axis[i], i < 3);
	}
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
	}

	// 读取三角摆形状: 摆幅、前进比例、后退比例
	float triWidth = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2, feedRatio = waveCfg.Length, backRatio =  waveCfg.Bias;
	// 摆焊参数
	float swingVel = (waveCfg.LeftWidth + waveCfg.RightWidth) * waveCfg.Freq, rightAngle = waveCfg.Angle_Ltype_top, leftAngle = waveCfg.Angle_Ltype_btm;
	// 轨迹时间
	float timeUnit = 1 / (2 * waveCfg.Freq), feedTime = timeUnit * feedRatio, backTime = timeUnit * backRatio;

	// 摆动开始
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++), relEndMove(axis.size(), 0);

		for (size_t i = 0; i < num; ++i) {
			// 终点到起点的相对运动
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}
		// 欧拉角转换到相对运动: beg -> mid -> end
		Eigen::Vector3f begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		begEuler = endEuler;
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		relEuler += get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}

		// 焊接速度
		float weldVel = (*infoIte)[7];

		// 圆弧运动
		if ((*infoIte)[3] > 0) {
			ZAux_Direct_SetForceSpeed(handle_, axis[0], weldVel);
			this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
		}
		// 直线运动
		else {
			// 轨迹段起点处的欧拉角(deg)
			Eigen::Vector3f zEuler(endBuffer[3], endBuffer[4], endBuffer[5]);
			zEuler *= M_PI / 180;
			// 缓冲最终位置的工具 Z 方向
			Eigen::Vector3f zDir(0,0,0);
			zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
			zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
			zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

			// 直线方向
			Eigen::Vector3f begTan((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
			// 三角摆运动距离
			float dist = std::fabs((*infoIte)[3]);

			// 修正的z方向
			Eigen::Vector3f begUprightDir = (begTan.cross(zDir).cross(begTan)).normalized();
			// 左右摆动方向
			Eigen::Vector3f rightDir = Eigen::AngleAxisf(rightAngle*M_PI/180 - M_PI / 2, begTan) * begUprightDir;
			Eigen::Vector3f leftDir = Eigen::AngleAxisf(leftAngle*M_PI / 180 + M_PI / 2, begTan) * begUprightDir;
			// 完整的三角形周期
			size_t numPeriod = std::floor((dist / weldVel - feedTime) / (feedTime - backTime));
			// 修正焊接速度
			weldVel = dist / (numPeriod * (feedTime - backTime) + feedTime);
			// 前进距离、后退距离
			float feedDist = weldVel * feedTime, backDist = weldVel * backTime;

			// 焊接速度
			ZAux_Direct_SetForceSpeed(handle_, axis[0], swingVel);

			std::vector<float> moveCmd(axis.size(), 0);

			for (size_t i = 0; i < numPeriod+1; ++i) {
				// 右侧向右
				for (size_t j = 0; j < moveCmd.size(); ++j) {
					moveCmd[j] = j < 3 ? rightDir[j] * triWidth : 0;
				}
				this->moveL(axis, moveCmd);
				ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

				// 右侧向前 + 姿态
				for (size_t j = 0; j < 3; ++j) {
					moveCmd[j] = -rightDir[j] * triWidth;
					moveCmd[moveCmd.size() - 3 + j] = begTan[j] * feedDist;
					moveCmd[3 + j] = relEndMove[3 + j] / numPeriod;
				}
				// 附加轴运动
				for (size_t j = 6; j < moveCmd.size()-3; ++j) {
					moveCmd[j] = relEndMove[j] / numPeriod;
				}
				ZAux_Direct_SetForceSpeed(handle_, axis[0], triWidth/feedTime);
				this->moveL(axis, moveCmd);
				ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_center);

				// 最后一段退出
				if (i == numPeriod) break;

				// 左测向后
				for (size_t j = 0; j < 3; ++j) {
					moveCmd[j] = leftDir[j] * triWidth;
					moveCmd[moveCmd.size() - 3 + j] = - begTan[j] * backDist;
				}
				// 附加轴运动和姿态清零
				for (size_t j = 3; j < moveCmd.size()-3; ++j) {
					moveCmd[j] = 0;
				}
				ZAux_Direct_SetForceSpeed(handle_, axis[0], triWidth / backTime);
				this->moveL(axis, moveCmd);
				ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);

				// 左侧向右
				for (size_t j = 0; j < moveCmd.size(); ++j) {
					moveCmd[j] = j < 3 ? -leftDir[j] * triWidth : 0;
				}
				ZAux_Direct_SetForceSpeed(handle_, axis[0], swingVel);
				this->moveL(axis, moveCmd);
			}
		}

		// 轨迹数+1
		ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);

		midPointIte++;
		infoIte++;
	}

	// 摆动停止
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	return 0;
}
uint8_t ZauxRobot::swing_sin(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	std::vector<int> axis = swingAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// 只计算TCP位置速度
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], i < 3);
	}
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
	}
	// 设置摆焊参数
	update_swing_table(waveCfg);
	// 摆动开始
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++);

		std::vector<float> relEndMove(axis.size(), 0);
		// 终点到起点的相对运动
		for (size_t i = 0; i < num; ++i) {
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动: beg -> mid -> end
		Eigen::Vector3f begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		begEuler = endEuler;
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		relEuler += get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
		begEuler = endEuler;

		// 轨迹速度
		float vel = (*infoIte)[7];
		// 设置速度
		ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);

		// 设置摆动参数
		if (waveCfg.Angle_Ltype_top != 0) {
			Eigen::Vector3f tanDir(0, 0, 0);
			if ((*infoIte)[3] > 0) {
				Eigen::Vector3f radiusDir(endBuffer[0] - (*infoIte)[0], endBuffer[1] - (*infoIte)[1], endBuffer[2] - (*infoIte)[2]);
				Eigen::Vector3f normDir((*infoIte)[4], (*infoIte)[5], (*infoIte)[6]);
				tanDir = normDir.cross(radiusDir).normalized();
			}
			else {
				tanDir = Eigen::Vector3f((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
			}
			swing_on(vel, waveCfg, { tanDir[0], tanDir[1], tanDir[2] });
		}
		else {
			swing_on(vel, waveCfg);
		}
		// 计算轴运动距离
		int numPeriod = std::round(std::fabs((*infoIte)[3]) / (vel / waveCfg.Freq));
		swing_off(std::fabs((*infoIte)[3]));

		// 圆弧运动
		if ((*infoIte)[3] > 0) {
			// 机器人停止
			if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0 && numPeriod > 0) {
				// 计算点位置
				Eigen::Vector3f tempPos, rotNorm((*infoIte)[4], (*infoIte)[5], (*infoIte)[6]);
				Eigen::Vector3f radiusDir(0, 0, 0), centerPos((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);

				std::vector<float> tempBegPoint = endBuffer, tempEndPoint(endBuffer.size(), 0), tempCenPoint(endBuffer.size(), 0);

				for (size_t i = 0; i < 3; ++i) {
					tempCenPoint[i] = centerPos[i];
					radiusDir[i] = endBuffer[i] - centerPos[i];
				}
				float partial = 0, theta = rotNorm.norm();
				rotNorm.normalize();

				// 1/4 周期
				partial += 1.0 / (4 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_right > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

				// 1/2 * 2周期
				for (size_t i = 0; i < numPeriod; ++i) {
					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_left > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					// 最后一个周期退出
					if (i +1 == numPeriod) break;

					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_right > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				}

				// 1/4 周期
				partial += 1.0 / (4 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				tempBegPoint = tempEndPoint;
			}
			// 摆动停止
			else {
				this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
			}
		}
		// 直线运动
		else {
			// 机器人停止
			if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0 && numPeriod > 0) {
				std::vector<float> detRelEndMove(relEndMove.size(), 0);

				// 1/4 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] = relEndMove[j] / (numPeriod * 4);
				int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
				if (waveCfg.Dwell_right > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

				// 1/2 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] *= 2;
				for (int i = 0; i < numPeriod; ++i) {
					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					if (waveCfg.Dwell_left > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					// 最后一个周期退出
					if (i+1 == numPeriod) break;

					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					if (waveCfg.Dwell_right > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				}
				// 1/4 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] /= 2;
				ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
			}
			// 摆动停止
			else {
				// 相对运动
				this->moveL(axis, relEndMove);
			}
		}

		// 轨迹数+1
		ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);

		midPointIte++;
		infoIte++;
	}

	// 摆动停止
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	return 0;
}
uint8_t ZauxRobot::swing_tri() {
	return 0;
}


uint8_t ZauxRobot::swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	int ret = 0;
	if (waveCfg.Shape == 3) {
		ret = swing_tri(discreteTrajectory, waveCfg);
	}
	else if (waveCfg.Shape == 0) {
		ret = swing_sin(discreteTrajectory, waveCfg);
	}
	else {
		ret = 1;
	}
	return ret;

	std::vector<int> axis = swingAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// 只计算TCP位置速度
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], i < 3);
	}
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
	}
	// 设置摆焊参数
	update_swing_table(waveCfg);
	// 摆动开始
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++);

		std::vector<float> relEndMove(axis.size(), 0);
		// 终点到起点的相对运动
		for (size_t i = 0; i < num; ++i) {
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// 欧拉角转换到相对运动: beg -> mid -> end
		Eigen::Vector3f begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		begEuler = endEuler;
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		relEuler += get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
		begEuler = endEuler;

		// 轨迹速度
		float vel = (*infoIte)[7];
		// 设置速度
		//ZAux_Direct_MovePara(handle_, axis[0], (char*)"SPEED", axis[0], vel);
		ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);

		// 设置摆动参数
		if (waveCfg.Angle_Ltype_top != 0) {
			Eigen::Vector3f tanDir(0, 0, 0);
			if ((*infoIte)[3] > 0) {
				Eigen::Vector3f radiusDir(endBuffer[0] - (*infoIte)[0], endBuffer[1] - (*infoIte)[1], endBuffer[2] - (*infoIte)[2]);
				Eigen::Vector3f normDir((*infoIte)[4], (*infoIte)[5], (*infoIte)[6]);
				tanDir = normDir.cross(radiusDir).normalized();
			}
			else {
				tanDir = Eigen::Vector3f((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
			}
			swing_on(vel, waveCfg, { tanDir[0], tanDir[1], tanDir[2] });
		}
		else {
			swing_on(vel, waveCfg);
		}
		// 计算轴运动距离
		int numPeriod = std::round(std::fabs((*infoIte)[3]) / (vel / waveCfg.Freq));
		swing_off(std::fabs((*infoIte)[3]));

		// 圆弧运动
		if ((*infoIte)[3] > 0) {
			// 机器人停止
			if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0 && numPeriod > 0) {
				// 计算点位置
				Eigen::Vector3f tempPos, rotNorm((*infoIte)[4], (*infoIte)[5], (*infoIte)[6]);
				Eigen::Vector3f radiusDir(0, 0, 0), centerPos((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);

				std::vector<float> tempBegPoint = endBuffer, tempEndPoint(endBuffer.size(), 0), tempCenPoint(endBuffer.size(), 0);

				for (size_t i = 0; i < 3; ++i) {
					tempCenPoint[i] = centerPos[i];
					radiusDir[i] = endBuffer[i] - centerPos[i];
				}
				float partial = 0, theta = rotNorm.norm();
				rotNorm.normalize();

				// 1/4 周期
				partial += 1.0 / (4 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_right > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

				// 1/2 * 2周期
				for (size_t i = 0; i < numPeriod - 1; ++i) {
					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_left > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);

					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_right > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				}

				// 1/2 周期
				partial += 1.0 / (2 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_left > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);

				// 1/4 周期
				partial += 1.0 / (4 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				tempBegPoint = tempEndPoint;
			}
			// 摆动停止
			else {
				this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
			}
		}
		// 直线运动
		else {
			// 机器人停止
			if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0 && numPeriod > 0) {
				std::vector<float> detRelEndMove(relEndMove.size(), 0);

				// 1/4 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] = relEndMove[j] / (numPeriod * 4);
				int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
				if (waveCfg.Dwell_right > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

				// 1/2 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] *= 2;
				for (int i = 0; i < numPeriod - 1; ++i) {
					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					if (waveCfg.Dwell_left > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					if (waveCfg.Dwell_right > 0)
						ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				}
				ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
				if (waveCfg.Dwell_left > 0)
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					
				// 1/4 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] /= 2;
				ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
			}
			// 摆动停止
			else {
				// 相对运动
				this->moveL(axis, relEndMove);
				//int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), relEndMove.data());
			}
		}

		// 轨迹数+1
		ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);

		midPointIte++;
		infoIte++;
	}

	// 摆动停止
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);

	return 0;
}


uint8_t ZauxRobot::arc_tracking_config(const Track& trackCfg) {
	std::vector<float> config(20, 0);
	size_t configTableStart = 1010;

	// 读取现有参数
	ZAux_Direct_GetTable(handle_, configTableStart, config.size(), config.data());

	// 左右跟踪参数
	config[0] = trackCfg.Lr_enable;
	config[1] = trackCfg.Lr_offset;
	config[2] = trackCfg.Lr_gain;
	// 积分常数
	config[3] = trackCfg.Lr_maxSingleCompensation;
	//config[4] = trackCfg.Lr_diffCoeff;
	config[5] = trackCfg.Lr_minCompensation;
	config[6] = trackCfg.Lr_maxCompensation;
	config[7] = trackCfg.Lr_MaxCorrectAngle;
	// 上下跟踪参数
	config[10] = trackCfg.Ud_enable;
	config[11] = trackCfg.Ud_offset;
	config[12] = trackCfg.Ud_gain;
	config[15] = trackCfg.Ud_minCompensation;
	config[16] = trackCfg.Ud_maxCompensation;
	config[17] = trackCfg.Ud_MaxCorrectAngle;
	// 其他参数
	config[18] = trackCfg.SegCorrectCycles;
	config[19] = trackCfg.Ud_refSampleCount;

	ZAux_Direct_SetTable(handle_, configTableStart, config.size(), config.data());
	return 0;
}


uint8_t ZauxRobot::emergency_stop() {
	float zero = 0.0, negtive = -1.0;
	// 摆动标志位复位
	ZAux_Direct_SetTable(handle_, 1000, 1, &negtive);

	return 0;
}


uint8_t ZauxRobot::emergency_pause() {
	return 0;
}


uint8_t ZauxRobot::emergency_resume() {
	return 0;
}

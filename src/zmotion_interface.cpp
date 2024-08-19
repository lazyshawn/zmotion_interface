
#include<chrono>
#include<thread>
#include<iostream>

#include"zmotion_interface.h"

// 轴号占用列表
std::vector<int> ZauxAxis::AxisIdxList = std::vector<int>(100, 0);

ZauxAxis::ZauxAxis() {
}


std::vector<float> serialize_weld_param(const Weave& waveCfg, const Arc_WeldingParaItem& weldCfg, const Track& trackCfg) {
	std::vector<float> param(40, 0);

	// 摆焊参数
	param[0] = waveCfg.Id;
	param[1] = waveCfg.Shape;
	param[2] = waveCfg.Freq;
	param[3] = waveCfg.LeftWidth;
	param[4] = waveCfg.RightWidth;
	param[5] = waveCfg.Dwell_type;
	param[6] = waveCfg.Dwell_left;
	param[7] = waveCfg.Dwell_right;

	// 焊接参数
	param[10] = weldCfg.Id;
	param[11] = weldCfg.WeldingCrt_Spd;
	param[12] = weldCfg.WeldingVtg_Strth;

	// 跟踪参数
	// 左右跟踪参数
	param[20] = trackCfg.Lr_enable;
	param[21] = trackCfg.Lr_offset;
	param[22] = trackCfg.Lr_gain;
	// 积分常数
	param[23] = trackCfg.Lr_maxSingleCompensation;
	//config[4] = trackCfg.Lr_diffCoeff;
	param[25] = trackCfg.Lr_minCompensation;
	param[26] = trackCfg.Lr_maxCompensation;
	param[27] = trackCfg.Lr_MaxCorrectAngle;
	// 上下跟踪参数
	param[30] = trackCfg.Ud_enable;
	param[31] = trackCfg.Ud_offset;
	param[32] = trackCfg.Ud_gain;
	param[35] = trackCfg.Ud_minCompensation;
	param[36] = trackCfg.Ud_maxCompensation;
	param[37] = trackCfg.Ud_MaxCorrectAngle;
	// 其他参数
	param[38] = trackCfg.SegCorrectCycles;
	param[39] = trackCfg.Ud_refSampleCount;

	return param;
}

uint8_t read_weld_param(const std::vector<float>& param, Weave& waveCfg, Arc_WeldingParaItem& weldCfg, Track& trackCfg) {
	if (param.size() < 40) {
		waveCfg = Weave();
		weldCfg = Arc_WeldingParaItem();
		trackCfg = Track();
		return 0;
	}

	// 摆焊参数
	waveCfg.Id = param[0];
	waveCfg.Shape = param[1];
	waveCfg.Freq = param[2];
	waveCfg.LeftWidth = param[3];
	waveCfg.RightWidth = param[4];
	waveCfg.Dwell_type = param[5];
	waveCfg.Dwell_left = param[6];
	waveCfg.Dwell_right = param[7];

	// 焊接参数
	weldCfg.Id = param[10];
	weldCfg.WeldingCrt_Spd = param[11];
	weldCfg.WeldingVtg_Strth = param[12];

	// 跟踪参数
	// 左右跟踪参数
	trackCfg.Lr_enable = param[20];
	trackCfg.Lr_offset = param[21];
	trackCfg.Lr_gain = param[22];
	// 积分常数
	trackCfg.Lr_maxSingleCompensation = param[23];
	trackCfg.Lr_minCompensation = param[25];
	trackCfg.Lr_maxCompensation = param[26];
	trackCfg.Lr_MaxCorrectAngle = param[27];
	// 上下跟踪参数
	trackCfg.Ud_enable = param[30];
	trackCfg.Ud_offset = param[31];
	trackCfg.Ud_gain = param[32];
	trackCfg.Ud_minCompensation = param[35];
	trackCfg.Ud_maxCompensation = param[36];
	trackCfg.Ud_MaxCorrectAngle = param[37];
	// 其他参数
	trackCfg.SegCorrectCycles = param[38];
	trackCfg.Ud_refSampleCount = param[39];
	return 0;
}


ZauxRobot::ZauxRobot() {
	derive_config();
}

ZauxRobot::ZauxRobot(const std::vector<int>& jointAxisIdx_, const std::vector<int>& appAxisIdx_,
	const std::vector<int>& tcpPosAxisIdx_, const std::vector<int>& tcpAngleAxisIdx_,
	const std::vector<int>& camAxisIdx_, const std::vector<int>& swingAxis_, const std::vector<int>& excuteAxis_) {

	set_axis(jointAxisIdx_, appAxisIdx_, tcpPosAxisIdx_, tcpAngleAxisIdx_, camAxisIdx_, swingAxis_, excuteAxis_);
}


int32 ZauxRobot::set_handle(ZMC_HANDLE handle) {
	handle_ = handle;

	return 0;
}


int32 ZauxRobot::set_axis(const std::vector<int>& jointAxisIdx_, const std::vector<int>& appAxisIdx_,
	const std::vector<int>& tcpPosAxisIdx_, const std::vector<int>& tcpAngleAxisIdx_,
	const std::vector<int>& camAxisIdx_, const std::vector<int>& swingAxis_, const std::vector<int>& excuteAxis_) {

	// 关节轴
	jointAxisIdx = jointAxisIdx_;
	// TCP 姿态轴
	tcpAngleAxisIdx = tcpAngleAxisIdx_;
	// TCP 位置轴
	tcpPosAxisIdx = tcpPosAxisIdx_;
	// 附加轴
	appAxisIdx = appAxisIdx_;
	// 凸轮轴
	camAxisIdx = camAxisIdx_;
	// 摆焊轴
	swingAxis = swingAxis_;
	// 执行轴
	excuteAxis = excuteAxis_;

	derive_config();

	return 0;
}

int32 ZauxRobot::derive_config() {
	// 正解轴
	fkAxis = jointAxisIdx;
	fkAxis.insert(fkAxis.end(), appAxisIdx.begin(), appAxisIdx.end());

	// 逆解轴
	ikAxis = tcpPosAxisIdx;
	ikAxis.insert(ikAxis.end(), tcpAngleAxisIdx.begin(), tcpAngleAxisIdx.end());
	ikAxis.insert(ikAxis.end(), appAxisIdx.begin(), appAxisIdx.end());

	// 摆焊轴
	swingAxis = swingAxisIdx;
	swingAxis.insert(swingAxis.end(), tcpAngleAxisIdx.begin(), tcpAngleAxisIdx.end());
	swingAxis.insert(swingAxis.end(), appAxisIdx.begin(), appAxisIdx.end());

	// 脉冲当量
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx[0], &axisUnits);
	return 0;
}

int32 ZauxRobot::connect_eth(char *ip_addr) {
	printf("Connecting to: %s... ", ip_addr);
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle_)) {
		printf("Failed!\n");
		handle_ = NULL;
		//getchar();
		return 1;
	}

	printf("Succeed\n"); 

	ZAux_SetTraceFile(4, "sdf");
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx[0], &axisUnits);
	return 0;
}


int32 ZauxRobot::connect_pci(uint32 cardNum) {
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
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx[0], &axisUnits);

	return 0;
}


int32 ZauxRobot::lazy_connect() {
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

int32 ZauxRobot::connect(std::string addr) {
	if (addr.size() > 2) {
		return connect_eth(const_cast<char *>(addr.c_str()));
	}
	else {
		// 转换成uint32
		uint32 cardNum = static_cast<uint32>(std::atoi(addr.c_str()));
		return connect_pci(cardNum);
	}
	return 0;
}

int32 ZauxRobot::disconnect() {
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


/* ******************************** 上位机软件交互 ********************************* */
int32 ZauxRobot::load_basic_pragma(const char *basPath, uint32_t mode) {
	// 加载 bas 程序
	if (ZAux_BasDown(handle_, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return 1;
	}
	// 等待 bas 程序下载
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}


int32 ZauxRobot::load_basic_project(const char *basPath, uint32_t mode) {
	return 1;
}


int32 ZauxRobot::forward_kinematics(int delay) {
	float flag = 1.0, readVal;
	int ret = 0;

	for (size_t i = 0; i < delay; ++i) {
		ret = ZAux_Direct_GetTable(handle_, 1003, 1, &readVal);

		// 判断是否切换完成
		if (readVal > 0) {
			return ret;
		}
		// 切换一次正解
		else if (i == 0) {
			ret = ZAux_Direct_SetTable(handle_, 1006, 1, &flag);
		}

		// 等待正逆解切换完成
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return -1;
}


int32 ZauxRobot::inverse_kinematics(int delay) {
	float flag = -1.0, readVal;
	int ret = 0;

	for (size_t i = 0; i < delay; ++i) {
		// 判断是否在正解模式
		ret = ZAux_Direct_GetTable(handle_, 1003, 1, &readVal);

		// 判断是否切换完成
		if (readVal < 0) {
			return ret;
		}
		// 切换一次逆解
		else if (i == 0) {
			ret = ZAux_Direct_SetTable(handle_, 1006, 1, &flag);
		}

		// 等待正逆解切换完成
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return -1;
}


int32 ZauxRobot::wait_idle(int axisIdx) {
	float IDLE;
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		ZAux_Direct_GetParam(handle_, (char *) "IDLE", axisIdx, &IDLE);
		if (IDLE < 0)break;
	}
	return 0;
}

int32 ZauxRobot::get_kinematic_mode() {
	int ret = 0;
	float readVal;

	ret = ZAux_Direct_GetTable(handle_, 1003, 1, &readVal);
	// 正解
	if (std::fabs(readVal - 1) < 1e-3) {
		return 1;
	}
	// 逆解
	else if (std::fabs(readVal + 1) < 1e-3) {
		return -1;
	}
	// 摆焊
	else if (std::fabs(readVal + 2) < 1e-3) {
		return -2;
	}
}


int32 ZauxRobot::switch_automation(bool setAuto) {
	// 切换到自动模式
	if (setAuto) {

	}
	// 切换到手动模式
	else {
		
	}
	return 0;
}


int32 ZauxRobot::jog_moving(int idx, int type, int dir) {
	std::vector<std::vector<int>> axisIdx;
	// 关节轴
	axisIdx.push_back(fkAxis);
	// 世界坐标轴
	axisIdx.push_back(ikAxis);
	// 工具坐标轴
	axisIdx.push_back(ikAxis);

	int ret = 0;
	if (type < 0|| type >= axisIdx.size() || idx < 0 || idx >= axisIdx[type].size()) {
		return -1;
	}

	// 运动类型与轴号不匹配
	float kinematics = 0.0;
	ZAux_Direct_GetTable(handle_, 1003, 1, &kinematics);
	// 空间运动(1) + 正解模式(1)
	if (type > 0 && std::fabs(kinematics - 1) < 1e-3) {
		return -2;
	}
	// 关节运动(0) + 逆解模式(-1)
	else if (type == 0 && kinematics < 0) {
		return -2;
	}

	// VMOVE 点动
	if (type < 2) {
		// 运动结束
		if (dir == 0) {
			ret = ZAux_Direct_Single_Cancel(handle_, axisIdx[type][idx], 4);
		}
		// 正向运动
		else if (dir > 0) {
			ret = ZAux_Direct_Single_Vmove(handle_, axisIdx[type][idx], 1);
		}
		// 负向运动
		else {
			ret = ZAux_Direct_Single_Vmove(handle_, axisIdx[type][idx], -1);
		}
	}
	// MOVE 点动
	else {
		// 当前姿态欧拉角
		std::vector<float> euler;
		ret = get_axis_param(tcpAngleAxisIdx, "DPOS", euler);

		// 计算当前方向
		//Eigen::Matrix3f trans = DiscreteTrajectory<float>::RzyxToRotMat(euler[5], euler[4], euler[3])
	}

	return ret;
}

int32 ZauxRobot::set_manual_SpeedRatio(float speedRatio) {
	float readVal;
	int ret = 0;

	ret = ZAux_Direct_GetTable(handle_, 1003, 1, &readVal);
	if (ret != 0)
		return handle_zaux_error(ret);

	// 逆解
	if (std::fabs(readVal + 1) < 1e-2) {
		// 最大末端速度
		std::vector<float> curSpeed(ikAxis.size(), 0);
		// 末端速度寄存器
		std::vector<int> jointSpeedVR = { 83,84,85, 1077,1078,1079,1080,1081,1082 };

		ret = get_axis_param(jointSpeedVR, "VR", curSpeed);
		if (ret != 0)
			return handle_zaux_error(ret);

		for (size_t i = 0; i < curSpeed.size(); ++i) {
			curSpeed[i] *= speedRatio;
		}

		// 修改末端速度
		ret = set_axis_param(ikAxis, "SPEED", curSpeed);
		if (ret != 0)
			return handle_zaux_error(ret);

		// 修改正解
		speedRatio = 1.0;
	}

	// 正解
	// 速度: 关节 + 附加轴
	std::vector<float> curSpeed(fkAxis.size(), 0);
	// 关节速度寄存器
	std::vector<int> jointSpeedVR = { 77,78,79,80,81,82,83,84,85 };

	ret = get_axis_param(jointSpeedVR, "VR", curSpeed);
	if (ret != 0)
		return handle_zaux_error(ret);

	for (size_t i = 0; i < curSpeed.size(); ++i) {
		curSpeed[i] *= speedRatio;
	}

	// 修改关节速度
	ret = set_axis_param(fkAxis, "SPEED", curSpeed);

	return 0;
}

int32 ZauxRobot::set_auto_SpeedRatio(float speedRatio) {
	int ret = 0;

	//ret = set_axis_param({ excuteAxis[0] }, "SPEED_RATIO", { speedRatio });
	ret = set_axis_param({ jointAxisIdx[0], tcpPosAxisIdx[0], swingAxisIdx[0] }, "SPEED_RATIO", { speedRatio, speedRatio, speedRatio });

	return handle_zaux_error(ret);
}

int32 ZauxRobot::set_acceleration_time(float time) {
	int ret = 0;

	// 速度: 关节 + 附加轴 + 位置 + 姿态
	std::vector<float> curSpeed(jointAxisIdx.size() + appAxisIdx.size() + tcpPosAxisIdx.size() + tcpAngleAxisIdx.size(), 0);

	ret = get_axis_param(jointSpeedVR, "VR", curSpeed);
	if (ret != 0)
		return handle_zaux_error(ret);

	// 速度 / 加速时间 = 加速度
	for (size_t i = 0; i < curSpeed.size(); ++i) {
		curSpeed[i] /= time;
	}

	// 关节 + 附加轴 + TCP位置 + TCP姿态
	std::vector<int> axisIdx = fkAxis;
	axisIdx.insert(axisIdx.end(), tcpPosAxisIdx.begin(), tcpPosAxisIdx.end());
	axisIdx.insert(axisIdx.end(), tcpAngleAxisIdx.begin(), tcpAngleAxisIdx.end());

	// 修改关节加减速度
	ret = set_axis_param(axisIdx, "ACCEL", curSpeed);
	if (ret != 0)
		return handle_zaux_error(ret);
	ret = set_axis_param(axisIdx, "DECEL", curSpeed);
	if (ret != 0)
		return handle_zaux_error(ret);
	
	return 0;
}

int32 ZauxRobot::set_axis_speed(const std::vector<float>& speedList) {
	int ret = set_axis_param(jointSpeedVR, "VR", speedList);
	return ret;
}

std::vector<float> ZauxRobot::get_axis_speed() {
	std::vector<float> curSpeed;

	int ret = get_axis_param(jointSpeedVR, "VR", curSpeed);

	return curSpeed;
}

/* ******************************** 基础运动指令封装 *********************************
* @brief 正运动接口封装。Zmotion -> ZauxRobot
*/
// 基础运动指令封装：move
int32 ZauxRobot::move(const std::vector<int>& axis, const std::vector<float>& relMove, float speed) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(relMove.size(), axis.size());
	if (num < 1)
		return 1;

	int ret = 0;
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

	// 单独设置速度
	if (speed > 0) {
		sprintf(tempbuff, "FORCE_SPEED(%d) = %f", axis[0], speed);
		strcat(cmdbuff, tempbuff);
	}

	strcat(cmdbuff, "MOVESP(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%f,", relMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", relMove[num - 1]);
	strcat(cmdbuff, tempbuff);

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}

// moveABS
int32 ZauxRobot::moveABS(const std::vector<int>& axis, const std::vector<float>& endMove, float speed) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(endMove.size(), axis.size());
	if (num < 1)
		return 1;

	int ret = 0;
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

	// 单独设置速度
	if (speed > 0) {
		sprintf(tempbuff, "FORCE_SPEED(%d) = %f", axis[0], speed);
		strcat(cmdbuff, tempbuff);
	}

	strcat(cmdbuff, "MOVEABSSP(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%f,", endMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", endMove[num - 1]);
	strcat(cmdbuff, tempbuff);

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}

// 获取轴号信息
int32 ZauxRobot::get_axis_param(const std::vector<int>& axisList, char* paramName, std::vector<float>& paramList) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	if (axisList.size() < 1) {
		return 1;
	}
	paramList = std::vector<float>(axisList.size(), 0);

	//生成命令
	sprintf(cmdbuff, "?%s(%d)", paramName, axisList[0]);
	for (size_t i = 1; i < axisList.size(); ++i) {
		sprintf(tempbuff, ",%s(%d)", paramName, axisList[i]);
		strcat(cmdbuff, tempbuff);
	}

	int32 iresult = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	// 判断返回状态
	if (ERR_OK != iresult) {
		return handle_zaux_error(iresult);
	}
	if (0 == strlen(cmdbuffAck)) {
		return handle_zaux_error(ERR_NOACK);
	}

	// 解析返回值
	std::stringstream ackStr(cmdbuffAck);
	std::string word;
	// Extract word from the stream
	size_t i = 0;
	while (ackStr >> word) {
		if (i >= axisList.size())
			break;
		//paramList.push_back(std::stof(word));
		paramList[i++] = std::stof(word);
	}

	return handle_zaux_error(ret);
}

// 设置轴号信息
int32 ZauxRobot::set_axis_param(const std::vector<int>& axisList, char* paramName, const std::vector<float>& paramList, int principal) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

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
	//std::cout << cmdbuff << std::endl;
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}


/* ******************************** 工业机械臂常用指令封装 *********************************
* @brief 常用指令接口的实现。ZauxRobot -> 工业机械臂常用指令
*/
// 点到点同步运动：move_ptp
int32 ZauxRobot::move_ptp(const std::vector<float>& relEndMove, float speedRatio) {
	std::vector<int> axisIdx = { 0,1,2,3,4,5,6,7,8 };

	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0, num = std::min(axisIdx.size(), relEndMove.size());

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%d,", axisIdx[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)", axisIdx[num - 1]);
	strcat(cmdbuff, tempbuff);
	strcat(cmdbuff, "\n");
	
	// 设置速度
	if (speedRatio > 0) {
		sprintf(tempbuff, "SPEED_RATIO = %f\n", speedRatio);
		strcat(cmdbuff, tempbuff);
	}

	strcat(cmdbuff, "MOVE_PTP(1,");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%f,", relEndMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", relEndMove[num - 1]);
	strcat(cmdbuff, tempbuff);

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}

int32 ZauxRobot::move_ptp_abs(const std::vector<float>& endMove, float speedRatio) {
	std::vector<int> axisIdx = { 0,1,2,3,4,5,6,7,8 };

	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0, num = std::min(axisIdx.size(), endMove.size());

	strcpy(cmdbuff, "BASE(");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%d,", axisIdx[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)", axisIdx[num - 1]);
	strcat(cmdbuff, tempbuff);
	strcat(cmdbuff, "\n");

	// 设置速度
	if (speedRatio > 0) {
		sprintf(tempbuff, "SPEED_RATIO = %f\n", speedRatio);
		strcat(cmdbuff, tempbuff);
	}

	strcat(cmdbuff, "MOVE_PTPABS(1,");
	for (size_t i = 0; i < num - 1; i++) {
		sprintf(tempbuff, "%f,", endMove[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%f)", endMove[num - 1]);
	strcat(cmdbuff, tempbuff);

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}

int32 ZauxRobot::moveJ(const std::vector<float>& relEndMove, float speedRatio, bool replace) {
	std::vector<int> axis = replace ? excuteAxis : fkAxis;
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(relEndMove.size(), axis.size());
	axis.assign(axis.begin(), axis.begin() + num);
	int ret = 0;

	std::vector<float> moveCmd(num, 0);
	for (size_t i = 0; i < num; ++i) {
		moveCmd[i] = relEndMove[i];
	}

	// 获取各个轴速度
	std::vector<float> axisSpeed;
	ret = get_axis_param(fkAxis, (char*)"SPEED", axisSpeed);

	// 计算时间运动时间最久的轴
	float maxTime = 0.0;
	int axisIdx = 0;
	for (size_t i = 0; i < num; ++i) {
		// 防止除零
		axisSpeed[i] = axisSpeed[i] < 1e-3 ? 1e3 : axisSpeed[i];
		float tmpTime = std::fabs(moveCmd[i]) / axisSpeed[i];
		if (tmpTime > maxTime) {
			maxTime = tmpTime;
			axisIdx = i;
		}
	}

	// 只计算最慢轴的速度
	std::vector<float> interpVec(axis.size(), 0);
	interpVec[axisIdx] = 1;
	ret = set_axis_param(axis, (char*)"INTERP_FACTOR", interpVec);
	if (ret != 0)
		return handle_zaux_error(ret);

	// 设置轴运动速度
	if (speedRatio > 0) {
		ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], axisSpeed[axisIdx] * speedRatio);
	}

	// 发送运动指令
	ret = moveL(axis, moveCmd);
	if (ret != 0)
		return handle_zaux_error(ret);

	return handle_zaux_error(ret);
}
int32 ZauxRobot::moveJ(const std::vector<int>& axis, const std::vector<float>& relEndMove, float speedRatio) {
	return 0;
}


int32 ZauxRobot::moveJ_abs(const std::vector<float>& endMove, float speedRatio, bool replace) {
	std::vector<int> axis = replace ? excuteAxis : fkAxis;
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = std::min(endMove.size(), axis.size());
	axis.assign(axis.begin(), axis.begin() + num);
	int ret = 0;

	// 获取各轴缓冲最后位置
	std::vector<float> relEndMove(fkAxis.size(), 0);
	ret = get_axis_param(fkAxis, (char*)"ENDMOVE_BUFFER", relEndMove);

	// 转换到相对运动
	for (size_t i = 0; i < num; ++i) {
		relEndMove[i] = endMove[i] - relEndMove[i];
	}

	ret = moveJ(relEndMove, speedRatio, replace);

	return ret;
}
int32 ZauxRobot::moveJ_abs(const std::vector<int>& axis, const std::vector<float>& endMove, float speedRatio) {
	return 0;
}


int32 ZauxRobot::moveL(const std::vector<int>& axis, const std::vector<float>& relEndMove) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(relEndMove.size(), axis.size());
	int ret = 0;
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

	//std::cout << cmdbuff << std::endl;
	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}
int32 ZauxRobot::moveL(const std::vector<int>& axis, const std::vector<float>& begPoint, std::vector<float>& endPoint) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(begPoint.size(), axis.size());
	int ret = 0;

	// 终点到起点的相对运动
	std::vector<float> relEndMove(num, 0);
	for (size_t i = 0; i < num; ++i) {
		relEndMove[i] = endPoint[i] - begPoint[i];
	}

	// 欧拉角转换到相对运动: beg -> mid -> end
	Eigen::Vector3f begEuler = Eigen::Vector3f(begPoint[3], begPoint[4], begPoint[5]);
	Eigen::Vector3f endEuler = Eigen::Vector3f(endPoint[3], endPoint[4], endPoint[5]);
	Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		// 欧拉角相对值
		relEndMove[3 + i] = relEuler[i];
		// 终点欧拉角转换到与起点同侧
		endPoint[3 + i] = endEuler[i];
	}

	// 相对运动
	ret = this->move(axis, relEndMove);
	if (ret != 0)
		return handle_zaux_error(ret);
	return 0;
}

int32 ZauxRobot::moveC(const std::vector<int>& axis, const std::vector<float>& begPoint, std::vector<float>& midPoint,
						 std::vector<float>& endPoint, int imode) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(begPoint.size(), axis.size());

	// 中间点相对值
	std::vector<float> relMidMove(num);
	for (size_t i = 0; i < num; ++i) {
		relMidMove[i] = midPoint[i] - begPoint[i];
	}
	// 终点相对值
	std::vector<float> relEndMove(num);
	for (size_t i = 0; i < num; ++i) {
		relEndMove[i] = endPoint[i] - begPoint[i];
	}
	// 相对角度: beg -> mid -> end
	Eigen::Vector3f begEuler, endEuler, relEuler(0,0,0);
	begEuler = Eigen::Vector3f(begPoint[3], begPoint[4], begPoint[5]);
	if (imode == 0) {
		endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
		relEuler = get_zyx_euler_distance(begEuler, endEuler);
		begEuler = endEuler;
	}
	endEuler = Eigen::Vector3f(endPoint[3], endPoint[4], endPoint[5]);
	relEuler += get_zyx_euler_distance(begEuler, endEuler);
	for (size_t i = 0; i < 3; ++i) {
		// 欧拉角相对值
		relEndMove[3 + i] = relEuler[i];
		// 中间点欧拉角转换到与起点同侧
		midPoint[3 + i] = begEuler[i];
		// 终点欧拉角转换到与起点同侧
		endPoint[3 + i] = endEuler[i];
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
	int ret =  ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}


// 摆焊功能
int32 ZauxRobot::save_table(size_t startIdx, size_t num, const std::string& path) {
	if (num == 0) {
		return 0;
	}
	int ret = 0;

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
		ret = ZAux_Direct_GetTable(handle_, startIdx + maxNum * i, maxNum, tableData.data());
		// 判断返回状态
		if (ret != 0)
			return handle_zaux_error(ret);
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

	return handle_zaux_error(ret);
}


int32 ZauxRobot::update_swing_table(const Weave& waveCfg) {
	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;
	int ret = 0;
	
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
		// 左右摆幅不同
		if (std::fabs(waveCfg.LeftWidth - waveCfg.RightWidth) > 1e-1) {
			for (size_t i = 0; i < numInterp; ++i) {
				waveGenerator[i] = std::sin(2 * M_PI * i / (numInterp - 1));
				// 左摆动
				waveGenerator[i] *= i > numInterp / 2 ? waveCfg.LeftWidth / waveCfg.RightWidth : 1;
			}
			sinTableBeg = 2100;
			for (size_t i = 0; i < numInterp; ++i) {
				ret = ZAux_Direct_MoveTable(handle_, swingAxisIdx[0], sinTableBeg + i, waveGenerator[i]);
				// 判断返回状态
				if (ret != 0)
					return handle_zaux_error(ret);
			}
		}
		else {
			robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
		}
	}
	// 摆动停留时间
	else if (holdType == 0 && waveCfg.Dwell_left + waveCfg.Dwell_right > 0) {
		swingHoldTime = waveCfg.Dwell_left + waveCfg.Dwell_right;
		// 周期时间(ms)
		float totalTime = 1000 / freq + swingHoldTime;
		// 四分之一摆动周期占用的 table 个数
		size_t numQuarter = numInterp * (1000 / freq) / totalTime / 4;
		// 右停留时间占用的 table 个数
		//size_t numRightHold = numInterp * waveCfg.Dwell_right / totalTime;
		size_t numRightHold = (numInterp - 4 * numQuarter) * waveCfg.Dwell_right / swingHoldTime;

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

		sinTableBeg = 2100;
		for (size_t i = 0; i < numInterp; ++i) {
			ret = ZAux_Direct_MoveTable(handle_, excuteAxis[0], sinTableBeg + i, waveGenerator[i]);
			// 判断返回状态
			if (ret != 0)
				return handle_zaux_error(ret);
		}
	}

	// 自定义摆焊函数

	return handle_zaux_error(ret);
}


int32 ZauxRobot::swing_on(float vel, const Weave& waveCfg, const std::vector<float>& toolDir) {
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
	if (holdType == 0) {
		swingHoldTime = waveCfg.Dwell_left + waveCfg.Dwell_right;
	}

	//sinTableBeg = holdType > 0 ? 2000 : 2100;
	sinTableBeg = ((holdType == 0 && waveCfg.Dwell_left + waveCfg.Dwell_right > 0) || (std::fabs(waveCfg.LeftWidth - waveCfg.RightWidth) > 1e-1)) ? 2100 : 2000;

	// 周期长度
	float dist = vel * (1/freq + swingHoldTime/1000);

	Eigen::Vector3f zDir(0, 0, 0);
	// 自动计算焊枪角度
	if (toolDir.size() < 3) {
		// 读取缓冲最终位置处的欧拉角(deg)
		Eigen::Vector3f zEuler(0, 0, 0);
		for (size_t i = 0; i < 3; ++i) {
			ret = ZAux_Direct_GetEndMoveBuffer(handle_, tcpAngleAxisIdx[i], &zEuler[i]);
			if (ret != 0)
				return handle_zaux_error(ret);
		}
		zEuler *= M_PI / 180;
		zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
		zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
		zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	}
	// 给定焊枪角度
	else {
		zDir = Eigen::Vector3f(toolDir[0], toolDir[1], toolDir[2]);
	}

	// 设置摆角
	//if (toolDir.size() > 0) {
	//	Eigen::Vector3f tanDir(toolDir[0], toolDir[1], toolDir[2]);
	//	tanDir.normalize();
	//	zDir = (Eigen::AngleAxisf(waveCfg.Angle_Ltype_top * M_PI / 180, tanDir) * zDir).eval();
	//}

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", swingAxisIdx[0]);
	float vectorBuffered2 = 0.0;
	ret = ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);
	if (ret != 0)
		return handle_zaux_error(ret);

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
		camAxisIdx[0], camAxisIdx[1], camAxisIdx[2],
		// mode, 主轴, 矢量距离, 周期长度, 左右摆幅, 开始Table, 结束Table
		5, swingAxisIdx[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1,
		zDir[0], zDir[1], zDir[2]
	);
	//std::cout << cmdbuff  << std::endl;

	//调用命令执行函数
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return handle_zaux_error(ret);
}


int32 ZauxRobot::swing_off(float displacement) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", swingAxisIdx[0]);
	float vectorBuffered2 = 0.0;
	ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);
	vectorBuffered2 += displacement;

	//生成命令
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f)",
		camAxisIdx[0], camAxisIdx[1], camAxisIdx[2],
		// mode, 主轴, 矢量距离
		-1, swingAxisIdx[0], vectorBuffered2
	);
	//std::cout << cmdbuff << std::endl;

	//调用命令执行函数
	int ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	return handle_zaux_error(ret);
}


int32 ZauxRobot::wlder_on(float current, float voltage) {
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


int32 ZauxRobot::wlder_off() {
	// 关气
	//ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 2, 5, 0);

	// 结束焊接
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 1, 5, 6);
	return 0;
}


/* ******************************** 特殊运动指令 *********************************
* @brief 连续轨迹运动
*/
int32 ZauxRobot::execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory, bool replace) {
	return execute_discrete_trajectory(discreteTrajectory, replace);
}
int32 ZauxRobot::execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory, bool replace) {
	std::vector<int> axis = replace ? excuteAxis : ikAxis;
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());
	axis.assign(axis.begin(), axis.begin() + num);
	int ret = 0;

	// 只计算TCP位置速度
	std::vector<float> interpVec(axis.size(), 0);
	for (size_t i = 0; i < 3; ++i) {
		interpVec[i] = 1;
	}
	ret = set_axis_param(axis, (char*)"INTERP_FACTOR", interpVec);
	if (ret != 0)
		return handle_zaux_error(ret);

	float trajIdx = 0;
	// 轨迹计数复位
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	if (ret != 0)
		return handle_zaux_error(ret);

	discreteTrajectory.reset_iterator();
	while (discreteTrajectory.next() == 0) {
		auto prePoint = discreteTrajectory.get_preNodePoint();
		auto curPoint = discreteTrajectory.get_nodePoint();
		auto midPoint = discreteTrajectory.get_midPoint();
		auto traj = discreteTrajectory.get_trajInfo();

		// 轨迹速度
		float vel = traj.speed;
		// 设置速度
		ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);
		if (ret != 0)
			return handle_zaux_error(ret);
		// 设置平滑度
		if (traj.smooth >= 0) {
			ret = ZAux_Direct_SetZsmooth(handle_, axis[0], traj.smooth);
			if (ret != 0)
				return handle_zaux_error(ret);
		}

		// 当前轨迹的相对运动量
		std::vector<float> relEndMove = discreteTrajectory.get_relative_distance();

		// 圆弧运动
		if (traj.isArc()) {
			ret = this->moveC(axis, prePoint, midPoint, curPoint);
			if (ret != 0)
				return handle_zaux_error(ret);
		}
		// 直线运动
		else {
			// 相对运动
			//ret = this->moveL(axis, prePoint, curPoint);
			ret = this->move(axis, relEndMove);
			if (ret != 0)
				return handle_zaux_error(ret);
		}

		// 轨迹数+1
		ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);
		if (ret != 0)
			return handle_zaux_error(ret);

	}

	return handle_zaux_error(ret);
}

int32 ZauxRobot::swing_tri(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	//std::vector<int> axis = swingAxisIdx, camAxis = camAxisIdx;
	//axis.insert(axis.end(), tcpAngleAxisIdx.begin(), tcpAngleAxisIdx.end());
	//axis.insert(axis.end(), appAxisIdx.begin(), appAxisIdx.end());
	//axis.insert(axis.end(), camAxisIdx.begin(), camAxisIdx.end());

	// 轨迹点维度与驱动轴维度的较小值
	//size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());
	//int ret = 0;
	/*
	// 只计算TCP位置速度
	//for (size_t i = 0; i < axis.size(); ++i) {
	//	ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], i < 3);
	//}
	std::vector<float> interpVec(axis.size(), 0);
	for (size_t i = 0; i < 3; ++i) {
		interpVec[i] = 1;
	}
	ret = set_axis_param(axis, (char*)"INTERP_FACTOR", interpVec);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ret = ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
		if (ret != 0)
			return handle_zaux_error(ret);
	}

	// 读取三角摆形状: 摆幅、前进比例、后退比例
	float triWidth = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2, feedRatio = waveCfg.Length, backRatio =  waveCfg.Bias;
	// 摆焊参数
	float swingVel = (waveCfg.LeftWidth + waveCfg.RightWidth) * waveCfg.Freq, rightAngle = waveCfg.Angle_Ltype_top, leftAngle = waveCfg.Angle_Ltype_btm;
	// 轨迹时间
	float timeUnit = 1 / (2 * waveCfg.Freq), feedTime = timeUnit * feedRatio, backTime = timeUnit * backRatio;

	// 摆动开始
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);
	if (ret != 0)
		return handle_zaux_error(ret);

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	if (ret != 0)
		return handle_zaux_error(ret);
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
			ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], weldVel);
			if (ret != 0)
				return handle_zaux_error(ret);
			ret = this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
			if (ret != 0)
				return handle_zaux_error(ret);
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
			ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], swingVel);
			if (ret != 0)
				return handle_zaux_error(ret);

			std::vector<float> moveCmd(axis.size(), 0);

			for (size_t i = 0; i < numPeriod+1; ++i) {
				// 右侧向右
				for (size_t j = 0; j < moveCmd.size(); ++j) {
					moveCmd[j] = j < 3 ? rightDir[j] * triWidth : 0;
				}
				ret = this->moveL(axis, moveCmd);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				if (ret != 0)
					return handle_zaux_error(ret);

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
				ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], triWidth/feedTime);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = this->moveL(axis, moveCmd);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_center);
				if (ret != 0)
					return handle_zaux_error(ret);

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
				ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], triWidth / backTime);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = this->moveL(axis, moveCmd);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
				if (ret != 0)
					return handle_zaux_error(ret);

				// 左侧向右
				for (size_t j = 0; j < moveCmd.size(); ++j) {
					moveCmd[j] = j < 3 ? -leftDir[j] * triWidth : 0;
				}
				ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], swingVel);
				if (ret != 0)
					return handle_zaux_error(ret);
				ret = this->moveL(axis, moveCmd);
				if (ret != 0)
					return handle_zaux_error(ret);
			}
		}

		// 轨迹数+1
		ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);
		if (ret != 0)
			return handle_zaux_error(ret);

		midPointIte++;
		infoIte++;
	}

	// 摆动停止
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	if (ret != 0)
		return handle_zaux_error(ret);
	*/
	return 0;
}
int32 ZauxRobot::swing_sin(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	//std::vector<int> axis = swingAxisIdx_;
	//axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	//axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	std::vector<int> axis = ikAxis;

	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());
	int ret = 0;
	/*
	// 只计算TCP位置速度
	std::vector<float> interpVec(axis.size(), 0);
	for (size_t i = 0; i < 3; ++i) {
		interpVec[i] = 1;
	}
	ret = set_axis_param(axis, (char*)"INTERP_FACTOR", interpVec);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ret = ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
		if (ret != 0)
			return handle_zaux_error(ret);
	}
	// 设置摆焊参数
	ret = update_swing_table(waveCfg);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 摆动开始
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);
	if (ret != 0)
		return handle_zaux_error(ret);

	// 遍历轨迹
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	float trajIdx = 0;
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, -1);
	if (ret != 0)
		return handle_zaux_error(ret);
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
			// 欧拉角相对值
			relEndMove[3 + i] = relEuler[i];
			// 终点欧拉角转换到与起点同侧
			(*nodePointIte)[3 + i] = endEuler[i];
		}
		begEuler = endEuler;

		// 轨迹速度
		float vel = (*infoIte)[7];
		// 设置速度
		ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);
		if (ret != 0)
			return handle_zaux_error(ret);
		// 设置平滑度
		if ((*infoIte)[8] >= 0) {
			ret = ZAux_Direct_SetZsmooth(handle_, axis[0], (*infoIte)[8]);
			if (ret != 0)
				return handle_zaux_error(ret);
		}

		int numPeriod = std::round(std::fabs((*infoIte)[3]) / (vel / waveCfg.Freq));
		// 小于1个周期则走不摆动
		if (numPeriod > 0) {
			ret = swing_on(vel, waveCfg);
			if (ret != 0)
				return handle_zaux_error(ret);

			// 计算轴运动距离
			ret = swing_off(std::fabs((*infoIte)[3]));
			if (ret != 0)
				return handle_zaux_error(ret);
		}

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
				ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				if (ret != 0)
					return handle_zaux_error(ret);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_right > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
					if (ret != 0)
						return handle_zaux_error(ret);
				}

				// 1/2 * 2周期
				for (size_t i = 0; i < numPeriod; ++i) {
					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					if (ret != 0)
						return handle_zaux_error(ret);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_left > 0) {
						ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
						if (ret != 0)
							return handle_zaux_error(ret);

					}
					// 最后一个周期退出
					if (i +1 == numPeriod) break;

					partial += 1.0 / (2 * numPeriod);
					tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
					for (size_t i = 0; i < num; ++i) {
						tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
					}
					ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
					if (ret != 0)
						return handle_zaux_error(ret);
					tempBegPoint = tempEndPoint;
					if (waveCfg.Dwell_right > 0) {
						ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
						if (ret != 0)
							return handle_zaux_error(ret);
						
					}
				}

				// 1/4 周期
				partial += 1.0 / (4 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (endBuffer[i] + relEndMove[i] * partial);
				}
				ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				if (ret != 0)
					return handle_zaux_error(ret);
				tempBegPoint = tempEndPoint;
			}
			// 摆动停止
			else {
				ret = this->moveC(axis, endBuffer, *midPointIte, *nodePointIte);
				if (ret != 0)
					return handle_zaux_error(ret);
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
				ret = this->moveL(axis, detRelEndMove);
				if (ret != 0)
					return handle_zaux_error(ret);
				if (waveCfg.Dwell_right > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
					if (ret != 0)
						return handle_zaux_error(ret);
				}

				// 1/2 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] *= 2;
				for (int i = 0; i < numPeriod; ++i) {
					ret = this->moveL(axis, detRelEndMove);
					if (ret != 0)
						return handle_zaux_error(ret);
					if (waveCfg.Dwell_left > 0) {
						ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
						if (ret != 0)
							return handle_zaux_error(ret);
					}
					// 最后一个周期退出
					if (i+1 == numPeriod) break;

					ret = this->moveL(axis, detRelEndMove);
					if (ret != 0)
						return handle_zaux_error(ret);
					if (waveCfg.Dwell_right > 0) {
						ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
						if (ret != 0)
							return handle_zaux_error(ret);
					}
				}
				// 1/4 周期运动量
				for (size_t j = 0; j < relEndMove.size(); ++j)
					detRelEndMove[j] /= 2;
				ret = this->moveL(axis, detRelEndMove);
				if (ret != 0)
					return handle_zaux_error(ret);
			}
			// 摆动停止
			else {
				// 相对运动
				ret = this->moveL(axis, relEndMove);
				if (ret != 0)
					return handle_zaux_error(ret);
			}
		}

		// 轨迹数+1
		ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);
		if (ret != 0)
			return handle_zaux_error(ret);

		midPointIte++;
		infoIte++;
	}

	// 摆动停止
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	if (ret != 0)
		return handle_zaux_error(ret);
	*/
	return 0;
}

int32 ZauxRobot::swing_tri() {
	return 0;
}


int32 ZauxRobot::swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
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
	return handle_zaux_error(ret);
}


// 连续轨迹摆焊
int32 ZauxRobot::swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), swingAxis.size());
	std::vector<int> axis(swingAxis.begin(), swingAxis.begin() + num);
	int ret = 0;
	
	// 只计算TCP位置速度
	std::vector<float> interpVec(axis.size(), 0);
	for (size_t i = 0; i < 3; ++i) {
		interpVec[i] = 1;
	}
	ret = set_axis_param(axis, (char*)"INTERP_FACTOR", interpVec);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 凸轮轴清零
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ret = ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx[i], 0);
		if (ret != 0)
			return handle_zaux_error(ret);
	}
	// 摆动开始
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);
	if (ret != 0)
		return handle_zaux_error(ret);

	
	float trajIdx = -1;
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);
	if (ret != 0)
		return handle_zaux_error(ret);

	discreteTrajectory.reset_iterator();
	while (discreteTrajectory.next() == 0) {
		auto prePoint = discreteTrajectory.get_preNodePoint();
		auto curPoint = discreteTrajectory.get_nodePoint();
		auto midPoint = discreteTrajectory.get_midPoint();
		auto traj = discreteTrajectory.get_trajInfo();

		// 轨迹速度
		float vel = traj.speed;
		if (ret != 0)
			return handle_zaux_error(ret);

		// 读取焊接参数
		Weave waveCfg;
		Arc_WeldingParaItem weldCfg;
		Track trackCfg;
		read_weld_param(traj.appendix, waveCfg, weldCfg, trackCfg);

		// 构建当前轨迹信息
		DiscreteTrajectory<float> curTraj;
		curTraj.set_starting_point(prePoint);
		// 圆弧轨迹
		if (traj.isArc()) {
			curTraj.add_arc(curPoint, midPoint, traj);
		}
		else {
			curTraj.add_line(curPoint, traj);
		}

		// 修改焊接参数
		//update_welder_config(weldCfg);
		// 修改跟踪参数
		//update_track_config(trackCfg);

		// 设置平滑度
		if (traj.smooth >= 0) {
			ret = ZAux_Direct_SetZsmooth(handle_, axis[0], traj.smooth);
			if (ret != 0)
				return handle_zaux_error(ret);
		}
		// 设置速度
		ret = ZAux_Direct_SetForceSpeed(handle_, axis[0], vel);

		// 不摆动
		if (waveCfg.Id == 0) {
			swing_single_empty(curTraj);
		}
		// 开始摆焊
		else if (waveCfg.Shape == 0) {
			swing_single_sin(curTraj, waveCfg);
		}
		else if (waveCfg.Shape == 3) {
			swing_single_tri(curTraj, waveCfg);
		}
		else {

		}
		// 轨迹计数+1
		ret = ZAux_Direct_MoveTable(handle_, axis[0], 1001, trajIdx++);
	}

	// 摆动停止
	ret = ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	if (ret != 0)
		return handle_zaux_error(ret);

	return handle_zaux_error(ret);
}

int32 ZauxRobot::swing_single_sin(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), swingAxis.size());
	std::vector<int> axis(swingAxis.begin(), swingAxis.begin() + num);
	int ret = 0;

	discreteTrajectory.reset_iterator();
	if (discreteTrajectory.next() != 0) {
		return 1;
	}

	auto prePoint = discreteTrajectory.get_preNodePoint();
	auto curPoint = discreteTrajectory.get_nodePoint();
	auto midPoint = discreteTrajectory.get_midPoint();
	auto traj = discreteTrajectory.get_trajInfo();

	// 设置摆动参数
	ret = update_swing_table(waveCfg);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 小于1个周期则不摆动
	int numPeriod = std::round(traj.get_dist() / (traj.speed / waveCfg.Freq));
	if (numPeriod > 0) {
		// 计算摆焊方向
		std::vector<float> zDir(3), zEuler = { static_cast<float>(prePoint[3] * M_PI / 180),
			static_cast<float>(prePoint[4] * M_PI / 180),
			static_cast<float>(prePoint[5] * M_PI / 180) };

		zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
		zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
		zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

		ret = swing_on(traj.speed, waveCfg, zDir);
		if (ret != 0)
			return handle_zaux_error(ret);

		// 计算轴运动距离
		ret = swing_off(traj.get_dist());
		if (ret != 0)
			return handle_zaux_error(ret);
	}

	// 当前轨迹的相对运动量
	std::vector<float> relEndMove = discreteTrajectory.get_relative_distance();

	// 圆弧运动
	if (traj.isArc()) {
		// 机器人停止
		if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0 && numPeriod > 0) {
			std::vector<float> cent = traj.get_knot(), rotDir = traj.get_dir();
			// 计算点位置
			Eigen::Vector3f tempPos, rotNorm(rotDir[0], rotDir[1], rotDir[2]);
			Eigen::Vector3f radiusDir(0, 0, 0), centerPos(cent[0], cent[1], cent[2]);

			std::vector<float> tempBegPoint = prePoint, tempEndPoint(prePoint.size(), 0), tempCenPoint(prePoint.size(), 0);

			for (size_t i = 0; i < 3; ++i) {
				tempCenPoint[i] = centerPos[i];
				radiusDir[i] = prePoint[i] - centerPos[i];
			}
			float partial = 0, theta = rotNorm.norm();
			rotNorm.normalize();

			// 1/4 周期
			partial += 1.0 / (4 * numPeriod);
			tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
			// 位置分量单独计算，姿态和附加值按线性累加
			for (size_t i = 0; i < num; ++i) {
				tempEndPoint[i] = i < 3 ? tempPos[i] : (prePoint[i] + relEndMove[i] * partial);
			}
			ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
			if (ret != 0)
				return handle_zaux_error(ret);
			tempBegPoint = tempEndPoint;
			if (waveCfg.Dwell_right > 0) {
				ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				if (ret != 0)
					return handle_zaux_error(ret);
			}

			// 1/2 * 2周期
			for (size_t i = 0; i < numPeriod; ++i) {
				partial += 1.0 / (2 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (prePoint[i] + relEndMove[i] * partial);
				}
				ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				if (ret != 0)
					return handle_zaux_error(ret);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_left > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					if (ret != 0)
						return handle_zaux_error(ret);

				}
				// 最后一个周期退出
				if (i + 1 == numPeriod) break;

				partial += 1.0 / (2 * numPeriod);
				tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
				for (size_t i = 0; i < num; ++i) {
					tempEndPoint[i] = i < 3 ? tempPos[i] : (prePoint[i] + relEndMove[i] * partial);
				}
				ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
				if (ret != 0)
					return handle_zaux_error(ret);
				tempBegPoint = tempEndPoint;
				if (waveCfg.Dwell_right > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
					if (ret != 0)
						return handle_zaux_error(ret);

				}
			}

			// 1/4 周期
			partial += 1.0 / (4 * numPeriod);
			tempPos = Eigen::AngleAxisf(partial * theta, rotNorm) * radiusDir + centerPos;
			for (size_t i = 0; i < num; ++i) {
				tempEndPoint[i] = i < 3 ? tempPos[i] : (prePoint[i] + relEndMove[i] * partial);
			}
			ret = this->moveC(axis, tempBegPoint, tempCenPoint, tempEndPoint, 1);
			if (ret != 0)
				return handle_zaux_error(ret);
			tempBegPoint = tempEndPoint;
		}
		// 摆动停止
		else {
			ret = this->moveC(axis, prePoint, midPoint, curPoint);
			if (ret != 0)
				return handle_zaux_error(ret);
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
			ret = this->moveL(axis, detRelEndMove);
			if (ret != 0)
				return handle_zaux_error(ret);
			if (waveCfg.Dwell_right > 0) {
				ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
				if (ret != 0)
					return handle_zaux_error(ret);
			}

			// 1/2 周期运动量
			for (size_t j = 0; j < relEndMove.size(); ++j)
				detRelEndMove[j] *= 2;
			for (int i = 0; i < numPeriod; ++i) {
				ret = this->moveL(axis, detRelEndMove);
				if (ret != 0)
					return handle_zaux_error(ret);
				if (waveCfg.Dwell_left > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_left);
					if (ret != 0)
						return handle_zaux_error(ret);
				}
				// 最后一个周期退出
				if (i + 1 == numPeriod) break;

				ret = this->moveL(axis, detRelEndMove);
				if (ret != 0)
					return handle_zaux_error(ret);
				if (waveCfg.Dwell_right > 0) {
					ret = ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);
					if (ret != 0)
						return handle_zaux_error(ret);
				}
			}
			// 1/4 周期运动量
			for (size_t j = 0; j < relEndMove.size(); ++j)
				detRelEndMove[j] /= 2;
			ret = this->moveL(axis, detRelEndMove);
			if (ret != 0)
				return handle_zaux_error(ret);
		}
		// 摆动停止
		else {
			// 相对运动
			ret = this->moveL(axis, relEndMove);
			if (ret != 0)
				return handle_zaux_error(ret);
		}
	}

	return handle_zaux_error(ret);
}

int32 ZauxRobot::swing_single_tri(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	return 0;
}

int32 ZauxRobot::swing_single_empty(DiscreteTrajectory<float>& discreteTrajectory) {
	// 轨迹点维度与驱动轴维度的较小值
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), swingAxis.size());
	std::vector<int> axis(swingAxis.begin(), swingAxis.begin() + num);
	int ret = 0;

	discreteTrajectory.reset_iterator();
	if (discreteTrajectory.next() != 0) {
		return 1;
	}

	auto prePoint = discreteTrajectory.get_preNodePoint();
	auto curPoint = discreteTrajectory.get_nodePoint();
	auto midPoint = discreteTrajectory.get_midPoint();
	auto traj = discreteTrajectory.get_trajInfo();
	// 当前轨迹的相对运动量
	std::vector<float> relEndMove = discreteTrajectory.get_relative_distance();

	// 圆弧运动
	if (traj.isArc()) {
		ret = this->moveC(axis, prePoint, midPoint, curPoint);
		if (ret != 0)
			return handle_zaux_error(ret);
	}
	// 直线运动
	else {
		ret = this->moveL(axis, relEndMove);
		if (ret != 0)
			return handle_zaux_error(ret);
	}
}

int32 ZauxRobot::update_track_config(const Track& trackCfg) {
	std::vector<float> config(20, 0);
	size_t configTableStart = 1010;
	int ret = 0;

	// 读取现有参数
	ret = ZAux_Direct_GetTable(handle_, configTableStart, config.size(), config.data());
	if (ret != 0)
		return handle_zaux_error(ret);

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

	//ret = ZAux_Direct_SetTable(handle_, configTableStart, config.size(), config.data());
	//if (ret != 0)
	//	return handle_zaux_error(ret);
	for (size_t i = 0; i < config.size(); ++i) {
		ret = ZAux_Direct_MoveTable(handle_, swingAxisIdx[0], 1010+i, config[i]);
		if (ret != 0)
			return handle_zaux_error(ret);
	}

	return handle_zaux_error(ret);
}

int32 ZauxRobot::update_welder_config(const Arc_WeldingParaItem& weldCfg) {
	//std::vector<float> config(20, 0);
	size_t configTableStart = 1030;
	int ret = 0;

	// 电流
	ret = ZAux_Direct_MoveTable(handle_, swingAxisIdx[0], 1035, weldCfg.WeldingCrt_Spd);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 电压
	ret = ZAux_Direct_MoveTable(handle_, swingAxisIdx[0], 1036, weldCfg.WeldingVtg_Strth);
	if (ret != 0)
		return handle_zaux_error(ret);
	// 执行修改
	ret = ZAux_Direct_MoveTable(handle_, swingAxisIdx[0], 1030, 1);
	if (ret != 0)
		return handle_zaux_error(ret);

	return handle_zaux_error(ret);
}

int32 ZauxRobot::handle_zaux_error(int32 errCode) {
	if (errCode != 0) {
		this->emergency_pause();
	}
	return errCode;
}


/* ******************************** 下位机交互 *********************************/
// 下位机紧急信号触发
int32 ZauxRobot::emergency_pause() {
	float zero = 0.0, pauseFlag = 1.0;
	int ret = ZAux_Direct_SetTable(handle_, 1004, 1, &pauseFlag);
	std::cout << "Active PAUSE triggered." << std::endl;
	return ret;
}

int32 ZauxRobot::emergency_resume() {
	float zero = 0.0, resumeFlag = 2.0;
	int ret = ZAux_Direct_SetTable(handle_, 1004, 1, &resumeFlag);
	std::cout << "Active PAUSE triggered." << std::endl;
	return ret;
}

int32 ZauxRobot::emergency_stop() {
	float zero = 0.0, stopFlag = 3.0, negtive = -1.0;
	int ret = 0;
	ZAux_Direct_SetTable(handle_, 1004, 1, &stopFlag);
	std::cout << "Active PAUSE triggered." << std::endl;

	// 摆动标志位复位
	ZAux_Direct_SetTable(handle_, 1000, 1, &negtive);

	return ret;
}


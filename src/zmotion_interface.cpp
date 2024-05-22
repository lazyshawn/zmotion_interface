
#include<chrono>
#include<thread>
#include<iostream>

#include"zmotion_interface.h"

// ���ռ���б�
std::vector<int> ZauxAxis::AxisIdxList = std::vector<int>(100, 0);

ZauxAxis::ZauxAxis() {
}

ZauxRobot::ZauxRobot() {
}

ZauxRobot::ZauxRobot(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
					 const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx,
					 const std::vector<int>& connpathAxisIdx) {

	set_axis(jointAxisIdx_, ikPosAxisIdx_, tcpAngleAxisIdx_, tcpPosAxisIdx_, appAxisIdx_, camAxisIdx_, connpathAxisIdx_);
}

uint8_t ZauxRobot::set_handle(ZMC_HANDLE handle) {
	handle_ = handle;

	return 0;
}

uint8_t ZauxRobot::set_axis(const std::vector<int>& jointAxisIdx, const std::vector<int>& ikPosAxisIdx, const std::vector<int>& tcpAngleAxisIdx,
	const std::vector<int>& tcpPosAxisIdx, const std::vector<int>& appAxisIdx, const std::vector<int>& camAxisIdx, const std::vector<int>& connpathAxisIdx) {
	jointAxisIdx_ = jointAxisIdx;
	ikPosAxisIdx_ = ikPosAxisIdx;
	tcpAngleAxisIdx_ = tcpAngleAxisIdx;
	tcpPosAxisIdx_ = tcpPosAxisIdx;
	appAxisIdx_ = appAxisIdx;
	camAxisIdx_ = camAxisIdx;
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
	else {
		printf("Succeed\n"); 
		ZAux_SetTraceFile(3, "sdf");
	}
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
	else {
		printf("Succeed\n");
	}
	ZAux_Direct_GetUnits(handle_, tcpPosAxisIdx_[0], &axisUnits);
	return 0;
}

uint8_t ZauxRobot::lazy_connect() {
	if (this->connect_eth("127.0.0.1") == 0) {
		return 0;
	}
	else if (this->connect_eth("192.168.1.14") == 0) {
		return 0;
	}
	else if (this->connect_pci(0) == 0) {
		return 0;
	}
	// ����ʧ��
	return 1;
}

uint8_t ZauxRobot::load_basic_pragma(const char *basPath, uint32_t mode) {
	// ���� bas ����
	if (ZAux_BasDown(handle_, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return 1;
	}
	// �ȴ� bas ��������
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::disconnect() {
	//�ر����� 
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
	// ����� = ���λ�� + TCP ��̬
	std::vector<int> ikAxisIdx = ikPosAxisIdx_;
	ikAxisIdx.insert(ikAxisIdx.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());

	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 0 && kinematics < 2) {
		return 1;
	}
	// �ȴ�ĩ���˶�������������+������
	wait_idle(20);
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
	ZAux_Direct_SetUserVar(handle_, "kinematic", 1);
	return 0;
}

uint8_t ZauxRobot::inverse_kinematics() {
	// ����� = ���λ�� + TCP ��̬
	std::vector<int> ikAxisIdx = ikPosAxisIdx_;
	ikAxisIdx.insert(ikAxisIdx.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());

	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle_, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 1) {
		return 1;
	}
	// �ȴ��ؽ��˶��������ؽ���+������
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
	ret = ZAux_Direct_SetUserVar(handle_, "kinematic", 2);

	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	wait_idle(tcpPosAxisIdx_[0]);
	sprintf(cmdbuff, "VECTOR_MOVED(%d)=0", tcpPosAxisIdx_[0]);
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
	// �л������ģʽ, �������
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
	// �л������ģʽ, �������
	inverse_kinematics();
	//ZAux_Direct_MSphericalAbs(handle_, 6, toolAxisIdx_.data(), 800, 400, 300, 900, 200, 600, 0, 20, 20, 50);
	ZAux_Direct_MSphericalAbs(handle_, 3, tcpPosAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	return 0;
}

uint8_t ZauxRobot::save_table(size_t startIdx, size_t num, const std::string& path) {
	// ��������ȡ����
	size_t maxNum = 1000;
	// ��ȡ����
	std::vector<float> tableData(maxNum, 0);
	// ��ȡ����
	size_t times = std::floor(num / maxNum);
	// ����ļ�
	std::ofstream out(path, std::ios::trunc);

	for (size_t i = 0; i < times; ++i) {
		// ��ȡ����
		ZAux_Direct_GetTable(handle_, startIdx + maxNum * i, maxNum, tableData.data());
		// ���浽�ļ�
		for (const auto& data : tableData) {
			out << data << std::endl;
		}
	}
	
	// ʣ�����ݳ���
	size_t remain = num - maxNum * times;
	// ��ȡʣ������
	ZAux_Direct_GetTable(handle_, startIdx + maxNum * times, remain, tableData.data());
	for (size_t i = 0; i < remain; ++i) {
		out << tableData[i] << std::endl;
	}

	return 0;
}

uint8_t ZauxRobot::swing_on(float vel, const Weave& waveCfg) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterp = 100;

	// *** ��ȡ�İں����� *************************************
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	//float vel = 10.0;
	// ֹͣģʽ
	int holdType = waveCfg.Dwell_type;
	// ������ͣ��ʱ��, �ڶ�ͣ��ʱ�� (��һ����Ч)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// ���Ұ�Table����
	std::vector<float> sinTableValue(numInterp);
	// ������ֹͣ
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// �ڶ�ͣ��ʱ��
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
		// ����ʱ��(ms)
		float totalTime = 1000 / freq + 2 * swingHoldTime;
		// �ķ�֮һ�ڶ�����ռ�õ� table ����
		size_t numQuarter = numInterp * (1000 / freq) / totalTime / 4;
		// ��ͣ��ʱ��ռ�õ� table ����
		size_t numRightHold = numInterp * waveCfg.Dwell_right / totalTime;

		// ������д��͹�ֱ�
		for (size_t i = 0; i < 4 * numQuarter; ++i) {
			// ��ֵ���Ӧ��ƫ�Ʒ�ֵ
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
	}

	// ������ֹͣ
	if (holdType > 0) {
		sinTableBeg = 2000;
	}
	else {
		sinTableBeg = 2100;
		for (size_t i = 0; i < 100; ++i) {
			ZAux_Direct_MoveTable(handle_, tcpPosAxisIdx_[0], sinTableBeg + i, sinTableValue[i]);
		}
	}

	// ���ڳ���
	float dist = vel * (1/freq + 2*swingHoldTime/1000);

	// ��ȡ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, tcpAngleAxisIdx_[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", tcpPosAxisIdx_[0]);
	float vectorBuffered2 = 0.0;
	ret = ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);

	//��������
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, ����, ʸ������, ���ڳ���, ���Ұڷ�, ��ʼTable, ����Table
		3, tcpPosAxisIdx_[0], vectorBuffered2, dist, ampl, sinTableBeg, sinTableBeg + numInterp - 1,
		zDir[0], zDir[1], zDir[2]
	);
	std::cout << cmdbuff  << std::endl;

	//��������ִ�к���
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	return ret;
}

uint8_t ZauxRobot::swing_off(float displacement) {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	sprintf(cmdbuff, "VECTOR_BUFFERED2(%d)", tcpPosAxisIdx_[0]);
	float vectorBuffered2 = 0.0;
	ZAux_Direct_GetVariablef(handle_, cmdbuff, &vectorBuffered2);
	vectorBuffered2 += displacement;

	//��������
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, ����, ʸ������
		-1, tcpPosAxisIdx_[0], vectorBuffered2
	);
	std::cout << cmdbuff << std::endl;

	//��������ִ�к���
	int ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);

	//wait_idle(20);
	//float setTableValue[1] = { -1 };
	//ZAux_Direct_SetTable(handle_, 1000, 1, setTableValue);

	return ret;
}

uint8_t ZauxRobot::wlder_on(float current, float voltage) {
	// ����
	//ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 2, 5, 1);

	uint16_t dec = 0;
	uint8_t low = 0, high = 0;
	// ���õ���
	dec = std::floor(current * 65535 / 550);
	low = dec & 0x00FF;
	high = (dec >> 8) & 0x00FF;
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 5, 5, low);
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 6, 5, high);

	// ���õ�ѹ
	dec = std::floor(voltage * 65535 / 60);
	low = dec & 0x00FF;
	high = (dec >> 8) & 0x00FF;
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 7, 5, low);
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 8, 5, high);

	// ����ģʽ(ֱ��һԪ) & ��ʼ����
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 1, 5, 7);

	return 0;
}

uint8_t ZauxRobot::wlder_off() {
	// ����
	//ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 2, 5, 0);

	// ��������
	ZAux_BusCmd_NodePdoWrite(handle_, 7, 0x2002, 1, 5, 6);
	return 0;
}

uint8_t ZauxRobot::swingL(const std::vector<float>& moveCmd, const Weave& waveCfg) {
	// �����
	ZMC_HANDLE handle = handle_;
	// TCP λ����� + ���������
	std::vector<int> toolAxisIdx = tcpPosAxisIdx_;
	toolAxisIdx.insert(toolAxisIdx.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	// TCP ��̬���
	std::vector<int> poseAxisIdx = tcpAngleAxisIdx_;
	// ͹�������
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2] };
	// ����ʸ�������
	int connpathAxisIdx = connpathAxisIdx_[0];

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterp = 100;

	// �л������ģʽ, �������
	inverse_kinematics();

	// *** ��ȡ�İں����� *************************************
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	float vel = 10.0;
	// ֹͣģʽ
	int holdType = waveCfg.Dwell_type;
	// ������ͣ��ʱ��, �ڶ�ͣ��ʱ�� (��һ����Ч)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// ������ֹͣ
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// �ڶ�ͣ��ʱ��
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}

	// *** ����ں����� ***************************************
	// ��ȡ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, poseAxisIdx[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// TCP ��λ��
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// ֱ�߹켣�ĳ���
	Eigen::Vector3f displDir = displ.normalized();
	// ƫ�Ʒ���
	Eigen::Vector3f offDir = zDir.cross(displ).normalized();
	// ���η���
	offDir *= waveCfg.Phase > 0 ? -1 : 1;
	// ��λ��(mm)
	float dist = displ.norm();
	// ��ʱ��(s)
	float totalTime = dist / vel;
	// ������
	size_t numPeriod = std::ceil(1000 * totalTime / (1000 / freq + 2 * swingHoldTime));

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle, connpathAxisIdx, (char*)"TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], connpathAxisIdx);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// �ȴ��ڶ���־λ��λ
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], (char*)"TABLE", swingFlagIdx, 1, 0);
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
		// �󶨸���͹�ֱ�
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg + numInterp - 1, ampl * units * offDir[i], dist / numPeriod, connpathAxisIdx, 4, 0);
	}

	// *** �����˶����� *************************************
	// ���������ٶ�
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// ���ø����᲻�����ٶȼ���
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	}

	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", connpathAxisIdx, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", camAxisIdx[i], 0);
	}

	// ƫ�Ʒ������ȷ���д�� Table
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], 1004 + i, offDir[i]);
		ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], 1007 + i, zDir[i]);
	}

	// �ķ�֮һ�ڶ�����ռ�õ� table ����
	size_t numQuarter = numInterp * (1000 / freq) / (1000 / freq + 2 * swingHoldTime) / 4;
	// ��ͣ��ʱ��ռ�õ� table ����
	size_t numRightHold = (numInterp - 4 * numQuarter) / 2;
	// ������д��͹�ֱ�
	for (size_t i = 0; i < 4 * numQuarter; ++i) {
		// ��ֵ���Ӧ��ƫ�Ʒ�ֵ
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

	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	std::vector<float> cmd = { moveCmd[0] / numPeriod / 4, moveCmd[1] / numPeriod / 4, moveCmd[2] / numPeriod / 4, moveCmd[3] / numPeriod / 4 };

	// 1/4 ����
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	if (robotHoldTime > 0)
		ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], robotHoldTime);
	
	for (size_t i = 0; i < cmd.size(); ++i) cmd[i] *= 2;
	for (size_t i = 0; i < numPeriod - 1; ++i) {
		// ��ʼ�ɼ�������ѹ
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

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], connpathAxisIdx, 0);

	// ������ָ�����岹�ٶ�
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}

	return 0;
}
uint8_t ZauxRobot::swingL_(const std::vector<float>& moveCmd, const Weave& waveCfg) {
	// �����
	ZMC_HANDLE handle = handle_;
	// TCP λ����� + ���������
	std::vector<int> toolAxisIdx = tcpPosAxisIdx_;
	toolAxisIdx.insert(toolAxisIdx.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	// TCP ��̬���
	std::vector<int> poseAxisIdx = tcpAngleAxisIdx_;
	// ͹�������
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_ [1], camAxisIdx_[2] };
	// ����ʸ�������
	int connpathAxisIdx = connpathAxisIdx_[0];

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterp = 100;

	// �л������ģʽ, �������
	inverse_kinematics();

	// *** ��ȡ�İں����� *************************************
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	float vel = 10.0;
	// ͣ��ʱ��
	float holdTime = (waveCfg.Dwell_left+ waveCfg.Dwell_right) / 2;

	// *** ����ں����� ***************************************
	// ��ȡ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, poseAxisIdx[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// TCP ��λ��
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// ֱ�߹켣�ĳ���
	Eigen::Vector3f displDir = displ.normalized();
	// ƫ�Ʒ���
	Eigen::Vector3f offDir = zDir.cross(displ).normalized();
	// ���η���
	offDir *= waveCfg.Phase > 0 ? -1 : 1;
	// ��λ��
	float dist = displ.norm();
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq * units) / units;

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle, connpathAxisIdx, (char*)"TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], connpathAxisIdx);

	// *** ����͹�������� *************************************
	// INT(SQR(dx*dx + dy * dy + dz * dz) * UNITS(20)) / UNITS(20)
	// �ȴ��ڶ���־λ��λ
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], (char*)"TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// �󶨸���͹�ֱ�
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg + numInterp - 1, ampl * units * offDir[i], dist / numPeriod, connpathAxisIdx, 4, 0);
	}

	// *** �����˶����� *************************************
	// ���������ٶ�
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// ���ø����᲻�����ٶȼ���
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	}

	// �ƶ������Ҳ�
	std::vector<float> cmd = moveCmd;
	for (size_t i = 0; i < 3; ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 4 + ampl * offDir[i];
	}
	cmd[3] = moveCmd[3] / numPeriod / 4;
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", connpathAxisIdx, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], (char*)"DPOS", camAxisIdx[i], 0);
	}

	// ƫ�Ʒ������ȷ���д�� Table
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], 1004 + i, offDir[i]);
		ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], 1007 + i, zDir[i]);
	}

	// ������д��͹�ֱ�
	for (size_t i = 0; i < numInterp; ++i) {
		// ��ֵ���Ӧ��ƫ�Ʒ�ֵ
		float tmp = std::sin(M_PI/2 + 2 * M_PI * i / (numInterp - 1));
		// ���ڿ�
		// tmp *= i > numInterp / 2 ? (waveCfg.LeftWidth / waveCfg.RightWidth) : 1;
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + i, tmp);
	}

	
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	//std::vector<float> cmd(moveCmd.begin(), moveCmd.begin() + toolAxisIdx.size());
	// ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());
	for (size_t i = 0; i < moveCmd.size(); ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 2;
	}
	for (size_t i = 0; i < numPeriod - 1; ++i) {
		// ��ʼ�ɼ�������ѹ
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

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], connpathAxisIdx, 0);

	// �˶����ڶ�����λ��
	for (size_t i = 0; i < 3; ++i) {
		cmd[i] = moveCmd[i] / numPeriod / 4 + ampl * offDir[i];
	}
	cmd[3] = moveCmd[3] / numPeriod / 4;
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// ������ָ�����岹�ٶ�
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}


	return 0;
}
uint8_t ZauxRobot::swingLAbs(const std::vector<float>& moveCmd, const Weave& waveCfg) {
	float curEndMove[4] = { 0,0,0,0 };
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 20 + i, &curEndMove[i]);
	}
	swingL({ moveCmd[0] - curEndMove[0], moveCmd[1] - curEndMove[1], moveCmd[2] - curEndMove[2], moveCmd[3] - curEndMove[3] }, waveCfg);
	return 0;
}

uint8_t ZauxRobot::swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig, const Weave& waveCfg) {
#pragma region swing_config
	// �����
	ZMC_HANDLE handle = handle_;
	// TCP λ����� + ���������
	std::vector<int> toolAxisIdx = tcpPosAxisIdx_;
	toolAxisIdx.insert(toolAxisIdx.end(), appAxisIdx_.begin(), appAxisIdx_.end());
	// TCP ��̬���
	std::vector<int> poseAxisIdx = tcpAngleAxisIdx_;
	// ͹�������
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2] };
	// ����ʸ�������
	int connpathAxisIdx = connpathAxisIdx_[0];

	
	// �л������ģʽ, �������
	inverse_kinematics();

	// *** ��ȡ�İں����� *************************************
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	float vel = 10.0;
	// ֹͣģʽ
	int holdType = waveCfg.Dwell_type;
	// ������ͣ��ʱ��, �ڶ�ͣ��ʱ�� (��һ����Ч)
	float robotHoldTime = 0.0, swingHoldTime = 0.0;
	// ������ֹͣ
	if (holdType > 0) {
		robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}
	// �ڶ�ͣ��ʱ��
	else {
		swingHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;
	}

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterpInPeriod = 20;
	
	Eigen::Vector3f begAngle, detAngle{ 0,0,0 };

	// *** ����ں����� ***************************************
	// ��ȡ��ǰ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, 10 + i, &zEuler[i]);
	}
	begAngle = zEuler;
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	float units = 0.0;
	ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle, toolAxisIdx[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// ����Բ������
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// �뾶����
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// ����뾶��������
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// Բ���˶�ƽ��ķ��߷���
	Eigen::Vector3f normal = op1.cross(op3);
	// Բ�ĽǽǶ�
	float theta = std::acos(op1.dot(op3));
	// ����Բ�ĽǺ�������
	// 2,3 �� 1 ������
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

	// ��ʼ���߷���
	//Eigen::Vector3f tanDirBase = (normal.cross(op1)).normalized();
	// ��ʼ�ڶ�����
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	// ��ʼ��ȷ���
	Eigen::Vector3f depthDirBase = (normal.cross(op1).cross(zDir)).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// �ܾ���
	float dist = std::floor((begPnt - center).norm() * theta * units) / units;
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq);
	// ����·���ϵĲ�ֵ��
	size_t numInterp = numPeriod * numInterpInPeriod;

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle, 15, "TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], 15);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	// �ȴ��ڶ���־λ��λ
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// �󶨸���͹�ֱ�
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg + numInterp * i, sinTableBeg + numInterp * (i + 1) - 1, ampl * 1000, dist, 15, 4, 0);
	}

	// *** �����˶����� *************************************
	// ���������ٶ�
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);
#pragma endregion swing_config

	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", camAxisIdx[i], 0);
	}

	int iBuffLen = 0;
	while (iBuffLen < 1000) {
		int tmpBuffLen = 9999;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// ��С�Ļ�����
		for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
			int tmp = 9999;
			ZAux_Direct_GetRemain_LineBuffer(handle, toolAxisIdx[0], &tmp);
			tmpBuffLen = std::min(tmpBuffLen, tmp);
		}
		iBuffLen = tmpBuffLen;
	}

	// ������д��͹�ֱ�
	// ��ֵ���Ӧ��Բ�Ľ�
	float dq = theta / (numInterp - 1), curQ = 0.0;
	Eigen::Vector3f offDir(0, 0, 0), depthDir(0, 0, 0);
	// ��¼͹�ֱ����ݵ�����
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			// ƫ�Ʒ���
			// Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			offDir = (Eigen::AngleAxisf(curQ, normal) * offDirBase).normalized();
			// ���߷���
			//Eigen::Vector3f tanDir = normal.cross(offDir);
			// ƫ����λ��
			float off = sin(float(j) / (numInterpInPeriod) * 2 * M_PI);

			// ��¼͹�ֱ�
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// ���µ�ǰ�Ƕ�
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 3 - 1, 0);
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
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

	// 1/4 ����
	curQ += dq/4;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle/4;
	curQ += dq/4;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle/4;
	// ���ڿ�ʼ��־
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);
	trans = Eigen::AngleAxisf(curQ, normal).matrix();
	// ƫ�Ʒ���
	offDir = (trans * offDirBase).normalized();
	// ��ȷ���
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

		// ���ڿ�ʼ��־
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], 1001, 1);

		//ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
		//std::cout << "ret = " << ret << std::endl;
		//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);
		trans = Eigen::AngleAxisf(curQ, normal).matrix();
		// ƫ�Ʒ���
		offDir = (trans * offDirBase).normalized();
		// ��ȷ���
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

	// 1/2 ����
	curQ += dq / 2;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 2;
	curQ += dq / 2;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle;
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);

	// 1/4 ����
	curQ += dq / 4;
	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 4;
	curQ += dq / 4;
	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	curAngle += detAngle / 4;
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisPoseIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//ZAux_Direct_MoveDelay(handle, toolAxisIdx[0], 100);

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	return 0;
}

uint8_t ZauxRobot::zswingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig, const Weave& waveCfg) {
	// TCP ָ��λ���� = TCP λ�� + ������
	std::vector<int> toolAxisIdx = tcpPosAxisIdx_;
	toolAxisIdx.insert(toolAxisIdx.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	float vel = 10.0;
	// ֹͣģʽ
	int holdType = waveCfg.Dwell_type;
	// ������ͣ��ʱ��, �ڶ�ͣ��ʱ�� (��һ����Ч)
	float robotHoldTime = holdType > 0 ? (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2 : 0;

	//// �л������ģʽ, �������
	//inverse_kinematics();

	// *** ����ں����� ***************************************
	// ��ȡ��ǰ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 10 + i, &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	float units = 0.0;
	ZAux_Direct_GetUnits(handle_, toolAxisIdx[0], &units);

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx[i], &begPnt[i]);
		begPnt[i] = std::floor(begPnt[i] * units) / units;
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// ����Բ������
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// �뾶����
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// ����뾶��������
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// Բ���˶�ƽ��ķ��߷���
	Eigen::Vector3f normal = op1.cross(op3);
	// Բ�ĽǽǶ�
	float theta = std::acos(op1.dot(op3));
	// ����Բ�ĽǺ�������
	// 2,3 �� 1 ������
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

	//// ��ʼ�ڶ�����
	//Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	//std::cout << "zDir       = " << zDir.transpose() << std::endl;
	//std::cout << "normal     = " << normal.transpose() << std::endl;
	//std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	//// �ܾ���
	//int distUnits = std::floor((begPnt - center).norm() * theta * units);
	//float dist = distUnits / units;
	//// ������
	//size_t numPeriod = std::ceil(dist / vel * freq);
	//
	//// ������д��͹�ֱ�
	//float dq = theta / numPeriod / 2, curQ = 0.0;
	//op1 = begPnt - center;

	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	size_t numPeriod = 10;
	float curQ = 0.0, dq = theta / numPeriod / 2;
	// ��¼͹�ֱ����ݵ�����
	for (size_t i = 0; i < numPeriod; ++i) {
		// ��ǰ���ڵ��˶�
		Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
		curQ += dq;
		curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		curQ += dq;
		curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
		int ret = ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		//ZAux_Direct_MoveDelay(handle_, toolAxisIdx_[0], 1000);
		std::cout << ret << ", pos = " << curEnd.transpose() << std::endl;
	}

	return 0;
}

uint8_t ZauxRobot::swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig, const Weave& waveCfg) {
	// TCP ָ��λ���� = TCP λ�� + ������
	std::vector<int> toolAxisIdx = tcpPosAxisIdx_;
	toolAxisIdx.insert(toolAxisIdx.end(), appAxisIdx_.begin(), appAxisIdx_.end());

#pragma region swing_config
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = (waveCfg.LeftWidth + waveCfg.RightWidth) / 2;
	// �����ٶ�
	float vel = 5.0;

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterpInPeriod = 20;

	// �л������ģʽ, �������
	inverse_kinematics();

	Eigen::Vector3f begAngle, detAngle{ 0,0,0 };

	// *** ����ں����� ***************************************
	// ��ȡ��ǰ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, 10 + i, &zEuler[i]);
	}
	begAngle = zEuler;
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	float units = 0.0;
	ZAux_Direct_GetUnits(handle_, toolAxisIdx[0], &units);

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// ����Բ������
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// �뾶����
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// ����뾶��������
	if (op1.norm() > 1e3) {
		return {};
	}
	float q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
	Eigen::Vector3f n12 = op1.cross(op2), n13 = op1.cross(op3);
	// Բ���˶�ƽ��ķ��߷���
	Eigen::Vector3f normal = op1.cross(op3);
	// Բ�ĽǽǶ�
	float theta = std::acos(op1.dot(op3));
	// ����Բ�ĽǺ�������
	// 2,3 �� 1 ������
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

	// ��ʼ�ڶ�����
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// �ܾ���
	float dist = std::floor((begPnt - center).norm() * theta * units) / units;
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq);
	// ����·���ϵĲ�ֵ��
	size_t numInterp = numPeriod * numInterpInPeriod;

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle_, 15, "TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle_, 1, toolAxisIdx[0], 15);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle_, toolAxisIdx[i], camAxisIdx_[i]);
	}
	// �ȴ��ڶ���־λ��λ
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveWait(handle_, camAxisIdx_[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// �󶨸���͹�ֱ�
		ZAux_Direct_Cambox(handle_, camAxisIdx_[i], sinTableBeg + numInterp * i, sinTableBeg + numInterp * (i + 1) - 1, ampl * 1000, dist, 15, 4, 0);
	}

	// *** �����˶����� *************************************
	// ���������ٶ�
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle_, toolAxisIdx[i], vel);
	}
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], swingFlagIdx, 1);
#pragma endregion swing_config

	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle_, toolAxisIdx[0], "DPOS", camAxisIdx_[i], 0);
	}

	// ������д��͹�ֱ�
	// ��ֵ���Ӧ��Բ�Ľ�
	float dq = theta / (numInterp - 1), curQ = 0.0;
	// ��¼͹�ֱ����ݵ�����
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			// ƫ�Ʒ���
			// Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			Eigen::Vector3f offDir = (Eigen::AngleAxisf(curQ, normal) * offDirBase).normalized();
			// ���߷���
			//Eigen::Vector3f tanDir = normal.cross(offDir);
			// ƫ����λ��
			float off = sin(float(j) / (numInterpInPeriod - 1) * 2 * M_PI);

			// ��¼͹�ֱ�
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// ���µ�ǰ�Ƕ�
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], sinTableBeg + numInterp * 3 - 1, 0);
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	dq = theta / numPeriod / 2;
	curQ = 0.0;
	op1 = begPnt - center;
	if (endConfig.size() > 5) {
		for (size_t i = 0; i < 3; ++i) {
			detAngle[i] = (endConfig[3 + i] - begAngle[i]) / numPeriod / 2;
		}
	}
	//int toolAxisIdx[6] = { 20,21,22,10,11,12 };
	//Eigen::Vector3f curAngle = begAngle;
	//for (size_t i = 0; i < numPeriod; ++i) {
	//	Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
	//	curQ += dq;
	//	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	curAngle = detAngle;
	//	curQ += dq;
	//	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	curAngle = detAngle;
	//	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
	//	ZAux_Direct_MSphericalAbs(handle_, 6, toolAxisIdx, curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, curAngle[0], curAngle[1], curAngle[2]);
	//	//std::cout << "ret = " << ret << std::endl;
	//	ZAux_Direct_MoveDelay(handle_, toolAxisIdx_[0], 100);
	//}

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveCancel(handle_, toolAxisIdx[0], camAxisIdx_[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle_, toolAxisIdx[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx[0], "DPOS", 15, 0);

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx[0], swingFlagIdx, -1);

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

	//	// �����յ㴦��ŷ������ռ�λ��
	//	Eigen::Matrix<float, 6, 1> begPnt;
	//	Eigen::Vector3f zEuler(0, 0, 0), zPos;
	//	for (size_t i = 0; i < 3; ++i) {
	//		ZAux_Direct_GetEndMoveBuffer(handle_, 9 + i, &begPnt[i+3]);
	//		ZAux_Direct_GetEndMoveBuffer(handle_, 6 + i, &begPnt[i]);
	//	}
	//	// ��ǰĿ���
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

uint8_t ZauxRobot::execute_discrete_trajectory_abs(DiscreteTrajectory<float>& discreteTrajectory) {
	std::vector<int> axis = tcpPosAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	// �켣��ά����������ά�ȵĽ�Сֵ
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// ֻ����TCPλ���ٶ�
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, axis[i], i < 3);
	}

	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	size_t i = 0;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++), relEndMove(axis.size(), 0);
		for (size_t i = 0; i < num; ++i) {
			// �յ㵽��������˶�
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// ŷ����ת��������˶�
		Eigen::Vector3f begEuler(endBuffer[3], endBuffer[4], endBuffer[5]), endEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}

		// �켣�ٶ�
		float vel = (*infoIte)[7];
		// �����ٶ�
		for (size_t i = 0; i < axis.size(); ++i) {
			ZAux_Direct_MovePara(handle_, axis[0], (char*)"SPEED", axis[i], vel);
		}

		// Բ���˶�
		if ((*infoIte)[3] > 0) {
			// Բ���м��
			std::vector<float> midPoint = *midPointIte;
			std::vector<float> relMidMove(axis.size());
			for (size_t i = 0; i < endBuffer.size(); ++i) {
				relMidMove[i] = midPoint[i] - endBuffer[i];
			}
			endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
			relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				relMidMove[3 + i] = relEuler[i];
			}

			// Բ��ģʽ
			int imode = 0;
			// ��������
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
			//��������ִ�к���
			return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
		}
		// ֱ���˶�
		else {
			// �����˶�
			//ZAux_Direct_MoveAbs(handle, axis.size(), axis.data(), (*nodePointIte).data());
			// ����˶�
			ZAux_Direct_Move(handle_, axis.size(), axis.data(), relEndMove.data());
		}

		midPointIte++;
		infoIte++;
	}

	// �ָ��ٶȲ岹
	for (size_t i = 0; i < axis.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, axis[i], 1);
	}

	return 0;
}

uint8_t ZauxRobot::execute_discrete_trajectory(DiscreteTrajectory<float>& discreteTrajectory) {
	//// ���������
	//ZMC_HANDLE handle = handle_;
	//// TCP λ�������
	//std::vector<int> tcpPosAxis = { 20, 21, 22 };
	//// TCP ŷ���������
	//std::vector<int> tcpAngleAxis = { 10, 11, 12 };
	//// ���������
	//std::vector<int> appendAxis = { 6 };

	//std::vector<int> axis = tcpPosAxis;
	//axis.insert(axis.end(), tcpAngleAxis.begin(), tcpAngleAxis.end());
	//axis.insert(axis.end(), appendAxis.begin(), appendAxis.end());

	//std::list<Eigen::Matrix<float, 7, 1>>::iterator nodePointIte = discreteTrajectory.nodePoint.begin();
	//std::list<Eigen::Matrix<float, 7, 1>>::iterator midPointIte = discreteTrajectory.midPoint.begin();
	//std::list<Eigen::Matrix<float, 7, 1>>::iterator infoIte = discreteTrajectory.trajInfo.begin();

	//Eigen::Vector3f begEuler = (*nodePointIte).segment<3>(3), endEuler = (*nodePointIte).segment<3>(3);
	//while (infoIte != discreteTrajectory.trajInfo.end()) {
	//	Eigen::Matrix<float, 7, 1> endBuffer = *(nodePointIte++);

	//	std::vector<float> relEndMove(axis.size());
	//	for (size_t i = 0; i < endBuffer.size(); ++i) {
	//		// �յ㵽��������˶�
	//		relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
	//	}

	//	// ŷ����ת��������˶�
	//	begEuler = endBuffer.segment<3>(3);
	//	endEuler = (*nodePointIte).segment<3>(3);
	//	Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
	//	for (size_t i = 0; i < 3; ++i) {
	//		relEndMove[3 + i] = relEuler[i];
	//	}
	//	begEuler = endEuler;

	//	// Բ���˶�
	//	if ((*infoIte)(3) > 0) {
	//		// Բ���м��
	//		Eigen::Matrix<float, 7, 1> midPoint = *midPointIte;
	//		std::vector<float> relMidMove(axis.size());
	//		for (size_t i = 0; i < endBuffer.size(); ++i) {
	//			relMidMove[i] = midPoint[i] - endBuffer[i];
	//		}
	//		endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
	//		relEuler = get_zyx_euler_distance(begEuler, endEuler);
	//		for (size_t i = 0; i < 3; ++i) {
	//			relMidMove[3 + i] = relEuler[i];
	//		}

	//		// Բ��ģʽ
	//		int imode = 0;
	//		// ��������
	//		char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	//		strcpy(cmdbuff, "BASE(");
	//		for (size_t i = 0; i < axis.size() - 1; i++) {
	//			sprintf(tempbuff, "%d,", axis[i]);
	//			strcat(cmdbuff, tempbuff);
	//		}
	//		sprintf(tempbuff, "%d)", axis.back());
	//		strcat(cmdbuff, tempbuff);
	//		strcat(cmdbuff, "\n");

	//		sprintf(tempbuff, "MSPHERICAL(%f,%f,%f,%f,%f,%f,%d", relEndMove[0], relEndMove[1], relEndMove[2], relMidMove[0], relMidMove[1], relMidMove[2], imode);
	//		strcat(cmdbuff, tempbuff);
	//		for (size_t i = 3; i < 7; ++i) {
	//			sprintf(tempbuff, ",%f", relEndMove[i]);
	//			strcat(cmdbuff, tempbuff);
	//		}
	//		strcat(cmdbuff, ")");
	//		std::cout << cmdbuff << std::endl;
	//		//��������ִ�к���
	//		return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//	}
	//	// ֱ���˶�
	//	else {
	//		// ����˶�
	//		ZAux_Direct_Move(handle, axis.size(), axis.data(), relEndMove.data());
	//	}

	//	midPointIte++;
	//	infoIte++;
	//}

	return 0;
}

uint8_t ZauxRobot::swing_tri() {
	//// ���������
	//ZMC_HANDLE handle = handle_;
	//// TCP λ�������
	//std::vector<int> tcpPosAxis = { 7, 8, 9 };
	//// TCP ŷ���������
	//std::vector<int> tcpAngleAxis = { 10, 11, 12 };
	//// ���������
	//std::vector<int> appendAxis = { 6 };

	std::vector<int> axis = tcpPosAxisIdx_;
	//axis.insert(axis.end(), tcpAngleAxis.begin(), tcpAngleAxis.end());
	//axis.insert(axis.end(), appendAxis.begin(), appendAxis.end());

	// ֱ���˶�ָ��
	std::vector<float> moveCmd = { 100,100,0 };
	// ���ǰ���״
	std::vector<float> triShape = { 10, 30, 20 };

	// ��ȡ��������λ�ô���ŷ����(deg)
	Eigen::Vector3f zEuler(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, tcpAngleAxisIdx_[i], &zEuler[i]);
	}
	zEuler *= M_PI / 180;
	// ��������λ�õĹ��� Z ����
	Eigen::Vector3f zDir(0, 0, 0);
	zDir[0] = sin(zEuler[2]) * sin(zEuler[0]) + cos(zEuler[2]) * cos(zEuler[0]) * sin(zEuler[1]);
	zDir[1] = cos(zEuler[0]) * sin(zEuler[2]) * sin(zEuler[1]) - cos(zEuler[2]) * sin(zEuler[0]);
	zDir[2] = cos(zEuler[0]) * cos(zEuler[1]);
	// ��ȡ���嵱��
	//float units = 0.0;
	//ZAux_Direct_GetUnits(handle, toolAxisIdx[0], &units);

	// ֱ�߷���
	Eigen::Vector3f begTan(moveCmd[0], moveCmd[1], moveCmd[2]);
	// �˶�����
	float dist = begTan.norm();
	begTan.normalize();

	// ������z����
	Eigen::Vector3f begUprightDir = (begTan.cross(zDir).cross(begTan)).normalized();
	// ���Ұڶ�����
	Eigen::Vector3f rightDir = Eigen::AngleAxisf(-M_PI / 4, begTan) * begUprightDir;
	Eigen::Vector3f leftDir = Eigen::AngleAxisf(M_PI / 4, begTan) * begUprightDir;

	size_t numPeriod = std::floor((dist - triShape[1]) / (triShape[1] - triShape[2]));

	Eigen::Vector3f cmd(0, 0, 0);

	for (size_t i = 0; i < numPeriod; ++i) {
		// ��һ��: ��
		cmd = -triShape[0] * rightDir;
		ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());
		// �ڶ���: ����
		cmd = triShape[0] * rightDir + triShape[1] * begTan;
		ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());
		// ������: ����
		cmd = -triShape[0] * leftDir - triShape[2] * begTan;
		ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());
		// ���Ķ�: ��
		cmd = triShape[0] * leftDir;
		ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());
	}
	cmd = -triShape[0] * rightDir;
	ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());
	cmd = triShape[0] * rightDir + (dist - numPeriod * (triShape[1] - triShape[2])) * begTan;
	ZAux_Direct_Move(handle_, axis.size(), axis.data(), cmd.data());

	return 0;
}

uint8_t ZauxRobot::swing_trajectory(DiscreteTrajectory<float>& discreteTrajectory, const Weave& waveCfg) {
	std::vector<int> axis = tcpPosAxisIdx_;
	axis.insert(axis.end(), tcpAngleAxisIdx_.begin(), tcpAngleAxisIdx_.end());
	axis.insert(axis.end(), appAxisIdx_.begin(), appAxisIdx_.end());

	
	// �켣��ά����������ά�ȵĽ�Сֵ
	size_t num = (std::min)(discreteTrajectory.nodePoint.begin()->size(), axis.size());

	// ֻ����TCPλ���ٶ�
	for (size_t i = 0; i < axis.size(); ++i) {
		//ZAux_Direct_SetInterpFactor(handle_, axis[i], i < 3);
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], i < 3);
	}
	// ͹��������
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"DPOS", camAxisIdx_[i], 0);
	}
	// �ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, 1);
	//ZAux_Direct_MovePara(handle_, axis[0], (char*)"VECTOR_MOVED", axis[0], 0);

	// �����켣
	auto nodePointIte = discreteTrajectory.nodePoint.begin();
	auto midPointIte = discreteTrajectory.midPoint.begin();
	auto infoIte = discreteTrajectory.trajInfo.begin();

	Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
	while (infoIte != discreteTrajectory.trajInfo.end()) {
		std::vector<float> endBuffer = *(nodePointIte++);

		std::vector<float> relEndMove(axis.size(), 0);
		// �յ㵽��������˶�
		for (size_t i = 0; i < num; ++i) {
			relEndMove[i] = (*nodePointIte)[i] - endBuffer[i];
		}

		// ŷ����ת��������˶�
		begEuler = Eigen::Vector3f(endBuffer[3], endBuffer[4], endBuffer[5]);
		endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
		Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
		for (size_t i = 0; i < 3; ++i) {
			relEndMove[3 + i] = relEuler[i];
		}
		begEuler = endEuler;

		// �켣�ٶ�
		float vel = (*infoIte)[7];
		// �����ٶ�
		ZAux_Direct_MovePara(handle_, axis[0], (char*)"SPEED", axis[0], vel);

		// ���ðڶ�����
		swing_on(vel, waveCfg);

		// �������˶�����
		int numPeriod = std::round(std::fabs((*infoIte)[3]) / (vel / waveCfg.Freq));
		if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0) {
			float totalDist = std::sqrt(relEndMove[0] * relEndMove[0] + relEndMove[1] * relEndMove[1] + relEndMove[2] * relEndMove[2]);
			float halfDist = std::floor(totalDist / numPeriod / 2 * axisUnits) / axisUnits;
			float quartDist = std::floor(totalDist / numPeriod / 4 * axisUnits) / axisUnits;
			float zmotionDist = halfDist * ((numPeriod - 1) * 2 + 1) + quartDist * 2;
			// �ڶ�����
			swing_off(zmotionDist);
		}
		else {
			swing_off(std::fabs((*infoIte)[3]));
		}
		

		// Բ���˶�
		if ((*infoIte)[3] > 0) {
			// Բ���м��
			std::vector<float> midPoint = *midPointIte;
			std::vector<float> relMidMove(axis.size());
			for (size_t i = 0; i < endBuffer.size(); ++i) {
				relMidMove[i] = midPoint[i] - endBuffer[i];
			}
			// �м��Ƕ����ֵ
			endEuler = Eigen::Vector3f(midPoint[3], midPoint[4], midPoint[5]);
			relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				relMidMove[3 + i] = relEuler[i];
			}

			// Բ��ģʽ
			int imode = 0;
			// ��������
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
			//��������ִ�к���
			return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
		}
		// ֱ���˶�
		else {
			// ������ֹͣ
			if (waveCfg.Dwell_type > 0 && (waveCfg.Dwell_left + waveCfg.Dwell_right) > 0) {
				float robotHoldTime = (waveCfg.Dwell_left + waveCfg.Dwell_right) / 2;

				if (numPeriod > 0) {
					std::vector<float> detRelEndMove(relEndMove.size(), 0);
					for (size_t j = 0; j < relEndMove.size(); ++j)
						detRelEndMove[j] = relEndMove[j] / (numPeriod * 4);

					int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					ZAux_Direct_MoveDelay(handle_, axis[0], waveCfg.Dwell_right);

					for (size_t j = 0; j < relEndMove.size(); ++j)
						detRelEndMove[j] *= 2;
					for (int i = 0; i < numPeriod - 1; ++i) {
						ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
						ZAux_Direct_MoveDelay(handle_, tcpPosAxisIdx_[0], waveCfg.Dwell_left);
						ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
						ZAux_Direct_MoveDelay(handle_, tcpPosAxisIdx_[0], waveCfg.Dwell_right);
					}

					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
					ZAux_Direct_MoveDelay(handle_, tcpPosAxisIdx_[0], waveCfg.Dwell_left);
					for (size_t j = 0; j < relEndMove.size(); ++j)
						detRelEndMove[j] /= 2;
					ZAux_Direct_Move(handle_, axis.size(), axis.data(), detRelEndMove.data());
				}
			}
			// �ڶ�ֹͣ
			else {
				// ����˶�
				int ret = ZAux_Direct_Move(handle_, axis.size(), axis.data(), relEndMove.data());
			}
		}

		midPointIte++;
		infoIte++;
	}

	// �ڶ�ֹͣ
	ZAux_Direct_MoveTable(handle_, axis[0], 1000, -1);
	// �ָ��ٶȲ岹
	//for (size_t i = 0; i < axis.size(); ++i) {
	//	//ZAux_Direct_SetInterpFactor(handle_, axis[i], 1);
	//	ZAux_Direct_MovePara(handle_, axis[0], (char*)"INTERP_FACTOR", axis[i], 1);
	//}
	return 0;
}

uint8_t ZauxRobot::arc_tracking_config(const Track& trackCfg) {
	// ����ʹ��
	//float enableFlag[2] = { trackCfg.Lr_enable, trackCfg.Ud_enable };
	//ZAux_Direct_SetTable(handle_, 1004, 2, enableFlag);

	std::vector<float> config(14, 0);
	// ���Ҹ��ٲ���
	config[0] = trackCfg.Lr_enable;
	config[1] = trackCfg.Lr_gain;
	config[2] = trackCfg.Lr_minCompensation;
	config[3] = trackCfg.Lr_maxCompensation;
	config[4] = trackCfg.Lr_offset;
	config[5] = trackCfg.Lr_MaxCorrectAngle;
	// ���¸��ٲ���
	config[6] = trackCfg.Ud_enable;
	config[7] = trackCfg.Ud_gain;
	config[8] = trackCfg.Ud_minCompensation;
	config[9] = trackCfg.Ud_maxCompensation;
	config[10] = trackCfg.Ud_offset;
	config[11] = trackCfg.Ud_MaxCorrectAngle;
	// ��������
	config[12] = trackCfg.SegCorrectCycles;
	config[13] = trackCfg.Ud_refSampleCount;

	ZAux_Direct_SetTable(handle_, 1010, config.size(), config.data());
	return 0;
}

uint8_t ZauxRobot::emergency_stop() {
	float zero = 0.0, negtive = -1.0;
	// �ڶ���־λ��λ
	ZAux_Direct_SetTable(handle_, 1000, 1, &negtive);

	return 0;
}
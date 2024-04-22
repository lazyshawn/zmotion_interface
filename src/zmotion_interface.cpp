
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
	// �ȴ��ؽ��˶��������ؽ���+������
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

uint8_t ZauxRobot::moveL() {
	// �л������ģʽ, �������
	inverse_kinematics();

	int axisList[] = { 20, 21, 22, 6 };
	float dist[] = { 0, -200, 0, 0 };
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
	ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	return 0;
}

uint8_t ZauxRobot::swing_on() {
	char  cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];
	int ret = 0;

	// �л������ģʽ, �������
	inverse_kinematics();

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterp = 100;
	// ���ڳ���
	float dist = 19.9385719;
	// �ڷ�
	float width = 5.0;
	// ��ǹ����
	Eigen::Vector3f zDir(0, 0, 1);

	// д�� Table
	std::vector<float> camTable(numInterp);
	for (size_t i = 0; i < numInterp; ++i) {
		camTable[i] = sin(2 * M_PI * (float)i / (numInterp - 1));
	}
	ZAux_Direct_SetTable(handle_, sinTableBeg, numInterp, camTable.data());

	float vectorBuffered2 = 0.0;
	ret = ZAux_Direct_GetVariablef(handle_, "VECTOR_BUFFERED2(20)", &vectorBuffered2);

	//��������
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nCONN_SWING(%d,%d,%f,%f,%f,%d,%d,%f,%f,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, ����, ʸ������, ���ڳ���, ���Ұڷ�, ��ʼTable, ����Table
		3, toolAxisIdx_[0], vectorBuffered2, dist, width, sinTableBeg, sinTableBeg + numInterp - 1,
		// ��ǹ����
		zDir[0], zDir[1], zDir[2]
	);
	std::cout << cmdbuff  << std::endl;

	//��������ִ�к���
	ret = ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
	return ret;
}

uint8_t ZauxRobot::swing_off() {
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];

	float vectorBuffered2 = 0.0;
	ZAux_Direct_GetVariablef(handle_, "VECTOR_BUFFERED2(20)", &vectorBuffered2);

	//��������
	sprintf(cmdbuff, "BASE(%d,%d,%d)\nconn_swing(%d,%d,%f)",
		camAxisIdx_[0], camAxisIdx_[1], camAxisIdx_[2],
		// mode, ����, ʸ������
		-1, toolAxisIdx_[0], vectorBuffered2
	);
	std::cout << cmdbuff << std::endl;

	//��������ִ�к���
	return ZAux_DirectCommand(handle_, cmdbuff, cmdbuffAck, 2048);
}

uint8_t ZauxRobot::swingL(const std::vector<float>& moveCmd, Eigen::Vector3f upper) {
	// �ڶ�Ƶ��
	float freq = 4.0;
	// �ڶ����
	float ampl = 2.0;
	// �����ٶ�
	float vel = 10.0;

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterp = 100;

	// �л������ģʽ, �������
	inverse_kinematics();

	// *** ����ں����� ***************************************
	// �켣ƽ����Ϸ��߷���
	upper.normalize();
	// TCP ��λ��
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// ֱ�߹켣�ĳ���
	Eigen::Vector3f displDir = displ.normalized();
	// ƫ�Ʒ���
	Eigen::Vector3f offDir = displ.normalized().cross(upper);
	// ��λ��
	float dist = displ.norm();
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq);

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle_, 15, "TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle_, 1, toolAxisIdx_[0], 15);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle_, toolAxisIdx_[i], camAxisIdx_[i]);
	}
	// �ȴ��ڶ���־λ��λ
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveWait(handle_, camAxisIdx_[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// �󶨸���͹�ֱ�
		ZAux_Direct_Cambox(handle_, camAxisIdx_[i], sinTableBeg, sinTableBeg+numInterp-1, ampl * 1000 * offDir[i], dist / numPeriod, 15, 4, 0);
	}

	// *** �����˶����� *************************************
	// ���������ٶ�
	for (size_t i = 0; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetSpeed(handle_, toolAxisIdx_[i], vel);
	}
	// ���ø����᲻�����ٶȼ���
	for (size_t i = 3; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, toolAxisIdx_[i], 0);
	}
	// ������д��͹�ֱ�
	for (size_t i = 0; i < numInterp; ++i) {
		// ��ֵ���Ӧ��ƫ�Ʒ�ֵ
		float tmp = std::sin(2 * M_PI * i / (numInterp - 1));
		ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg+i, tmp);
	}
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	std::vector<float> cmd(moveCmd.begin(), moveCmd.begin() + toolAxisIdx_.size());
	ZAux_Direct_Move(handle_, toolAxisIdx_.size(), toolAxisIdx_.data(), cmd.data());

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], camAxisIdx_[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);
	// ������ָ�����岹�ٶ�
	for (size_t i = 3; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle_, toolAxisIdx_[i], 1);
	}

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, -1);

	// ÿһ�ε�λ��
	//std::vector<float> detDispl = { displ[0] / 20, displ[1] / 20, displ[2] / 20 };
	//std::cout << numPeriod << std::endl;
	//std::cout << detDispl[0] << ", " << detDispl[1] << ", " << detDispl[2] << std::endl;
	//for (size_t i = 0; i <  20; ++i) {
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//}
	//for (size_t i = 0; i < numPeriod; ++i) {
	//	// ��λ��
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// ����ʱ
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, leftHoldT);
	//	}
	//	// ����
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// ������ʱ
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, midHoldT);
	//	}
	//	// ��λ��
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// ����ʱ
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, rightHoldT);
	//	}
	//	// ����
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// ������ʱ
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, midHoldT);
	//	}
	//}
	return 0;
}

uint8_t ZauxRobot::swingL(const std::vector<float>& moveCmd) {
	// �����
	ZMC_HANDLE handle = handle_;
	// TCP λ����� + ���������
	std::vector<int> toolAxisIdx = { toolAxisIdx_[0],toolAxisIdx_[1],toolAxisIdx_[2],toolAxisIdx_[3] };
	// TCP ��̬���
	std::vector<int> poseAxisIdx = { ikAxisIdx_[3], ikAxisIdx_[4], ikAxisIdx_[5] };
	// ͹�������
	std::vector<int> camAxisIdx = { camAxisIdx_[0], camAxisIdx_ [1], camAxisIdx_[2] };
	// ����ʸ�������
	int connpathAxisIdx = 15;

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
	float ampl = waveCfg.Width / 2;
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
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = waveCfg.Width / 2;
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
	ZAux_Direct_GetUnits(handle_, toolAxisIdx_[0], &units);

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx_[i], &begPnt[i]);
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
	ZAux_Direct_Connpath(handle_, 1, toolAxisIdx_[0], 15);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle_, toolAxisIdx_[i], camAxisIdx_[i]);
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
	for (size_t i = 0; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetSpeed(handle_, toolAxisIdx_[i], vel);
	}
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);
#pragma endregion swing_config

	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", camAxisIdx_[i], 0);
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
			float off = sin(float(j) / (numInterpInPeriod-1) * 2 * M_PI);

			// ��¼͹�ֱ�
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// ���µ�ǰ�Ƕ�
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 3 - 1, 0);
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);

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
	int toolAxisIdx[6] = {20,21,22,10,11,12};
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

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], camAxisIdx_[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, -1);

	return 0;
}

uint8_t ZauxRobot::zswingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = waveCfg.Width / 2;
	// �����ٶ�
	float vel = 20.0;

	// �л������ģʽ, �������
	inverse_kinematics();

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
	ZAux_Direct_GetUnits(handle_, toolAxisIdx_[0], &units);

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx_[i], &begPnt[i]);
		//volatile float tmpPos = begPnt[i];
		int tmpPos = std::floor(begPnt[i] * 1000);
		//tmpPos /= 1000;
		begPnt[i] = tmpPos / 1000.0;
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
	// ��ʼ�ڶ�����
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// �ܾ���
	int distUnits = std::floor((begPnt - center).norm() * theta * units);
	float dist = distUnits / units;
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq);
	
	// ������д��͹�ֱ�
	float dq = theta / numPeriod / 2, curQ = 0.0;
	op1 = begPnt - center;

	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endConfig[0], endConfig[1], endConfig[2], midConfig[0], midConfig[1], midConfig[2], 0, 0, 0, 0);
	//// ��¼͹�ֱ����ݵ�����
	//for (size_t i = 0; i < numPeriod; ++i) {
	//	// ��ǰ���ڵ��˶�
	//	Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
	//	curQ += dq;
	//	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	curQ += dq;
	//	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	int ret = ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
	//	ZAux_Direct_MoveDelay(handle_, toolAxisIdx_[0], 100);
	//	std::cout << ret << ", pos = " << curEnd.transpose() << std::endl;
	//}

	return 0;
}

uint8_t ZauxRobot::swingC_(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
	// �ڶ�Ƶ��
	float freq = waveCfg.Freq;
	// �ڶ����
	float ampl = waveCfg.Width / 2;
	// �����ٶ�
	float vel = 10.0;

	// ͹�ֱ���ʼ����
	size_t sinTableBeg = 2000;
	// һ���ڶ����ڵĲ�ֵ����
	size_t numInterpInPeriod = 10;

	// �л������ģʽ, �������
	inverse_kinematics();

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

	// ��ȡԲ����㣬��ǰ�����е�����λ��
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetEndMoveBuffer(handle_, toolAxisIdx_[i], &begPnt[i]);
		//ZAux_Direct_GetMpos(handle, toolAxisIdx[i], &begPnt[i]);
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
	// Բ���˶�ƽ��ķ��߷���
	Eigen::Vector3f normal = op1.cross(op3);
	if (normal.norm() < 1) {
		normal = op1.cross(op2);
	}
	// Բ�ĽǽǶ�
	float theta = std::acos(op1.dot(op3));
	// ����Բ�ĽǺ�������
	if (op1.cross(op2).dot(op2.cross(op3)) <= 0) {
		theta = 2 * M_PI - theta;
		normal *= -1;
	}
	// ��ʼ�ڶ�����
	Eigen::Vector3f offDirBase = zDir.cross(normal.cross(op1)).normalized();
	// zDir.cross(normal).cross(zDir).normalized();
	std::cout << "zDir       = " << zDir.transpose() << std::endl;
	std::cout << "normal     = " << normal.transpose() << std::endl;
	std::cout << "offDirBase = " << offDirBase.transpose() << std::endl;
	// �ܾ���
	float dist = (begPnt - center).norm() * theta;
	// ������
	size_t numPeriod = std::ceil(dist / vel * freq);
	// ����·���ϵĲ�ֵ��
	size_t numInterp = numPeriod * numInterpInPeriod + 1;

	// *** ���ò岹ʸ���� *************************************
	// �ȴ��ڶ���־λ��λ
	ZAux_Direct_MoveWait(handle_, 15, "TABLE", swingFlagIdx, 1, 0);
	// ʹ�ò岹ʸ����������Ϊ͹����ĸ������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle_, 1, toolAxisIdx_[0], 15);

	// *** ����͹�������� *************************************
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle_, toolAxisIdx_[i], camAxisIdx_[i]);
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
	for (size_t i = 0; i < toolAxisIdx_.size(); ++i) {
		ZAux_Direct_SetSpeed(handle_, toolAxisIdx_[i], vel);
	}
	// ���ø����᲻�����ٶȼ���
	//for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
	//	ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	//}

	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		// ͹����λ������
		ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", camAxisIdx_[i], 0);
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
			float off = sin(float(j) / numInterpInPeriod * 2 * M_PI);

			// ��¼͹�ֱ�
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + interpIdx, offDir[0] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp + interpIdx, offDir[1] * off);
			ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// ���µ�ǰ�Ƕ�
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 2 - 1, 0);
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], sinTableBeg + numInterp * 3 - 1, 0);
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	//ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	dq = theta / numPeriod / 2;
	curQ = 0.0;
	op1 = begPnt - center;
	//std::cout << (op1 + center).transpose() << std::endl;
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			Eigen::Vector3f curMid(0, 0, 0), curEnd(0, 0, 0);
			curQ += dq;
			curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
			curQ += dq;
			curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
			ZAux_Direct_MSphericalAbs(handle_, 3, toolAxisIdx_.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
		}
	}

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx_.size(); ++i) {
		ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], camAxisIdx_[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle_, toolAxisIdx_[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle_, toolAxisIdx_[0], "DPOS", 15, 0);
	// ������ָ�����岹�ٶ�
	//for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
	//	ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	//}

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle_, toolAxisIdx_[0], swingFlagIdx, -1);

	return 0;
}

uint8_t circle_swing() {
	// Բ����㡢���ɵ㡢�յ�
	Eigen::Vector3d begPnt, viaPnt, endPnt;
	return 0;
}

Eigen::Vector3f triangular_circumcenter(Eigen::Vector3f beg, Eigen::Vector3f mid, Eigen::Vector3f end) {
	Eigen::Vector3f a = beg - mid, b = end - mid;
	if (a.cross(b).squaredNorm() < 1e-12) {
		float inf = std::numeric_limits<float>::max();
		return { inf, inf, inf };
	}

	return (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + mid;
}

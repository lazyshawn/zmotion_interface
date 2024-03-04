
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
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle)) {
		printf("Connect controller failed!\nPress <Enter> exist!\n");
		handle = NULL;
		getchar();
		return 1;
	}
	return 0;
}

uint8_t ZauxRobot::load_basic_pragma(const char *basPath, uint32_t mode) {
	// ���� bas ����
	if (ZAux_BasDown(handle, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return 1;
	}
	// �ȴ� bas ��������
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::disconnect() {
	//�ر����� 
	if (ZAux_Close(handle) > 0) {
		printf("Error: # ZauxRobot::disconnect()!\n");
		return 1;
	}
	printf("connection closed!\n");
	handle = NULL;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::trigger_scope() {
	ZAux_Trigger(handle);
	return 0;
}

uint8_t ZauxRobot::forward_kinematics() {
	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 0) {
		return 1;
	}
	// �ȴ�ĩ���˶�����
	wait_idle(20);
	//if (connType == ConnType::FK) {
	//	return 1;
	//}
	int ret = 0;
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connreframe()
	//ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	ret = ZAux_Direct_Connreframe(handle, baseAxis.size(), baseAxis.data(), 93, 100, connreAxis.size(), connreAxis.data());
	std::cout << "1. ret = " << ret << std::endl;
	ret = ZAux_Direct_Connreframe(handle, ikAxisIdx.size(), ikAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	std::cout << "2. ret = " << ret << std::endl;

	connType = ConnType::FK;
	ZAux_Direct_SetUserVar(handle, "kinematic", 1);
	return 0;
}

uint8_t ZauxRobot::inverse_kinematics() {
	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics < 0) {
		return 1;
	}
	// �ȴ��ؽ��˶�����
	wait_idle(jointAxisIdx[0]);
	//if (connType == ConnType::IK) {
	//	return 1;
	//}
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connframe()
	//ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, toolAxisIdx.size(), toolAxisIdx.data());
	int ret = 0;
	ret = ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, ikAxisIdx.size(), ikAxisIdx.data());
	ret = ZAux_Direct_Connframe(handle, connreAxis.size(), connreAxis.data(), 93, 100, baseAxis.size(), baseAxis.data());
	connType = ConnType::IK;
	ret = ZAux_Direct_SetUserVar(handle, "kinematic", -1);
	return 0;
}

uint8_t ZauxRobot::wait_idle(int axisIdx) {
	float IDLE;
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		ZAux_Direct_GetParam(handle, "IDLE", axisIdx, &IDLE);
		if (IDLE < 0)break;
	}
	return 0;
}

uint8_t ZauxRobot::moveJ(const std::vector<float>& jntDPos) {
	forward_kinematics();

	std::vector<float> cmd(jntDPos.begin(), jntDPos.begin() + jointAxisIdx.size() + appAxisIdx.size());
	std::vector<int> axis = jointAxisIdx;
	axis.insert(axis.end(), appAxisIdx.begin(), appAxisIdx.end());
	ZAux_Direct_MoveAbs(handle, axis.size(), axis.data(), cmd.data());
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
	ZAux_Direct_Move(handle, 3, axisList, dist);
	return 0;
}

uint8_t ZauxRobot::moveL_single() {
	return 0;
}

uint8_t ZauxRobot::moveC() {
	inverse_kinematics();
	ZAux_Direct_MSphericalAbs(handle, 6, toolAxisIdx.data(), 800, 400, 300, 900, 200, 600, 0, 20, 20, 50);
	return 0;
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

	// ����ں�����
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
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg+numInterp-1, ampl * 1000 * offDir[i], dist / numPeriod, 15, 4, 0);
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
	// ������д��͹�ֱ�
	for (size_t i = 0; i < numInterp; ++i) {
		// ��ֵ���Ӧ��ƫ�Ʒ�ֵ
		float tmp = std::sin(2 * M_PI * i / (numInterp - 1));
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg+i, tmp);
		std::cout << tmp << std::endl;
	}
	// ���Ỻ���н��ڶ���־λ��λ���ڶ���ʼ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// ��ʼ�˶�ָ��
	std::vector<float> cmd(moveCmd.begin(), moveCmd.begin() + toolAxisIdx.size());
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// �ڶ�����
	// ֹͣ͹�����˶�
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// �岹ʸ����ȡ����
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], 15, 0);
	// �岹ʸ����λ������
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);
	// ������ָ�����岹�ٶ�
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}

	// �ڶ���־λ��λ
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

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

uint8_t ZauxRobot::swingC() {
	// �������
	inverse_kinematics();

	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_SetMpos(handle, camAxisIdx[i], 0);
		ZAux_Direct_SetDpos(handle, camAxisIdx[i], 0);
	}
	// �˶�����
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[0] + i, camAxisIdx[0] + i);
	}

	Eigen::Vector3f begPnt(0, 0, 0);
	// ��ȡԲ�����
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetMpos(handle, toolAxisIdx[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(0, 0, 0), endPnt(0, 0, 0);
	midPnt = begPnt + Eigen::Vector3f(-200, 0, -200);
	endPnt = begPnt + Eigen::Vector3f(-400, 0, 0);

	// ����Բ������
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// �뾶����
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// �뾶����
	if (op1.norm() > 1e3) return {};

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
	// �ܾ���
	float dist = (begPnt - center).norm() * theta;

	// �ڶ�Ƶ��
	float freq = 1;
	// ��ȡ�����ٶ�
	float vel = 0.0;
	ZAux_Direct_GetSpeed(handle, toolAxisIdx[0], &vel);
	// ������
	int numPeriod = std::ceil(dist / vel * freq);

	// һ�����ڵĲ�ֵ��
	size_t num = 50;
	// ����·���ϵĲ�ֵ��
	size_t numInterp = numPeriod * num + 1;
	float dq = theta / (numInterp - 1), curQ = 0.0;
	// ��¼͹�ֱ����ݵ�����
	std::vector<float> camX(numInterp, 0), camY(numInterp, 0), camZ(numInterp, 0);
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < num; j++) {
			// ƫ�Ʒ���
			Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			// ���߷���
			Eigen::Vector3f tanDir = normal.cross(offDir);
			// ƫ����λ��
			float off = sin(float(j) / num * 2 * M_PI);

			// ��¼͹�ֱ�
			camX[i * num + j] = offDir[0] * off;
			camY[i * num + j] = offDir[1] * off;
			camZ[i * num + j] = offDir[2] * off;
			//std::cout << camX[i * num + j] << ", " << camZ[i * num + j] << std::endl;

			// ���µ�ǰ�Ƕ�
			curQ += dq;
		}
	}

	// ͹�ֱ�д�� Table
	ZAux_Direct_SetTable(handle, 1000, numInterp, camX.data());
	ZAux_Direct_SetTable(handle, 1000+numInterp, numInterp, camY.data());
	ZAux_Direct_SetTable(handle, 1000+2*numInterp, numInterp, camZ.data());

	// ����͹���˶�
	float tableMult = 10 * 1000;
	ZAux_Direct_Cambox(handle, camAxisIdx[0], 1000, 1000 + numInterp - 1,                     tableMult, dist, 15, 4, 0);
	ZAux_Direct_Cambox(handle, camAxisIdx[1], 1000 + numInterp, 1000 + 2 * numInterp - 1,     tableMult, dist, 15, 4, 0);
	ZAux_Direct_Cambox(handle, camAxisIdx[2], 1000 + 2 * numInterp, 1000 + 3 * numInterp - 1, tableMult, dist, 15, 4, 0);

	// �岹ʸ�������ᣬ��¼�����ʸ���˶����룬���ܵ�����Ӱ��
	ZAux_Direct_Connpath(handle, 1, 20, 15);

	// Բ���˶�
	//ZAux_Direct_MSphericalAbs(handle, 6, toolAxisIdx.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
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

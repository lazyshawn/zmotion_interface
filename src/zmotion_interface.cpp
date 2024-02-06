
#include<chrono>
#include<thread>

#include"zmotion_interface.h"

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
	if (connType == ConnType::FK) {
		return 1;
	}
	//! @param handle, base(), type, tableBegin, connreframe()
	ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	connType = ConnType::FK;
	return 0;
}

uint8_t ZauxRobot::inverse_kinematics() {
	if (connType == ConnType::IK) {
		return 1;
	}
	//! @param handle, base(), type, tableBegin, connframe()
	ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, toolAxisIdx.size(), toolAxisIdx.data());
	connType = ConnType::IK;
	return 0;
}

uint8_t ZauxRobot::moveJ() {
	return 0;
}

uint8_t ZauxRobot::moveJ_single() {
	return 0;
}

uint8_t ZauxRobot::moveL() {
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

uint8_t ZauxRobot::swingL(Eigen::Vector3f displ, Eigen::Vector3f upper) {
	// �л������ģʽ, �������
	inverse_kinematics();
	// ͬ�����⹤����λ��
	for (size_t i = 0; i < virtualAxisIdx.size(); ++i) {
		float pos;
		ZAux_Direct_GetMpos(handle, toolAxisIdx[0] + i, &pos);
		ZAux_Direct_SetMpos(handle, virtualAxisIdx[0] + i, pos);
		ZAux_Direct_SetDpos(handle, virtualAxisIdx[0] + i, pos);
	}

	for (size_t i = 0; i < virtualAxisIdx.size(); ++i) {
		// �����������˶����ӵ�͹������
		ZAux_Direct_Single_Addax(handle, camAxisIdx[0] + i, virtualAxisIdx[0] + i);
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[0] + i, camAxisIdx[0] + i);
	}
	
	// �켣ƽ����Ϸ��߷���
	upper.normalize();
	// ֱ�߹켣�ĳ���
	Eigen::Vector3f displDir = displ.normalized();
	// ƫ�Ʒ���
	Eigen::Vector3f offDir = displ.normalized().cross(upper);
	// ��λ��
	float dist = displ.norm();

	// ��λ������������д�� 
	std::vector<float> sinTable(100);
	for (int i = 0; i < 100; ++i) {
		sinTable[i] = sin(2 * M_PI * i / 99);
	}

	// ��͹�ֱ�����д�� Table �Ĵ���ֵ
	ZAux_Direct_SetTable(handle, 200, 100, sinTable.data());

	// ��ȡ���ٶ�
	std::vector<float> vel(3);
	for (size_t i = 0; i < vel.size(); ++i) {
		ZAux_Direct_GetSpeed(handle, 0, &vel[i]);
		printf("%d, vel = %f\n", i, vel[i]);
	}

	// ɸѡ�˶�����Ϊ�����
	size_t primeAxis = 0;
	float minT = std::numeric_limits<float>::max();
	for (size_t i = 0; i < 3; ++i) {
		float time = displ[i] / vel[i];
		if (time < minT) {
			minT = time;
			primeAxis = i;
		}
	}
	// ����λ��
	float primeDist = std::fabs(displ[primeAxis]);
	float freq = 1;
	size_t numPeriod = std::ceil(primeDist / vel[primeAxis] * freq);
	// �����ʵ������
	primeAxis += virtualAxisIdx[0];

	// ����һ���˶�����Ϊ�������͹���˶�
	// ��� = ϵ������ / ���嵱��; ���� = �ο��˶��ľ��� / ���ٶ�
	ZAux_Direct_Cambox(handle, 20, 200, 299, 100000 * offDir[0], primeDist / numPeriod, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 21, 200, 299, 100000 * offDir[1], primeDist / numPeriod, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 22, 200, 299, 100000 * offDir[2], primeDist / numPeriod, primeAxis, 4, 0);

	//ZAux_Direct_Single_Move(handle, 28, -500);

	int axisList[] = { 26,27,28 };
	ZAux_Direct_Move(handle, 3, axisList, displ.data());
	return 0;
}

uint8_t ZauxRobot::swingC(std::vector<Eigen::Vector3f> traj) {
	// �������
	inverse_kinematics();
	// ͬ�����⹤����λ��
	for (size_t i = 0; i < virtualAxisIdx.size(); ++i) {
		float pos;
		ZAux_Direct_GetMpos(handle, toolAxisIdx[0] + i, &pos);
		ZAux_Direct_SetMpos(handle, virtualAxisIdx[0] + i, pos);
		ZAux_Direct_SetDpos(handle, virtualAxisIdx[0] + i, pos);
	}

	for (size_t i = 0; i < virtualAxisIdx.size(); ++i) {
		// �����������˶����ӵ�͹������
		ZAux_Direct_Single_Addax(handle, camAxisIdx[0] + i, virtualAxisIdx[0] + i);
		// ͹�����˶����ӵ���ʵ��������
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[0] + i, camAxisIdx[0] + i);
	}

	// Բ���˶�ƽ��ķ��߷���

	// Բ���˶�
	//ZAux_Direct_Single_Move(handle, 30, 20);
	ZAux_Direct_MSphericalAbs(handle, 6, virtualAxisIdx.data(), 800, 400, 300, 900, 200, 600, 0, 20, 20, 50);
	return 0;
}

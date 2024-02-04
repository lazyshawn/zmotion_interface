
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
	//! @param handle, base(), type, tableBegin, connreframe()
	ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	return 0;
}

uint8_t ZauxRobot::inverse_kinematics() {
	//! @param handle, base(), type, tableBegin, connframe()
	ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, toolAxisIdx.size(), toolAxisIdx.data());

	ZAux_Direct_Connframe(handle, jointAxisIdx.size(), jointAxisIdx.data(), 6, 0, virtualAxisIdx.size(), virtualAxisIdx.data());
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
	return 0;
}

uint8_t ZauxRobot::sin_swingL(Eigen::Vector3f displ, Eigen::Vector3f upper) {
	// �����������˶����ӵ���ʵ��������
	ZAux_Direct_Single_Addax(handle, 20, 26);
	ZAux_Direct_Single_Addax(handle, 21, 27);
	ZAux_Direct_Single_Addax(handle, 22, 28);
	// ͹�����˶����ӵ���ʵ��������
	ZAux_Direct_Single_Addax(handle, 6, 20);
	ZAux_Direct_Single_Addax(handle, 7, 21);
	ZAux_Direct_Single_Addax(handle, 8, 22);

	upper.normalize();
	Eigen::Vector3f displDir = displ.normalized();
	// ��λ��
	float dist = displ.norm();
	// ƫ�Ʒ���
	Eigen::Vector3f offDir = displ.normalized().cross(upper);

	// ��λ������������д�� 
	std::vector<float> sinTable(100);
	for (int i = 0; i < 100; ++i) {
		sinTable[i] = sin(2 * M_PI * i / 99);
	}

	// ��͹�ֱ�����д�� Table �Ĵ���ֵ
	ZAux_Direct_SetTable(handle, 200, 100, sinTable.data());
	// ����ʾ����
	trigger_scope();

	// �л������ģʽ
	inverse_kinematics();

	// ɸѡ�˶�����Ϊ�����
	size_t primeAxis = 0;
	for (size_t i = 0; i < 3; ++i) {
		if (std::fabs(displDir[i]) > 0.5) {
			primeAxis = i;
			break;
		}
	}
	primeAxis += virtualAxisIdx[0];

	// ����һ���˶�����Ϊ�������͹���˶�
	// ��� = ϵ������ / ���嵱��
	// ���� = �ο��˶��ľ��� / ���ٶ�
	ZAux_Direct_Cambox(handle, 20, 200, 299, 100000 * offDir[0], 100, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 21, 200, 299, 100000 * offDir[1], 100, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 22, 200, 299, 100000 * offDir[2], 100, primeAxis, 4, 0);

	//ZAux_Direct_Single_Move(handle, 28, -500);

	int axisList[] = { 26,27,28 };
	ZAux_Direct_Move(handle, 3, axisList, displ.data());
	return 0;
}


#include <windows.h>
#include<iostream>

#include"zmotion_interface.h"

int test();
void commandCheckHandler(const char *command, int ret) {
	// �� 0��ʧ��
	if (ret) {
		printf("%s return code is %d\n", command, ret);
	}
}

int main() {
	ZauxRobot robot;
	if (robot.connect("127.0.0.1") > 0) {
		std::cout << "Connect error" << std::endl;
		getchar();
		return 1;
	}

	//std::string basPath = "D:\\CIMC\\CppPro\\zmotion\\config\\test\\main.bas";
	//if (robot.load_basic_pragma(basPath.c_str()) > 0) {
	//	return 1;
	//}

	// ����ʾ����
	robot.trigger_scope();

	robot.moveJ({ 0, -6.8221, 14.3160, -0.0000, 55.1156, 0, 0 });
	//robot.moveL();
	robot.swingL({ -200, 0, 0, 200 }, {0,0,1});
	//robot.swingC();
	//robot.swingC({ {800, 200, 600}, {300, 500, 1200} });

	// �Ͽ�����
	robot.disconnect();
	return 0;
}

int test() {
	std::cout << "hello world" << std::endl;
	ZauxRobot robot;

	int ret;
	// Ĭ�Ͽ����� IP��ַ 192.168.0.11, ������ IP ��ַ 127.0.0.1
	char *ip_addr = (char *)"127.0.0.1";
	// ���������
	ZMC_HANDLE handle = NULL;
	// ͨ���������ӿ�����
	if (ERR_SUCCESS != ZAux_OpenEth(ip_addr, &handle))	{
		printf("Connect controller failed!\nPress <Enter> exist!\n");
		handle = NULL;
		getchar();
		return -1;
	}
	printf("���������ӳɹ���\n");

	// bas ����·��
	std::string basPath = "D:\\CIMC\\zmotion\\robot_configuration.bas";
	// ���� bas ����
	if (ZAux_BasDown(handle, basPath.c_str(), 0) != 0) {
		printf("Error.");
		return 0;
	}
	// �ȴ� bas ��������
	Sleep(1000);
	// ���
	int m_nAxis = 0;

	// ��λ������������д�� 
	std::vector<float> sinTable(100);
	for (int i = 0; i < 100; ++i) {
		sinTable[i] = sin(2 * M_PI * i / 99);
	}

	int firstAxis = 0, secondAxis = 1;
	// ��͹�ֱ�����д�� Table �Ĵ���ֵ
	ZAux_Direct_SetTable(handle, 200, 100, sinTable.data());
	// ����ʾ����
	ZAux_Trigger(handle);

	// �л������ģʽ��ÿ��ִ�й�������ϵ�˶�ʱ��Ҫ��һ�Σ�ͬ������ؽ�����仯
	ZAux_Direct_Connframe(handle, robot.jointAxisIdx.size(), robot.jointAxisIdx.data(), 6, 0, robot.toolAxisIdx.size(), robot.toolAxisIdx.data());

	// �ο��� 1 �˶��� 100 λ��ʱ�������� 0����͹�ֱ��˶�
	/**
	* @param handle ���
	* @param ͹�������
	* @param ͹�ֱ���ʼ����
	* @param ͹�ֱ���ֹ����
	* @param λ�ñ���
	* @param һ��͹���˶����ڵľ��룬�˶����� = ���� / ͹�����ٶ�
	* @param �ο������
	* @param �ο���Ĺ�����ʽ
	* @param �ο���λ��
	*/
	ZAux_Direct_Cambox(handle, 6, 200, 299, 10000, 100, 8, 4, 0);

	ZAux_Direct_Single_Move(handle, 8, -500);

	Sleep(500);
	ret = ZAux_Close(handle);    //�ر����� 
	printf("connection closed!\n");
	handle = NULL;
	return 0;
}

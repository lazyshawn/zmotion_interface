#include <windows.h>
#include<iostream>
#include<fstream>
#include<bitset>

#include"zmotion_interface.h"

int test();
void commandCheckHandler(const char *command, int ret) {
	// �� 0��ʧ��
	if (ret) {
		printf("%s return code is %d\n", command, ret);
	}
}

ZauxRobot robot;

void test_dec_to_hex();
void test_hex_to_dec(uint16_t high, uint16_t low);
void format_float(std::string ifname, std::string ofname);
void test_swing();
void test_zswing();

int main() {
	//format_float("C:\\Users\\15874\\Desktop\\SD0.BIN", "C:\\Users\\15874\\Desktop\\SD0.txt");

	if (robot.connect("127.0.0.1") > 0) {
	//if (robot.connect("192.168.1.14") > 0) {
		std::cout << "Connect error" << std::endl;
		getchar();
		return 1;
	}
	//robot.forward_kinematics();
	//robot.inverse_kinematics();

	////std::string basPath = "D:\\CIMC\\CppPro\\zmotion\\config\\test\\main.bas";
	////if (robot.load_basic_pragma(basPath.c_str()) > 0) {
	////	return 1;
	////}

	// ����ʾ����
	//robot.trigger_scope();

	//robot.moveJ({ 0, -6.8221, 14.3160, -0.0000, 55.1156, 0, 0 });
	////robot.moveL();
	////robot.swingL({ -200, 0, 0, 200 }, {0,0,1});
	//robot.swingC({1467.749, -2.06, 163.68}, {1267.749, -2.06, 363.68});
	////robot.swingL({ -200, 0, 0, 200 }, {0,0,1});

	test_swing();
	//test_zswing();

	//// �Ͽ�����
	//robot.disconnect();
	return 0;
}

void test_zswing() {
	robot.waveCfg.Freq = 1;
	robot.waveCfg.Width = 20;
	robot.waveCfg.Dwell_left = 1000;
	robot.waveCfg.Dwell_right = 1000;

	int ret = 0;
	robot.moveJ({ -1.8055, 7.2945, 24.1143, -2.5585, 69.5887, 5.4886, 0.0 });


	//ZAux_Direct_MovePara(robot.handle_, 0, "VECTOR_MOVED", robot.toolAxisIdx_[0], 0);

	robot.wait_idle(0);
	std::this_thread::sleep_for(std::chrono::seconds(2));

	ret += robot.swing_on();
	//robot.moveL();
	// ��ȡ��ǰλ��

	robot.zswingC({ 1467.749, 200.06, 163.68 }, { 1267.749, -200.06, 663.68 });
	ret += robot.swing_off();

	//robot.moveL();

}

void test_swing() {
	robot.waveCfg.Freq = 1;
	robot.waveCfg.Width = 5;
	robot.waveCfg.Dwell_left = 1000;
	robot.waveCfg.Dwell_right = 1000;
	robot.waveCfg.Dwell_type = 0;
	
	robot.moveJ({ -1.8055, 7.2945, 24.1143, -2.5585, 69.5887, 5.4886, 0.0 });
	robot.swingL({ 0, -100, 0, 0 });
	//robot.swingL({ 0, -100, 0, 0 });

	//robot.moveJ({ 0, -6.8221, 14.3160, -0.0000, 55.1156, 30, 0 });
	//robot.swingC_({ 1034.4090, -118.5060, 78.8420 }, { 1034.4090, -18.5060, 78.8420 });
	//robot.swingC({ 1467.749, 200.06, 163.68 }, { 1267.749, -200.06, 663.68 });

	//robot.moveJ({ -15.9641, 33.5979, 3.9740, 16.2066, 76.0944, 31.1940, 0.0 });
	//robot.wait_idle(0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//std::cout << "Begin" << std::endl;
	////robot.wlder_on(240, 30);
	//std::this_thread::sleep_for(std::chrono::milliseconds(200));

	//robot.swingC({ 1244.7410, -44.3710, -73.6200, -201.6840, -40.8120, 227.0230 }, { 1265.5150, -90.9640, -73.2900, -177.6410, -41.2930, 175.6090 });
	////robot.swingC({ 1244.7410, -44.3710, -73.6200 }, { 1265.5150, -90.9640, -73.2900 });

	robot.wait_idle(20);
	std::cout << "End" << std::endl;
	//robot.wlder_off();

	//robot.inverse_kinematics();
	//int toolAxis[] = { 20,21,22,10,11,12 }, ret = 0.0;
	//ret = ZAux_Direct_MSphericalAbs(robot.handle_, 6, toolAxis, 1467.749, 200.06, 163.68, 1267.749, -200.06, 663.68, 0, 0, 0, 0);
	//robot.swingC({ 884.4090, -68.5060, 78.8420 }, { 934.4090, -18.5060, 78.8420 });
}

void format_float(std::string ifname, std::string ofname) {
	std::ifstream in(ifname, std::ios::binary);
	std::ofstream out(ofname, std::ios::trunc);
	if (!in.is_open() || !out.is_open()) {
		std::cout << "ERROR: file is not opened." << std::endl;
	}

	char s;
	size_t idx = 0;
	std::string numStr;
	numStr.clear();
	while (in.get(s)) {
		std::bitset<8> bits(s);
		numStr = bits.to_string() + numStr;
		if (idx++ == 3) {
			std::bitset<32> num(numStr);
			float fNum = *reinterpret_cast<float*>(&num);
			std::cout << fNum << std::endl;
			out << fNum << std::endl;
			numStr.clear();
			idx = 0;
		}
	}
	in.close();
}

void test_dec_to_hex() {
	int dec = 65533;
	uint16_t bin = dec;
	// �Ͱ�λ
	uint8_t low = bin & 0x00FF;
	// �߰�λ
	uint8_t high = bin >> 8;
	printf("high = %d\n", high);
	printf("low = %d\n", low);
}

void test_hex_to_dec(uint16_t high, uint16_t low) {
	//uint16_t high = 12;
	//uint16_t low = 12;
	uint16_t dec = (high << 8) + low;

	std::cout << high << ", " << low << " -> ";
	printf("dec = %d\n", dec);
	std::cout << dec * 1000 / 65535.0 << std::endl;
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
	ZAux_Direct_Connframe(handle, robot.jointAxisIdx_.size(), robot.jointAxisIdx_.data(), 6, 0, robot.toolAxisIdx_.size(), robot.toolAxisIdx_.data());

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

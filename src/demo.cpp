#include <windows.h>
#include<iostream>
#include<bitset>

#include"zmotion_interface.h"


ZauxRobot robot, robotB;
// handle, �����ؽ���, ��е��λ��, TCP��̬, ��������ϵ��TCPλ��, ������, ͹����, �岹ʸ����
//ZauxRobot robotB(robot.handle_, { 0,1,2,3,4,5 }, { 9,10,11 }, { 12,13,14 }, { 15,16,17 }, { 18,19,20 }, { 36,37,38 }, { 39 });

// �켣����
DiscreteTrajectory<float> discreteTrajectory;
// �ں�����
Weave waveCfg;
// �绡���ٲ���
Track trackCfg;

namespace shawn_test {
	int test();
	void dec_to_hex();
	void hex_to_dec(uint16_t high, uint16_t low);
	void format_float(std::string ifname, std::string ofname);
	void swing();
	void zswing();
	// ��ȡ���������ֵ
	void save_current();
}
using namespace shawn_test;

int main() {
	//format_float("C:\\Users\\15874\\Desktop\\SD0.BIN", "C:\\Users\\15874\\Desktop\\SD0.txt");

	if (robot.lazy_connect() > 0) {
	//if (robot.connect_eth("127.0.0.1") > 0) {
	//if (robot.connect_eth("192.168.1.14") > 0) {
		std::cout << "\nConnect error, Press <Enter> exist!\n" << std::endl;
		getchar();
		return 1;
	}

	//// ���þ��
	//robotB.set_handle(robot.handle_);
	//// �������: �����ؽ���, ��е������ϵ��TCPλ��, TCP��̬, ��������ϵ��TCPλ��, ������, ͹����, �岹ʸ����
	//robotB.set_axis({ 0,1,2,3,4,5 }, { 9,10,11 }, { 12,13,14 }, { 15,16,17 }, { 18,19,20 }, { 36,37,38 }, { 39 });

	//swing();
	zswing();

	printf("Press <Enter> to exit.\n");
	getchar();

	//// �Ͽ�����
	//robot.disconnect();
	return 0;
}

void shawn_test::zswing() {
	//robot.test();

	// �ں�����
	waveCfg.Freq = 2.3;
	waveCfg.LeftWidth = 3.5;
	waveCfg.RightWidth = 3.5;
	waveCfg.Dwell_left = 500;
	waveCfg.Dwell_right = 50;
	waveCfg.Dwell_type = 0;

	// �绡���ٲ���
	trackCfg.Lr_enable = 0;
	trackCfg.Ud_enable = 0;

	//robot.arc_tracking_config(trackCfg);

	/* ****  **** */
	// base(0,1,2,3,4,5) moveabs(-10.9961, -11.7299, 34.4224, 0, 67.3076, -55.9961)
	//discreteTrajectory.set_starting_point({ 1000, -200, 200, 179.9990, -28.8890, -135, 0 });
	//discreteTrajectory.add_line({ 1000, 200, 200, 179.9990, -28.8890, 135, 0.0 }, 20);
	//discreteTrajectory.add_line({ 1400, 200, 200, 179.9990, -28.8900, 45, 0.0 });
	//discreteTrajectory.add_arc({ 1000, -200, 200, 179.9990, -28.8890, -135, 0 }, { 1000, 200, 200, 179.9990, -28.8890, 135, 0.0 });

	discreteTrajectory.set_starting_point({ 1310.7,-224.628,-69.899,-179.737,-48.0991,-8.9635, 0.0 });
	discreteTrajectory.add_line({ 1308.7,-263.008,-69.99,-179.737,-48.0991,-8.9635, 0.0 }, 7);
	//discreteTrajectory.add_line({ 1400, 200, 200, 179.9990, -28.8900, 45, 0.0 });

	robot.swing_trajectory(discreteTrajectory, waveCfg);
	//for (size_t i = 0; i < 5; ++i) {
	//	robot.swing_trajectory(discreteTrajectory, waveCfg);
	//	ZAux_Direct_MoveDelay(robot.handle_, robot.tcpPosAxisIdx_[0], 2000);
	//}

	//robot.discreteTrajectory.set_starting_point({ 1015.1998, -100.0001, 139.4812, 177.8830, -31.1151, -139.9815, 0 });
	//robot.discreteTrajectory.add_line({ 1013.0623, 99.9997, 139.5929, -177.4258, -30.7125, 128.5668, 0.0 });
	////robot.discreteTrajectory.add_line({ 1200.0044, 102.7630, 139.2698, -177.3642, -27.1874, 53.1885, 0.0 });

	//robot.discreteTrajectory.corner_transition(50, 50);

	//robot.swing_on();
	//robot.execute_discrete_trajectory_abs(discreteTrajectory);
	//robot.swing_off();

	/* ****  **** */
	//robot.swing_tri();

	/* ****  **** */

}

void shawn_test::swing() {
	waveCfg.Freq = 1;
	waveCfg.Width = 5;
	waveCfg.Dwell_left = 1000;
	waveCfg.Dwell_right = 1000;
	waveCfg.Dwell_type = 0;
	
	//robot.moveJ({ -1.8055, 7.2945, 24.1143, -2.5585, 69.5887, 5.4886, 0.0 });
	robot.swingL({ 0, -100, 0, 0 }, waveCfg);
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

	//robot.wait_idle(20);
	//std::cout << "End" << std::endl;
	//robot.wlder_off();

	//robot.inverse_kinematics();
	//int toolAxis[] = { 20,21,22,10,11,12 }, ret = 0.0;
	//ret = ZAux_Direct_MSphericalAbs(robot.handle_, 6, toolAxis, 1467.749, 200.06, 163.68, 1267.749, -200.06, 663.68, 0, 0, 0, 0);
	//robot.swingC({ 884.4090, -68.5060, 78.8420 }, { 934.4090, -18.5060, 78.8420 });
}

void shawn_test::format_float(std::string ifname, std::string ofname) {
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

void shawn_test::dec_to_hex() {
	int dec = 65533;
	uint16_t bin = dec;
	// �Ͱ�λ
	uint8_t low = bin & 0x00FF;
	// �߰�λ
	uint8_t high = bin >> 8;
	printf("high = %d\n", high);
	printf("low = %d\n", low);
}

void shawn_test::hex_to_dec(uint16_t high, uint16_t low) {
	//uint16_t high = 12;
	//uint16_t low = 12;
	uint16_t dec = (high << 8) + low;

	std::cout << high << ", " << low << " -> ";
	printf("dec = %d\n", dec);
	std::cout << dec * 1000 / 65535.0 << std::endl;
}

int shawn_test::test() {
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
	//ZAux_Direct_Connframe(handle, robot.jointAxisIdx_.size(), robot.jointAxisIdx_.data(), 6, 0, robot.toolAxisIdx_.size(), robot.toolAxisIdx_.data());

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

void shawn_test::save_current() {
	// ��ȡ����
	float tmpTable = 0.0;
	int numPoint = 0;
	size_t maxNum = 1000, times = 0, saverIdx = 0;
	std::vector<float> tableData(maxNum, 0);

	// ���ڿ�ʼ������
	saverIdx = 5000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint;
	printf("num of sin period: %d\n", numPoint);
	robot.save_table(saverIdx+1, numPoint, "./arc_tracking/SD0_idx.txt");

	// ԭʼ����ֵ
	saverIdx = 10000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint;
	printf("num of raw current: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, "./arc_tracking/SD0.txt");

	// �˲���ĵ���ֵ
	saverIdx = 110000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint;
	printf("num of fined current: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, "./arc_tracking/SD0_f.txt");

	// �˲���ĵ�ѹֵ
	saverIdx = 210000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint;
	printf("num of voltage: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, "./arc_tracking/SD0_v.txt");

	return;
}

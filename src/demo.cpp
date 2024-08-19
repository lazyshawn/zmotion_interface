#include <windows.h>
#include<iostream>
#include<bitset>

#include"zmotion_interface.h"


ZauxRobot robot, robotB;
// handle, 六个关节轴, 机械臂位置, TCP姿态, 世界坐标系下TCP位置, 附加轴, 凸轮轴, 插补矢量轴
//ZauxRobot robotB(robot.handle_, { 0,1,2,3,4,5 }, { 9,10,11 }, { 12,13,14 }, { 15,16,17 }, { 18,19,20 }, { 36,37,38 }, { 39 });

namespace shawn_test {
	// 轨迹处理
	DiscreteTrajectory<float> discreteTrajectory;
	// 摆焊参数
	Weave waveCfg, waveCfg2;
	// 电弧跟踪参数
	Track trackCfg, trackCfg2;
	// 焊接参数
	Arc_WeldingParaItem weldCfg, weldCfg2;

	void dec_to_hex();
	void hex_to_dec(uint16_t high, uint16_t low);
	void format_float(std::string ifname, std::string ofname);
	void zswing();
	// 读取并保存电流值
	void save_current(const std::string& fold = "arc_tracking/");
}
//using namespace shawn_test;

int main() {
	//format_float("C:\\Users\\15874\\Desktop\\SD0.BIN", "C:\\Users\\15874\\Desktop\\SD0.txt");

	if (robot.lazy_connect() > 0) {
		std::cout << "\nConnect error, Press <Enter> exist!\n" << std::endl;
		getchar();
		return 1;
	}

	//// 设置句柄
	//robotB.set_handle(robot.handle_);
	//// 设置轴号: 六个关节轴, 机械臂坐标系下TCP位置, TCP姿态, 世界坐标系下TCP位置, 附加轴, 凸轮轴, 摆动轴, 插补矢量轴
	//robotB.set_axis({ 0,1,2,3,4,5 }, { 7,8,9 }, { 10,11,12 }, { 7,8,9 }, {  }, { 36,37,38 }, { 40,41,42 }, { 39 });

	shawn_test::zswing();
	//shawn_test::save_current("./");

	printf("Press <Enter> to exit.\n");
	//getchar();

	//// 断开连接
	//robot.disconnect();
	return 0;
}

void shawn_test::zswing() {
	//robot.test();

	// 摆焊参数
	waveCfg.Shape = 0;
	waveCfg.Length = 3;
	waveCfg.Bias = 2;
	waveCfg.Freq = 1.0;
	waveCfg.LeftWidth = 4;
	waveCfg.RightWidth = 2;
	waveCfg.Dwell_left = 100;
	waveCfg.Dwell_right = 100;
	waveCfg.Dwell_type = 1;
	waveCfg.Angle_Ltype_top = 0;
	waveCfg.Angle_Ltype_btm = 0;

	waveCfg2.Shape = 0;
	waveCfg2.Freq = 2.0;
	waveCfg2.LeftWidth = 2;
	waveCfg2.RightWidth = 2;
	waveCfg2.Dwell_left = 200;
	waveCfg2.Dwell_right = 200;
	waveCfg2.Dwell_type = 0;

	// 电弧跟踪参数
	trackCfg.Lr_enable = 0;
	trackCfg.Ud_enable = 0;
	//robot.update_track_config(trackCfg);

	// 焊接参数
	weldCfg.WeldingSpeed = 20;
	weldCfg2.WeldingSpeed = 30;


	/* ****  **** */
	// base(0,1,2,3,4,5) moveabs(-10.9961, -11.7299, 34.4224, 0, 67.3076, -55.9961)
	discreteTrajectory.set_starting_point({ 132.561005,3.092700,-420.473999,-191.216995,-55.311699,-102.778000, 0 });
	//discreteTrajectory.add_line({ 124.508003,4.327800,-410.992004,-8.733237,-111.726524,65.269234, 0.0 }, waveCfg, trackCfg, weldCfg);
	//discreteTrajectory.add_line({ 120.498001,1.379500,-352.433014,-7.894470,-104.864510,65.795158, 0.0 }, waveCfg, trackCfg, weldCfg);
	discreteTrajectory.add_line({ 121.116997,0.437300,-344.458008,-7.285388,-104.983093,64.644684, 0.0 }, waveCfg, trackCfg, weldCfg);
	//discreteTrajectory.equally_divide({ 200, 100 });

	//18,19,20,21,22,23,24,25,26
	//discreteTrajectory.set_starting_point({ 1000, -200, 200, 179.9990, -28.8890, 135, 0 });
	//discreteTrajectory.add_line({ 1000, 0, 200, 179.9990, -28.8890, 135, 0.0 }, waveCfg, trackCfg, weldCfg);
	//discreteTrajectory.add_line({ 1000, 200, 200, 179.9990, -28.8890, 135, 0.0 }, waveCfg2, trackCfg2, weldCfg2);
	//discreteTrajectory.add_line({ 1200, 0, 200, 179.9990, -28.8900, 45, 0.0 }, 50);

	//discreteTrajectory.set_starting_point({ -22.873100,-1467.822754,-17.759701,178.017395,-43.356602,-46.023201,-85.892502, 0 });
	//discreteTrajectory.add_arc({ 759.797,97.0257,93.728,168.197,-46.3449,78.4946, 0 }, { 720.393,158.951,92.2991,-179.413,-48.6368,1.0271, 0 }, 20);
	//discreteTrajectory.add_line({ -22.873072, -1467.822754, -17.759697, -1.982573, -136.643463, 133.976715, -85.892509, 0.0 }, 25, 5);
	//robot.swing_trajectory(discreteTrajectory, waveCfg);

	/* ****  **** */
	//robot.swing_tri();

	/* ****  **** */
	//discreteTrajectory.corner_transition();
	//discreteTrajectory.corner_slowdown(10);
	//robot.execute_discrete_trajectory_abs(discreteTrajectory);
	//robot.swing_trajectory(discreteTrajectory, waveCfg);
	//robot.move_ptp_abs({ 100,120,80,50,80,30 }, 0.5);
	robot.swing_trajectory(discreteTrajectory);
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
	// 低八位
	uint8_t low = bin & 0x00FF;
	// 高八位
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


void shawn_test::save_current(const std::string& fold) {
	// 读取点数
	float tmpTable = 0.0;
	int numPoint = 0;
	size_t maxNum = 1000, times = 0, saverIdx = 0;
	std::vector<float> tableData(maxNum, 0);

	// 周期开始的索引
	saverIdx = 100000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint % 5000;
	printf("num of sin period: %d\n", numPoint);
	robot.save_table(saverIdx+1, numPoint, fold + "SD0_idx.txt");

	// 原始电流值
	saverIdx = 105000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint % 5000;
	printf("num of raw current: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, fold + "SD0.txt");

	// 滤波后的电流值
	saverIdx = 110000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint % 5000;
	printf("num of fined current: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, fold + "SD0_f.txt");

	// 滤波后的电压值
	saverIdx = 115000;
	ZAux_Direct_GetTable(robot.handle_, saverIdx, 1, (float*)&tmpTable);
	numPoint = std::floor(tmpTable);
	numPoint = numPoint < 0 ? 0 : numPoint % 5000;
	printf("num of voltage: %d\n", numPoint);
	robot.save_table(saverIdx + 1, numPoint, fold + "SD0_v.txt");

	return;
}


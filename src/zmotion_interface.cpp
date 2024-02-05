
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
	// 加载 bas 程序
	if (ZAux_BasDown(handle, basPath, mode) != 0) {
		printf("Error: # ZauxRobot::load_basic_pragma() check basPath.\n");
		return 1;
	}
	// 等待 bas 程序下载
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	return 0;
}

uint8_t ZauxRobot::disconnect() {
	//关闭连接 
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
	inverse_kinematics();
	ZAux_Direct_MSpherical(handle, 5, toolAxisIdx.data(), 100, -100, 200, -300, 0, 0, 0, 20, 20);
	return 0;
}

uint8_t ZauxRobot::sin_swingL(Eigen::Vector3f displ, Eigen::Vector3f upper) {
	// 工具虚拟轴运动叠加到真实工具轴上
	ZAux_Direct_Single_Addax(handle, 20, 26);
	ZAux_Direct_Single_Addax(handle, 21, 27);
	ZAux_Direct_Single_Addax(handle, 22, 28);
	// 凸轮轴运动叠加到真实工具轴上
	ZAux_Direct_Single_Addax(handle, 6, 20);
	ZAux_Direct_Single_Addax(handle, 7, 21);
	ZAux_Direct_Single_Addax(handle, 8, 22);
	
	// 轨迹平面的上法线方向
	upper.normalize();
	// 直线轨迹的朝向
	Eigen::Vector3f displDir = displ.normalized();
	// 偏移方向
	Eigen::Vector3f offDir = displ.normalized().cross(upper);
	// 总位移
	float dist = displ.norm();

	// 单位正弦曲线数据写入 
	std::vector<float> sinTable(100);
	for (int i = 0; i < 100; ++i) {
		sinTable[i] = sin(2 * M_PI * i / 99);
	}

	// 把凸轮表数据写入 Table 寄存器值
	ZAux_Direct_SetTable(handle, 200, 100, sinTable.data());

	// 切换到逆解模式
	inverse_kinematics();

	// 读取轴速度
	std::vector<float> vel(3);
	for (size_t i = 0; i < vel.size(); ++i) {
		ZAux_Direct_GetSpeed(handle, 0, &vel[i]);
		printf("%d, vel = %f\n", i, vel[i]);
	}

	// 筛选运动量不为零的轴
	size_t primeAxis = 0;
	float minT = std::numeric_limits<float>::max();
	for (size_t i = 0; i < 3; ++i) {
		float time = displ[i] / vel[i];
		if (time < minT) {
			minT = time;
			primeAxis = i;
		}
	}
	// 主轴位移
	float primeDist = std::fabs(displ[primeAxis]);
	float freq = 1;
	size_t numPeriod = std::ceil(primeDist / vel[primeAxis] * freq);
	// 主轴的实际索引
	primeAxis += virtualAxisIdx[0];

	// 跟随一个运动量不为零的轴做凸轮运动
	// 振幅 = 系数比例 / 脉冲当量; 周期 = 参考运动的距离 / 轴速度
	ZAux_Direct_Cambox(handle, 20, 200, 299, 100000 * offDir[0], primeDist / numPeriod, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 21, 200, 299, 100000 * offDir[1], primeDist / numPeriod, primeAxis, 4, 0);
	ZAux_Direct_Cambox(handle, 22, 200, 299, 100000 * offDir[2], primeDist / numPeriod, primeAxis, 4, 0);

	//ZAux_Direct_Single_Move(handle, 28, -500);

	int axisList[] = { 26,27,28 };
	ZAux_Direct_Move(handle, 3, axisList, displ.data());
	return 0;
}


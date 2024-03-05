
#include<chrono>
#include<thread>
#include<iostream>

#include"zmotion_interface.h"

// 轴号占用列表
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
	float kinematics = 0.0;
	ZAux_Direct_GetUserVar(handle, "kinematic", &kinematics);
	std::cout << "beg k = " << kinematics << std::endl;
	if (kinematics > 0) {
		return 1;
	}
	// 等待末端运动结束，工具轴+附加轴
	wait_idle(20);
	//if (connType == ConnType::FK) {
	//	return 1;
	//}
	int ret = 0;
	std::vector<int> baseAxis = { 20,21,22,6 }, connreAxis = { 7,8,9,6 };
	//! @param handle, base(), type, tableBegin, connreframe()
	//ZAux_Direct_Connreframe(handle, toolAxisIdx.size(), toolAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());
	ret = ZAux_Direct_Connreframe(handle, baseAxis.size(), baseAxis.data(), 93, 100, connreAxis.size(), connreAxis.data());
	ret = ZAux_Direct_Connreframe(handle, ikAxisIdx.size(), ikAxisIdx.data(), 6, 0, jointAxisIdx.size(), jointAxisIdx.data());

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
	// 等待关节运动结束，关节轴+附加轴
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
	// 切换到逆解模式, 关联逆解
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
	// 摆动频率
	float freq = 4.0;
	// 摆动振幅
	float ampl = 2.0;
	// 焊接速度
	float vel = 10.0;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterp = 100;

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();

	// *** 计算摆焊参数 ***************************************
	// 轨迹平面的上法线方向
	upper.normalize();
	// TCP 点位移
	Eigen::Vector3f displ = { moveCmd[0], moveCmd[1], moveCmd[2] };
	// 直线轨迹的朝向
	Eigen::Vector3f displDir = displ.normalized();
	// 偏移方向
	Eigen::Vector3f offDir = displ.normalized().cross(upper);
	// 总位移
	float dist = displ.norm();
	// 周期数
	size_t numPeriod = std::ceil(dist / vel * freq);

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle, 15, "TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], 15);

	// *** 设置凸轮虚拟轴 *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	// 等待摆动标志位置位
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg, sinTableBeg+numInterp-1, ampl * 1000 * offDir[i], dist / numPeriod, 15, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// 设置附加轴不参与速度计算
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	}
	// 缓冲中写入凸轮表
	for (size_t i = 0; i < numInterp; ++i) {
		// 插值点对应的偏移幅值
		float tmp = std::sin(2 * M_PI * i / (numInterp - 1));
		ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg+i, tmp);
	}
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// 开始运动指令
	std::vector<float> cmd(moveCmd.begin(), moveCmd.begin() + toolAxisIdx.size());
	ZAux_Direct_Move(handle, toolAxisIdx.size(), toolAxisIdx.data(), cmd.data());

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], 15, 0);
	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);
	// 附加轴恢复计算插补速度
	for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	}

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	// 每一段的位移
	//std::vector<float> detDispl = { displ[0] / 20, displ[1] / 20, displ[2] / 20 };
	//std::cout << numPeriod << std::endl;
	//std::cout << detDispl[0] << ", " << detDispl[1] << ", " << detDispl[2] << std::endl;
	//for (size_t i = 0; i <  20; ++i) {
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//}
	//for (size_t i = 0; i < numPeriod; ++i) {
	//	// 左位移
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// 左延时
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, leftHoldT);
	//	}
	//	// 回中
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// 回中延时
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, midHoldT);
	//	}
	//	// 右位移
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// 右延时
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, rightHoldT);
	//	}
	//	// 回中
	//	ZAux_Direct_Move(handle, 3, axisList, detDispl.data());
	//	// 回中延时
	//	for (auto& axis : axisList) {
	//		ZAux_Direct_MoveDelay(handle, axis, midHoldT);
	//	}
	//}
	return 0;
}

uint8_t ZauxRobot::swingC(const std::vector<float>& endConfig, const std::vector<float>& midConfig) {
	// 摆动频率
	float freq = 0.2;
	// 摆动振幅
	float ampl = 2.0;
	// 焊接速度
	float vel = 10.0;

	// 凸轮表起始索引
	size_t sinTableBeg = 2000;
	// 一个摆动周期的插值点数
	size_t numInterpInPeriod = 10;

	// 切换到逆解模式, 关联逆解
	inverse_kinematics();
	
	// *** 计算摆焊参数 ***************************************
	// 读取圆弧起点
	Eigen::Vector3f begPnt(0, 0, 0);
	for (size_t i = 0; i < 3; ++i) {
		ZAux_Direct_GetMpos(handle, toolAxisIdx[i], &begPnt[i]);
	}
	Eigen::Vector3f midPnt(midConfig[0], midConfig[1], midConfig[2]), endPnt(endConfig[0], endConfig[1], endConfig[2]);
	// 计算圆心坐标
	Eigen::Vector3f center = triangular_circumcenter(begPnt, midPnt, endPnt);
	// 半径方向
	Eigen::Vector3f op1 = (begPnt - center).normalized(), op2 = (midPnt - center).normalized(), op3 = (endPnt - center).normalized();
	// 处理半径过大的情况
	if (op1.norm() > 1e3) {
		return {};
	}
	// 圆弧运动平面的法线方向
	Eigen::Vector3f normal = op1.cross(op3);
	if (normal.norm() < 1) {
		normal = op1.cross(op2);
	}
	// 圆心角角度
	float theta = std::acos(op1.dot(op3));
	// 修正圆心角和正法向
	if (op1.cross(op2).dot(op2.cross(op3)) <= 0) {
		theta = 2 * M_PI - theta;
		normal *= -1;
	}
	// 总距离
	float dist = (begPnt - center).norm() * theta;
	// 周期数
	size_t numPeriod = std::ceil(dist / vel * freq);
	// 整条路径上的插值点
	size_t numInterp = numPeriod * numInterpInPeriod + 1;

	// *** 设置插补矢量轴 *************************************
	// 等待摆动标志位置位
	ZAux_Direct_MoveWait(handle, 15, "TABLE", swingFlagIdx, 1, 0);
	// 使用插补矢量长度轴作为凸轮轴的跟随主轴，记录主轴的矢量运动距离，不受叠加轴影响
	ZAux_Direct_Connpath(handle, 1, toolAxisIdx[0], 15);

	// *** 设置凸轮虚拟轴 *************************************
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 凸轮轴运动叠加到真实工具轴上
		ZAux_Direct_Single_Addax(handle, toolAxisIdx[i], camAxisIdx[i]);
	}
	// 等待摆动标志位置位
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveWait(handle, camAxisIdx[i], "TABLE", swingFlagIdx, 1, 0);
	}
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		// 绑定跟随凸轮表
		ZAux_Direct_Cambox(handle, camAxisIdx[i], sinTableBeg + numInterp * i, sinTableBeg + numInterp*(i+1) - 1, ampl * 1000, dist, 15, 4, 0);
	}

	// *** 设置运动主轴 *************************************
	// 设置主轴速度
	for (size_t i = 0; i < toolAxisIdx.size(); ++i) {
		ZAux_Direct_SetSpeed(handle, toolAxisIdx[i], vel);
	}
	// 设置附加轴不参与速度计算
	//for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
	//	ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 0);
	//}
	// 缓冲中写入凸轮表
	// 插值点对应的圆心角
	float dq = theta / (numInterp - 1), curQ = 0.0;
	// 记录凸轮表数据的数组
	for (size_t i = 0; i < numPeriod; ++i) {
		for (size_t j = 0; j < numInterpInPeriod; j++) {
			// 偏移方向
			Eigen::Vector3f offDir = Eigen::AngleAxisf(curQ, normal) * op1;
			// 切线方向
			Eigen::Vector3f tanDir = normal.cross(offDir);
			// 偏移相位角
			float off = sin(float(j) / numInterpInPeriod * 2 * M_PI);

			// 记录凸轮表
			size_t interpIdx = i * numInterpInPeriod + j;
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + interpIdx,                 offDir[0] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp + interpIdx,     offDir[1] * off);
			ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp * 2 + interpIdx, offDir[2] * off);

			// 更新当前角度
			curQ += dq;
		}
	}
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp*2 - 1, 0);
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], sinTableBeg + numInterp*3 - 1, 0);
	// 主轴缓冲中将摆动标志位置位，摆动开始
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, 1);

	// 开始运动指令
	ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), endPnt[0], endPnt[1], endPnt[2], midPnt[0], midPnt[1], midPnt[2], 0, 0, 0, 0);
	//dq = theta / numPeriod / 2;
	//curQ = 0.0;
	//op1 = begPnt - center;
	//std::cout << (op1 + center).transpose() << std::endl;
	//for (size_t i = 0; i < numPeriod; ++i) {
	//	Eigen::Vector3f curMid(0,0,0), curEnd(0,0,0);
	//	curQ += dq;
	//	curMid = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	std::cout << curMid.transpose() << std::endl;
	//	curQ += dq;
	//	curEnd = Eigen::AngleAxisf(curQ, normal) * op1 + center;
	//	std::cout << curEnd.transpose() << std::endl;
	//	ZAux_Direct_MSphericalAbs(handle, 3, toolAxisIdx.data(), curEnd[0], curEnd[1], curEnd[2], curMid[0], curMid[1], curMid[2], 0, 0, 0, 0);
	//	// 延时
	//	for (auto& axis : toolAxisIdx) {
	//		ZAux_Direct_MoveDelay(handle, axis, 1000);
	//	}
	//}

	// 摆动结束
	// 停止凸轮轴运动
	for (size_t i = 0; i < camAxisIdx.size(); ++i) {
		ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], camAxisIdx[i], 0);
	}
	// 插补矢量轴取消绑定
	ZAux_Direct_MoveCancel(handle, toolAxisIdx[0], 15, 0);
	// 插补矢量轴位置清零
	ZAux_Direct_MovePara(handle, toolAxisIdx[0], "DPOS", 15, 0);
	// 附加轴恢复计算插补速度
	//for (size_t i = 3; i < toolAxisIdx.size(); ++i) {
	//	ZAux_Direct_SetInterpFactor(handle, toolAxisIdx[i], 1);
	//}

	// 摆动标志位复位
	ZAux_Direct_MoveTable(handle, toolAxisIdx[0], swingFlagIdx, -1);

	return 0;
}

uint8_t circle_swing() {
	// 圆弧起点、过渡点、终点
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

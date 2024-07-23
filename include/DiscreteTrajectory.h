#pragma once

#include<vector>
#include<list>
#include<eigen3/Eigen/Dense>
#include<cmath>

#include<iostream>

#include"FsCraftDef.h"


/**
* @brief  计算等效的 Rzyx 欧拉角
* @param  endEuler    给定欧拉角 <Rx, Ry, Rz>
* @return 等效的 Rzyx 欧拉角
*/
template <typename T>
Eigen::Matrix<T, 3, 1> get_equivalent_zyx_euler(const Eigen::Matrix<T, 3, 1>& endEuler) {

	Eigen::Matrix<T, 3, 1> equivEuler = endEuler;

	while (std::fabs(equivEuler[1]) - 180 > 0) {
		equivEuler[1] += equivEuler[1] > 0 ? -360 : 360;
	}

	// Ry == 90
	if (endEuler[1] == 90) {
		equivEuler[0] = 0;
		equivEuler[2] = endEuler[2] - endEuler[0];
	}
	// Ry == -90
	else if (endEuler[1] == -90) {
		equivEuler[0] = 0;
		equivEuler[2] = endEuler[2] + endEuler[0];
	}
	else {
		equivEuler[0] = endEuler[0] + 180;
		equivEuler[1] = 180 - endEuler[1];
		equivEuler[2] = endEuler[2] + 180;
	}
	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(equivEuler[i]) - 180 > 0) {
			equivEuler[i] += equivEuler[i] > 0 ? -360 : 360;
		}
	}

	return equivEuler;
}

/**
* @brief  计算两组 Rzyx 欧拉角之间的相对运动距离
* @param  begEuler           起点欧拉角 <Rx, Ry, Rz>
* @param  endEuler           绕 y 转角(deg)
* @param  chooseMimumDist    绕 z 转角(deg)
* @return 相对运动距离
*/
template <typename T>
Eigen::Matrix<T, 3, 1> get_zyx_euler_distance(Eigen::Matrix<T, 3, 1>& begEuler, Eigen::Matrix<T, 3, 1>& endEuler, bool chooseMimumDist = 1) {
	Eigen::Matrix<T, 3, 1> ans(0, 0, 0);

	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(begEuler[i]) - 180 > 0) {
			begEuler[i] += begEuler[i] > 0 ? -360 : 360;
		}
		while (std::fabs(endEuler[i]) - 180 > 0) {
			endEuler[i] += endEuler[i] > 0 ? -360 : 360;
		}
	}

	// equivalent of endEuler
	Eigen::Matrix<T, 3, 1> equivEuler = get_equivalent_zyx_euler(endEuler);
	if (endEuler[1] == 90) {
		equivEuler[0] = (equivEuler[2] > 0) ? (std::min)(begEuler[0], begEuler[2]) : (std::max)(begEuler[0], begEuler[2]);
		equivEuler[2] += equivEuler[0];
	}
	else if (endEuler[1] == -90) {
		T sumQ = endEuler[2] + endEuler[0];
		equivEuler[0] = begEuler[0];
		equivEuler[2] -= equivEuler[0];
	}
	// wrap to (-pi, pi]
	for (size_t i = 0; i < 3; ++i) {
		while (std::fabs(equivEuler[i]) - 180 > 0) {
			equivEuler[i] += equivEuler[i] > 0 ? -360 : 360;
		}
	}

	// 欧拉角相对值
	Eigen::Matrix<T, 3, 1> endRel(0, 0, 0), equivRel(0, 0, 0);
	T endSum = 0.0, equivSum = 0.0;
	for (size_t i = 0; i < 3; ++i) {
		T directDist = endEuler[i] - begEuler[i];
		T hopDist = 360 - std::fabs(endEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			endRel[i] = directDist;
		}
		else {
			// 此时 begEuler[i] ！= 0 成立
			endRel[i] = begEuler[i] < 0 ? -hopDist : hopDist;
		}
		endSum += std::fabs(endRel[i]);

		directDist = equivEuler[i] - begEuler[i];
		hopDist = 360 - std::fabs(equivEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			equivRel[i] = directDist;
		}
		else {
			equivRel[i] = begEuler[i] < 0 ? -hopDist : hopDist;
		}
		equivSum += std::fabs(equivRel[i]);
	}

	// 选择最短 / 最长的相对运动距离
	if (chooseMimumDist) {
		//ans = endSum < equivSum ? endRel : equivRel;
		if (endSum < equivSum) {
			ans = endRel;
		}
		else {
			ans = equivRel;
			endEuler = equivEuler;
		}
	}
	else {
		//ans = endSum < equivSum ? equivRel : endRel;
		if (endSum > equivSum) {
			ans = endRel;
		}
		else {
			ans = equivRel;
			endEuler = equivEuler;
		}
	}

	return ans;
}


template <typename T = float>
class DiscreteTrajectory {
public:
	//! 轨迹端点 <位置, Rzyx(deg), 附加轴>
	std::list<std::vector<T>> nodePoint;
	//! 轨迹中间点 <位置, Rzyx(deg), 附加轴>
	std::list<std::vector<T>> midPoint;
	//! 轨迹信息    直线    方向(0:2),    -直线长度(3), null(4:6),     速度(7)
	//              圆弧    圆心坐标(0:2),+圆弧长度(3), 旋转矢量(4:6), 速度(7)
	std::list<std::vector<T>> trajInfo;

	//! 摆焊参数
	std::list<Weave> waveInfo;
	//! 跟踪参数
	std::list<Track> trackInfo;
	//! 焊接参数
	std::list<Arc_WeldingParaItem> weldInfo;

public:
	DiscreteTrajectory() {};
	~DiscreteTrajectory() {};

	/**
	* @brief  清除数据并重新记录起点
	* @param  pnt    起点数据
	*/
	uint8_t set_starting_point(const std::vector<T>& pnt = {}) {

		nodePoint.clear();
		midPoint.clear();
		trajInfo.clear();

		nodePoint.push_back(pnt);

		return 0;
	}

	uint8_t reset_starting_point(const std::vector<T>& pnt = {}) {
		return 0;
	}

	/**
	* @brief  记录直线轨迹
	* @param  pnt    直线终点数据
	*/
	uint8_t add_line(const std::vector<T>& pnt, T vel = 10, T smooth = -1) {
		if (nodePoint.size() < 1) return 1;

		size_t N = nodePoint.front().size(), num = (std::min)(N, pnt.size());

		// 添加轨迹点
		std::vector<T> tmp(N, 0), cur = nodePoint.back();
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = pnt[i];
		}
		nodePoint.push_back(tmp);

		// 添加中间点
		for (size_t i = 0; i < N; ++i) {
			tmp[i] = (cur[i] + tmp[i]) / 2;
		}
		for (size_t i = 3; i < 6; ++i) {
			tmp[i] = cur[i];
		}
		midPoint.push_back(tmp);

		// 计算轨迹信息
		std::vector<T> info(9, 0);
		Eigen::Matrix<T, 3, 1> curPos(cur[0], cur[1], cur[2]), endPos(pnt[0], pnt[1], pnt[2]);

		// 直线方向
		Eigen::Matrix<T, 3, 1> dir = (endPos - curPos).normalized();
		for (size_t i = 0; i < 3; ++i) {
			info[i] = dir[i];
		}
		// 直线长度
		info[3] = -1 * (endPos - curPos).norm();
		// 轨迹速度
		info[7] = vel;
		// 平滑度
		info[8] = smooth;
		trajInfo.push_back(info);
		
		return 0;
	}
	uint8_t add_line(const std::vector<T>& pnt, const Weave& waveCfg, const Track& trackCfg, const Arc_WeldingParaItem& weldCfg) {
		add_line(pnt);

		waveInfo.push_back(waveCfg);
		trackInfo.push_back(trackCfg);
		weldInfo.push_back(weldCfg);
	}

	/**
	* @brief  记录圆弧轨迹
	* @param  end    圆弧终点
	* @param  mid    圆弧中间点
	*/
	uint8_t add_arc(const std::vector<T>& end, const std::vector<T>& mid, T vel = 10, T smooth = -1) {
		if (nodePoint.size() < 1) return 1;
		
		size_t N = nodePoint.front().size(), num = (std::min)(N, end.size());

		// 添加轨迹点
		std::vector<T> tmp(N, 0), cur = nodePoint.back();
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = end[i];
		}
		nodePoint.push_back(tmp);

		// 添加中间点
		tmp = std::vector<T>(N, 0);
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = mid[i];
		}
		midPoint.push_back(tmp);

		// 计算轨迹信息
		Eigen::Matrix<T, 3, 1> begPnt(cur[0], cur[1], cur[2]), midPnt(mid[0], mid[1], mid[2]), endPnt(end[0], end[1], end[2]);
		Eigen::Matrix<T, 7, 1> arcInfo = construct_arc_trajectory(begPnt, midPnt, endPnt);
		std::vector<T> info(9);
		// 轨迹形状
		for (size_t i = 0; i < 7; ++i) {
			info[i] = arcInfo[i];
		}
		// 轨迹速度
		info[7] = vel;
		// 平滑度
		info[8] = smooth;

		trajInfo.push_back(info);

		return 0;
	}
	uint8_t add_arc(const std::vector<T>& end, const std::vector<T>& mid, const Weave& waveCfg, const Track& trackCfg, const Arc_WeldingParaItem& weldCfg) {
		add_arc(end, mid);

		waveInfo.push_back(waveCfg);
		trackInfo.push_back(trackCfg);
		weldInfo.push_back(weldCfg);
	}
	/**
	* @brief  拐角过渡
	* @param  end    圆弧终点
	* @param  mid    圆弧中间点
	*/
	uint8_t corner_transition(T inDist = -1, T outDist = -1) {
		if (nodePoint.size() < 1) return 1;

		size_t N = nodePoint.front().size();
		// N < 6 时不支持该函数
		if (N < 6) return 2;

		// 节点迭代器
		auto endPntIte = nodePoint.begin(), begPntIte = endPntIte++;
		// 中间点迭代器
		auto midPntIte = midPoint.begin();
		// 轨迹信息迭代器
		auto infoIte = trajInfo.begin();

		while (infoIte != trajInfo.end()) {
			auto transPnt = transition_interpolate(*begPntIte, *endPntIte, *infoIte, { inDist, inDist/2, std::fabs((*infoIte)[3])/2, -outDist/2, -outDist });
			// 插入路径点
			nodePoint.insert(endPntIte, transPnt.front());
			nodePoint.insert(endPntIte, transPnt.back());
			begPntIte = endPntIte++;
			// 插入中间点
			midPoint.insert(midPntIte++, transPnt[1]);
			midPoint.insert(midPntIte, transPnt[3]);

			// 起点过渡轨迹
			std::vector<T> begTraj(9, 0);
			for (size_t i = 0; i < 3; ++i) {
				begTraj[i] = (*infoIte)[i];
			}
			begTraj[3] = -inDist;
			begTraj[7] = (*infoIte)[7];

			// 终点过渡轨迹
			std::vector<T> endTraj(9, 0);
			for (size_t i = 0; i < 3; ++i) {
				endTraj[i] = (*infoIte)[i];
			}
			endTraj[3] = -outDist;
			endTraj[7] = (*infoIte)[7];

			// 插入起点过渡轨迹信息
			trajInfo.insert(infoIte, begTraj);
			// 原始轨迹信息修改
			(*infoIte++)[3] -= begTraj[3] + endTraj[3];
			// 插入终点过渡轨迹信息
			trajInfo.insert(infoIte, endTraj);

		}
		
		//begPntIte = nodePoint.begin();
		//// 上一位姿
		//Eigen::Matrix<T, N, 1> prePnt = (*begPntIte++);
		//std::cout << "curPnt = " << prePnt.transpose() << std::endl;

		//Eigen::Matrix<T, 3, 1> begEuler(prePnt[3], prePnt[4], prePnt[5]), endEuler = (*begPntIte).segment<3>(3);

		//while (begPntIte != nodePoint.end()) {
		//	// 当前位姿
		//	Eigen::Matrix<T, N, 1> curPnt = (*begPntIte++);
		//	std::cout << "curPnt = " << curPnt.transpose() << std::endl;
		//	std::cout << curPnt[0] - prePnt[0] << ", " << curPnt[1] - prePnt[1] << ", " << curPnt[2] - prePnt[2] << ", ";
		//	endEuler = curPnt.segment<3>(3);
		//	// 欧拉角相对值
		//	Eigen::Matrix<T, 3, 1> relEuler = get_zyx_euler_distance(begEuler, endEuler);
		//	for (size_t i = 0; i < 3; ++i) {
		//		std::cout << relEuler[i] << ", ";
		//	}
		//	begEuler = endEuler;

		//	prePnt = curPnt;
		//	std::cout << "\n" << std::endl;
		//}
	}

	/**
	* @brief  拐角减速
	* @param  angularVel    轨迹最大角速度，欧拉角矢量速度
	*/
	uint8_t corner_slowdown(T angularVel) {
		// 遍历轨迹
		auto nodePointIte = nodePoint.begin();
		auto midPointIte = midPoint.begin();
		auto infoIte = trajInfo.begin();

		while (infoIte != trajInfo.end()) {
			std::vector<float> curBuffer = *(nodePointIte++);

			// 欧拉角转换到相对运动: beg -> mid -> end
			Eigen::Vector3f begEuler = Eigen::Vector3f(curBuffer[3], curBuffer[4], curBuffer[5]);
			Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
			Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
			begEuler = endEuler;
			endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
			relEuler += get_zyx_euler_distance(begEuler, endEuler);

			T sumAngle = std::sqrt(relEuler[0]* relEuler[0] + relEuler[1]* relEuler[1] + relEuler[2]* relEuler[2]);
			// 速度修正
			if (sumAngle / angularVel > std::fabs((*infoIte)[3]) / (*infoIte)[7]) {
				(*infoIte)[7] = angularVel * std::fabs((*infoIte)[3]) / sumAngle;
			}

			midPointIte++;
			infoIte++;
		}

		return 0;
	}
	
	/**
	* @brief  均匀分隔
	* @param  separation    间隔宽度
	*/
	uint8_t equally_divide(const std::vector<T>& separation) {
		// 分段总距离
		T phaseDist = 0.0;
		for (size_t i = 0; i < separation.size(); ++i) {
			phaseDist += separation[i];
		}

		// 遍历轨迹
		auto nodePointIte = nodePoint.begin();
		auto midPointIte = midPoint.begin();
		auto infoIte = trajInfo.begin();

		Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
		while (infoIte != trajInfo.end()) {
			std::vector<T> begBuffer = *(nodePointIte++);
			// 当前点距离
			T curDist = 0.0, midDist = 0.0, trajDist = std::fabs((*infoIte)[3]);

			std::vector<T> relEndMove(begBuffer.size(), 0);
			// 终点到起点的相对运动
			for (size_t i = 0; i < begBuffer.size(); ++i) {
				relEndMove[i] = (*nodePointIte)[i] - begBuffer[i];
			}

			// 欧拉角相对距离
			begEuler = Eigen::Vector3f(begBuffer[3], begBuffer[4], begBuffer[5]);
			endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
			Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
			begEuler = endEuler;

			// 完整周期个数
			int numPeriod = std::floor(std::fabs((*infoIte)[3]) / phaseDist);
			bool isRedundant = true;
			for (size_t i = 0; i < numPeriod + 1; ++i) {
				if (!isRedundant)
					break;
				for (size_t j = 0; j < separation.size(); ++j) {
					// 不够添加新的轨迹
					if (std::fabs((*infoIte)[3]) < separation[j]) {
						isRedundant = false;
						break;
					}

					curDist += separation[j];
					midDist = curDist - separation[j] / 2;
					
					std::vector<T> sepaPoint(begBuffer.size(), 0), curMidPoint(begBuffer.size(), 0), sepaTraj = *infoIte;
					// 间隔点位置
					Eigen::Matrix<T, 3, 1> curPos(0,0,0), midPos(0,0,0), endMidPos(0,0,0);

					if ((*infoIte)[3] < 0) {
						Eigen::Matrix<T, 3, 1> begPos(begBuffer[0], begBuffer[1], begBuffer[2]);
						Eigen::Matrix<T, 3, 1> dir((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
						curPos = begPos + dir * curDist;
						midPos = begPos + dir * midDist;
						endMidPos = begPos + dir * (trajDist + curDist) / 2;
						// 新增轨迹信息
						sepaTraj[3] = -separation[j];
						// 原始轨迹修改
						(*infoIte)[3] += separation[j];
					}
					else if ((*infoIte)[3] > 0) {
						Eigen::Matrix<T, 3, 1> radius(begBuffer[0] - (*infoIte)[0], begBuffer[1] - (*infoIte)[1], begBuffer[2] - (*infoIte)[2]);
						Eigen::Matrix<T, 3, 1> center((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
						Eigen::Matrix<T, 3, 1> norm((*infoIte)[4], (*infoIte)[5], (*infoIte)[6]);
						T theta = trajDist / radius.norm();
						norm.normalize();
						curPos = Eigen::AngleAxis<T>(theta * curDist / trajDist, norm) * radius + center;
						midPos = Eigen::AngleAxis<T>(theta * midDist / trajDist, norm) * radius + center;
						endMidPos = Eigen::AngleAxis<T>(theta * (trajDist + curDist) / 2 / trajDist, norm) * radius + center;
						// 新增轨迹信息
						sepaTraj[3] = separation[j];
						sepaTraj[4] = norm[0] * separation[j] / trajDist * theta;
						sepaTraj[5] = norm[1] * separation[j] / trajDist * theta;
						sepaTraj[6] = norm[2] * separation[j] / trajDist * theta;
						// 原始轨迹修改
						(*infoIte)[3] -= separation[j];
						(*infoIte)[4] = norm[0] * (trajDist - curDist) / trajDist * theta;
						(*infoIte)[5] = norm[1] * (trajDist - curDist) / trajDist * theta;
						(*infoIte)[6] = norm[2] * (trajDist - curDist) / trajDist * theta;
					}
					for (size_t k = 0; k < 3; ++k) {
						sepaPoint[k] = curPos[k];
						curMidPoint[k] = midPos[k];
						// 原始轨迹中间点修改
						(*midPointIte)[k] = endMidPos[k];
					}
					// 间隔点欧拉角
					for (size_t k = 3; k < 6; ++k) {
						sepaPoint[k] = begBuffer[k] + curDist / std::fabs((*infoIte)[3]) * relEuler[k - 3];
						curMidPoint[k] = begBuffer[k] + midDist / std::fabs((*infoIte)[3]) * relEuler[k - 3];
						(*midPointIte)[k] = begBuffer[k] + (trajDist + curDist) / 2 / std::fabs((*infoIte)[3]) * relEuler[k - 3];
					}
					for (size_t k = 6; k < begBuffer.size(); ++k) {
						sepaPoint[k] = begBuffer[k] + curDist / trajDist * relEndMove[k];
						curMidPoint[k] = begBuffer[k] + midDist / trajDist * relEndMove[k];
						(*midPointIte)[k] = begBuffer[k] + (trajDist + curDist) / 2 / trajDist * relEndMove[k];
					}

					nodePoint.insert(nodePointIte, sepaPoint);
					midPoint.insert(midPointIte, curMidPoint);
					trajInfo.insert(infoIte, sepaTraj);
				}
			}

			midPointIte++;
			infoIte++;
		}

		return 0;
	}

	/**
	* @brief  拐角过渡点插值
	* @param  begPnt      轨迹起点
	* @param  endPnt      轨迹终点
	* @param  info        轨迹信息
	* @param  distList    过渡点距离数组, + 从起点计算; - 从终点计算
	* @return 过渡点位置数组
	*/
	std::vector<std::vector<T>> transition_interpolate(const std::vector<T>& begPnt, const std::vector<T>& endPnt, const std::vector<T>& info, const std::vector<T>& distList) {
		// 返回值
		std::vector<std::vector<T>> ans(distList.size(), std::vector<T>(begPnt.size(), 0));

		// 起点处 TCP 姿态
		Eigen::Matrix<T, 3, 3> begMat = RzyxToRotMat(begPnt[5], begPnt[4], begPnt[3]);
		// 终点处 TCP 姿态
		Eigen::Matrix<T, 3, 3> endMat = RzyxToRotMat(endPnt[5], endPnt[4], endPnt[3]);
		// 切线方向
		Eigen::Matrix<T, 3, 1> infoTail3(info[4], info[5], info[6]), infoHead3(info[0], info[1], info[2]);
		Eigen::Matrix<T, 3, 1> begHead3(begPnt[0], begPnt[1], begPnt[2]), endHead3(endPnt[0], endPnt[1], endPnt[2]);
		Eigen::Matrix<T, 3, 1> begTan = info[3] > 0 ? (infoTail3.cross(begHead3 - infoHead3)).normalized() : infoHead3;
		Eigen::Matrix<T, 3, 1> endTan = info[3] > 0 ? (infoTail3.cross(endHead3 - infoHead3)).normalized() : infoHead3;
		// 法线方向
		Eigen::Matrix<T, 3, 1> begNorm = begTan.cross(begMat.col(2)).normalized();
		Eigen::Matrix<T, 3, 1> endNorm = endTan.cross(endMat.col(2)).normalized();
		// 标准焊枪方向
		Eigen::Matrix<T, 3, 1> begUprightDir = begNorm.cross(begTan).normalized();
		Eigen::Matrix<T, 3, 1> endUprightDir = endNorm.cross(endTan).normalized();
		// 起点焊枪方向转换到终点
		// todo: Eigen::AngleAxis<T>(1, infoTail3)
		Eigen::Matrix<T, 3, 1> begUprightDir_end = info[3] > 0 ? (Eigen::AngleAxis<T>(infoTail3.norm(), infoTail3.normalized())*begUprightDir).eval() : begUprightDir;

		// 俯仰角变化: 绕切线方向, 默认小角度变化
		T dotProd = begUprightDir_end.dot(endUprightDir);
		dotProd = std::fabs(dotProd) > 1 ? (dotProd / std::fabs(dotProd)) : dotProd;
		T pitchAngle = std::acos(dotProd);
		pitchAngle *= (begUprightDir_end.cross(endUprightDir).dot(endTan) < 0) ? -1 : 1;

		//std::cout << "\n===========\nbegMat = \n" << begMat << std::endl;
		//std::cout << "endMat = \n" << endMat << "\n===========\n" << std::endl;
		for (size_t i = 0; i < distList.size(); ++i) {
			
			// 过渡点
			std::vector<T> transPnt(begPnt.size(), 0);
			// 过渡点姿态
			Eigen::Matrix<T, 3, 3> transMat = Eigen::Matrix<T, 3, 3>::Identity();
			// 过渡点距离占比
			T lambda = 0.0;

			// 从起点算起
			if (distList[i] > 0) {

				//std::cout << "\nNodeMat = \n" << begMat << std::endl;
				//std::cout << "NodePnt = " << begPnt.transpose() << std::endl;

				T inDist = distList[i];
				// 推拉角纠正: 绕法线方向, 默认小角度变化
				transMat.col(2) = begUprightDir;
				transMat.col(1) = begMat.col(1).dot(begTan) >= 0 ? begTan : -begTan;
				transMat.col(0) = (transMat.col(1).cross(transMat.col(2))).normalized();
				// 俯仰角修正
				transMat = (Eigen::AngleAxis<T>(pitchAngle * inDist / std::fabs(info[3]), begTan) * transMat).eval();

				// 直线
				if (info[3] < 0) {
					Eigen::Matrix<T, 3, 1> dir = begHead3 + begTan * inDist;
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
				}
				// 圆弧
				else if (info[3] > 0) {
					Eigen::Matrix<T, 3, 1> center = infoHead3 + Eigen::AngleAxis<T>(inDist / std::fabs(info[3]), infoTail3) * (begHead3 - infoHead3);
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = center[i];
					}
					transMat = (Eigen::AngleAxis<T>(inDist / std::fabs(info[3]), infoTail3) * transMat).eval();
				}

				lambda = inDist / std::fabs(info[3]);
			}
			// 从终点算起
			else if (distList[i] < 0) {
				T outDist = -distList[i];

				// 推拉角纠正: 绕法线方向, 默认小角度变化
				transMat.col(2) = endUprightDir;
				transMat.col(1) = endMat.col(1).dot(endTan) >= 0 ? endTan : -endTan;
				transMat.col(0) = (transMat.col(1).cross(transMat.col(2))).normalized();
				// 俯仰角修正
				transMat = (Eigen::AngleAxis<T>(pitchAngle * outDist / std::fabs(info[3]), -endTan) * transMat).eval();

				// 直线
				if (info[3] < 0) {
					Eigen::Matrix<T, 3, 1> dir = endHead3 - endTan * outDist;
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
				}
				// 圆弧
				else if (info[3] > 0) {
					Eigen::Matrix<T, 3, 1> center = infoHead3 + Eigen::AngleAxis<T>(outDist / std::fabs(info[3]), -infoTail3) * (endHead3 - infoHead3);
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = center[i];
					}
					transMat = (Eigen::AngleAxis<T>(outDist / std::fabs(info[3]), -infoTail3) * transMat).eval();
				}

				lambda = 1.0 - outDist / std::fabs(info[3]);
			}

			// 过渡点欧拉角
			Eigen::Matrix<T, 3, 1> transEuler = transMat.eulerAngles(2, 1, 0).reverse() * 180 / DT_PI;
			for (size_t j = 0; j < 3; ++j) {
				transPnt[3 + j] = transEuler[j];
			}
			// 附加轴插补
			for (size_t j = 6; j < begPnt.size(); ++j) {
				transPnt[j] = begPnt[j] + (endPnt[j] - begPnt[j]) * lambda;
			}
			ans[i] = transPnt;
		}

		return ans;
	}

private:
	// 坐标轴单位向向量
	static const Eigen::Matrix<T, 3, 1> unitX, unitY, unitZ;
	static const T DT_PI;

	/**
	* @brief  空间三点构建圆弧轨迹
	* @param  beg    起点坐标
	* @param  mid    中间点坐标
	* @param  end    终点坐标
	* @return 圆弧轨迹信息
	*/
	Eigen::Matrix<T, 7, 1> construct_arc_trajectory(Eigen::Matrix<T, 3, 1> beg, Eigen::Matrix<T, 3, 1> mid, Eigen::Matrix<T, 3, 1> end) {
		Eigen::Matrix<T, 7, 1> info;

		Eigen::Matrix<T, 3, 1> a = beg - mid, b = end - mid;
		// 当直线处理
		if (a.cross(b).squaredNorm() < 1e-12) {
			info.head(3) = (end - beg).normalized();
			info(3) = -1 * (end - beg).norm();
			return info;
		}

		// 圆心位置
		info.head(3) = (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + mid;
		// 半径方向
		Eigen::Matrix<T, 3, 1> op1 = (beg - info.head(3)).normalized(), op2 = (mid - info.head(3)).normalized(), op3 = (end - info.head(3)).normalized();
		// 半径夹角
		T q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
		// 法线方向
		Eigen::Matrix<T, 3, 1> n12 = op1.cross(op2), n13 = op1.cross(op3);
		// 圆弧运动平面的法线方向
		Eigen::Matrix<T, 3, 1> normal = op1.cross(op3);
		// 圆心角角度
		float theta = std::acos(op1.dot(op3));
		// 修正圆心角和正法向
		// 2,3 在 1 的两侧
		if (n12.dot(n13) < 0) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}
		// q12 > q13
		else if (q12 > q13) {
			normal = -n13;
			theta = 2 * DT_PI - theta;
		}
		// q13 > q12
		else if (q13 > q12) {
			normal = n12;
		}

		// 圆弧长度
		info(3) = (beg - info.head(3)).norm() * theta;
		// 附加旋转角度的法线方向
		info.tail(3) = normal.normalized() * theta;

		return info;
	}

	/**
	* @brief  XYZ 欧拉角转旋转矩阵
	* @param  qz    绕 z 转角(deg)
	* @param  qy    绕 y 转角(deg)
	* @param  qx    绕 x 转角(deg)
	*/
	Eigen::Matrix<T, 3, 3> RzyxToRotMat(T qz, T qy, T qx) {
		return (Eigen::AngleAxis<T>(qz * DT_PI / 180, unitZ) * Eigen::AngleAxis<T>(qy * DT_PI / 180, unitY) * Eigen::AngleAxis<T>(qx * DT_PI / 180, unitX)).matrix();
	}

	/**
	* @brief  ZYX 欧拉角转旋转矩阵
	* @param  qx    绕 x 转角(deg)
	* @param  qy    绕 y 转角(deg)
	* @param  qz    绕 z 转角(deg)
	*/
	Eigen::Matrix<T, 3, 3> RxyzToRotMat(T qx, T qy, T qz) {
		return (Eigen::AngleAxis<T>(qx * DT_PI / 180, unitX) * Eigen::AngleAxis<T>(qy * DT_PI / 180, unitY) * Eigen::AngleAxis<T>(qz * DT_PI / 180, unitZ)).matrix();
	}
};

template <typename T>
const T DiscreteTrajectory<T>::DT_PI = 3.14159265358979323846;
// 坐标轴单位向向量
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitX = Eigen::Matrix<T, 3, 1>(1, 0, 0);
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitY = Eigen::Matrix<T, 3, 1>(0, 1, 0);
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitZ = Eigen::Matrix<T, 3, 1>(0, 0, 1);


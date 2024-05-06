#pragma once

#include<vector>
#include<list>
#include<eigen3/Eigen/Dense>

#include<iostream>

template <typename T = float, size_t N = 6>
class DiscreteTrajectory {
public:
	//! 轨迹端点 <位置, Rzyx(deg), 附加轴>
	std::list<Eigen::Matrix<T, N, 1>> nodePoint;
	//! 轨迹中间点 <位置, Rzyx(deg), 附加轴>
	std::list<Eigen::Matrix<T, N, 1>> midPoint;
	//! 轨迹信息
	std::list<Eigen::Matrix<T, 7, 1>> trajInfo;

public:
	DiscreteTrajectory() {};
	~DiscreteTrajectory() {};

	/**
	* @brief  清除数据并重新记录起点
	* @param  pnt    起点数据
	*/
	uint8_t set_starting_point(const std::vector<T>& pnt = std::vector<T>(0, N)) {
		if (pnt.size() < N) return 2;

		nodePoint.clear();
		midPoint.clear();
		trajInfo.clear();

		Eigen::Matrix<T, N, 1> tmp;
		for (size_t i = 0; i < N; ++i) {
			tmp(i) = pnt[i];
		}
		nodePoint.push_back(tmp);

		return 0;
	}

	uint8_t reset_starting_point(const std::vector<T>& pnt = std::vector<T>(0, N)) {
		return 0;
	}

	/**
	* @brief  记录直线轨迹
	* @param  pnt    直线终点数据
	*/
	uint8_t add_line(const std::vector<T>& pnt) {
		if (nodePoint.size() < 1) return 1;
		if (pnt.size() < N) return 2;

		// 添加轨迹点
		Eigen::Matrix<T, N, 1> tmp, cur = nodePoint.back();
		for (size_t i = 0; i < N; ++i) {
			tmp(i) = pnt[i];
		}
		nodePoint.push_back(tmp);

		midPoint.push_back((cur+tmp)/2);

		// 计算轨迹信息
		Eigen::Matrix<T, 7, 1> info = Eigen::Matrix<T, 7, 1>::Zero();
		Eigen::Matrix<T, 3, 1> curPos(cur[0], cur[1], cur[2]), endPos(pnt[0], pnt[1], pnt[2]);
		info.head(3) = (endPos - curPos).normalized();
		// 直线长度
		info(3) = -1 * (endPos - curPos).norm();
		trajInfo.push_back(info);
		

		return 0;
	}

	/**
	* @brief  记录圆弧轨迹
	* @param  end    圆弧终点
	* @param  mid    圆弧中间点
	* @todo   法向量计算
	*/
	uint8_t add_arc(const std::vector<T>& end, const std::vector<T>& mid) {
		if (nodePoint.size() < 1) return 1;
		if (end.size() < N || mid.size() < N) return 2;

		// 添加轨迹点
		Eigen::Matrix<T, N, 1> tmp, cur = nodePoint.back();
		for (size_t i = 0; i < N; ++i) {
			tmp(i) = end[i];
		}
		nodePoint.push_back(tmp);

		for (size_t i = 0; i < N; ++i) {
			tmp(i) = mid[i];
		}
		midPoint.push_back(tmp);

		// 计算轨迹信息
		Eigen::Matrix<T, 7, 1> info = construct_arc_trajectory(cur.head(3), midPoint.back().head(3), nodePoint.back().head(3));

		trajInfo.push_back(info);

		return 0;
	}

	/**
	* @brief  拐角过渡
	* @param  end    圆弧终点
	* @param  mid    圆弧中间点
	*/
	uint8_t corner_transition(T inDist = -1, T outDist = -1) {
		if (nodePoint.size() < 1) return 1;
		// N < 6 时不支持该函数
		if (N < 6) return 2;

		// 节点迭代器
		std::list<Eigen::Matrix<T, N, 1>>::iterator endPntIte = nodePoint.begin(), begPntIte = endPntIte++;
		// 中间点迭代器
		std::list<Eigen::Matrix<T, N, 1>>::iterator midPntIte = midPoint.begin();
		// 轨迹信息迭代器
		std::list<Eigen::Matrix<T, 7, 1>>::iterator infoIte = trajInfo.begin();

		while (infoIte != trajInfo.end()) {
			std::vector< Eigen::Matrix<T, N, 1>> transPnt = transition_interpolate(*begPntIte, *endPntIte, *infoIte, { inDist, inDist/2, std::fabs((*infoIte)(3))/2, -outDist/2, -outDist });
			// 插入路径点
			nodePoint.insert(endPntIte, transPnt.front());
			nodePoint.insert(endPntIte, transPnt.back());
			begPntIte = endPntIte++;
			// 插入中间点
			midPoint.insert(midPntIte++, transPnt[1]);
			midPoint.insert(midPntIte, transPnt[3]);

			// 起点过渡轨迹
			Eigen::Matrix<T, 7, 1> begTraj = Eigen::Matrix<T, 7, 1>::Zero();
			begTraj.head(3) = (*infoIte).head(3);
			begTraj(3) = -inDist;
			// 终点过渡轨迹
			Eigen::Matrix<T, 7, 1> endTraj = Eigen::Matrix<T, 7, 1>::Zero();
			endTraj.head(3) = (*infoIte).head(3);
			endTraj(3) = -outDist;

			// 插入轨迹信息
			trajInfo.insert(infoIte, begTraj);
			(*infoIte++)(3) -= begTraj(3) + endTraj(3);
			trajInfo.insert(infoIte, endTraj);
		}
		
		begPntIte = nodePoint.begin();
		// 上一位姿
		Eigen::Matrix<T, N, 1> prePnt = (*begPntIte++);
		std::cout << "curPnt = " << prePnt.transpose() << std::endl;

		while (begPntIte != nodePoint.end()) {
			// 当前位姿
			Eigen::Matrix<T, N, 1> curPnt = (*begPntIte++);
			std::cout << "curPnt = " << curPnt.transpose() << std::endl;
			// 位置相对值
			std::cout << curPnt[0]-prePnt[0] << ", " << curPnt[1] - prePnt[1] << ", " << curPnt[2] - prePnt[2] << ", ";
			// 欧拉角相对值
			Eigen::Matrix<T, 3, 1> begEuler(prePnt[3], prePnt[4], prePnt[5]), endEuler(curPnt[3], curPnt[4], curPnt[5]);
			Eigen::Matrix<T, 3, 1> relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				std::cout << relEuler[i] << ", ";
			}

			prePnt = curPnt;
			std::cout << "\n" << std::endl;

			// 绝对值
			//std::cout << (*begPntIte)[0] << ", " << (*begPntIte)[1] << ", " << (*begPntIte)[2] << ", ";
			//std::cout << (*begPntIte)[3] << ", " << (*begPntIte)[4] << ", " << (*begPntIte)[5] << std::endl;
			//begPntIte++;
		}
	}

	/**
	* @brief  拐角过渡点插值
	* @param  begPnt      轨迹起点
	* @param  endPnt      轨迹终点
	* @param  info        轨迹信息
	* @param  distList    过渡点距离数组, + 从起点计算; - 从终点计算
	* @return 过渡点位置数组
	*/
	std::vector< Eigen::Matrix<T, N, 1>>
	transition_interpolate(const Eigen::Matrix<T, N, 1>& begPnt, const Eigen::Matrix<T, N, 1>& endPnt, const Eigen::Matrix<T, 7, 1>& info, std::vector<T> distList) {
		// 返回值
		std::vector<Eigen::Matrix<T, N, 1>> ans(distList.size(), Eigen::Matrix<T, N, 1>::Zero());

		// 起点处 TCP 姿态
		Eigen::Matrix<T, 3, 3> begMat = RzyxToRotMat(begPnt[5], begPnt[4], begPnt[3]);
		// 终点处 TCP 姿态
		Eigen::Matrix<T, 3, 3> endMat = RzyxToRotMat(endPnt[5], endPnt[4], endPnt[3]);
		// 切线方向
		Eigen::Matrix<T, 3, 1> begTan = info(3) > 0 ? (info.tail<3>().cross(begPnt.head<3>() - info.head<3>())).normalized() : info.head<3>();
		Eigen::Matrix<T, 3, 1> endTan = info(3) > 0 ? (info.tail<3>().cross(endPnt.head<3>() - info.head<3>())).normalized() : info.head<3>();
		// 法线方向
		Eigen::Matrix<T, 3, 1> begNorm = begTan.cross(begMat.col(2)).normalized();
		Eigen::Matrix<T, 3, 1> endNorm = endTan.cross(endMat.col(2)).normalized();
		// 标准焊枪方向
		Eigen::Matrix<T, 3, 1> begUprightDir = begNorm.cross(begTan).normalized();
		Eigen::Matrix<T, 3, 1> endUprightDir = endNorm.cross(endTan).normalized();
		// 起点焊枪方向转换到终点
		Eigen::Matrix<T, 3, 1> begUprightDir_end = info(3) > 0 ? (Eigen::AngleAxis<T>(info.tail(3).norm(), info.tail(3).normalized())*begUprightDir).eval() : begUprightDir;

		// 俯仰角变化: 绕切线方向, 默认小角度变化
		T dotProd = begUprightDir_end.dot(endUprightDir);
		dotProd = std::fabs(dotProd) > 1 ? (dotProd / std::fabs(dotProd)) : dotProd;
		T pitchAngle = std::acos(dotProd);
		pitchAngle *= (begUprightDir_end.cross(endUprightDir).dot(endTan) < 0) ? -1 : 1;

		std::cout << "\n===========>\nbegMat = \n" << begMat << std::endl;
		std::cout << "endMat = \n" << endMat << std::endl;
		for (size_t i = 0; i < distList.size(); ++i) {
			
			// 过渡点
			Eigen::Matrix<T, N, 1> transPnt = Eigen::Matrix<T, N, 1>::Zero();
			// 过渡点姿态
			Eigen::Matrix<T, 3, 3> transMat = Eigen::Matrix<T, 3, 3>::Identity();
			// 过渡点距离占比
			T lambda = 0.0;

			// 从起点算起
			if (distList[i] > 0) {

				std::cout << "\nNodeMat = \n" << begMat << std::endl;
				std::cout << "NodePnt = " << begPnt.transpose() << std::endl;

				T inDist = distList[i];
				// 推拉角纠正: 绕法线方向, 默认小角度变化
				dotProd = begMat.col(2).dot(begUprightDir);
				dotProd = std::fabs(dotProd) > 1 ? (dotProd / std::fabs(dotProd)) : dotProd;
				T thrustAngle = std::acos(dotProd);
				thrustAngle *= begMat.col(2).cross(begUprightDir).dot(begNorm) < 0 ? -1 : 1;
				transMat = Eigen::AngleAxis<T>(pitchAngle * inDist / std::fabs(info(3)), begTan) * Eigen::AngleAxis<T>(thrustAngle, begNorm) * begMat;

				// 直线
				if (info(3) < 0) {
					transPnt.head(3) = begPnt.head(3) + begTan * inDist;
				}
				// 圆弧
				else if (info(3) > 0) {
					transPnt.head(3) = info.head(3) + Eigen::AngleAxis<T>(inDist / std::fabs(info(3)), info.tail(3)) * (begPnt.head(3)-info.head(3));
					transMat = (Eigen::AngleAxis<T>(inDist / std::fabs(info(3)), info.tail(3)) * transMat).eval();
				}

				lambda = inDist / std::fabs(info(3));
			}
			// 从终点算起
			else if (distList[i] < 0) {
				T outDist = -distList[i];

				// 推拉角纠正: 绕法线方向, 默认小角度变化
				dotProd = endMat.col(2).dot(endUprightDir);
				dotProd = std::fabs(dotProd) > 1 ? (dotProd / std::fabs(dotProd)) : dotProd;
				T thrustAngle = std::acos(dotProd);
				thrustAngle *= endMat.col(2).cross(endUprightDir).dot(endNorm) < 0 ? -1 : 1;
				transMat = Eigen::AngleAxis<T>(pitchAngle * outDist / std::fabs(info(3)), -endTan) * Eigen::AngleAxis<T>(thrustAngle, endNorm) * endMat;

				// 直线
				if (info(3) < 0) {
					transPnt.head(3) = endPnt.head(3) - endTan * outDist;
				}
				// 圆弧
				else if (info(3) > 0) {
					transPnt.head(3) = info.head(3) + Eigen::AngleAxis<T>(outDist / std::fabs(info(3)), -info.tail(3)) * (endPnt.head(3) - info.head(3));
					transMat = (Eigen::AngleAxis<T>(outDist / std::fabs(info(3)), -info.tail(3)) * transMat).eval();
				}

				lambda = 1.0 - outDist / std::fabs(info(3));
			}

			//std::cout << "TransMat = \n" << transMat << std::endl;
			// 过渡点欧拉角
			transPnt.segment<3>(3) = transMat.eulerAngles(2, 1, 0).reverse() * 180 / M_PI;
			// 附加轴插补
			for (size_t j = 6; j < N; ++j) {
				transPnt[j] = begPnt[j] + (endPnt[j] - begPnt[j]) * lambda;
			}
			ans[i] = transPnt;
			//std::cout << "transPnt = \n" << transPnt.transpose() << "\n" << std::endl;
		}

		return ans;
	}

private:
	// 坐标轴单位向向量
	static const Eigen::Matrix<T, 3, 1> unitX, unitY, unitZ;
	static const T M_PI;

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
			theta = 2 * M_PI - theta;
		}
		// q12 > q13
		else if (q12 > q13) {
			normal = -n13;
			theta = 2 * M_PI - theta;
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
		return (Eigen::AngleAxis<T>(qz * M_PI / 180, unitZ) * Eigen::AngleAxis<T>(qy * M_PI / 180, unitY) * Eigen::AngleAxis<T>(qx * M_PI / 180, unitX)).matrix();
	}

	/**
	* @brief  ZYX 欧拉角转旋转矩阵
	* @param  qx    绕 x 转角(deg)
	* @param  qy    绕 y 转角(deg)
	* @param  qz    绕 z 转角(deg)
	*/
	Eigen::Matrix<T, 3, 3> RxyzToRotMat(T qx, T qy, T qz) {
		return (Eigen::AngleAxis<T>(qx * M_PI / 180, unitX) * Eigen::AngleAxis<T>(qy * M_PI / 180, unitY) * Eigen::AngleAxis<T>(qz * M_PI / 180, unitZ)).matrix();
	}
};

template <typename T, size_t N>
const T DiscreteTrajectory<T, N>::M_PI = 3.14159265358979323846;
// 坐标轴单位向向量
template <typename T, size_t N>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T, N>::unitX = Eigen::Matrix<T, 3, 1>(1, 0, 0);
template <typename T, size_t N>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T, N>::unitY = Eigen::Matrix<T, 3, 1>(0, 1, 0);
template <typename T, size_t N>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T, N>::unitZ = Eigen::Matrix<T, 3, 1>(0, 0, 1);

/**
* @brief  计算两组 Rzyx 欧拉角之间的相对运动距离
* @param  begEuler           起点欧拉角 <Rx, Ry, Rz>
* @param  endEuler           绕 y 转角(deg)
* @param  chooseMimumDist    绕 z 转角(deg)
* @return 相对运动距离
*/
template <typename T>
Eigen::Matrix<T, 3, 1> get_zyx_euler_distance(Eigen::Matrix<T, 3, 1> begEuler, Eigen::Matrix<T, 3, 1> endEuler, bool chooseMimumDist = 1) {
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
	Eigen::Matrix<T, 3, 1> equivEuler = endEuler;
	if (endEuler[1] == 90) {
		T detQ = endEuler[2] - endEuler[0];
		equivEuler[0] = (detQ > 0) ? (std::min)(begEuler[0], begEuler[2]) : (std::max)(begEuler[0], begEuler[2]);
		equivEuler[2] = equivEuler[0] + detQ;
	}
	else if (endEuler[1] == -90) {
		T sumQ = endEuler[2] + endEuler[0];
		equivEuler[0] = begEuler[0];
		equivEuler[2] = sumQ - equivEuler[0];
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

	// 欧拉角相对值
	Eigen::Matrix<T, 3, 1> endRel(0,0,0), equivRel(0,0,0);
	T endSum = 0.0, equivSum = 0.0;
	for (size_t i = 0; i < 3; ++i) {
		T directDist = endEuler[i] - begEuler[i];
		T hopDist = 360 - std::fabs(endEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			endRel[i] = directDist;
		} else {
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
		ans = endSum < equivSum ? endRel : equivRel;
	}
	else {
		ans = ans = endSum < equivSum ? equivRel : endRel;
	}

	//std::cout << "begEuler = " << begEuler.transpose() << std::endl;
	//std::cout << "endEuler = " << endEuler.transpose() << "  -> " << endRel.transpose() << " | sum = " << endSum << std::endl;
	//std::cout << "equEuler = " << equivEuler.transpose() << "  -> " << equivRel.transpose() << " | sum = " << equivSum << std::endl;
	//std::cout << "return: " << ans.transpose() << std::endl;
	return ans;
}

#pragma once

#include<vector>
#include<list>
#include<eigen3/Eigen/Dense>
#include<cmath>

#include<iostream>


template <typename T = float>
class DiscreteTrajectory {
public:
	//! �켣�˵� <λ��, Rzyx(deg), ������>
	std::list<std::vector<T>> nodePoint;
	//! �켣�м�� <λ��, Rzyx(deg), ������>
	std::list<std::vector<T>> midPoint;
	//! �켣��Ϣ
	std::list<std::vector<T>> trajInfo;

public:
	DiscreteTrajectory() {};
	~DiscreteTrajectory() {};

	/**
	* @brief  ������ݲ����¼�¼���
	* @param  pnt    �������
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
	* @brief  ��¼ֱ�߹켣
	* @param  pnt    ֱ���յ�����
	*/
	uint8_t add_line(const std::vector<T>& pnt, T vel = 10) {
		if (nodePoint.size() < 1) return 1;

		size_t N = nodePoint.front().size(), num = (std::min)(N, pnt.size());

		// ��ӹ켣��
		std::vector<T> tmp(N, 0), cur = nodePoint.back();
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = pnt[i];
		}
		nodePoint.push_back(tmp);

		// ����м��
		for (size_t i = 0; i < N; ++i) {
			tmp[i] = (cur[i] + tmp[i]) / 2;
		}
		midPoint.push_back(tmp);

		// ����켣��Ϣ
		std::vector<T> info(8, 0);
		Eigen::Matrix<T, 3, 1> curPos(cur[0], cur[1], cur[2]), endPos(pnt[0], pnt[1], pnt[2]);

		// ֱ�߷���
		Eigen::Matrix<T, 3, 1> dir = (endPos - curPos).normalized();
		for (size_t i = 0; i < 3; ++i) {
			info[i] = dir[i];
		}
		// ֱ�߳���
		info[3] = -1 * (endPos - curPos).norm();
		// �켣�ٶ�
		info[7] = vel;
		trajInfo.push_back(info);
		
		return 0;
	}

	/**
	* @brief  ��¼Բ���켣
	* @param  end    Բ���յ�
	* @param  mid    Բ���м��
	*/
	uint8_t add_arc(const std::vector<T>& end, const std::vector<T>& mid, T vel = 10) {
		if (nodePoint.size() < 1) return 1;
		
		size_t N = nodePoint.front().size(), num = (std::min)(N, end.size());

		// ��ӹ켣��
		std::vector<T> tmp(N, 0), cur = nodePoint.back();
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = end[i];
		}
		nodePoint.push_back(tmp);

		// ����м��
		tmp = std::vector<T>(N, 0);
		for (size_t i = 0; i < num; ++i) {
			tmp[i] = mid[i];
		}
		midPoint.push_back(tmp);

		// ����켣��Ϣ
		Eigen::Matrix<T, 3, 1> begPnt(cur[0], cur[1], cur[2]), midPnt(mid[0], mid[1], mid[2]), endPnt(end[0], end[1], end[2]);
		Eigen::Matrix<T, 7, 1> arcInfo = construct_arc_trajectory(begPnt, midPnt, endPnt);
		std::vector<T> info(8);
		// �켣��״
		for (size_t i = 0; i < 7; ++i) {
			info[i] = arcInfo[i];
		}
		// �켣�ٶ�
		info[7] = vel;

		trajInfo.push_back(info);

		return 0;
	}

	/**
	* @brief  �սǹ���
	* @param  end    Բ���յ�
	* @param  mid    Բ���м��
	*/
	uint8_t corner_transition(T inDist = -1, T outDist = -1) {
		if (nodePoint.size() < 1) return 1;

		size_t N = nodePoint.front().size();
		// N < 6 ʱ��֧�ָú���
		if (N < 6) return 2;

		// �ڵ������
		auto endPntIte = nodePoint.begin(), begPntIte = endPntIte++;
		// �м�������
		auto midPntIte = midPoint.begin();
		// �켣��Ϣ������
		auto infoIte = trajInfo.begin();

		while (infoIte != trajInfo.end()) {
			std::vector< Eigen::Matrix<T, N, 1>> transPnt = transition_interpolate(*begPntIte, *endPntIte, *infoIte, { inDist, inDist/2, std::fabs((*infoIte)(3))/2, -outDist/2, -outDist });
			// ����·����
			nodePoint.insert(endPntIte, transPnt.front());
			nodePoint.insert(endPntIte, transPnt.back());
			begPntIte = endPntIte++;
			// �����м��
			midPoint.insert(midPntIte++, transPnt[1]);
			midPoint.insert(midPntIte, transPnt[3]);

			// �����ɹ켣
			Eigen::Matrix<T, 7, 1> begTraj = Eigen::Matrix<T, 7, 1>::Zero();
			begTraj.head(3) = (*infoIte).head(3);
			begTraj(3) = -inDist;
			// �յ���ɹ켣
			Eigen::Matrix<T, 7, 1> endTraj = Eigen::Matrix<T, 7, 1>::Zero();
			endTraj.head(3) = (*infoIte).head(3);
			endTraj(3) = -outDist;

			// ����켣��Ϣ
			trajInfo.insert(infoIte, begTraj);
			(*infoIte++)(3) -= begTraj(3) + endTraj(3);
			trajInfo.insert(infoIte, endTraj);
		}
		
		begPntIte = nodePoint.begin();
		// ��һλ��
		Eigen::Matrix<T, N, 1> prePnt = (*begPntIte++);
		std::cout << "curPnt = " << prePnt.transpose() << std::endl;

		Eigen::Matrix<T, 3, 1> begEuler(prePnt[3], prePnt[4], prePnt[5]), endEuler = (*begPntIte).segment<3>(3);

		while (begPntIte != nodePoint.end()) {
			// ��ǰλ��
			Eigen::Matrix<T, N, 1> curPnt = (*begPntIte++);
			std::cout << "curPnt = " << curPnt.transpose() << std::endl;
			std::cout << curPnt[0] - prePnt[0] << ", " << curPnt[1] - prePnt[1] << ", " << curPnt[2] - prePnt[2] << ", ";
			endEuler = curPnt.segment<3>(3);
			// ŷ�������ֵ
			Eigen::Matrix<T, 3, 1> relEuler = get_zyx_euler_distance(begEuler, endEuler);
			for (size_t i = 0; i < 3; ++i) {
				std::cout << relEuler[i] << ", ";
			}
			begEuler = endEuler;

			prePnt = curPnt;
			std::cout << "\n" << std::endl;
		}
	}

	/**
	* @brief  �սǹ��ɵ��ֵ
	* @param  begPnt      �켣���
	* @param  endPnt      �켣�յ�
	* @param  info        �켣��Ϣ
	* @param  distList    ���ɵ��������, + ��������; - ���յ����
	* @return ���ɵ�λ������
	*/
	std::vector<std::vector<T>> transition_interpolate(const std::vector<T>& begPnt, const std::vector<T>& endPnt, const std::vector<T>& info, const std::vector<T>& distList) {
		// ����ֵ
		std::vector<std::vector<T>> ans(distList.size(), std::vector<T>(begPnt.size(), 0));

		// ��㴦 TCP ��̬
		Eigen::Matrix<T, 3, 3> begMat = RzyxToRotMat(begPnt[5], begPnt[4], begPnt[3]);
		// �յ㴦 TCP ��̬
		Eigen::Matrix<T, 3, 3> endMat = RzyxToRotMat(endPnt[5], endPnt[4], endPnt[3]);
		// ���߷���
		Eigen::Matrix<T, 3, 1> infoTail3(info[4], info[5], info[6]), infoHead3(info[0], info[1], info[2]);
		Eigen::Matrix<T, 3, 1> begHead3(begPnt[0], begPnt[1], begPnt[2]), endHead3(endPnt[0], endPnt[1], endPnt[2]);
		Eigen::Matrix<T, 3, 1> begTan = info[3] > 0 ? (infoTail3.cross(begHead3 - infoHead3)).normalized() : infoHead3;
		Eigen::Matrix<T, 3, 1> endTan = info[3] > 0 ? (infoTail3.cross(endHead3 - infoHead3)).normalized() : infoHead3;
		// ���߷���
		Eigen::Matrix<T, 3, 1> begNorm = begTan.cross(begMat.col(2)).normalized();
		Eigen::Matrix<T, 3, 1> endNorm = endTan.cross(endMat.col(2)).normalized();
		// ��׼��ǹ����
		Eigen::Matrix<T, 3, 1> begUprightDir = begNorm.cross(begTan).normalized();
		Eigen::Matrix<T, 3, 1> endUprightDir = endNorm.cross(endTan).normalized();
		// ��㺸ǹ����ת�����յ�
		// todo: Eigen::AngleAxis<T>(1, infoTail3)
		Eigen::Matrix<T, 3, 1> begUprightDir_end = info[3] > 0 ? (Eigen::AngleAxis<T>(infoTail3.norm(), infoTail3.normalized())*begUprightDir).eval() : begUprightDir;

		// �����Ǳ仯: �����߷���, Ĭ��С�Ƕȱ仯
		T dotProd = begUprightDir_end.dot(endUprightDir);
		dotProd = std::fabs(dotProd) > 1 ? (dotProd / std::fabs(dotProd)) : dotProd;
		T pitchAngle = std::acos(dotProd);
		pitchAngle *= (begUprightDir_end.cross(endUprightDir).dot(endTan) < 0) ? -1 : 1;

		std::cout << "\n===========\nbegMat = \n" << begMat << std::endl;
		std::cout << "endMat = \n" << endMat << "\n===========\n" << std::endl;
		for (size_t i = 0; i < distList.size(); ++i) {
			
			// ���ɵ�
			std::vector<T> transPnt(begPnt.size(), 0);
			// ���ɵ���̬
			Eigen::Matrix<T, 3, 3> transMat = Eigen::Matrix<T, 3, 3>::Identity();
			// ���ɵ����ռ��
			T lambda = 0.0;

			// ���������
			if (distList[i] > 0) {

				//std::cout << "\nNodeMat = \n" << begMat << std::endl;
				//std::cout << "NodePnt = " << begPnt.transpose() << std::endl;

				T inDist = distList[i];
				// �����Ǿ���: �Ʒ��߷���, Ĭ��С�Ƕȱ仯
				transMat.col(2) = begUprightDir;
				transMat.col(1) = begMat.col(1).dot(begTan) >= 0 ? begTan : -begTan;
				transMat.col(0) = (transMat.col(1).cross(transMat.col(2))).normalized();
				// ����������
				transMat = (Eigen::AngleAxis<T>(pitchAngle * inDist / std::fabs(info(3)), begTan) * transMat).eval();

				// ֱ��
				if (info[3] < 0) {
					Eigen::Matrix<T, 3, 1> dir = begHead3 + begTan * inDist;
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
				}
				// Բ��
				else if (info[3] > 0) {
					Eigen::Matrix<T, 3, 1> center = infoHead3 + Eigen::AngleAxis<T>(inDist / std::fabs(info[3]), infoTail3) * (begHad3 - infoHead3);
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
					transMat = (Eigen::AngleAxis<T>(inDist / std::fabs(info[3]), infoTail3) * transMat).eval();
				}

				lambda = inDist / std::fabs(info[3]);
			}
			// ���յ�����
			else if (distList[i] < 0) {
				T outDist = -distList[i];

				// �����Ǿ���: �Ʒ��߷���, Ĭ��С�Ƕȱ仯
				transMat.col(2) = endUprightDir;
				transMat.col(1) = endMat.col(1).dot(endTan) >= 0 ? endTan : -endTan;
				transMat.col(0) = (transMat.col(1).cross(transMat.col(2))).normalized();
				// ����������
				transMat = (Eigen::AngleAxis<T>(pitchAngle * outDist / std::fabs(info[3]), -endTan) * transMat).eval();

				// ֱ��
				if (info[3] < 0) {
					Eigen::Matrix<T, 3, 1> dir = endHead3 - endTan * outDist;
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
				}
				// Բ��
				else if (info[3] > 0) {
					Eigen::Matrix<T, 3, 1> center = infoHead3 + Eigen::AngleAxis<T>(outDist / std::fabs(info[3]), -infoTail3) * (endHead3 - infoHead3);
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
					transMat = (Eigen::AngleAxis<T>(outDist / std::fabs(info[3]), -infoTail3) * transMat).eval();
				}

				lambda = 1.0 - outDist / std::fabs(info[3]);
			}

			if (i == 0 || i == 4) {
				std::cout << "TransMat = \n" << transMat << std::endl;
			}
			// ���ɵ�ŷ����
			Eigen::Matrix<T, 3, 1> transEuler = transMat.eulerAngles(2, 1, 0).reverse() * 180 / DT_PI;
			for (size_t j = 0; j < 3; ++j) {
				transPnt[3 + j] = transEuler[j];
			}
			// ������岹
			for (size_t j = 6; j < begPnt.size(); ++j) {
				transPnt[j] = begPnt[j] + (endPnt[j] - begPnt[j]) * lambda;
			}
			ans[i] = transPnt;
			if (i == 0 || i == 4) {
				std::cout << "transPnt = \n" << transPnt.transpose() << "\n" << std::endl;
			}
		}

		return ans;
	}

private:
	// �����ᵥλ������
	static const Eigen::Matrix<T, 3, 1> unitX, unitY, unitZ;
	static const T DT_PI;

	/**
	* @brief  �ռ����㹹��Բ���켣
	* @param  beg    �������
	* @param  mid    �м������
	* @param  end    �յ�����
	* @return Բ���켣��Ϣ
	*/
	Eigen::Matrix<T, 7, 1> construct_arc_trajectory(Eigen::Matrix<T, 3, 1> beg, Eigen::Matrix<T, 3, 1> mid, Eigen::Matrix<T, 3, 1> end) {
		Eigen::Matrix<T, 7, 1> info;

		Eigen::Matrix<T, 3, 1> a = beg - mid, b = end - mid;
		// ��ֱ�ߴ���
		if (a.cross(b).squaredNorm() < 1e-12) {
			info.head(3) = (end - beg).normalized();
			info(3) = -1 * (end - beg).norm();
			return info;
		}

		// Բ��λ��
		info.head(3) = (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2 * (a.cross(b)).squaredNorm()) + mid;
		// �뾶����
		Eigen::Matrix<T, 3, 1> op1 = (beg - info.head(3)).normalized(), op2 = (mid - info.head(3)).normalized(), op3 = (end - info.head(3)).normalized();
		// �뾶�н�
		T q12 = std::acos(op1.dot(op2)), q13 = std::acos(op1.dot(op3));
		// ���߷���
		Eigen::Matrix<T, 3, 1> n12 = op1.cross(op2), n13 = op1.cross(op3);
		// Բ���˶�ƽ��ķ��߷���
		Eigen::Matrix<T, 3, 1> normal = op1.cross(op3);
		// Բ�ĽǽǶ�
		float theta = std::acos(op1.dot(op3));
		// ����Բ�ĽǺ�������
		// 2,3 �� 1 ������
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

		// Բ������
		info(3) = (beg - info.head(3)).norm() * theta;
		// ������ת�Ƕȵķ��߷���
		info.tail(3) = normal.normalized() * theta;

		return info;
	}

	/**
	* @brief  XYZ ŷ����ת��ת����
	* @param  qz    �� z ת��(deg)
	* @param  qy    �� y ת��(deg)
	* @param  qx    �� x ת��(deg)
	*/
	Eigen::Matrix<T, 3, 3> RzyxToRotMat(T qz, T qy, T qx) {
		return (Eigen::AngleAxis<T>(qz * DT_PI / 180, unitZ) * Eigen::AngleAxis<T>(qy * DT_PI / 180, unitY) * Eigen::AngleAxis<T>(qx * DT_PI / 180, unitX)).matrix();
	}

	/**
	* @brief  ZYX ŷ����ת��ת����
	* @param  qx    �� x ת��(deg)
	* @param  qy    �� y ת��(deg)
	* @param  qz    �� z ת��(deg)
	*/
	Eigen::Matrix<T, 3, 3> RxyzToRotMat(T qx, T qy, T qz) {
		return (Eigen::AngleAxis<T>(qx * DT_PI / 180, unitX) * Eigen::AngleAxis<T>(qy * DT_PI / 180, unitY) * Eigen::AngleAxis<T>(qz * DT_PI / 180, unitZ)).matrix();
	}
};

template <typename T>
const T DiscreteTrajectory<T>::DT_PI = 3.14159265358979323846;
// �����ᵥλ������
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitX = Eigen::Matrix<T, 3, 1>(1, 0, 0);
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitY = Eigen::Matrix<T, 3, 1>(0, 1, 0);
template <typename T>
const Eigen::Matrix<T, 3, 1> DiscreteTrajectory<T>::unitZ = Eigen::Matrix<T, 3, 1>(0, 0, 1);

/**
* @brief  �������� Rzyx ŷ����֮�������˶�����
* @param  begEuler           ���ŷ���� <Rx, Ry, Rz>
* @param  endEuler           �� y ת��(deg)
* @param  chooseMimumDist    �� z ת��(deg)
* @return ����˶�����
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
		equivEuler[0] = (equivEuler[2] > 0) ? std::min(begEuler[0], begEuler[2]) : (std::max)(begEuler[0], begEuler[2]);
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

	// ŷ�������ֵ
	Eigen::Matrix<T, 3, 1> endRel(0,0,0), equivRel(0,0,0);
	T endSum = 0.0, equivSum = 0.0;
	for (size_t i = 0; i < 3; ++i) {
		T directDist = endEuler[i] - begEuler[i];
		T hopDist = 360 - std::fabs(endEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			endRel[i] = directDist;
		} else {
			// ��ʱ begEuler[i] ��= 0 ����
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

	// ѡ����� / �������˶�����
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

	//std::cout << "begEuler = " << begEuler.transpose() << std::endl;
	//std::cout << "endEuler = " << endEuler.transpose() << "  -> " << endRel.transpose() << " | sum = " << endSum << std::endl;
	//std::cout << "equEuler = " << equivEuler.transpose() << "  -> " << equivRel.transpose() << " | sum = " << equivSum << std::endl;
	//std::cout << "return: " << ans.transpose() << std::endl;
	return ans;
}

/**
* @brief  �����Ч�� Rzyx ŷ����
* @param  endEuler    ����ŷ���� <Rx, Ry, Rz>
* @return ��Ч�� Rzyx ŷ����
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

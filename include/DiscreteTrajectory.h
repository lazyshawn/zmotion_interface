#pragma once

#include<vector>
#include<list>
#include<eigen3/Eigen/Dense>
#include<cmath>

#include<iostream>

#include"FsCraftDef.h"


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

	// ŷ�������ֵ
	Eigen::Matrix<T, 3, 1> endRel(0, 0, 0), equivRel(0, 0, 0);
	T endSum = 0.0, equivSum = 0.0;
	for (size_t i = 0; i < 3; ++i) {
		T directDist = endEuler[i] - begEuler[i];
		T hopDist = 360 - std::fabs(endEuler[i]) - std::fabs(begEuler[i]);
		if (std::fabs(directDist) <= hopDist) {
			endRel[i] = directDist;
		}
		else {
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

	return ans;
}


template <typename T = float>
class DiscreteTrajectory {
public:
	//! �켣�˵� <λ��, Rzyx(deg), ������>
	std::list<std::vector<T>> nodePoint;
	//! �켣�м�� <λ��, Rzyx(deg), ������>
	std::list<std::vector<T>> midPoint;
	//! �켣��Ϣ    ֱ��    ����(0:2),    -ֱ�߳���(3), null(4:6),     �ٶ�(7)
	//              Բ��    Բ������(0:2),+Բ������(3), ��תʸ��(4:6), �ٶ�(7)
	std::list<std::vector<T>> trajInfo;

	//! �ں�����
	std::list<Weave> waveInfo;
	//! ���ٲ���
	std::list<Track> trackInfo;
	//! ���Ӳ���
	std::list<Arc_WeldingParaItem> weldInfo;

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
	uint8_t add_line(const std::vector<T>& pnt, T vel = 10, T smooth = -1) {
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
		for (size_t i = 3; i < 6; ++i) {
			tmp[i] = cur[i];
		}
		midPoint.push_back(tmp);

		// ����켣��Ϣ
		std::vector<T> info(9, 0);
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
		// ƽ����
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
	* @brief  ��¼Բ���켣
	* @param  end    Բ���յ�
	* @param  mid    Բ���м��
	*/
	uint8_t add_arc(const std::vector<T>& end, const std::vector<T>& mid, T vel = 10, T smooth = -1) {
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
		std::vector<T> info(9);
		// �켣��״
		for (size_t i = 0; i < 7; ++i) {
			info[i] = arcInfo[i];
		}
		// �켣�ٶ�
		info[7] = vel;
		// ƽ����
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
			auto transPnt = transition_interpolate(*begPntIte, *endPntIte, *infoIte, { inDist, inDist/2, std::fabs((*infoIte)[3])/2, -outDist/2, -outDist });
			// ����·����
			nodePoint.insert(endPntIte, transPnt.front());
			nodePoint.insert(endPntIte, transPnt.back());
			begPntIte = endPntIte++;
			// �����м��
			midPoint.insert(midPntIte++, transPnt[1]);
			midPoint.insert(midPntIte, transPnt[3]);

			// �����ɹ켣
			std::vector<T> begTraj(9, 0);
			for (size_t i = 0; i < 3; ++i) {
				begTraj[i] = (*infoIte)[i];
			}
			begTraj[3] = -inDist;
			begTraj[7] = (*infoIte)[7];

			// �յ���ɹ켣
			std::vector<T> endTraj(9, 0);
			for (size_t i = 0; i < 3; ++i) {
				endTraj[i] = (*infoIte)[i];
			}
			endTraj[3] = -outDist;
			endTraj[7] = (*infoIte)[7];

			// ���������ɹ켣��Ϣ
			trajInfo.insert(infoIte, begTraj);
			// ԭʼ�켣��Ϣ�޸�
			(*infoIte++)[3] -= begTraj[3] + endTraj[3];
			// �����յ���ɹ켣��Ϣ
			trajInfo.insert(infoIte, endTraj);

		}
		
		//begPntIte = nodePoint.begin();
		//// ��һλ��
		//Eigen::Matrix<T, N, 1> prePnt = (*begPntIte++);
		//std::cout << "curPnt = " << prePnt.transpose() << std::endl;

		//Eigen::Matrix<T, 3, 1> begEuler(prePnt[3], prePnt[4], prePnt[5]), endEuler = (*begPntIte).segment<3>(3);

		//while (begPntIte != nodePoint.end()) {
		//	// ��ǰλ��
		//	Eigen::Matrix<T, N, 1> curPnt = (*begPntIte++);
		//	std::cout << "curPnt = " << curPnt.transpose() << std::endl;
		//	std::cout << curPnt[0] - prePnt[0] << ", " << curPnt[1] - prePnt[1] << ", " << curPnt[2] - prePnt[2] << ", ";
		//	endEuler = curPnt.segment<3>(3);
		//	// ŷ�������ֵ
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
	* @brief  �սǼ���
	* @param  angularVel    �켣�����ٶȣ�ŷ����ʸ���ٶ�
	*/
	uint8_t corner_slowdown(T angularVel) {
		// �����켣
		auto nodePointIte = nodePoint.begin();
		auto midPointIte = midPoint.begin();
		auto infoIte = trajInfo.begin();

		while (infoIte != trajInfo.end()) {
			std::vector<float> curBuffer = *(nodePointIte++);

			// ŷ����ת��������˶�: beg -> mid -> end
			Eigen::Vector3f begEuler = Eigen::Vector3f(curBuffer[3], curBuffer[4], curBuffer[5]);
			Eigen::Vector3f endEuler = Eigen::Vector3f((*midPointIte)[3], (*midPointIte)[4], (*midPointIte)[5]);
			Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
			begEuler = endEuler;
			endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
			relEuler += get_zyx_euler_distance(begEuler, endEuler);

			T sumAngle = std::sqrt(relEuler[0]* relEuler[0] + relEuler[1]* relEuler[1] + relEuler[2]* relEuler[2]);
			// �ٶ�����
			if (sumAngle / angularVel > std::fabs((*infoIte)[3]) / (*infoIte)[7]) {
				(*infoIte)[7] = angularVel * std::fabs((*infoIte)[3]) / sumAngle;
			}

			midPointIte++;
			infoIte++;
		}

		return 0;
	}
	
	/**
	* @brief  ���ȷָ�
	* @param  separation    ������
	*/
	uint8_t equally_divide(const std::vector<T>& separation) {
		// �ֶ��ܾ���
		T phaseDist = 0.0;
		for (size_t i = 0; i < separation.size(); ++i) {
			phaseDist += separation[i];
		}

		// �����켣
		auto nodePointIte = nodePoint.begin();
		auto midPointIte = midPoint.begin();
		auto infoIte = trajInfo.begin();

		Eigen::Vector3f begEuler((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]), endEuler = begEuler;
		while (infoIte != trajInfo.end()) {
			std::vector<T> begBuffer = *(nodePointIte++);
			// ��ǰ�����
			T curDist = 0.0, midDist = 0.0, trajDist = std::fabs((*infoIte)[3]);

			std::vector<T> relEndMove(begBuffer.size(), 0);
			// �յ㵽��������˶�
			for (size_t i = 0; i < begBuffer.size(); ++i) {
				relEndMove[i] = (*nodePointIte)[i] - begBuffer[i];
			}

			// ŷ������Ծ���
			begEuler = Eigen::Vector3f(begBuffer[3], begBuffer[4], begBuffer[5]);
			endEuler = Eigen::Vector3f((*nodePointIte)[3], (*nodePointIte)[4], (*nodePointIte)[5]);
			Eigen::Vector3f relEuler = get_zyx_euler_distance(begEuler, endEuler);
			begEuler = endEuler;

			// �������ڸ���
			int numPeriod = std::floor(std::fabs((*infoIte)[3]) / phaseDist);
			bool isRedundant = true;
			for (size_t i = 0; i < numPeriod + 1; ++i) {
				if (!isRedundant)
					break;
				for (size_t j = 0; j < separation.size(); ++j) {
					// ��������µĹ켣
					if (std::fabs((*infoIte)[3]) < separation[j]) {
						isRedundant = false;
						break;
					}

					curDist += separation[j];
					midDist = curDist - separation[j] / 2;
					
					std::vector<T> sepaPoint(begBuffer.size(), 0), curMidPoint(begBuffer.size(), 0), sepaTraj = *infoIte;
					// �����λ��
					Eigen::Matrix<T, 3, 1> curPos(0,0,0), midPos(0,0,0), endMidPos(0,0,0);

					if ((*infoIte)[3] < 0) {
						Eigen::Matrix<T, 3, 1> begPos(begBuffer[0], begBuffer[1], begBuffer[2]);
						Eigen::Matrix<T, 3, 1> dir((*infoIte)[0], (*infoIte)[1], (*infoIte)[2]);
						curPos = begPos + dir * curDist;
						midPos = begPos + dir * midDist;
						endMidPos = begPos + dir * (trajDist + curDist) / 2;
						// �����켣��Ϣ
						sepaTraj[3] = -separation[j];
						// ԭʼ�켣�޸�
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
						// �����켣��Ϣ
						sepaTraj[3] = separation[j];
						sepaTraj[4] = norm[0] * separation[j] / trajDist * theta;
						sepaTraj[5] = norm[1] * separation[j] / trajDist * theta;
						sepaTraj[6] = norm[2] * separation[j] / trajDist * theta;
						// ԭʼ�켣�޸�
						(*infoIte)[3] -= separation[j];
						(*infoIte)[4] = norm[0] * (trajDist - curDist) / trajDist * theta;
						(*infoIte)[5] = norm[1] * (trajDist - curDist) / trajDist * theta;
						(*infoIte)[6] = norm[2] * (trajDist - curDist) / trajDist * theta;
					}
					for (size_t k = 0; k < 3; ++k) {
						sepaPoint[k] = curPos[k];
						curMidPoint[k] = midPos[k];
						// ԭʼ�켣�м���޸�
						(*midPointIte)[k] = endMidPos[k];
					}
					// �����ŷ����
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

		//std::cout << "\n===========\nbegMat = \n" << begMat << std::endl;
		//std::cout << "endMat = \n" << endMat << "\n===========\n" << std::endl;
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
				transMat = (Eigen::AngleAxis<T>(pitchAngle * inDist / std::fabs(info[3]), begTan) * transMat).eval();

				// ֱ��
				if (info[3] < 0) {
					Eigen::Matrix<T, 3, 1> dir = begHead3 + begTan * inDist;
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = dir[i];
					}
				}
				// Բ��
				else if (info[3] > 0) {
					Eigen::Matrix<T, 3, 1> center = infoHead3 + Eigen::AngleAxis<T>(inDist / std::fabs(info[3]), infoTail3) * (begHead3 - infoHead3);
					for (size_t i = 0; i < 3; ++i) {
						transPnt[i] = center[i];
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
						transPnt[i] = center[i];
					}
					transMat = (Eigen::AngleAxis<T>(outDist / std::fabs(info[3]), -infoTail3) * transMat).eval();
				}

				lambda = 1.0 - outDist / std::fabs(info[3]);
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


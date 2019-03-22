#pragma once
#include "dbtype.h"
#include "dbkdtree.h"

#include <iostream>
#include <time.h>
#include <algorithm>
#include <Eigen/Dense>
#include <valarray>

class AngleRegister
{
public:
	AngleRegister()
	{
	}

	~AngleRegister()
	{
	}
	//��ά��תƽ��
	void toProjecImage(float& x, float& y, unsigned int& rows, unsigned int& cols);

	//����һϵ�е�����꣬�����Щ������ϵ��ߵ�k bֵ ��С���˷�
	std::vector<double> LineFitting(std::vector<db::Point2f> &rPoints);
	std::vector<double> leastSquareFitting(std::vector<db::Point2f> &rPoints);

	//��ֱ֪�߷���  ����һ��  ���䴹��
	db::Point2f getFootPoint(std::vector<double>& kb, const db::Point2f& P1);
	//��ֱ֪�߷���  ����һ��  ��㵽ֱ�߾���
	double getPointToline_Distance(std::vector<double>& kb, const db::Point2f& P1);
	//��ƽ�� ��������
	double getPointToPoint_Distance(const db::Point2f& P1, const db::Point2f& P2);

	//��ȡpointset�е�ֱ�߶�
	void getLinesFromPointset(const std::vector<db::Point2f> &rPoints, std::vector<db::line> &lineSets);

	//ֱ�߶ε��Ż�����  �ϲ���ɢ�߶�
	int reprocessLineset(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets);
	int reprocessLineset2(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets);

	// ��ȡֱ��KB����  
	void getLinePara(float& x1, float& y1, float& x2, float& y2, db::LinePara & LP);

	//������������ļн�
	void getTwoVector_angle(double(&a)[2], double(&b)[2], double & sigema);

	//�����ֱ�ߵĽ���
	db::Point2f getTwoline_cornerPoint(db::line firstline, db::line secondline);

	//���һ��������x�����������ʱ��н�
	void getVector_Xangle(double(&a)[2], double &sigema);

	// ���2d ������ ֱ���������ڼн�  �Լ�����ֱ�ߵĽ���
	void LRangleFromLines2(std::vector<db::line> &lineSets);

	//�������ƽ������������ת��
	/*
	* ��������֮�����ת��
	* ������ȷ������ѧ���
	* 1. ��������ʱ��ת���ķ�����������
	* 2. ��������֮��ļн�theta�� ��ָ(A^B)/(|A|*|B|) = cos(theta)��0<=theta<=180 �ȣ� ����û�з���֮��
	* 3. ������������ת�ǣ���ָ������p1��ʼ����ʱ����ת��ת������p2ʱ����ת���ĽǶȣ� ��Χ�� 0 ~ 360��
	* ��������p1��p2����ת�ǣ��㷨���£�
	* ����ͨ����˺�arccosine�ĵõ���������֮��ļн�
	* Ȼ���ж�ͨ��������ж���������֮���λ�ù�ϵ
	* ���p2��p1����ʱ�뷽��, ����arccose�ĽǶ�ֵ, ��Χ0 ~ 180.0(�������ֶ���,���Թ����������)
	* ���򷵻� 360.0 - arecose��ֵ, ����180��360(�������ֶ���,���Ϊ��)
	*/
	double getRotateAngle(double x1, double y1, double x2, double y2);

	// �任������� 2dlidar ƥ��
	int registration_2dLidar(
		std::vector<db::Point2f>& src_points,
		std::vector<db::Point2f>& targ_points,
		Eigen::Matrix<float, 2, 3>& RT,
		float &theta);

private:

};

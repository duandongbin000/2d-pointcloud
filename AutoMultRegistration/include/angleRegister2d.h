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
	//二维点转平面
	void toProjecImage(float& x, float& y, unsigned int& rows, unsigned int& cols);

	//输入一系列点的坐标，输出这些点所拟合的线的k b值 最小二乘法
	std::vector<double> LineFitting(std::vector<db::Point2f> &rPoints);
	std::vector<double> leastSquareFitting(std::vector<db::Point2f> &rPoints);

	//已知直线方程  线外一点  求其垂足
	db::Point2f getFootPoint(std::vector<double>& kb, const db::Point2f& P1);
	//已知直线方程  线外一点  求点到直线距离
	double getPointToline_Distance(std::vector<double>& kb, const db::Point2f& P1);
	//求平面 两点间距离
	double getPointToPoint_Distance(const db::Point2f& P1, const db::Point2f& P2);

	//获取pointset中的直线段
	void getLinesFromPointset(const std::vector<db::Point2f> &rPoints, std::vector<db::line> &lineSets);

	//直线段的优化处理  合并零散线段
	int reprocessLineset(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets);
	int reprocessLineset2(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets);

	// 获取直线KB参数  
	void getLinePara(float& x1, float& y1, float& x2, float& y2, db::LinePara & LP);

	//求解两个向量的夹角
	void getTwoVector_angle(double(&a)[2], double(&b)[2], double & sigema);

	//求解两直线的交点
	db::Point2f getTwoline_cornerPoint(db::line firstline, db::line secondline);

	//求解一个向量与x轴正方向的逆时针夹角
	void getVector_Xangle(double(&a)[2], double &sigema);

	// 求解2d 点云中 直线左右相邻夹角  以及左右直线的交点
	void LRangleFromLines2(std::vector<db::line> &lineSets);

	//求解两个平面两向量的旋转角
	/*
	* 两个向量之间的旋转角
	* 首先明确几个数学概念：
	* 1. 极轴沿逆时针转动的方向是正方向
	* 2. 两个向量之间的夹角theta， 是指(A^B)/(|A|*|B|) = cos(theta)，0<=theta<=180 度， 而且没有方向之分
	* 3. 两个向量的旋转角，是指从向量p1开始，逆时针旋转，转到向量p2时，所转过的角度， 范围是 0 ~ 360度
	* 计算向量p1到p2的旋转角，算法如下：
	* 首先通过点乘和arccosine的得到两个向量之间的夹角
	* 然后判断通过差乘来判断两个向量之间的位置关系
	* 如果p2在p1的逆时针方向, 返回arccose的角度值, 范围0 ~ 180.0(根据右手定理,可以构成正的面积)
	* 否则返回 360.0 - arecose的值, 返回180到360(根据右手定理,面积为负)
	*/
	double getRotateAngle(double x1, double y1, double x2, double y2);

	// 变换矩阵估计 2dlidar 匹配
	int registration_2dLidar(
		std::vector<db::Point2f>& src_points,
		std::vector<db::Point2f>& targ_points,
		Eigen::Matrix<float, 2, 3>& RT,
		float &theta);

private:

};

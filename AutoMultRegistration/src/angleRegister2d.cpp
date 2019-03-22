#include "angleRegister2d.h"

//二维点转平面
void AngleRegister::toProjecImage(float& x, float& y, unsigned int& rows, unsigned int& cols)    //  640X640的图像
{
	//	int nthrds = 4;
	//	omp_set_num_threads(nthrds);
	//#pragma omp parallel for
	/*int rows = 0;
	int cols = 0;*/

	if (x >= -16.0 && x<0.0)
	{
		cols = (unsigned int)((x + 16) * 20);
		if (y >= -16.0 && y<0.0)
		{
			rows = (unsigned int)(y*(-20)) + 320;
		}
		if (y >= 0.0 && y <= 16.0)
		{
			rows = (unsigned int)((16 - y) * 20);
		}
	}
	else if (x >= 0.0 && x <= 16.0)
	{
		cols = (unsigned int)((x) * 20) + 320;
		if (y >= -16.0 && y<0.0)
		{
			rows = (unsigned int)(y*(-20)) + 320;
		}
		if (y >= 0.0 && y <= 16.0)
		{
			rows = (unsigned int)((16 - y) * 20);
		}
	}
}

//输入一系列点的坐标，输出这些点所拟合的线的k b值 最小二乘法
std::vector<double> AngleRegister::LineFitting(std::vector<db::Point2f> &rPoints)
{
	// y = Ax + B，根据最小二乘法求出A，B
	std::vector<double > resLine(2);
	int size = rPoints.size();
	float *x = new float[size];
	float *y = new float[size];
	float A = 1.0, B = 0.0;
	float xmean = 0.0f;
	float ymean = 0.0f;

	for (int i = 0; i < size; i++)
	{
		x[i] = rPoints[i].x;
		y[i] = rPoints[i].y;
	}

	for (int i = 0; i < size; i++)
	{
		xmean += x[i];
		ymean += y[i];
	}
	xmean /= size;
	ymean /= size;
	float sumx2 = 0.0f;
	float sumxy = 0.0f;
	for (int i = 0; i < size; i++)
	{
		sumx2 += (x[i] - xmean) * (x[i] - xmean);
		sumxy += (y[i] - ymean) * (x[i] - xmean);
	}

	if (sumx2 != 0)
	{
		A = sumxy / sumx2;
		B = ymean - A*xmean;
	}
	else
	{
		A = 1.0;
		B = 0.0;
	}

	resLine[0] = A;
	resLine[1] = B;
	return resLine;
}
std::vector<double> AngleRegister::leastSquareFitting(std::vector<db::Point2f> &rPoints)
{
	std::vector<double > resLine(2);
	int num_points = rPoints.size();
	std::valarray<float> data_x(num_points);
	std::valarray<float> data_y(num_points);
	for (int i = 0; i < num_points; i++)
	{
		data_x[i] = rPoints[i].x;
		data_y[i] = rPoints[i].y;
	}
	float A = 0.0;
	float B = 0.0;
	float C = 0.0;
	float D = 0.0;
	A = (data_x*data_x).sum();
	B = data_x.sum();
	C = (data_x*data_y).sum();
	D = data_y.sum();
	float k, b, tmp = 0;
	if (tmp = (A*data_x.size() - B*B))   //temp！=0
	{
		k = (C*data_x.size() - B*D) / tmp;
		b = (A*D - C*B) / tmp;
	}
	else
	{
		k = 1;
		b = 0;
	}
	resLine[0] = k;
	resLine[1] = b;
	return resLine;
}

//已知直线方程  线外一点  求其垂足
db::Point2f AngleRegister::getFootPoint(std::vector<double>& kb, const db::Point2f& P1)
{
	db::Point2f FootPoint;
	double x = (kb[0] * P1.y + P1.x - kb[0] * kb[1]) / (1 + kb[0] * kb[0]);
	double y = x*kb[0] + kb[1];
	FootPoint.x = (float)x;
	FootPoint.y = (float)y;

	return FootPoint;
}
//已知直线方程  线外一点  求点到直线距离
double AngleRegister::getPointToline_Distance(std::vector<double>& kb, const db::Point2f& P1)
{
	db::Point2f pointtemp = getFootPoint(kb, P1);
	double distance = 0.0;
	distance = sqrt((pointtemp.x - P1.x)*(pointtemp.x - P1.x) + (pointtemp.y - P1.y)*(pointtemp.y - P1.y));
	return distance;
}
//求平面 两点间距离
double AngleRegister::getPointToPoint_Distance(const db::Point2f& P1, const db::Point2f& P2)
{
	double distance = 0.0;
	distance = sqrt((P2.x - P1.x)*(P2.x - P1.x) + (P2.y - P1.y)*(P2.y - P1.y));
	return distance;
}

//获取pointset中的直线段
void AngleRegister::getLinesFromPointset(const std::vector<db::Point2f> &rPoints, std::vector<db::line> &lineSets)  //2018.11.7 修正
{
	if (rPoints.size()<3)
	{
		std::cout << "point cloud have NuLL data !!!" << std::endl;
		return;
	}

	double avg = 0.0, sum = 0.0;
	std::vector<double> kb;
	std::vector<db::Point2f> tempPoints;
	db::line tempLine;
	std::vector<double> kb1;

	std::vector<double> kb2;
	std::vector<double> kb3;

	for (size_t i = 0; i < rPoints.size(); i++)
	{
		if (tempPoints.size() == 0)
		{
			if (i == 0)
			{
				double distance_0 = getPointToPoint_Distance(rPoints[i], rPoints[i + 1]);
				if (distance_0 <0.5)
				{
					tempPoints.push_back(rPoints[i]);

					tempLine.Points.push_back(rPoints[i]);
					tempLine.PointsIndex.push_back(i);
				}
				continue;
			}
			if (i == rPoints.size() - 1)
			{
				double distance_0 = getPointToPoint_Distance(rPoints[i], rPoints[i - 1]);
				if (distance_0 <0.5)
				{
					tempPoints.push_back(rPoints[i]);

					tempLine.Points.push_back(rPoints[i]);
					tempLine.PointsIndex.push_back(i);
				}
				continue;
			}

			//判断孤点
			double distance_1 = getPointToPoint_Distance(rPoints[i], rPoints[i - 1]);
			double distance_2 = getPointToPoint_Distance(rPoints[i], rPoints[i + 1]);
			if (distance_1<0.5 || distance_2<0.5)
			{
				tempPoints.push_back(rPoints[i]);

				tempLine.Points.push_back(rPoints[i]);
				tempLine.PointsIndex.push_back(i);
			}

			continue;
		}
		else
		{
			tempPoints.push_back(rPoints[i]);
		}

		if (tempPoints.size() < 4)
		{
			double distance_1 = getPointToPoint_Distance(rPoints[i], rPoints[i - 1]);
			if (distance_1>0.5)
			{
				tempLine.Points.clear();
				std::vector<db::Point2f>(tempLine.Points).swap(tempLine.Points);   //tempLine.Points.swap(std::vector<db::Point2f>()); //v.clear()  vector<string>(v).swap(v);
				tempLine.PointsIndex.clear();
				std::vector<int>(tempLine.PointsIndex).swap(tempLine.PointsIndex);  //tempLine.PointsIndex.swap(std::vector<int>());
				tempLine.kb.clear();
				std::vector<double>(tempLine.kb).swap(tempLine.kb);   //tempLine.kb.swap(std::vector<double>());

				tempPoints.clear();
				std::vector<db::Point2f>(tempPoints).swap(tempPoints);  //tempPoints.swap(std::vector<db::Point2f>());
				continue;
			}
			kb = leastSquareFitting(tempPoints);
			kb1 = kb;
			kb2 = kb1;
			kb3 = kb1;

			tempLine.Points.push_back(rPoints[i]);
			tempLine.PointsIndex.push_back(i);
			continue;
		}

		//点到直线的距离
		double dis_1 = getPointToline_Distance(kb1, rPoints[i]);
		double dis_2 = getPointToline_Distance(kb2, rPoints[i]);
		double dis_3 = getPointToline_Distance(kb3, rPoints[i]);

		double distance_2 = getPointToPoint_Distance(rPoints[i], rPoints[i - 1]);

		if (dis_1 > 0.05 || distance_2 > 0.5)      //偏差较大的 点20cm
		{
			//sum = 0.0;
			if (i != 0)
				i--;

			tempLine.kb = kb1;
			if (atan(tempLine.kb[0]) >= 0)
				tempLine.sigma = atan(tempLine.kb[0]);   //0-180度
			else
				tempLine.sigma = atan(tempLine.kb[0]) + 3.1415926;

			lineSets.push_back(tempLine);

			tempLine.Points.clear();
			std::vector<db::Point2f>(tempLine.Points).swap(tempLine.Points);//tempLine.Points.swap(std::vector<db::Point2f>());
			tempLine.PointsIndex.clear();
			std::vector<int>(tempLine.PointsIndex).swap(tempLine.PointsIndex);//tempLine.PointsIndex.swap(std::vector<int>());
			tempLine.kb.clear();
			std::vector<double>(tempLine.kb).swap(tempLine.kb);  //tempLine.kb.swap(std::vector<double>());

			tempPoints.clear();
			std::vector<db::Point2f>(tempPoints).swap(tempPoints);  //tempPoints.swap(std::vector<db::Point2f>());
		}
		else
		{
			if (dis_2 > 0.08)
			{
				if (i != 0)
					i--;

				tempLine.kb = kb2;
				if (atan(tempLine.kb[0]) >= 0)
					tempLine.sigma = atan(tempLine.kb[0]);   //0-180度
				else
					tempLine.sigma = atan(tempLine.kb[0]) + 3.1415926;

				lineSets.push_back(tempLine);

				tempLine.Points.clear();
				std::vector<db::Point2f>(tempLine.Points).swap(tempLine.Points);//tempLine.Points.swap(std::vector<db::Point2f>());
				tempLine.PointsIndex.clear();
				std::vector<int>(tempLine.PointsIndex).swap(tempLine.PointsIndex);//tempLine.PointsIndex.swap(std::vector<int>());
				tempLine.kb.clear();
				std::vector<double>(tempLine.kb).swap(tempLine.kb);  //tempLine.kb.swap(std::vector<double>());

				tempPoints.clear();
				std::vector<db::Point2f>(tempPoints).swap(tempPoints);  //tempPoints.swap(std::vector<db::Point2f>());
				continue;
			}
			if (dis_3 > 0.08)
			{
				if (i != 0)
					i--;

				tempLine.kb = kb3;
				if (atan(tempLine.kb[0]) >= 0)
					tempLine.sigma = atan(tempLine.kb[0]);   //0-180度
				else
					tempLine.sigma = atan(tempLine.kb[0]) + 3.1415926;

				lineSets.push_back(tempLine);

				tempLine.Points.clear();
				std::vector<db::Point2f>(tempLine.Points).swap(tempLine.Points);//tempLine.Points.swap(std::vector<db::Point2f>());
				tempLine.PointsIndex.clear();
				std::vector<int>(tempLine.PointsIndex).swap(tempLine.PointsIndex);//tempLine.PointsIndex.swap(std::vector<int>());
				tempLine.kb.clear();
				std::vector<double>(tempLine.kb).swap(tempLine.kb);  //tempLine.kb.swap(std::vector<double>());

				tempPoints.clear();
				std::vector<db::Point2f>(tempPoints).swap(tempPoints);  //tempPoints.swap(std::vector<db::Point2f>());
				continue;
			}
			kb = LineFitting(tempPoints);
			//kb = leastSquareFitting(tempPoints);
			kb3 = kb2;
			kb2 = kb1;
			kb1 = kb;

			tempLine.Points.push_back(rPoints[i]);
			tempLine.PointsIndex.push_back(i);

			if (i == (rPoints.size() - 1))
			{
				tempLine.kb = kb1;
				if (atan(tempLine.kb[0]) >= 0)
					tempLine.sigma = atan(tempLine.kb[0]);   //0-180度
				else
					tempLine.sigma = atan(tempLine.kb[0]) + 3.1415926;

				lineSets.push_back(tempLine);
			}
		}
	}
	return;
}

//直线段的优化处理  合并零散线段
int AngleRegister::reprocessLineset(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets)
{
	if (lineSets.size() == 0)
		return -1;

	std::vector<db::line> templineSets;
	std::vector<db::line> temp;
	bool errorline = false;
	for (size_t i = 0; i < lineSets.size(); i++)
	{
		if (lineSets[i].Points.size() < 5)
		{
			temp.push_back(lineSets[i]);
			errorline = true;
		}
		else
		{
			if (errorline && temp.size()>0)
			{
				db::line templine;
				double k = 0.0, b = 0.0, sigma = 0.0;
				for (size_t j = 0; j < temp.size(); j++)
				{
					templine.Points.insert(templine.Points.end(), temp[j].Points.begin(), temp[j].Points.end());
					templine.PointsIndex.insert(templine.PointsIndex.end(), temp[j].PointsIndex.begin(), temp[j].PointsIndex.end());
					k += temp[j].kb[0];
					b += temp[j].kb[1];
					sigma += temp[j].sigma;
				}
				templine.kb.push_back(k / temp.size());
				templine.kb.push_back(b / temp.size());
				templine.sigma = sigma / temp.size();

				finalLineSets.push_back(templine);

				errorline = false;

				temp.clear();
				std::vector<db::line>(temp).swap(temp); //temp.swap(std::vector<db::line>());
			}
			finalLineSets.push_back(lineSets[i]);
		}
	}

}
int AngleRegister::reprocessLineset2(std::vector<db::line> &lineSets, std::vector<db::line>& finalLineSets)
{
	if (lineSets.size() == 0)
	{
		return -1;
	}
	//合并近似平行直线
	std::vector<db::line> lineSets_temp;
	double distance_R = 100.0;
	for (size_t i = 0; i < lineSets.size(); i++)
	{
		double distance_L = 100.0;
		if (i != (lineSets.size() - 1))
		{
			distance_L = sqrt(
				(lineSets[i].LRPoints.endPoint.x - lineSets[i + 1].LRPoints.firstPoint.x)
				*(lineSets[i].LRPoints.endPoint.x - lineSets[i + 1].LRPoints.firstPoint.x)
				+ (lineSets[i].LRPoints.endPoint.y - lineSets[i + 1].LRPoints.firstPoint.y)
				*(lineSets[i].LRPoints.endPoint.y - lineSets[i + 1].LRPoints.firstPoint.y));
		}

		if ((lineSets[i].LRangle.leftangle == 0 && distance_L <= 0.5) ||
			(lineSets[i].LRangle.rightangle == 0 && distance_R <= 0.5))
		{
			lineSets_temp.push_back(lineSets[i]);
		}
		else
		{
			if (lineSets_temp.size()>0)
			{
				db::line templine;
				double k = 0.0, b = 0.0, sigma = 0.0;
				for (size_t j = 0; j < lineSets_temp.size(); j++)
				{
					templine.Points.insert(templine.Points.end(), lineSets_temp[j].Points.begin(), lineSets_temp[j].Points.end());
					templine.PointsIndex.insert(templine.PointsIndex.end(), lineSets_temp[j].PointsIndex.begin(), lineSets_temp[j].PointsIndex.end());
					k += lineSets_temp[j].kb[0];
					b += lineSets_temp[j].kb[1];
					sigma += lineSets_temp[j].sigma;
				}
				templine.kb.push_back(k / lineSets_temp.size());
				templine.kb.push_back(b / lineSets_temp.size());
				templine.sigma = sigma / lineSets_temp.size();

				finalLineSets.push_back(templine);

				lineSets_temp.clear();
				vector<db::line>(lineSets_temp).swap(lineSets_temp);  //lineSets_temp.swap(std::vector<db::line>());
			}

			finalLineSets.push_back(lineSets[i]);
		}

		distance_R = distance_L;
	}
	return 0;
}

// 获取直线KB参数
void AngleRegister::getLinePara(float& x1, float& y1, float& x2, float& y2, db::LinePara & LP)
{
	double m = 0;
	// 计算分子
	m = x2 - x1;
	if (0 == m)
	{
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	}
	else

	{
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}

}

//求解两个向量的夹角
void AngleRegister::getTwoVector_angle(double(&a)[2], double(&b)[2], double & sigema)
{
	double ab, a1, b1, cosr;
	ab = a[0] * b[0] + a[1] * b[1];
	a1 = sqrt(a[0] * a[0] + a[1] * a[1]);
	b1 = sqrt(b[0] * b[0] + b[1] * b[1]);
	cosr = ab / a1 / b1;
	sigema = acos(cosr);//* 180 / 3.1415926;
}

//求解两直线的交点
db::Point2f AngleRegister::getTwoline_cornerPoint(db::line firstline, db::line secondline)
{
	db::Point2f point;
	point.x = (firstline.kb[1] - secondline.kb[1]) / (firstline.kb[0] - secondline.kb[0]);
	point.y = firstline.kb[0] * point.x + firstline.kb[1];
	return point;
}

//求解一个向量与x轴正方向的逆时针夹角
void AngleRegister::getVector_Xangle(double(&a)[2], double &sigema)
{

	double b[2] = { 1,0 };
	if (a[0]>0 && a[1] >= 0)
	{
		getTwoVector_angle(a, b, sigema);
	}
	else if (a[0] <= 0 && a[1] >0)
	{
		getTwoVector_angle(a, b, sigema);
	}
	else if (a[0] < 0 && a[1] <= 0)
	{
		getTwoVector_angle(a, b, sigema);
		sigema += 3.1415926;
	}
	else if (a[0] >= 0 && a[1] < 0)
	{
		getTwoVector_angle(a, b, sigema);
		sigema += 3.1415926;
	}
}

// 求解2d 点云中 直线左右相邻夹角  以及左右直线的交点
//  2018.11.7 修正
void AngleRegister::LRangleFromLines2(std::vector<db::line> &lineSets)
{
	//功能：：求解直线的左右夹角特征。
	//定理：两个向量与X轴正方向的同向夹角，大夹角减去小夹角 一定等于这两个向量的方向夹角 或 夹角+PI。
	//****  在数学中斜率夹角是逆时针的，方向射线，左值空间与右值空间  来判断向量的同向性？
	//****  在这里规定 绕原点逆时针 循环 前个直线段i 在后个直线段i+1的左值空间内。

	db::line firstline;
	db::line secondline;

	for (size_t i = 0; i < lineSets.size(); i++)
	{
		if (i == 0)
		{
			firstline = lineSets[i];
			db::Point2f endFootPoint = getFootPoint(firstline.kb, firstline.Points[0]);   //第1条直线的第一个点 垂足
			lineSets[i].LRPoints.firstPoint = endFootPoint;
			continue;
		}
		secondline = lineSets[i];

		double distance = getPointToPoint_Distance(firstline.Points[firstline.Points.size() - 1], secondline.Points[0]);   // 2018.11.7加入
		bool have_little_line = !((firstline.Points.size() < 6) && (secondline.Points.size() < 6));
		// 判断是否平行   求取交点
		if (abs(firstline.kb[0] - secondline.kb[0]) > 1.0 && distance< 0.3 && have_little_line)//0.5
		{
			//交点
			db::Point2f pointtemp;
			pointtemp.x = (secondline.kb[1] - firstline.kb[1]) / (firstline.kb[0] - secondline.kb[0]);
			pointtemp.y = firstline.kb[0] * pointtemp.x + firstline.kb[1];

			db::Point2f R_endFootPoint = getFootPoint(firstline.kb, firstline.Points[firstline.Points.size() - 1]);  //第一条直线的最后一个点 垂足
			db::Point2f L_endFootPoint = getFootPoint(secondline.kb, secondline.Points[0]);   //第二条直线的第一个点 垂足

																							  //判断第二个向量在第一个向量的左值空间 还是右值空间
																							  //R L向量	 x轴正方向的夹角比较
			double R[2] = { R_endFootPoint.x - pointtemp.x, R_endFootPoint.y - pointtemp.y },
				L[2] = { L_endFootPoint.x - pointtemp.x, L_endFootPoint.y - pointtemp.y };
			double R_sigema = 0.0;
			double L_sigema = 0.0;   //x轴正方向的夹角
			getVector_Xangle(R, R_sigema);
			getVector_Xangle(L, L_sigema);

			double sigema_temp = 0.0;      //(0-PI)
			if ((L_sigema - R_sigema)>0 && (L_sigema - R_sigema)<3.1415926)
			{
				getTwoVector_angle(R, L, sigema_temp);
			}
			else
			{
				double Ltemp[2] = { pointtemp.x - L_endFootPoint.x, pointtemp.y - L_endFootPoint.y };
				getTwoVector_angle(R, Ltemp, sigema_temp);
			}

			lineSets[i - 1].LRangle.leftangle = sigema_temp;
			lineSets[i].LRangle.rightangle = sigema_temp;

			lineSets[i - 1].LRPoints.endPoint = pointtemp;
			lineSets[i].LRPoints.firstPoint = pointtemp;
			//lineSets[i - 1].LRPoints.endPoint = R_endFootPoint;
			//lineSets[i].LRPoints.firstPoint = L_endFootPoint;
			firstline = secondline;
		}
		else    //  如果前后两条直线近似平行
		{
			db::Point2f R_endFootPoint = getFootPoint(firstline.kb, firstline.Points[firstline.Points.size() - 1]);  //第一条直线的最后一个点 垂足
			db::Point2f L_endFootPoint = getFootPoint(secondline.kb, secondline.Points[0]);   //第二条直线的第一个点 垂足

			lineSets[i - 1].LRPoints.endPoint = R_endFootPoint;
			lineSets[i].LRPoints.firstPoint = L_endFootPoint;

			firstline = secondline;
		}

		//db::Point2f R_endFootPoint = getFootPoint(firstline.kb, firstline.Points[firstline.Points.size() - 1]);  //第一条直线的最后一个点 垂足
		//db::Point2f L_endFootPoint = getFootPoint(secondline.kb, secondline.Points[0]);   //第二条直线的第一个点 垂足
		//lineSets[i - 1].LRPoints.endPoint = R_endFootPoint;
		//lineSets[i].LRPoints.firstPoint = L_endFootPoint;
		//firstline = secondline;

		if (i == (lineSets.size() - 1))
		{
			db::Point2f endFootPoint = getFootPoint(secondline.kb, secondline.Points[secondline.Points.size() - 1]);   //第1条直线的第一个点 垂足
			lineSets[i].LRPoints.endPoint = endFootPoint;
		}
	}

	//
	return;
}
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
double AngleRegister::getRotateAngle(double x1, double y1, double x2, double y2)

{
	const double epsilon = 1.0e-6;
	const double nyPI = acos(-1.0);
	double dist, dot, degree, angle;

	// normalize
	dist = sqrt(x1 * x1 + y1 * y1);
	x1 /= dist;
	y1 /= dist;
	dist = sqrt(x2 * x2 + y2 * y2);
	x2 /= dist;
	y2 /= dist;

	// dot product
	dot = x1 * x2 + y1 * y2;
	if (fabs(dot - 1.0) <= epsilon)
		angle = 0.0;
	else if (fabs(dot + 1.0) <= epsilon)
		angle = nyPI;
	else {
		double cross;
		angle = acos(dot);
		//cross product
		cross = x1 * y2 - x2 * y1;
		// vector p2 is clockwise from vector p1
		// with respect to the origin (0.0)
		if (cross < 0) {
			angle = 2 * nyPI - angle;   //p2在p1的顺时针方向
		}
	}
	//degree = angle *  180.0 / nyPI;
	return angle;
}

// 变换矩阵估计 2dlidar 匹配
int AngleRegister::registration_2dLidar(
	std::vector<db::Point2f>& src_points,
	std::vector<db::Point2f>& targ_points,
	Eigen::Matrix<float, 2, 3>& RT,
	float &theta)
{
	if (src_points.size() == 0 || targ_points.size() == 0)
	{
		return -1;
	}
	std::vector<db::line> src_lineSets;
	std::vector<db::line> src_templineSets;

	getLinesFromPointset(src_points, src_templineSets);
	LRangleFromLines2(src_templineSets);
	//reprocessLineset2(src_lineSets, src_templineSets);
	//LRangleFromLines(src_templineSets);

	std::vector<db::line> targ_lineSets;
	std::vector<db::line> targ_templineSets;

	getLinesFromPointset(targ_points, targ_templineSets);
	LRangleFromLines2(targ_templineSets);
	//reprocessLineset2(targ_lineSets, targ_templineSets);
	//LRangleFromLines(targ_templineSets);

	//1、寻找最优匹配线段
	struct paraLines
	{
		int src_line;
		int targ_line;
		int score = 0;    //if one match 1   two match 2
	};
	std::vector<paraLines> matchLines;
	std::vector<paraLines> matchLines_dis;

	for (size_t i = 0; i < src_templineSets.size(); i++)
	{
		for (size_t j = 0; j < targ_templineSets.size(); j++)
		{
			//line corner
			db::Point2f firstPoint;
			db::Point2f secondPoint;
			firstPoint = src_templineSets[i].LRPoints.firstPoint;
			secondPoint = src_templineSets[i].LRPoints.endPoint;
			float src_distance = sqrt((firstPoint.x - secondPoint.x)*(firstPoint.x - secondPoint.x) +
				(firstPoint.y - secondPoint.y)*(firstPoint.y - secondPoint.y));

			firstPoint = targ_templineSets[j].LRPoints.firstPoint;
			secondPoint = targ_templineSets[j].LRPoints.endPoint;
			float targ_distance = sqrt((firstPoint.x - secondPoint.x)*(firstPoint.x - secondPoint.x) +
				(firstPoint.y - secondPoint.y)*(firstPoint.y - secondPoint.y));

			//判断匹配夹角信息
			if (src_templineSets[i].LRangle.leftangle != 0.0 && targ_templineSets[j].LRangle.leftangle != 0.0
				&& abs(src_templineSets[i].LRangle.leftangle - targ_templineSets[j].LRangle.leftangle) < 0.1)    //0-PI   0.05-0.1
			{
				paraLines ml;
				ml.src_line = i;
				ml.targ_line = j;
				ml.score = 1;
				if (src_templineSets[i].LRangle.rightangle != 0.0 && targ_templineSets[j].LRangle.rightangle != 0.0 &&
					abs(src_templineSets[i].LRangle.rightangle - targ_templineSets[j].LRangle.rightangle) < 0.2)
				{
					ml.score = 2;
				}
				matchLines.push_back(ml);
			}

			// //判断直线相等匹配
			bool pointsNUM = (src_templineSets[i].Points.size() > 5) && (targ_templineSets[j].Points.size() > 5);
			if (pointsNUM && abs(src_distance - targ_distance) < 0.1)
			{
				paraLines ml;
				ml.src_line = i;
				ml.targ_line = j;
				ml.score = 1;
				matchLines_dis.push_back(ml);
			}
		}
	}

	if (matchLines.size() == 0 && matchLines_dis.size() == 0)
	{
		std::cout << "cannot find match lines" << std::endl;
		return -1;
	}

	int numTemp = 0;
	size_t i_temp;

	for (size_t i = 0; i < matchLines.size(); i++)
	{
		//1、变换矩阵 估计
		//db::Point2f src_firtpoint = src_templineSets[matchLines[i].src_line].LRPoints.firstPoint;
		db::Point2f src_firtpoint = getFootPoint(src_templineSets[matchLines[i].src_line].kb, src_templineSets[matchLines[i].src_line].Points[0]);
		db::Point2f src_secondpoint = src_templineSets[matchLines[i].src_line].LRPoints.endPoint;
		db::Point2f firstvector;
		firstvector.x = src_firtpoint.x - src_secondpoint.x;
		firstvector.y = src_firtpoint.y - src_secondpoint.y;

		//db::Point2f targ_firtpoint = targ_templineSets[matchLines[i].targ_line].LRPoints.firstPoint;
		db::Point2f targ_firtpoint = getFootPoint(targ_templineSets[matchLines[i].targ_line].kb, targ_templineSets[matchLines[i].targ_line].Points[0]);
		db::Point2f targ_secondpoint = targ_templineSets[matchLines[i].targ_line].LRPoints.endPoint;
		db::Point2f secondvector;
		secondvector.x = targ_firtpoint.x - targ_secondpoint.x;
		secondvector.y = targ_firtpoint.y - targ_secondpoint.y;

		Eigen::Matrix<float, 2, 3> RT_temp;
		theta = getRotateAngle(firstvector.x, firstvector.y, secondvector.x, secondvector.y);
		RT_temp(0, 0) = cos(theta);
		RT_temp(0, 1) = -sin(theta);
		RT_temp(1, 0) = sin(theta);
		RT_temp(1, 1) = cos(theta);

		float x0 = src_secondpoint.x*cos(theta) - src_secondpoint.y*sin(theta);
		float y0 = src_secondpoint.x*sin(theta) + src_secondpoint.y*cos(theta);

		RT_temp(0, 2) = targ_secondpoint.x - x0;
		RT_temp(1, 2) = targ_secondpoint.y - y0;


		//2、匹配精度结果验证
		struct Tnode * root = NULL;
		root = build_kdtree(targ_points);  //创建kdtree

		int num = 0;
		std::vector<double> distanceVector;

		for (size_t j = 0; j < src_points.size(); j++)
		{
			db::Point2f target;
			target.x = RT_temp(0, 0)*src_points[j].x + RT_temp(0, 1)*src_points[j].y + RT_temp(0, 2);
			target.y = RT_temp(1, 0)*src_points[j].x + RT_temp(1, 1)*src_points[j].y + RT_temp(1, 2);

			db::Point2f nearestpoint;
			double distance = 0.0;
			searchNearest(root, target, nearestpoint, distance);    //临近搜索  最近点

			if (distance<0.3)    //统计距离小于30cm的点个数
			{
				num++;
			}
			distanceVector.push_back(distance);
		}

		if (numTemp < num)
		{
			numTemp = num;
			i_temp = i;
			RT = RT_temp;
		}
	}

	for (size_t i = 0; i < matchLines_dis.size(); i++)
	{
		//1、变换矩阵 估计
		//db::Point2f src_firtpoint = src_templineSets[matchLines[i].src_line].LRPoints.firstPoint;
		db::Point2f src_firtpoint = getFootPoint(src_templineSets[matchLines_dis[i].src_line].kb, src_templineSets[matchLines_dis[i].src_line].Points[0]);
		db::Point2f src_secondpoint = src_templineSets[matchLines_dis[i].src_line].LRPoints.endPoint;
		db::Point2f firstvector;
		firstvector.x = src_firtpoint.x - src_secondpoint.x;
		firstvector.y = src_firtpoint.y - src_secondpoint.y;

		//db::Point2f targ_firtpoint = targ_templineSets[matchLines[i].targ_line].LRPoints.firstPoint;
		db::Point2f targ_firtpoint = getFootPoint(targ_templineSets[matchLines_dis[i].targ_line].kb, targ_templineSets[matchLines_dis[i].targ_line].Points[0]);
		db::Point2f targ_secondpoint = targ_templineSets[matchLines_dis[i].targ_line].LRPoints.endPoint;
		db::Point2f secondvector;
		secondvector.x = targ_firtpoint.x - targ_secondpoint.x;
		secondvector.y = targ_firtpoint.y - targ_secondpoint.y;

		Eigen::Matrix<float, 2, 3> RT_temp;
		double theta = getRotateAngle(firstvector.x, firstvector.y, secondvector.x, secondvector.y);
		RT_temp(0, 0) = cos(theta);
		RT_temp(0, 1) = -sin(theta);
		RT_temp(1, 0) = sin(theta);
		RT_temp(1, 1) = cos(theta);

		float x0 = src_secondpoint.x*cos(theta) - src_secondpoint.y*sin(theta);
		float y0 = src_secondpoint.x*sin(theta) + src_secondpoint.y*cos(theta);

		RT_temp(0, 2) = targ_secondpoint.x - x0;
		RT_temp(1, 2) = targ_secondpoint.y - y0;


		//2、匹配精度结果验证
		struct Tnode * root = NULL;
		root = build_kdtree(targ_points);  //创建kdtree

		int num = 0;
		std::vector<double> distanceVector;

		for (size_t j = 0; j < src_points.size(); j++)
		{
			db::Point2f target;
			target.x = RT_temp(0, 0)*src_points[j].x + RT_temp(0, 1)*src_points[j].y + RT_temp(0, 2);
			target.y = RT_temp(1, 0)*src_points[j].x + RT_temp(1, 1)*src_points[j].y + RT_temp(1, 2);

			db::Point2f nearestpoint;
			double distance = 0.0;
			searchNearest(root, target, nearestpoint, distance);    //临近搜索  最近点

			if (distance<0.3)    //统计距离小于30cm的点个数
			{
				num++;
			}
			distanceVector.push_back(distance);
		}

		if (numTemp < num)
		{
			numTemp = num;
			i_temp = i;
			RT = RT_temp;
		}
	}

	return 0;
}

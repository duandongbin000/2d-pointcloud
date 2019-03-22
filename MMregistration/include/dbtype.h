#pragma once

#include <iostream>
#include <vector>

namespace db
{
	//定义Point2f结构体  

	struct Point2f
	{
		float x;
		float y;
	};

	// 定义直线参数结构体  
	struct LinePara
	{
		float k;
		float b;
	};

	struct line
	{
		std::vector<double> kb;
		std::vector<db::Point2f> Points;       //typedef Point_<int> Point  in opencv
		std::vector<int> PointsIndex;

		struct LRangle
		{
			float leftangle = 0.0;
			float rightangle = 0.0;
		}LRangle;
		//定义向量
		struct LRPoints
		{
			Point2f firstPoint;
			Point2f endPoint;
		}LRPoints;
		double sigma;
	};
}



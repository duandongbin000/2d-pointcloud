#pragma once

#include <iostream>
#include <vector>

namespace db
{
	//����Point2f�ṹ��  

	struct Point2f
	{
		float x;
		float y;
	};

	// ����ֱ�߲����ṹ��  
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
		//��������
		struct LRPoints
		{
			Point2f firstPoint;
			Point2f endPoint;
		}LRPoints;
		double sigma;
	};
}



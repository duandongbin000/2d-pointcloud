/*   ���ߣ�ddb    2018.09.13
*
*    2d/3d ICP SLAM pointcloud registeration
*/

#include"multiRegistration.h"

//system
#include<fstream>
#include<math.h>
#include<random>
#include<stdint.h>
#include <string> 
#include <sstream>

/** \brief NormalEstimation to ponitsCloud
* \param[in]  cloud_in
* \param[out] ponitsCloud with Normal,
*/

multiRegistration::multiRegistration()
	:cloud_icp(new pcl::PointCloud<PointT>)
{
	transformation_matrix=Eigen::Matrix4f::Identity();
	uniform_samplingRadiusSearch=0.1;  // �����������뾶R1
	NormalEstimationRadiusSearch = 0.5;  // ���߼��������뾶R2 > 2R1;
	iterations = 35;  // Ĭ�ϵ�ICP��������
}

multiRegistration::~multiRegistration()
{
}


/********ȡ��λ�ַ�******/
char* Substrend(const char*str, int n)
{
	char *substr = (char*)malloc(n + 1);
	int length = strlen(str);
	if (n >= length)
	{
		strcpy(substr, str);
		return substr;
	}

	int k = 0;
	for (int i = length - n; i<length; i++)
	{
		substr[k] = str[i];
		k++;
	}
	substr[k] = '\0';
	return substr;
}


void multiRegistration::SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

/***********��������ת2d��3d����������**************/

int
multiRegistration::lidarDistanceToPointCloud_2(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud)
{
	if (cloud->size() != 0)
	{
		cloud->clear();
		std::vector<PointT, Eigen::aligned_allocator<PointT>>().swap(cloud->points);
	}

	double pi = 3.1415926;
	double distanceData, angle = 0;
	PointT point;

	ifstream infile(filename);
	string sline;
	std::vector<double> distanceVector;
	if (infile) // �и��ļ�  
	{
		while (getline(infile, sline)) // Fline�в�����ÿ�еĻ��з�  
		{
			sline = sline.replace(sline.find("angle:"), 6, "");
			sline = sline.replace(sline.find("   ,   distance:"), 16, ",");

			vector<string> v;
			SplitString(sline, v, ","); //�ɰ�����ַ����ָ�;
			if (v.size() > 1) {
				angle = atof(v[0].c_str());
				distanceData = atof(v[1].c_str());

				if (distanceData > 100)//����10cm�ڵ���ֵ
				{
					distanceVector.push_back(angle);
					distanceVector.push_back(distanceData);
				}
			}

		}
		infile.close();
	}
	else // û�и��ļ�  
	{
		cout << "error::  no such file: " << filename << endl;
	}

	point.z = 0;
	for (size_t i = 0; i < distanceVector.size(); i += 2)
	{
		angle =distanceVector[i] * pi / 180;
		point.x = (distanceVector[i+1] * cos(angle)) / 1000;
		point.y = -(distanceVector[i+1] * sin(angle)) / 1000;    //2dlidar ��˳ʱ����ת  ���Y��������ת
		point.z = 1.5;
		cloud->points.push_back(point);
	}

	if (cloud->size()==0)
	{
		return -1;
	}
	cloud->resize(cloud->size());
	//cout << "cloud->size" << cloud->size() << endl;

	stringstream ss;
	std::vector<std::string> str;
	SplitString(filename, str, ".");
	ss << str[0] << ".pcd";
	pcl::io::savePCDFile(ss.str(), *cloud);

	return 0;
}

int
multiRegistration::lidarDistanceToPointCloud_3(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud)
{
	if (cloud->size() != 0)
	{
		cloud->clear();
		std::vector<PointT, Eigen::aligned_allocator<PointT>>().swap(cloud->points);
	}

	double pi = 3.1415926;
	double distanceData, angle = 0;
	PointT point;

	ifstream infile(filename);
	string sline;
	std::vector<double> distanceVector;
	if (infile) // �и��ļ�  
	{
		while (getline(infile, sline)) // Fline�в�����ÿ�еĻ��з�  
		{
			vector<string> v;
			SplitString(sline, v, ";"); //�ɰ�����ַ����ָ�;
			if (v.size() > 1) 
			{
				for (size_t i = 0; i < v.size(); i++)
				{
					vector<string> temp;
					SplitString(v[i], temp, ",");

					angle = atof(temp[1].c_str());
					distanceData = atof(temp[2].c_str());

					if (distanceData > 100)//����10cm�ڵ���ֵ
					{
						distanceVector.push_back(angle);
						distanceVector.push_back(distanceData);
					}
				}			
			}
		}
		infile.close();
	}
	else // û�и��ļ�  
	{
		cout << "error::  no lidar file: " << filename << endl;
		return -1;
	}

	point.z = 0;
	for (size_t i = 0; i < distanceVector.size(); i += 2)
	{
		angle = distanceVector[i] * pi / 180;
		point.x = (distanceVector[i + 1] * cos(angle)) / 1000;
		point.y = -(distanceVector[i + 1] * sin(angle)) / 1000;   ////2dlidar ��˳ʱ����ת  ���Y��������ת
		point.z = 1.5;
		cloud->points.push_back(point);
	}

	if (cloud->size() == 0)
	{
		cout << "error::  no data in lidar file : " << filename << endl;
		return -1;
	}
	cloud->resize(cloud->size());
	//cout << "cloud->size" << cloud->size() << endl;

	//���ݴ洢
	//stringstream ss;
	//std::vector<std::string> str;
	//SplitString(filename, str, ".");
	//ss << str[0] << ".pcd";
	//pcl::io::savePCDFile(ss.str(), *cloud);
	return 0;
}

/************ֱ��ͼ��������ƥ��************/
int 
multiRegistration::FpfhSACAlignment(std::vector<pcl::PointCloud<PointT>::Ptr>& pointCloudPtrVect, 
	Eigen::Matrix4f& sac_trans)
{
	pcl::PointCloud<PointT>::Ptr cloud_src_o = pointCloudPtrVect[1];
	pcl::PointCloud<PointT>::Ptr cloud_tgt_o = pointCloudPtrVect[0];

	//������淨��
	//std::vector<int> indices_src; //����ȥ���ĵ������
	//pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src_o);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_src.setKSearch(10);
	ne_src.setRadiusSearch(0.1);
	ne_src.compute(*cloud_src_normals);

	//*********ͬ������Ŀ�����*******
	//std::vector<int> indices_tgt;
	//pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt_o);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(10);
	ne_tgt.setRadiusSearch(0.1);   //Ҫ���ڵ��Ƶ��ܶȰ뾶��2����
	ne_tgt.compute(*cloud_tgt_normals);

	//����FPFH
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src_o);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.15);
	fpfh_src.compute(*fpfhs_src);
	//std::cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt_o);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.15);      //һ�㡾0.02��0.1��  ���ݲ�������ͷ��߹��ư뾶����
	fpfh_tgt.compute(*fpfhs_tgt);
	//std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC��׼
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src_o);
	scia.setInputTarget(cloud_tgt_o);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);

	//scia.setMinSampleDistance(1);
	scia.setNumberOfSamples(3);   //Ĭ��nr_samples_(3), min_sample_distance_ (0.0f), k_correspondences_ (10)
	//sac_ia.setMaximumIterations(100);
	scia.setEuclideanFitnessEpsilon(0.001);
	scia.setTransformationEpsilon(1e-10);
	scia.setRANSACIterations(35);

	pcl::PointCloud<PointT>::Ptr sac_result(new pcl::PointCloud<PointT>);
	double scoreTemp=100.0;
	scia.setCorrespondenceRandomness(15);   //������һ���Թ��Ƶ��ٽ���������Ŀ   Ӱ��SAC�ľ��� [5-20] 10,15,20
	scia.align(*sac_result);
	
	//for (size_t i = 10; i < 21;)
	//{
	//	scia.setCorrespondenceRandomness(i);   //������һ���Թ��Ƶ��ٽ���������Ŀ   Ӱ��SAC�ľ��� [5-20] 10,15,20
	//	scia.align(*sac_result);
	//	if (scoreTemp > scia.getFitnessScore())
	//	{
	//		scoreTemp = scia.getFitnessScore();
	//		sac_trans = scia.getFinalTransformation();
	//	};
	//	i+=5;
	//}
	
	//std::cout << "fpfh sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	sac_trans = scia.getFinalTransformation();
	//std::cout << sac_trans << endl;
	return 0;
}

/************��վ����վ���icp************/
void
multiRegistration::multiStationRegistration(pcl::PointCloud<PointT>::Ptr& ScanSetTarget, 
	pcl::PointCloud<PointT>::Ptr& ScanSetSource, double& ICPscore)
{
	//pcl::PointCloud<PointT>::Ptr cloud_icp(new pcl::PointCloud<PointT>());  // ICP �������
	
	// ����������㷨
	//time.tic();        //ʱ��
	//std::cout << "��ʼmultiStationRegistrationƥ��......" << std::endl;
	Eigen::Matrix4f sac_trans = Eigen::Matrix4f::Identity();
	//FpfhSACAlignment(pointCloudPtrVect, sac_trans);

	//�����˲�
	pcl::PointCloud<PointT>::Ptr temp000(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr temp001(new pcl::PointCloud<PointT>());
	pcl::VoxelGrid<PointT> vox_grid;
	vox_grid.setLeafSize(0.1, 0.1, 0.1);
	vox_grid.setInputCloud(ScanSetTarget);
	vox_grid.filter(*temp000);
	//�����˲�
	vox_grid.setInputCloud(ScanSetSource);
	vox_grid.filter(*temp001);

	//kfpcs
	pcl::PointCloud <PointT>::Ptr keypoint_cloud_source_aligned(new pcl::PointCloud<PointT>);
	float voxel_size = 0.1f; // optimally set to estimated scan resolution in overlap
	float min_contrast = 0.02f; // set to extract approximately 1000-5000 keypoints
	float approx_overlap = 0.9f; // rough estimation of scan overlap
	float abort_score = 0.0f; // zero to avoid early termination
	pcl::registration::KFPCSInitialAlignment <PointT, PointT> kfpcs_ia;
	kfpcs_ia.setInputSource(temp001);
	kfpcs_ia.setInputTarget(temp000);
	kfpcs_ia.setApproxOverlap(approx_overlap);    //
	kfpcs_ia.setScoreThreshold(abort_score);
	kfpcs_ia.setDelta(voxel_size, false);
	kfpcs_ia.setNumberOfSamples(5000);
	kfpcs_ia.setNumberOfThreads(1);
	kfpcs_ia.align(*keypoint_cloud_source_aligned);

	Eigen::Matrix4f init_transform;
	init_transform = kfpcs_ia.getFinalTransformation();
	//std::cout << "kfpcs_ia_transform::" << init_transform << std::endl;

	//icp
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);    //��������������iterations=true
	icp.setInputSource(ScanSetSource);   //��������ĵ���
	icp.setInputTarget(ScanSetTarget);    //Ŀ�����
	icp.setMaxCorrespondenceDistance(0.5); // ����������
	icp.setMaximumIterations(100); // ���α仯����֮��Ĳ�ֵ
	icp.setTransformationEpsilon(1e-10); // �������
	icp.setEuclideanFitnessEpsilon(0.00001);   //������ƥ��ľ���   ԽС����Խ�� 0.001
	icp.align(*cloud_icp, init_transform);    // icp �����ó�ʼ������sac_trans�������� 
	cloud_icp->resize(ScanSetTarget->size());

	std::vector<pcl::PointCloud<PointT>::Ptr> pointCloudPtrVect;
	pointCloudPtrVect.push_back(ScanSetTarget);
	pointCloudPtrVect.push_back(ScanSetSource);
	if (icp.hasConverged())                 //icp.hasConverged ()=1��true������任������ʺ�������
	{
		//std::cout << "\n multiStationRegistration has converged, score_1 is " << icp.getFitnessScore() << std::endl;
		if (icp.getFitnessScore() > 1.0)
		{
			FpfhSACAlignment(pointCloudPtrVect, sac_trans);
			icp.align(*cloud_icp, sac_trans);
			//std::cout << "\nICP has converged, score_2 is " << icp.getFitnessScore() << std::endl;
		}

		ICPscore=icp.getFitnessScore();

		transformation_matrix = icp.getFinalTransformation();
		print4x4Matrix(transformation_matrix);
	}
}

void
multiRegistration::print4x4Matrix(const Eigen::Matrix4f & matrix)    //��ӡ��ת�����ƽ�ƾ���
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

int 
multiRegistration::Save4x4MatrixVect(std::vector<Eigen::Matrix4f> &transformation_matrix_Vect, 
	std::vector<double>& ICPscore, std::string &MatrixName, std::string &fliePath)   //  Save4x4MatrixVect
{
	ofstream ofile;   //��������ļ�
	std::string fileName = fliePath +"/" +MatrixName;
	ofile.open(fileName, ios::app);     //���ļ�ʱ��������ļ������ڣ����Զ������ļ�����ָ��Ŀ¼������ڣ��������ļ�ʧ�ܡ�
	for (size_t i = 0; i < transformation_matrix_Vect.size(); i++)
	{
		ofile << "pointCloud_station_" << i + 2 << "::" << endl << transformation_matrix_Vect[i] << endl
			<< "ICP score ::" << ICPscore[i] << endl
			<< "վ��λ��(x,y,z)Ϊ��" << transformation_matrix_Vect[i](0, 3) << "," << transformation_matrix_Vect[i](1, 3) << "," << transformation_matrix_Vect[i](2, 3) << endl << endl;   //д���ļ�
		
	}
	return 0;
}

/********�ݹ������ת����***************/
Eigen::Matrix4f faction(int i, const std::vector<Eigen::Matrix4f> &M)
{
	if (M.size() == 0)
	{
		std::cout << "transformation_matrix_Vect is empty...." << std::endl;
	}

	Eigen::Matrix4f product;    //�˻�
	if (i == 0)
	{
		product = M[0];
	}
	else
	{
		product = faction(i-1,M)* M[i];
	}
	return product;
}

int
multiRegistration::getAbsTransationMatrix(const std::vector<Eigen::Matrix4f> &transformation_matrix_Vect, std::vector<Eigen::Matrix4f> &AbsTransationMatrix)
{
	if (transformation_matrix_Vect.size()==0)
	{
		std::cout << "transformation_matrix_Vect is empty...." << std::endl;
		return -1;
	}
	for (size_t i = 0; i < transformation_matrix_Vect.size(); i++)
	{
		Eigen::Matrix4f product = faction(i, transformation_matrix_Vect);
		AbsTransationMatrix.push_back(product);
	}
	return 0;
}
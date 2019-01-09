#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp> //pcl�еĵ�������
#include <pcl/filters/statistical_outlier_removal.h>//������ɢ��
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include "cv.h"
#include <iostream>
#include <sstream>

using namespace pcl;

template <class PointT>
bool readPointCloud(const std::string &fileName, pcl::PointCloud<PointT> &outPointCloud)
{
	string _format = fileName.substr(fileName.length() - 3, 3);
	if (_format == "ply")
	{
		if (pcl::io::loadPLYFile<PointT>(fileName, outPointCloud) < 0)
		{
			pcl::console::print_error("Error loading pointCloud file!\n");
			return false;
		}
	}
	else if (_format == "pcd")
	{
		if (pcl::io::loadPCDFile(fileName, outPointCloud) < 0)
		{
			pcl::console::print_error("Error loading pointCloud file!\n");
			return false;
		}
	}
	else
	{
		cout << "file format error!" << endl;
		return false;
	}
	return true;
}

//��ӡ4*4�任����
//print 4*4 matrix
inline void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
	return;
}

template <class PointT>
bool pointCloudICP(pcl::PointCloud<PointT> PointSource, pcl::PointCloud<PointT> PointTarget, pcl::PointCloud<PointT> &outPointCloud)
{
	return false;
}

template <class PointT>
bool addPointCloud(pcl::visualization::PCLVisualizer &viewer, typename pcl::PointCloud<PointT>::Ptr pointCloud, const std::string pcloudID, cv::Scalar &color, int size, bool holdon)  //Ϊʲô����typename�Ϳ����ˣ���������
{
	if (!holdon)
	{
		viewer.removeAllPointClouds();
	}
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(pointCloud, (int)color[0], (int)color[1], (int)color[2]);
	viewer.addPointCloud(pointCloud, cloud_in_color_h, pcloudID);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, pcloudID);
	//viewer.updatePointCloud(pointCloud, pcloudID);//��������ʲô��û��
	//viewer.spin();
	return true;
}

//����ͳ�Ʒ���������Ⱥ��
template <class PointT>
void filterOutlier(typename PointCloud<PointT>::Ptr &srcPointCloud)
{
	//����ԭ���Ե������е������������K����ľ���ľ�ֵ�������еĵ�ľ����ֵ��ƽ��ֵ�ͱ�׼�ȷ��ȫ�־�ֵ�ͱ�׼���
	//mean + std_mul_ * stddev ��Ϊ������ֵdistance_threshold������ std_mul_Ϊ��׼��ķŴ�����������distance_threshold�ж�Ϊ��ɢ�㡣
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(srcPointCloud);
	sor.setMeanK(20);//Set the number of nearest neighbors to use for mean distance estimation.
	sor.setStddevMulThresh(1.0);
	sor.filter(*srcPointCloud);
	return;
}

//������Ƶķֱ��ʣ����ڴ��ڽ϶���ɢ��ĳ�����Ӧ�ȶ���ɢ���޳���
template <class PointT>
double computeCloudResolution(typename PointCloud<PointT>::ConstPtr srcPointCloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(srcPointCloud);
	for (size_t i = 0; i < srcPointCloud->size(); ++i)
	{
		if (!pcl_isfinite((*srcPointCloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		//����˵����1. Ϊ��Ҫ��ѯ�ĵ�������� 2.ΪK����ĸ�����=1ʱ��ʾ��ѯ���Լ���3.Ϊ�������������Ӧ�������� 4.Ϊ����㵽��ѯ���ŷʽ����
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


//�Ե��ƽ����²���
template <class PointT>
bool downSamplePointCloud(typename pcl::PointCloud<PointT>::Ptr &srcPointCloud, const double relSamplingDistance,
	typename pcl::PointCloud<PointT>::Ptr &outCloud, const int method)
{
	//��uniformSampling�Ե��ƽ����²���
	if (method == 1)
	{
		pcl::UniformSampling<PointT> uniform_sampling;
		uniform_sampling.setInputCloud(srcPointCloud);
		uniform_sampling.setRadiusSearch(relSamplingDistance);
		uniform_sampling.filter(*outCloud);
		return true;
	}
	//��voxelGraid�����²���
	if (method == 2)
	{
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(srcPointCloud);
		sor.setLeafSize(relSamplingDistance, relSamplingDistance, relSamplingDistance);
		sor.filter(*outCloud);
		return true;
	}
	//ͨ��ʵ�鷢�֣�����һ���Ƕ�ԭ��������в�����������������ͨ�����ؼ������ķ����в���������Ч���ϣ�����2���죬����Ƿ���һ��6��
	return true;
}

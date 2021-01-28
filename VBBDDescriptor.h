#ifndef __LiDAR_POINT_PROCESS_TOPOLOGY_QUANTIFY_CLASS_DEFINE_H__
#define __LiDAR_POINT_PROCESS_TOPOLOGY_QUANTIFY_CLASS_DEFINE_H__

#include <math.h>
#include "ccPointCloud.h"
#include "DgmOctree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>
#include <vector>
#include "ccStdPluginInterface.h "
//#include "opencv2/shape.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include <opencv2/core/utility.hpp>
#include <iostream>
#include <string>


#include"VBBDDescriptorSC.h"




//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  patches;//存储邻域列表
														  //typedef std::vector<double> FeatureList;   //对于每个点存储它的特征编码的类型
//std::vector<FeatureList>  CodingList; //存储编码列表

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getBBofCCVector3list(QVector<CCVector3> *pts, CCVector3 *left_down, CCVector3 *right_up);

void PointCloud2ccPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, ccPointCloud *output);

void ccPointCloud2PointCloud(ccPointCloud *input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);

void CCVector2ccPointCloud(QVector<CCVector3> *input, ccPointCloud *output);

void ccPointCloud2CCVector(ccPointCloud *input, QVector<CCVector3> *output);

string DoubleToString(double Input);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void KM_algorithm(int n, int **weight);

void VBBDDescriptor_ISS(ccPointCloud *theCloud, ccPointCloud *TransCC);

void VBBDDescriptor_Harris3D(ccPointCloud *theCloud, ccPointCloud *TransCC);

void VBBDDescriptor_NARF(ccPointCloud *theCloud, ccPointCloud *TransCC);

void VBBDDescriptor_SIFT3D(ccPointCloud *theCloud, ccPointCloud *TransCC);

void VBBDDescriptor_NARF(ccPointCloud *theCloud, ccPointCloud *TransCC);

void ConstructLRF(ccPointCloud *theCloud, ccPointCloud *TransCC, ccPointCloud *Neighbor, ccPointCloud *KeyTrans);

CCVector3 TransformMultiScale(Eigen::MatrixXd  m_IsPlaneVector);

void Coordinate_Transform(Eigen::MatrixXd  &m_IsPlaneVector, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

void FeatureComputation(ccPointCloud *keypoints, double gridSize, double h);

void FeatureCoding(ccPointCloud *keypoints, double gridSize, double radius);

void BinaryCoding(ccPointCloud *keypoints);


pcl::Correspondences Large_Scaled_registration(HybridSC RP1, HybridSC  RP2, double i_radius, double gridsize, double i_scale , Eigen::Matrix4f &best_transform, double _ratio);

pcl::Correspondences  Small_Scaled_registration(HybridSC RP1, HybridSC  RP2, double i_radius, double gridsize, double i_scale, Eigen::Matrix4f &best_transform);

void Registation_3DSC(ccPointCloud *model_cloud, ccPointCloud *scene_cloud,
	ccPointCloud* model_key, ccPointCloud *scene_key,
	ccPointCloud *model_corrs, ccPointCloud *scene_corrs,
	ccPointCloud* scene_trans, ccPointCloud * point_cloud,
	ccPointCloud * line_cloud, ccPointCloud * plane_cloud,
	ccMainAppInterface* m_app,
	int current_index,
    int _radius ,
    int _grid ,
    int _sampling,
    double _ratio);

double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud);

//void Registation(ccPointCloud *scene_cloud, ccPointCloud *model_cloud, ccPointCloud* scene_key, ccPointCloud *model_key);
//
//int Compute_3DHoPD(ccPointCloud *scene_cloud, ccPointCloud *model_cloud, ccPointCloud* scene_key, ccPointCloud *model_key);
//
int compute(ccPointCloud *scene_cloud, ccPointCloud *model_cloud, ccPointCloud* scene_key, ccPointCloud *model_key, ccPointCloud *scene_trans);

int TransformationS();

Eigen::Matrix4f RANSAC_Corresponding(pcl::Correspondences corrs, HybridSC RP1, HybridSC RP2, double resolution, pcl::Correspondences corrs_1);

double CalculateAccuracy(pcl::PointCloud<pcl::PointXYZ> cloud_keypoints_1, pcl::PointCloud<pcl::PointXYZ> cloud_keypoints_2, double resolution, double *distance);

int CalulateMultiScalePairs(ccPointCloud *input_1, ccPointCloud* input_2, double resolution);



#include <cstdio>
#include <memory.h>
#include <algorithm> // 使用其中的 min 函数
using namespace std;

struct index_similarity
{
	int index_i;
	int index_j;
	double similarity;
};

class Kuhn_Munkres
{
public:
	int n; // X 的大小
	std::vector<int> weight; // X 到 Y 的映射（权重）
	std::vector<int> lx; // X标号
	std::vector<int> ly; // Y标号	  
	std::vector<bool> sx;// x是否被搜索过
	std::vector<bool> sy;// y是否被搜索过
	std::vector<int> match;// Y(i) 与 X(match [i]) 匹配
public:
	bool path(int u)
	{
		sx[u] = true;
		for (int v = 0; v < n; v++)
			if (!sy[v] && lx[u] + ly[v] == weight[u*n + v])
			{
				sy[v] = true;
				if (match[v] == -1 || path(match[v]))
				{
					match[v] = u;
					return true;
				}
			}
		return false;
	}

	int bestmatch(bool maxsum)
	{
		int i, j;
		if (!maxsum)
		{
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					weight[i*n +j] = -weight[i*n+j];
		}

		// 初始化标号
		for (i = 0; i < n; i++)
		{
			lx[i]= -0x1FFFFFFF;
			ly[i] = 0;
			for (j = 0; j < n; j++)
				if (lx[i] < weight[i*n+j])
					lx[i] = weight[i*n +j];
		}

		for (int  ix = 0; ix < n; ix++)
		{
			match.push_back(-1); 
		}
		for (int u = 0; u < n; u++)
			while (true)
			{
				for (int ix = 0; ix < n; ix++)
				{
					sx[ix]= 0;
				}
				for (int ix = 0; ix < n; ix++)
				{
					sy[ix] = 0;
				}
				if (path(u))
					break;

				// 修改标号
				int dx = 0x7FFFFFFF;
				for (i = 0; i < n; i++)
					if (sx[i])
						for (j = 0; j < n; j++)
							if (!sy[j])
								dx = min(lx[i] + ly[j] - weight[i*n+j], dx);

				for (i = 0; i < n; i++)
				{
					if (sx[i])
						lx[i] -= dx;
					if (sy[i])
						ly[i] += dx;
				}
			}

		int sum = 0;
		for (i = 0; i < n; i++)
			sum += weight[match[i]*n+i];

		if (!maxsum)
		{
			sum = -sum;
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					weight[i*n+j] = -weight[i*n+j]; // 如果需要保持 weight [ ] [ ] 原来的值，这里需要将其还原
		}
		return sum;
	}

	std::vector<index_similarity>  KM_algorithm()
	{
		std::vector<index_similarity> _final;
		for (int ix = 0; ix < n; ix++)
		{
			match.push_back(-1);
			ly.push_back(0);
			lx.push_back(0);
			sy.push_back(0);
			sx.push_back(0);
		}
		
		int cost = bestmatch(true);

		ccLog::Print(QString("[cost] ") + QString::fromStdString(DoubleToString((double)cost)));

		for (int i = 0; i < n; i++)
		{
			index_similarity _current;
			_current.index_i =  match[i];
			_current.index_j = i;
			_current.similarity = weight[match[i] * n + i];
			_final.push_back(_current);
		}
		return _final;
	}
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

#include "VBBDDescriptor.h"
#include <math.h>
#include <vector>
#include "ScalarField.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>

#include "ccColorTypes.h"
#include "ccPolyline.h"
#include "ccPlane.h"
#include "ccStdPluginInterface.h "

#include <io.h>
#include <direct.h>
#include <iostream>
#include <fstream>
#include <list>


#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>//�ؼ�����

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>


#include <pcl/range_image/range_image.h>   
#include <pcl/features/range_image_border_extractor.h>  
#include <pcl/keypoints/narf_keypoint.h>  
#include <pcl/features/narf_descriptor.h>  

#include <pcl/point_cloud.h>     //��������ͷ�ļ�
#include <pcl/correspondence.h>   //��Ӧ��ʾ����ʵ��֮���ƥ�䣨���磬�㣬�������ȣ���
#include <pcl/features/normal_3d_omp.h>   //����
#include <pcl/features/shot_omp.h>    //������
#include <pcl/features/board.h>       
#include <pcl/filters/uniform_sampling.h>   //���Ȳ���
#include <pcl/recognition/cg/hough_3d.h>    //hough����
#include <pcl/recognition/cg/geometric_consistency.h>  //����һ����
#include <pcl/kdtree/kdtree_flann.h>             //��׼����
#include <pcl/kdtree/impl/kdtree_flann.hpp>      //
#include <pcl/common/centroid.h>             //ת������

#include "pcl/common/transforms.h"




typedef QVector<CCVector3 >  LinePtsList;
using namespace CCLib;
using namespace std;

typedef QVector<int > IDList;
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define PI 3.1415926


void getBBofCCVector3list(QVector<CCVector3> *pts, CCVector3 *left_down, CCVector3 *right_up)
{
	if (pts->size()< 0) return;
	left_down->x = right_up->x = pts->front().x;
	left_down->y = right_up->y = pts->front().y;
	left_down->z = right_up->z = pts->front().z;
	CCVector3 *pt_now = NULL;
	for (int i = 0; i< pts->size(); i++)
	{
		pt_now = &pts->data()[(i)];
		if (pt_now->x < left_down->x) left_down->x = pt_now->x;
		if (pt_now->y < left_down->y) left_down->y = pt_now->y;
		if (pt_now->z < left_down->z) left_down->z = pt_now->z;
		if (pt_now->x > right_up->x) right_up->x = pt_now->x;
		if (pt_now->y > right_up->y) right_up->y = pt_now->y;
		if (pt_now->z > right_up->z) right_up->z = pt_now->z;
	}
}

//��ʽת����pcl:PointCloud to ccPointCloud
void PointCloud2ccPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, ccPointCloud *output)
{
	long npts = input->size();
	output->reserve(npts + 1);
	for (long i = 0; i<npts; i++)
	{
		pcl::PointXYZ pAdd = input->at(i);
		const CCVector3 pp(pAdd.x, pAdd.y, pAdd.z);
		output->addPoint(pp);
	}
}

//��ʽת����ccPointCloud to pcl:PointCloud
void ccPointCloud2PointCloud(ccPointCloud *input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
	long npts = input->size();
	output->reserve(npts + 1);
	for (long i = 0; i<npts; i++)
	{
		const CCVector3* P_Residual = input->getPoint(i);
		pcl::PointXYZ *p = new pcl::PointXYZ;
		p->x = P_Residual->x;
		p->y = P_Residual->y;
		p->z = P_Residual->z;
		output->push_back(*p);
		delete[]p;
	}
}

//��ʽת����ccVector to ccPointCloud
void CCVector2ccPointCloud(QVector<CCVector3> *input, ccPointCloud *output)
{
	long npt = input->size();
	output->reserve(npt + 1);
	for (long i = 0; i<npt; i++)
	{
		const CCVector3 pt_now = input->at(i);
		output->addPoint(pt_now);
	}
}

//��ʽת����ccPointCloud to ccVector
void ccPointCloud2CCVector(ccPointCloud *input, QVector<CCVector3> *output)
{
	for (long i = 0; i < input->size(); i++)
	{
		CCVector3 pt_add = *input->getPoint(i);
		output->push_back(pt_add);//��pt_add��ָ��ĵ㸳ֵ��_pts
	}
}

//double2string
string DoubleToString(double Input)
{
	stringstream Oss;
	Oss << Input;
	return Oss.str();
}

//string2double
double StringToDouble(string Input)
{
	double Result;
	stringstream Oss;
	Oss << Input;
	Oss >> Result;
	return Result;
}

/**
* ����������ζ����Ʊ��λ����ͬ������
* @param m ����m
* @param n ����n
* @return ����
*/
int countBitDiff(int m, int n)
{
	//��λ���
	int c = m ^ n;
	char counter = 0;
	for (; c; ++counter) c &= (c - 1);
	return counter;
}


//��ȡ���������Ӳ�������׼
void Registation_3DSC(ccPointCloud *model_cloud, ccPointCloud *scene_cloud,
	ccPointCloud* model_key, ccPointCloud *scene_key,
	ccPointCloud *model_corrs, ccPointCloud *scene_corrs,
	ccPointCloud* scene_trans, ccPointCloud * point_cloud,
	ccPointCloud * line_cloud, ccPointCloud * plane_cloud,
	ccMainAppInterface* m_app, int current_index,
	int _radius,
	int _grid,
	int _sampling,
	double _ratio)
{
	//���������Ϣ
	ccLog::Print(QString("[current_index _radius  _grid  _sampling  _ratio:] ")
		+ QString::fromStdString(DoubleToString(current_index)) + QString("  ") + QString::fromStdString(DoubleToString(_radius))
		+ QString("  ") + QString::fromStdString(DoubleToString(_grid)) + QString("  ") + QString::fromStdString(DoubleToString(_sampling))
		+ QString("  ") + QString::fromStdString(DoubleToString(_ratio)));

	//����������
	HybridSC RP1, RP2;

	//����ģ������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<PointType>());
	ccPointCloud2PointCloud(model_cloud, cloud1);
	RP1.cloud_original = *cloud1;

	//���볡������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<PointType>());
	ccPointCloud2PointCloud(scene_cloud, cloud2);
	RP2.cloud_original = *cloud2;

	//��ȡģ�����ݷֱ���
	RP1.m_resolution = static_cast<double> (RP1.computeCloudResolution(cloud1));
	ccLog::Print(QString("[Resolution on Model] ") + QString::fromStdString(DoubleToString(RP1.m_resolution)));

	//��ȡ�������ݷֱ���
	RP2.m_resolution = static_cast<double> (RP2.computeCloudResolution(cloud2));;//static_cast<double> (computeCloudResolution(cloud2));
	ccLog::Print(QString("[Resolution on Scene] ") + QString::fromStdString(DoubleToString(RP2.m_resolution)));

	//�ؼ��㿪ʼ����ʱ��
	clock_t start1, end1;
	double cpu_time_used1;
	start1 = clock();

	//��ȡģ�͵��ƵĹؼ���,�����
	switch (current_index)
	{
	case 0:
		RP1.detect_keypoints_on_cloud_ISS(2 * RP1.m_resolution, 0.995, 0.995, 10, 20); //SIFT3D(0.0001f, 20, 4, 0.0001f); //
		RP2.detect_keypoints_on_cloud_ISS(2 * RP1.m_resolution, 0.995, 0.995, 10, 20); //SIFT3D(0.0001f, 20, 4, 0.0001f);
		break;
	case 1:
		RP1.detect_keypoints_on_cloud_Harris3D(2 * RP1.m_resolution, 2 * RP1.m_resolution);
		RP2.detect_keypoints_on_cloud_Harris3D(2 * RP1.m_resolution, 2 * RP1.m_resolution);
		break;
	case 2:
		RP1.detect_keypoints_on_cloud_NARF(15 * RP1.m_resolution, 15 * RP1.m_resolution);
		RP2.detect_keypoints_on_cloud_NARF(15 * RP1.m_resolution, 15 * RP1.m_resolution);
		break;
	case 3:
		RP1.detect_keypoints_on_cloud_SIFT3D(0.001f, 20, 4, 0.0002f);
		RP2.detect_keypoints_on_cloud_SIFT3D(0.001f, 20, 4, 0.0002f);
		break;
	case 4:
		RP1.detect_keypoints_on_cloud_uniform(30 * RP1.m_resolution);
		RP2.detect_keypoints_on_cloud_uniform(30 * RP1.m_resolution);
		break;
	case 5:
		RP1.detect_keypoints_on_cloud_voxel(30 * RP1.m_resolution);
		RP2.detect_keypoints_on_cloud_voxel(30 * RP1.m_resolution);
		break;
	}

	ccLog::Print(QString("[Number of Key points on Model] ") + QString::fromStdString(DoubleToString(RP1.cloud_keypoints.size())));
	ccLog::Print(QString("[Number of Key points on Scene] ") + QString::fromStdString(DoubleToString(RP2.cloud_keypoints.size())));

	//�ؼ�������ɵ�ʱ��
	end1 = clock();
	cpu_time_used1 = ((double)(end1 - start1)) / CLOCKS_PER_SEC;
	ccLog::Print(QString("[Time taken for KeyPoints Detection ] ") + QString::fromStdString(DoubleToString((double)cpu_time_used1)));

	//���������ӿ�ʼ����ʱ��
	clock_t start_2, end_2;
	double cpu_time_used_2;
	start_2 = clock();

	//����������
	Eigen::Matrix4f best_transform_1;
	pcl::Correspondences  corrs = Large_Scaled_registration(RP1, RP2, _radius *RP1.m_resolution, _grid, _sampling, best_transform_1, _ratio);

	//��ת�����model���ݴ�����
	pcl::transformPointCloud(RP1.cloud_keypoints, RP1.cloud_keypoints, best_transform_1);
	pcl::transformPointCloud(RP1.cloud_original, RP1.cloud_original, best_transform_1);

	PointCloud2ccPointCloud(RP1.cloud_original.makeShared(), scene_trans);

	//������ȡ��ԭʼ�ؼ���
	PointCloud2ccPointCloud(RP1.cloud_keypoints.makeShared(), model_key);
	PointCloud2ccPointCloud(RP2.cloud_keypoints.makeShared(), scene_key);

	model_corrs->reserve(corrs.size());
	scene_corrs->reserve(corrs.size());

	for (int i = 0; i < corrs.size(); i++)
	{
		ccPointCloud* Correspondings = new ccPointCloud(QString("Correspondings"));

		Correspondings->reserve(2);
		ccPolyline * m_Correspondings = new ccPolyline(Correspondings);
		m_Correspondings->addPointIndex(0, 2);

		CCVector3 pt_o(RP1.cloud_keypoints.at(corrs.at(i).index_query).x, RP1.cloud_keypoints.at(corrs.at(i).index_query).y, RP1.cloud_keypoints.at(corrs.at(i).index_query).z);
		CCVector3 pt_x(RP2.cloud_keypoints.at(corrs.at(i).index_match).x, RP2.cloud_keypoints.at(corrs.at(i).index_match).y, RP2.cloud_keypoints.at(corrs.at(i).index_match).z);

		model_corrs->addPoint(pt_x);
		scene_corrs->addPoint(pt_o);

		Correspondings->addPoint(pt_x);
		Correspondings->addPoint(pt_o);

		ccColor::Rgb col;
		col.r = int(float(255)*float(rand()) / float(RAND_MAX));
		col.g = int(float(255)*float(rand()) / float(RAND_MAX));
		col.b = int(float(255)*float(rand()) / float(RAND_MAX));
		m_Correspondings->setColor(col);
		m_Correspondings->setWidth(1);
		scene_cloud->addChild(m_Correspondings);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------------------------------------------------------*/
pcl::Correspondences Large_Scaled_registration(HybridSC RP1, HybridSC  RP2, double i_radius, double gridsize, double i_scale, Eigen::Matrix4f &best_transform, double _ratio)
{
	//�ؼ��㿪ʼ����ʱ��
	clock_t start1, end1;
	double cpu_time_used1;
	start1 = clock();

	//�����ֲ��ο�����ϵ     
	RP1.ConstructLRF(i_radius, i_scale); //0.015 30
	RP2.ConstructLRF(i_radius, i_scale); //0.015 30

	RP1.FeatureComputation_1(2 * i_radius / gridsize, gridsize, gridsize, gridsize, 4 * i_radius / gridsize);//2 * i_radius / gridsize
	RP2.FeatureComputation_1(2 * i_radius / gridsize, gridsize, gridsize, gridsize, 4 * i_radius / gridsize);//2 * i_radius / gridsize


	//������ȡ��ɵ�ʱ��
	end1 = clock();
	cpu_time_used1 = ((double)(end1 - start1)) / CLOCKS_PER_SEC;
	ccLog::Print(QString("[Time taken for feature extraction ] ") + QString::fromStdString(DoubleToString((double)cpu_time_used1)));


	//����ƥ�俪ʼ����ʱ��
	clock_t start2, end2;
	double cpu_time_used2;
	start2= clock();

	//����ģ�͹ؼ���������ҵ������е����ڽ���
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_LRF;
	kdtree_LRF.setInputCloud(RP2.cloud_keypoints_trans.makeShared());

    //һ���泡���йؼ���ƥ���ģ�͵�����,��ʼ��Ϊ0
	std::vector <int> scene_index(RP2.cloud_keypoints_trans.size(), -1);                                                                
	//һ���泡���йؼ���ƥ���ģ�͵�����ƶȣ���ʼ��Ϊ0,
	std::vector<double> scene_similarity(RP2.cloud_keypoints_trans.size(), -1);
	////��¼���йؼ�����������ƶ�
	int _size = max(RP1.cloud_keypoints.size(), RP2.cloud_keypoints_trans.size());
	std::vector<int> all_similarity(_size*_size, 0);

	pcl::Correspondences corrs;

	//�������ƶȴӸߵ��͵�˳�����˫�����
	for (int i = 0; i < RP1.cloud_keypoints_trans.size(); i++)
	{
		//ģ�͵�ǰ��
		pcl::PointXYZ searchPoint = RP1.cloud_keypoints_trans.at(i);
		//ģ�͵�ǰ�����
		FeatureList search_hybrid_code = RP1.CodingList.at(i);

		float threshold_to_remove_false_positives = i_radius ;   //0.0015 0.003
		std::vector<int> nn_indices;
		std::vector<float> nn_sqr_distances;
		std::vector<double> _similarity_vector;

		if (kdtree_LRF.radiusSearch(searchPoint, threshold_to_remove_false_positives, nn_indices, nn_sqr_distances) > 0)// Based on this threshold, false positives are removed!
		{
			//�ھ���threshold_to_remove_false_positives���ҵ�nn_indices.size()����
			for (int j = 0; j < nn_indices.size(); j++)
			{
					double similarity_coordinate = 0;
					double similarity_tensor = 0;
					double similarity_binary = 0;
					double similarity_lrf = 0;

					//����ÿ���ڽ��㣬�������ƶ�
					FeatureList near_hybrid_code = RP2.CodingList.at(nn_indices.at(j));

					//��ֵ�������ƶ�
					double search_sum = 0, near_sum = 0;
					for (int search_k = 0, near_k = 0; search_k < search_hybrid_code.size() && near_k < near_hybrid_code.size(); search_k++, near_k++)
					{
						//��ͬ�ĸ�����Խ��˵��Խ����ͬ
						if (search_hybrid_code.at(search_k) == near_hybrid_code.at(near_k) /*&& search_hybrid_code.binary.at(search_k) == 1*/)
							similarity_binary++;
					}

					index_similarity _i_j_similarity;
					_i_j_similarity.index_i = i;
					_i_j_similarity.index_j = nn_indices[j];
					_i_j_similarity.similarity = (similarity_binary) ;
					all_similarity[i*_size+ nn_indices[j]] =(int) _i_j_similarity.similarity;

					_similarity_vector.push_back(_i_j_similarity.similarity);
			}

			//std::vector<double>::iterator result;
			//result = std::max_element(_similarity_vector.begin(), _similarity_vector.end());
			//int max_element_index = std::distance(_similarity_vector.begin(), result);
			//double max_similarity = _similarity_vector.at(max_element_index);
	
			//if (max_similarity / (gridsize*gridsize*gridsize) > _ratio)//similarity_vector.at(max_element_index)> 0max_similarity_2/ max_similarity <0.9
			//{
			//	//flag[current_index.scene_index] = 1;
			//	ccLog::Print(QString("[max similarity ] ") + QString::fromStdString(DoubleToString((double)_similarity_vector.at(max_element_index) / (gridsize*gridsize*gridsize))));
			//	pcl::Correspondence corr;
			//	corr.index_query = i;
			//	corr.index_match = nn_indices[max_element_index];
			//	corrs.push_back(corr);
			//}
		}
	}
	//KM���ж���ͼ������ƥ��
	Kuhn_Munkres _km;
	_km.n = _size;
	_km.weight = all_similarity;
	std::vector<index_similarity> _final = _km.KM_algorithm();

	for (int i = 0; i < _final.size(); i++)
	{
		if (_final.at(i).similarity / (gridsize*gridsize*gridsize) > _ratio)
		{
			pcl::Correspondence corr;
			corr.index_query = _final.at(i).index_i;
			corr.index_match = _final.at(i).index_j;
			corrs.push_back(corr);

			//�����Ӧ��ϵ�����ƶ�
			ccLog::Print(QString("[Y -> X] ") + QString::fromStdString(DoubleToString((double)_final.at(i).index_i)) + QString(" -> ")
				+ QString::fromStdString(DoubleToString((double)_final.at(i).index_j)) + QString(" ; ")
				+ QString::fromStdString(DoubleToString((double)_final.at(i).similarity / (gridsize*gridsize*gridsize))));

		}
	}
	ccLog::Print(QString("[feature match ] ") + QString::fromStdString(DoubleToString((double)corrs.size())));

	//�ؼ�������ɵ�ʱ��
	end2 = clock();
	cpu_time_used2 = ((double)(end2 - start2)) / CLOCKS_PER_SEC;
	ccLog::Print(QString("[Time taken for surface matching ] ") + QString::fromStdString(DoubleToString((double)cpu_time_used2)));


	//��׼��ʼ����ʱ��
	clock_t start3, end3;
	double cpu_time_used3;
	start3 = clock();


	//ransac �޳��ֲ�
	pcl::CorrespondencesConstPtr corrs_const_ptr = boost::make_shared< pcl::Correspondences >(corrs);
	pcl::Correspondences corr_shot;
	pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ > Ransac_based_Rejection_shot;
	Ransac_based_Rejection_shot.setInputSource(RP1.cloud_keypoints.makeShared());
	Ransac_based_Rejection_shot.setInputTarget(RP2.cloud_keypoints.makeShared());
	Ransac_based_Rejection_shot.setInlierThreshold(10 * RP1.m_resolution);//0.005
	Ransac_based_Rejection_shot.setMaximumIterations(10000);
	Ransac_based_Rejection_shot.setInputCorrespondences(corrs_const_ptr);
	Ransac_based_Rejection_shot.getRemainingCorrespondences(corrs, corr_shot);


	//����任����
	Eigen::Matrix4f _transform = Ransac_based_Rejection_shot.getBestTransformation();
	
	//�ؼ�������ɵ�ʱ��
	end3 = clock();
	cpu_time_used3 = ((double)(end3 - start3)) / CLOCKS_PER_SEC;
	ccLog::Print(QString("[Time taken for registration ] ") + QString::fromStdString(DoubleToString((double)cpu_time_used3)));

	best_transform = _transform;
	ccLog::Print(QString("Large Scale Best Transformation: ] ") + QString::fromStdString(DoubleToString(_transform(0, 0))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(0, 1))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(0, 2))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(1, 0))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(1, 1))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(1, 2))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(2, 0))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(2, 1))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(2, 2))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(3, 0))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(3, 1))) + QString(" ")
		+ QString::fromStdString(DoubleToString(_transform(3, 2))) + QString(" ") + QString::fromStdString(DoubleToString(_transform(3, 3))) + QString(" "));

	ccLog::Print(QString("True correspondences after RANSAC: ] ") + QString::fromStdString(DoubleToString((double)corr_shot.size())));

	return corrs;
}





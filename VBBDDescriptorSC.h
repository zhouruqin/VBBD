
#pragma once
#include <iostream>
#include <string>
#include <bitset>
#include <vector>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <time.h>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <valarray>
//#include <sys/time.h>
#include <ctime>
#include <math.h>
//#include "ccPointCloud.h"
#include "DgmOctree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>
#include <vector>
//#include "ccStdPluginInterface.h "
//#include "opencv2/shape.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include <opencv2/core/utility.hpp>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>//�ؼ�����
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>


#include <pcl/range_image/range_image.h>   
#include <pcl/features/range_image_border_extractor.h>  
#include <pcl/keypoints/narf_keypoint.h>  
#include <pcl/features/narf_descriptor.h>  
#include <pcl/filters/uniform_sampling.h>   //���Ȳ���


using namespace std;
using namespace Eigen;
using namespace pcl;





#define max3(a, b, c)  (a > b ? a : b) > c ? (a > b ? a : b) : c
#define PI 3.14159265
#define eps 1e-10

typedef pcl::PointXYZ PointType;

//����Ǹ�SIFT3D׼����
namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () (const PointXYZ &p) const
		{
			return p.z;
		}
	};
}


typedef std::vector<BYTE> FeatureList;   //����ÿ����洢�����������������

										 //�洢����������
struct Nomal
{
	std::vector<double> v1;
	std::vector<double> v2;
	std::vector<double> v3;
};

class HybridSC
{
public:

	pcl::PointCloud<pcl::PointXYZ> cloud_original;  //�洢����ԭʼ����
	pcl::PointCloud<pcl::PointXYZ> cloud_keypoints;  //�洢ԭʼ�ؼ������
	pcl::PointCloud<pcl::PointXYZ> cloud_keypoints_trans;  //�洢ת����ؼ������

														   //��¼ԭʼ���Ƶķֱ���
	double m_resolution;
	//�����ֲ��ο�����ϵLRF
	std::vector<Eigen::Matrix3f> cloud_LRF_descriptors;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_original; //һ��������¼ԭʼ����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_key;//һ��������¼�ؼ������
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  patches;//�洢�����б�
	std::vector<FeatureList>  CodingList; //�洢�����б�

public:
	//������Ƶķֱ���
	double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
	{
		double res = 0.0;
		int n_points = 0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<PointType> tree;
		tree.setInputCloud(cloud);   //�����������

		for (size_t i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
			{
				continue;
			}
			//Considering the second neighbor since the first is the point itself.
			//����ڶ����ٽ��㣬��Ϊ��һ������������
			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);//return :number of neighbors found 
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

	//ƽ������
	void detect_keypoints_on_cloud_uniform(float keypoint_radius)
	{
		pcl::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud(cloud_original.makeShared());  //�������
		uniform_sampling.setRadiusSearch(keypoint_radius);   //���ð뾶
		uniform_sampling.filter(cloud_keypoints);   //�˲�
	}

	//���ز���
	void detect_keypoints_on_cloud_voxel(float keypoint_radius)
	{
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
		voxel_grid.setLeafSize(keypoint_radius, keypoint_radius, keypoint_radius);
		voxel_grid.setInputCloud(cloud_original.makeShared());
		voxel_grid.filter(cloud_keypoints);
	}

	//ISS�㷨
	void detect_keypoints_on_cloud_ISS(double model_resolution, double iss_threshold21, double iss_threshold32, double iss_minneighbors, double iss_numodthread)
	{
		//detect_keypoints_on_cloud_ISS(0.001,0.975,0.975,5,4);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		// Fill in the model cloud
		//	double model_resolution = 0.001;
		// Compute model_resolution
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
		iss_detector.setSearchMethod(tree);
		iss_detector.setSalientRadius(6 * model_resolution);
		iss_detector.setNonMaxRadius(4 * model_resolution);
		iss_detector.setThreshold21(iss_threshold21/*0.975*/);
		iss_detector.setThreshold32(iss_threshold32/*0.975*/);
		iss_detector.setMinNeighbors(iss_minneighbors/*5*/);
		iss_detector.setNumberOfThreads(iss_numodthread/*4*/);
		iss_detector.setInputCloud(cloud_original.makeShared());
		iss_detector.compute(cloud_keypoints);
	}

	//Harris3D�㷨
	void detect_keypoints_on_cloud_Harris3D(double harris_radius, double harris_threshold)
	{
		//ע��Harris��������Ʊ�������ǿ��(I)��Ϣ�ģ���Ϊ����ֵ������I������
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
		harris.setInputCloud(cloud_original.makeShared());
		harris.setNonMaxSupression(true);
		harris.setRadius(0.0015f);//0.01  0.001f
		harris.setThreshold(0.0005f);//0.001    0.0005f

									 //�½��ĵ��Ʊ����ʼ�������㣬����ָ���Խ��
		cloud_out->height = 1;
		cloud_out->width = 100;
		cloud_out->resize(cloud_out->height*cloud_original.makeShared()->width);
		cloud_out->clear();
		harris.compute(*cloud_out);
		int size = cloud_out->size();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_harris->height = 1;
		cloud_harris->width = 100;
		cloud_harris->resize(cloud_out->height*cloud_original.makeShared()->width);
		cloud_harris->clear();

		pcl::PointXYZ point;
		//���ӻ������֧��XYZI��ʽ���ƣ�������Ҫ����XYZ��ʽ��������
		for (int i = 0; i < size; i++)
		{
			point.x = cloud_out->at(i).x;
			point.y = cloud_out->at(i).y;
			point.z = cloud_out->at(i).z;
			cloud_keypoints.push_back(point);
		}
	}

	//sift3D�㷨
	void detect_keypoints_on_cloud_SIFT3D(float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_xyz = cloud_original.makeShared();
		//ccPointCloud2PointCloud(theCloud, cloud_xyz);

		// Parameters for sift computation
		//const float min_scale = 0.0001f; //the standard deviation of the smallest scale in the scale space //ԽС��Խ��
		//const int n_octaves = 20;//the number of octaves (i.e. doublings of scale) to compute
		//const int n_scales_per_octave = 4;//the number of scales to compute within each octave  //ԽС��Խ��
		//const float min_contrast = 0.0001f;//the minimum contrast required for detection 

		// Estimate the sift interest points using z values from xyz as the Intensity variants
		pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
		pcl::PointCloud<pcl::PointWithScale> result;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		sift.setSearchMethod(tree);
		sift.setScales(min_scale, n_octaves, n_scales_per_octave);
		sift.setMinimumContrast(min_contrast);
		sift.setInputCloud(cloud_xyz);
		sift.compute(result);

		// Copying the pointwithscale to pointxyz so as visualize the cloud
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(result, cloud_keypoints);
		//PointCloud2ccPointCloud(cloud_temp, TransCC);

		//TransCC->setRGBColor(255, 0, 0);
	}

	//narf�㷨
	void detect_keypoints_on_cloud_NARF(float angular_resolution, float support_size)
	{
		//float angular_resolution = 0.005f; //Խ���ԽС
		//float support_size = 0.002f;  //Խ���ԽС
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		bool rotation_invariant = true;

		// -----��ȡpcd�ļ����������û�и������򴴽�һ����������-----  
		setUnseenToMaxRange = true;
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		ccPointCloud2PointCloud(theCloud, point_cloud_ptr);

		pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;*/
		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

		// -----�ӵ����д������ͼ��----  
		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		range_image.createFromPointCloud(cloud_original, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
			scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		range_image.integrateFarRanges(far_ranges);
		if (setUnseenToMaxRange)
			range_image.setUnseenToMaxRange();

		// -----��ȡ NARF �ؼ���-----  
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage(&range_image);
		narf_keypoint_detector.getParameters().support_size = support_size;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute(keypoint_indices);

		/*pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;*/
		cloud_keypoints.points.resize(keypoint_indices.points.size());
		for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
			cloud_keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	}

	//��ȡ�ֲ�LRF   ����İ뾶���γ�patch�Ĵ�С
	//��ȡ�ֲ�LRF   ����İ뾶���γ�patch�Ĵ�С
	bool get_LRF(pcl::PointXYZ current_point, float lrf_radius, Eigen::Matrix3f &rf, Eigen::Matrix3f &eVal)
	{
		int current_point_idx;

		pcl::PointCloud<pcl::PointXYZ> cloud_here;
		std::vector<int> nn_indices;
		std::vector<float> nn_sqr_distances;

		if (kdtree_original.radiusSearch(current_point, lrf_radius, nn_indices, nn_sqr_distances) > 0)
		{
			pcl::PointCloud<pcl::PointXYZ> raw_patch;
			pcl::copyPointCloud(cloud_original, nn_indices, cloud_here);
			

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud_here.makeShared());

			std::vector<int> n_indices;
			std::vector<float> n_sqr_distances;

			if (kdtree.radiusSearch(current_point, lrf_radius, n_indices, n_sqr_distances) > 0)
			{
				current_point_idx = n_indices[0];

			}

			const Eigen::Vector4f& central_point = (cloud_here)[current_point_idx].getVector4fMap();

			pcl::PointXYZ searchPoint;
			searchPoint = cloud_here[current_point_idx];

			Eigen::Matrix<double, Eigen::Dynamic, 4> vij(n_indices.size(), 4);

			Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero();

			double distance = 0.0;
			double sum = 0.0;

			int valid_nn_points = 0;
			
			if (n_indices.size() < 5 )
				return false;

			for (size_t i_idx = 0; i_idx < n_indices.size(); i_idx++)
			{
				Eigen::Vector4f pt = cloud_here.points[n_indices[i_idx]].getVector4fMap();
				if (pt.head<3>() == central_point.head<3>())
					continue;

				// Difference between current point and origin
				vij.row(valid_nn_points).matrix() = (pt - central_point).cast<double>();
				vij(valid_nn_points, 3) = 0;

				//��ͬ��Ȩ��
				//distance =1;
				//distance = lrf_radius - sqrt(n_sqr_distances[i_idx]);
				//distance = 1/sqrtf(n_sqr_distances[i_idx] );
				distance = powf(lrf_radius - sqrt(n_sqr_distances[i_idx]), 2);
				//distance = exp(-powf(sqrtf(n_sqr_distances[i_idx] )/ lrf_radius, 2));
				//distance = cos(PI/2 * sqrtf(n_sqr_distances[i_idx] )/lrf_radius);

				// Multiply vij * vij'
				cov_m += distance * (vij.row(valid_nn_points).head<3>().transpose() * vij.row(valid_nn_points).head<3>());

				sum += distance;
				valid_nn_points++;
			}

			if (valid_nn_points < 5)
			{
				//PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
				rf.setConstant(std::numeric_limits<float>::quiet_NaN());
				return false;

				//cout <<"\n\n\n\ Something CRAZY is Happening dude!!! \n\n\n"<< endl;
			}

			cov_m /= sum;

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov_m);

			const double& e1c = solver.eigenvalues()[0];
			const double& e2c = solver.eigenvalues()[1];
			const double& e3c = solver.eigenvalues()[2];

			eVal(0, 0) = e1c; eVal(1, 0) = e2c; eVal(2, 0) = e3c;

			if (!pcl_isfinite(e1c) || !pcl_isfinite(e2c) || !pcl_isfinite(e3c))
			{
				//PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
				rf.setConstant(std::numeric_limits<float>::quiet_NaN());
				return false;
				//cout <<"\n\n\n\ Something CRAZY is Happening dude!!! \n\n\n"<< endl;
			}

			// Disambiguation
			Eigen::Vector4d v1 = Eigen::Vector4d::Zero();
			Eigen::Vector4d v3 = Eigen::Vector4d::Zero();
			v1.head<3>().matrix() = solver.eigenvectors().col(2);
			v3.head<3>().matrix() = solver.eigenvectors().col(0);

			//�ж������᷽��
			int plusNormal = 0, plusTangentDirection1 = 0;
			for (int ne = 0; ne < valid_nn_points; ne++)
			{
				double dp = vij.row(ne).dot(v1);
				if (dp >= 0)
					plusTangentDirection1++;//x��

				dp = vij.row(ne).dot(v3);
				if (dp >= 0)
					plusNormal++;//z��
			}

			//TANGENT
			plusTangentDirection1 = 2 * plusTangentDirection1 - valid_nn_points;
			if (plusTangentDirection1 == 0)
			{
				int points = 5; //std::min(valid_nn_points*2/2+1, 11);
				int medianIndex = valid_nn_points / 2;

				for (int i = -points / 2; i <= points / 2; i++)
					if (vij.row(medianIndex - i).dot(v1) > 0)
						plusTangentDirection1++;

				if (plusTangentDirection1 < points / 2 + 1)
					v1 *= -1;
			}
			else if (plusTangentDirection1 < 0)
				v1 *= -1;

			//Normal
			plusNormal = 2 * plusNormal - valid_nn_points;
			if (plusNormal == 0)
			{
				int points = 5; //std::min(valid_nn_points*2/2+1, 11);
				int medianIndex = valid_nn_points / 2;

				for (int i = -points / 2; i <= points / 2; i++)
					if (vij.row(medianIndex - i).dot(v3) > 0)
						plusNormal++;

				if (plusNormal < points / 2 + 1)
					v3 *= -1;
			}
			else if (plusNormal < 0)
				v3 *= -1;



			////�ж������᷽��2
			//int plusNormal = 0, plusTangentDirection1 = 0;
			//for (int ne = 0; ne < valid_nn_points; ne++)
			//{
			//	double dp = vij.row(ne).dot(v1);
			//	if (dp >= 0)
			//		plusTangentDirection1++;//x��

			//	dp = vij.row(ne).dot(v3);
			//	if (dp >= 0)
			//		plusNormal++;//z��
			//}

			////TANGENT
			//if (plusTangentDirection1 >= valid_nn_points - plusTangentDirection1)
			//	v1 *= 1;
			//else if (plusTangentDirection1 < valid_nn_points - plusTangentDirection1)
			//	v1 *= -1;

			////Normal
			//if (plusNormal >= valid_nn_points - plusNormal)
			//	v3 *= 1;
			//else if (plusNormal < valid_nn_points - plusNormal)
			//	v3 *= -1;

			////�ж������᷽��2
			//int plusNormal = 0, plusTangentDirection1 = 0;
			//for (int ne = 0; ne < valid_nn_points; ne++)
			//{
			//	double dp = vij.row(ne).dot(v1);
			//		plusTangentDirection1 += dp;//x��

			//	dp = vij.row(ne).dot(v3);
			//		plusNormal+= dp;//z��
			//}

			////TANGENT
			//if (plusTangentDirection1 >=0)
			//	v1 *= 1;
			//else if (plusTangentDirection1 < 0)
			//	v1 *= -1;

			////Normal
			//if (plusNormal >= 0)
			//	v3 *= 1;
			//else if (plusNormal < 0)
			//	v3 *= -1;


			rf.row(0).matrix() = v1.head<3>().cast<float>();
			rf.row(2).matrix() = v3.head<3>().cast<float>();
			rf.row(1).matrix() = rf.row(2).cross(rf.row(0));
			//rf.row(3).matrix() = solver.eigenvalues();

			return true;
		}
		else
	    	return false;
	}

	//����LRF
	void ConstructLRF(float patch_radius, int i_scale)
	{
		//�����е��ƽ���KD�����Ա��γɾֲ�����
		kdtree_original.setInputCloud(cloud_original.makeShared());// 

		if (cloud_keypoints.size() == 0)
			return;
														  //����ÿ���ؼ���������´���
		for (int i = 0; i < cloud_keypoints.size(); i++)
		{
			//��ȡ�ؼ����ԭʼ����
			pcl::PointXYZ currentPoint = cloud_keypoints[i];

			std::vector<int> nn_indices;
			std::vector<float> nn_sqr_distances;

			//�����ؼ��������
			if (kdtree_original.radiusSearch(currentPoint, patch_radius, nn_indices, nn_sqr_distances) > 0)
			{
				//��ȡ��ǰ�ؼ���ľֲ����棬����raw_patch����
				pcl::PointCloud<pcl::PointXYZ> raw_patch;
				pcl::copyPointCloud(cloud_original, nn_indices, raw_patch);


				//��������в������Լ��ٵ�������
				pcl::UniformSampling<PointType> uniform_sampling;
				uniform_sampling.setInputCloud(raw_patch.makeShared());  //�������
				uniform_sampling.setRadiusSearch(m_resolution  * i_scale);  //���ð뾶
				uniform_sampling.filter(raw_patch);   //�˲�

				//����LRF
				Eigen::Matrix3f LRF;
				Eigen::Matrix3f eVal;
				get_LRF(currentPoint, 1.5 *patch_radius, LRF, eVal);
				
				//���㵱ǰ�ؼ������ھֲ����������
				Eigen::Vector4f mean_raw_patch;
				pcl::compute3DCentroid(raw_patch, mean_raw_patch);

				//�������ת��������Ϊ���ĵ�����ϵ
				for (int j = 0; j < raw_patch.size(); j++)
				{
					raw_patch.points[j].x = raw_patch.points[j].x - mean_raw_patch[0];
					raw_patch.points[j].y = raw_patch.points[j].y - mean_raw_patch[1];
					raw_patch.points[j].z = raw_patch.points[j].z - mean_raw_patch[2];
				}

				//���ؼ���ת�Ƶ�������Ϊԭ�������ϵ
				currentPoint.x = currentPoint.x - mean_raw_patch[0];
				currentPoint.y = currentPoint.y - mean_raw_patch[1];
				currentPoint.z = currentPoint.z - mean_raw_patch[2];
		
			
				Eigen::Vector3f point_here = LRF * currentPoint.getVector3fMap();
				pcl:PointXYZ pt_o;
				pt_o.x = point_here[0];
				pt_o.y = point_here[1];
				pt_o.z = point_here[2];

				Eigen::Matrix4f TRX = Eigen::Matrix4f::Identity();
				TRX(0, 0) = LRF(0, 0); TRX(0, 1) = LRF(0, 1); TRX(0, 2) = LRF(0, 2);
				TRX(1, 0) = LRF(1, 0); TRX(1, 1) = LRF(1, 1); TRX(1, 2) = LRF(1, 2);
				TRX(2, 0) = LRF(2, 0); TRX(2, 1) = LRF(2, 1); TRX(2, 2) = LRF(2, 2);

				//��¼��LRF
				cloud_LRF_descriptors.push_back(LRF);

				//���ֲ�������ת����LRFΪ�����������ϵ��
				transformPointCloud(raw_patch, raw_patch, TRX);

				//����ÿ���ؼ���ľֲ�����
				patches.push_back(raw_patch.makeShared());

				//�洢ת���Ĺؼ���
				cloud_keypoints_trans.push_back(pt_o);
			}

		}
		kdtree_key.setInputCloud(cloud_keypoints.makeShared());// Ϊ�ؼ��㽨��kd����������ڲ����Ҳ��ظ�
	}

	//������˹��Ȩ�ܶȼ���
	void FeatureComputation_1(double gridSize, int _col, int _row, int _height, double h)
	{
		//��ȡ�ؼ�����Ŀ
		long npt = cloud_keypoints_trans.size();

		//����ÿ���ؼ���
		for (int i = 0; i < npt; i++)
		{
			//��ȡ��ǰ�ؼ���ת��������Ϊ���ĺ������
			pcl::PointXYZ current_key = cloud_keypoints_trans.at(i);

			//�Թؼ���������kd-tree
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_neighbor;
			kdtree_neighbor.setInputCloud(patches.at(i));

			//��¼neighbor�е�ļ�Ȩ���ܶȲ���ʼ��
			double weighted_density_neighbor = 0;

			//��¼��ǰ�ؼ������ڸ������ܶ�
			std::vector<double> grid_density;

			//�Ը������ĵ�Ϊԭ�㣬�����������򣬲������˹��Ȩ�ܶ�
			for (int irow = 0; irow < _row; irow++)
				for (int icol = 0; icol < _col; icol++)
					for (int iheight = 0; iheight < _height; iheight++)
					{
						//���ݸ�����index��������������ĵ�����
						pcl::PointXYZ center_grid;
						center_grid.x = current_key.x + (icol - _col / 2) * gridSize;
						center_grid.y = current_key.y + (irow - _row / 2) * gridSize;
						center_grid.z = current_key.z + (iheight - _height / 2) * gridSize;

						//���㵱ǰ�����ļ�Ȩ�ܶȣ���ʼ��
						double weighted_density = 0;
						//��¼�ڻ��������ڵ��ܵ�������ʼ��
						int mj = 0;

						//������ǰ����h��Χ�ڵ��ٽ���
						std::vector<int> nn_indices;
						std::vector<float> nn_sqr_distances;
						if (kdtree_neighbor.radiusSearch(center_grid, h, nn_indices, nn_sqr_distances) > 0)
						{
							//�ҵ���ǰ�����Ļ��������ڵ����е�
							pcl::PointCloud<pcl::PointXYZ> raw_patch;
							pcl::copyPointCloud(*patches.at(i), nn_indices, raw_patch);

							//����h��Χ�ڵ��ٽ�����м�Ȩ�ܶȼ���
							for (int k = 0; k < raw_patch.size(); k++)
							{
								pcl::PointXYZ tn = raw_patch.at(k);
								double _distance, _angle;
								//�������
								if (fabs(tn.x- center_grid.x )<= gridSize/2 && fabs(tn.y- center_grid.y)<= gridSize/2 && fabs(tn.z - center_grid.z)<= gridSize/2)
								{
									_distance = 0;
								}
								else
								{
									_distance = powf(tn.x - center_grid.x, 2) + powf(tn.y - center_grid.y, 2) + powf(tn.z - center_grid.z, 2);
								}
								//�����˹��Ȩ�ܶ�
								if (_distance < 3 * h)
								{
									mj++;
									weighted_density += 1 / (sqrtf(2 * PI)*h)* exp(-_distance / (2 * h*h));
								}
							}

							//��¼ÿ�������ļ�Ȩ�ܶ�����
							if (mj == 0)
								weighted_density = 0;
							else
								weighted_density = weighted_density / mj;

							//��ǰ�ؼ���ĸ������ܼ�Ȩ�ܶ�����
							weighted_density_neighbor += weighted_density;
						}
						//��¼��ǰ�ؼ������и����ļ�Ȩ�ܶȣ�����������
						grid_density.push_back(weighted_density);

					}

			//��ǰ�ؼ���ĸ������ܼ�Ȩ�ܶ�����
			weighted_density_neighbor = weighted_density_neighbor / (_col *_row *_height);

			//����CodingList����¼��ǰ�ؼ����������
			FeatureList _vector;
			CodingList.push_back(_vector);

			//���������Ӳ���ֵ��
			for (int irow = 0; irow < _row; irow++)
				for (int icol = 0; icol < _col; icol++)
				{
					for (int iheight = 0; iheight < _height; iheight++)
					{

						BYTE fb_density;
						if (weighted_density_neighbor == 0)
							fb_density = 0;
						else
						{
							double _d = grid_density.at((icol + irow * _col) * _height + iheight) / weighted_density_neighbor;

							if (_d >= 1)
								fb_density = 1;
							else
								fb_density = 0;
						}
						//��fb������������
						CodingList.at(i).push_back(fb_density);
					}

				}

		}
	}
};



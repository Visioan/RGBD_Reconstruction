#ifndef HELPER_H
#define HELPER_H
//Geometric Registration

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <boost/filesystem.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::FPFHEstimation<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

typedef Eigen::Matrix< double, 6, 6, Eigen::RowMajor > InformationMatrix;

class Configuration {
public:

	std::string path_voc_;
	std::string path_settings_;
	std::string path_sequence_;
	std::string path_association_;
	std::string path_oni_;
	std::string path_traj_;
	std::string path_trajRT_;
	std::string path_KeyPointsRel_;
	std::string path_pcdSave_;
	std::string path_fusedata_; 
	std::string finaltrajQuaternion_;

	bool usebufdata_;

	int segnum_;
	std::string strdatabuf_;
	std::string GrAndOptbufPath_;
	bool vis_;

	int max_iteration_;
	float resample_leaf_;
	float feature_radius;

	int num_of_samples_;
	int correspondence_randomness_;
	float edge_similarity_;
	float inlier_fraction_;
	float inlier_number_ratio;
	float angle_difference_;
	float max_correspondence_distance_;
	bool bcompute_feature_;
	float reg_dist_;

	std::string initraj_;
	std::string segcloudsave_;
	std::string segtraj_;
	int seg_num_;
	float normal_radius_;
	float mcormaxdis_;

	bool usepoint_cache_;
	std::string pointcachetxt_;

	bool global_usecache;
	std::string	global_cache_looptraj;
	std::string	global_cache_loopinfo;
	std::string	global_cache_odomtraj;
	std::string	global_cache_odominfo;
	std::string	global_cache_posetraj;
	std::string cache_featurepath_;
	std::string finaltraj_;

	bool binit_looptraj_;
	std::string strinit_looptraj_;

	std::string finalmodel_;
	std::string cameraparam_;

	std::string congfigname_;

public:
	Configuration(std::string congigname)
		: vis_(false)
		, congfigname_(congigname)
		, binit_looptraj_(false)
		, cameraparam_("cameraparam.txt")
		, finalmodel_("finalmodel.ply")
		, strinit_looptraj_("init_looptraj.txt")
		, reg_dist_(0.02)
		, bcompute_feature_(true)
		, num_of_samples_(-1)
		, correspondence_randomness_(-1)
		, edge_similarity_(-1)
		, inlier_fraction_(-1)
		, inlier_number_ratio(-1)
		, angle_difference_(-1)
		, max_correspondence_distance_(-1)
		, mcormaxdis_(1.0)
		,feature_radius(0.25)
		, global_usecache(false)
		, cache_featurepath_("")
		, finaltrajQuaternion_("final_trajQuaternion.txt")
		, finaltraj_("final_traj.txt")
		, global_cache_looptraj("cache_looptraj.txt")
		, global_cache_loopinfo("cache_loopinfo.txt")
		, global_cache_odomtraj("cache_odomtraj.txt")
		, global_cache_odominfo("cache_odominfo.txt")
		, global_cache_posetraj("cache_posetraj.txt")
		, usepoint_cache_(false)
		, pointcachetxt_("pointbuff.txt")
		, seg_num_(50)
		, normal_radius_(0.02)
		, segtraj_("segtraj.txt")
		, segcloudsave_("")
		, initraj_("initraj_.txt")
		, path_voc_("")
		, path_settings_("")
		, path_sequence_("")
		, path_association_("")
		, path_oni_("")
		, path_traj_("")
		, path_trajRT_("")
		, path_KeyPointsRel_("")
		, path_pcdSave_("")
		, usebufdata_(false)
		, segnum_(0)
		, strdatabuf_("")
		, GrAndOptbufPath_("")
		, path_fusedata_("")
	{
		Init();
	}

	Configuration()
		: vis_(false)
		, congfigname_("config.config")
		, binit_looptraj_(false)
		, cameraparam_("cameraparam.txt")
		, finalmodel_("finalmodel.ply")
		, strinit_looptraj_("init_looptraj.txt")
		, reg_dist_(0.02)
		, bcompute_feature_(true)
		, num_of_samples_(-1)
		, correspondence_randomness_(-1)
		, edge_similarity_(-1)
		, inlier_fraction_(-1)
		, inlier_number_ratio(-1)
		, angle_difference_(-1)
		, max_correspondence_distance_(-1)
		, mcormaxdis_(1.0)
		, feature_radius(0.25)
		, global_usecache(false)
		, cache_featurepath_("")
		, finaltrajQuaternion_("final_trajQuaternion.txt")
		, finaltraj_("final_traj.txt")
		, global_cache_looptraj("cache_looptraj.txt")
		, global_cache_loopinfo("cache_loopinfo.txt")
		, global_cache_odomtraj("cache_odomtraj.txt")
		, global_cache_odominfo("cache_odominfo.txt")
		, global_cache_posetraj("cache_posetraj.txt")
		, usepoint_cache_(false)
		, pointcachetxt_("pointbuff.txt")
		, seg_num_(50)
		, normal_radius_(0.02)
		, segtraj_("segtraj.txt")
		, segcloudsave_("")
		, initraj_("initraj_.txt")
		, path_voc_("")
		, path_settings_("")
		, path_sequence_("")
		, path_association_("")
		, path_oni_("")
		, path_traj_("")
		, path_trajRT_("")
		, path_KeyPointsRel_("")
		, path_pcdSave_("")
		, usebufdata_(false)
		, segnum_(0)
		, strdatabuf_("")
		, GrAndOptbufPath_("")
		, path_fusedata_("")
	{
		Init();
	}

	private:
		void Init()
		{
			if (boost::filesystem::exists(congfigname_)) {
				FILE * f = fopen(congfigname_.c_str(), "r");
				if (f != NULL) {
					char buffer[1024];
					while (fgets(buffer, 1024, f) != NULL) {
						std::string str(buffer);
						std::stringstream ss(str);
						std::string item1, item2;
						std::getline(ss, item1, '=');
						std::getline(ss, item2);
						//
						if (item1.compare("cameraparam_") == 0) {
							cameraparam_ = item2;
							std::cout << "cameraparam_：\t" << cameraparam_ << std::endl;
						}
						if (item1.compare("finalmodel_") == 0) {
							finalmodel_ = item2;
							std::cout << "finalmodel_：\t" << finalmodel_ << std::endl;
						}
						if (item1.compare("binit_looptraj_") == 0) {
							binit_looptraj_ = (item2.compare("true") == 0);
							std::cout << "binit_looptraj_：\t" << binit_looptraj_ << std::endl;
						}
						if (item1.compare("strinit_looptraj_") == 0) {
							strinit_looptraj_ = item2;
							std::cout << "strinit_looptraj_：\t" << strinit_looptraj_ << std::endl;
						}
						if (item1.compare("seg_num_") == 0) {
							seg_num_ = boost::lexical_cast< int >(item2);
							std::cout << "seg_num_ 为：\t" << seg_num_ << std::endl;
						}
						if (item1.compare("reg_dist_") == 0) {
							reg_dist_ = boost::lexical_cast< float >(item2);
							std::cout << "reg_dist_ 为：\t" << reg_dist_ << std::endl;
						}
						if (item1.compare("bcompute_feature_") == 0) {
							bcompute_feature_ = (item2.compare("true") == 0);
							std::cout << "bcompute_feature_：\t" << bcompute_feature_ << std::endl;
						}
						if (item1.compare("num_of_samples_") == 0) {
							num_of_samples_ = boost::lexical_cast< int >(item2);
							std::cout << "num_of_samples_ 为：\t" << num_of_samples_ << std::endl;
						}
						if (item1.compare("correspondence_randomness_") == 0) {
							correspondence_randomness_ = boost::lexical_cast< int >(item2);
							std::cout << "correspondence_randomness_ 为：\t" << correspondence_randomness_ << std::endl;
						}
						if (item1.compare("edge_similarity_") == 0) {
							edge_similarity_ = boost::lexical_cast< float >(item2);
							std::cout << "edge_similarity_ 为：\t" << edge_similarity_ << std::endl;
						}
						if (item1.compare("inlier_fraction_") == 0) {
							inlier_fraction_ = boost::lexical_cast< float >(item2);
							std::cout << "inlier_fraction_ 为：\t" << inlier_fraction_ << std::endl;
						}
						if (item1.compare("inlier_number_ratio") == 0) {
							inlier_number_ratio = boost::lexical_cast< float >(item2);
							std::cout << "inlier_number_ratio 为：\t" << inlier_number_ratio << std::endl;
						}
						if (item1.compare("angle_difference_") == 0) {
							angle_difference_ = boost::lexical_cast< float >(item2);
							std::cout << "angle_difference_ 为：\t" << angle_difference_ << std::endl;
						}
						if (item1.compare("max_correspondence_distance_") == 0) {
							max_correspondence_distance_ = boost::lexical_cast< float >(item2);
							std::cout << "max_correspondence_distance_ 为：\t" << max_correspondence_distance_ << std::endl;
						}
						if (item1.compare("feature_radius") == 0) {
							feature_radius = boost::lexical_cast< float >(item2);
							std::cout << "feature_radius 为：\t" << feature_radius << std::endl;
						}
						if (item1.compare("feature_radius") == 0) {
							feature_radius = boost::lexical_cast< float >(item2);
							std::cout << "feature_radius 为：\t" << feature_radius << std::endl;
						}
						if (item1.compare("cache_featurepath_") == 0) {
							cache_featurepath_ = item2;
							std::cout << "cache_featurepath_：\t" << cache_featurepath_ << std::endl;
						}
						if (item1.compare("mcormaxdis_") == 0) {
							mcormaxdis_ = boost::lexical_cast< float >(item2);
							std::cout << "mcormaxdis_ 为：\t" << mcormaxdis_ << std::endl;
						}
						if (item1.compare("finaltrajQuaternion_") == 0) {
							finaltrajQuaternion_ = item2;
							std::cout << "finaltrajQuaternion_：\t" << finaltrajQuaternion_ << std::endl;
						}
						if (item1.compare("finaltraj_") == 0) {
							finaltraj_ = item2;
							std::cout << "finaltraj_：\t" << finaltraj_ << std::endl;
						}
						if (item1.compare("global_usecache") == 0) {
							global_usecache = (item2.compare("true") == 0);
							std::cout << "global_usecache：\t" << global_usecache << std::endl;
						}
						if (item1.compare("global_cache_looptraj") == 0) {
							global_cache_looptraj = item2;
							std::cout << "global_cache_looptraj 为：\t" << global_cache_looptraj << std::endl;
						}
						if (item1.compare("global_cache_loopinfo") == 0) {
							global_cache_loopinfo = item2;
							std::cout << "global_cache_loopinfo 为：\t" << global_cache_loopinfo << std::endl;
						}
						if (item1.compare("global_cache_odomtraj") == 0) {
							global_cache_odomtraj = item2;
							std::cout << "global_cache_odomtraj 为：\t" << global_cache_odomtraj << std::endl;
						}
						if (item1.compare("global_cache_odominfo") == 0) {
							global_cache_odominfo = item2;
							std::cout << "global_cache_odominfo 为：\t" << global_cache_odominfo << std::endl;
						}
						if (item1.compare("global_cache_posetraj") == 0) {
							global_cache_posetraj = item2;
							std::cout << "global_cache_posetraj 为：\t" << global_cache_posetraj << std::endl;
						}

						if (item1.compare("usepoint_cache_") == 0) {
							usepoint_cache_ = (item2.compare("true") == 0);
							std::cout << "usepointbuff_：\t" << usepoint_cache_ << std::endl;
						}
						if (item1.compare("pointcachetxt_") == 0) {
							pointcachetxt_ = item2;
							std::cout << "pointcachetxt_ 为：\t" << pointcachetxt_ << std::endl;
						}
						if (item1.compare("segtraj_") == 0) {
							segtraj_ = item2;
							std::cout << "segtraj_ 为：\t" << segtraj_ << std::endl;
						}
						if (item1.compare("segcloudsave_") == 0) {
							segcloudsave_ = item2;
							std::cout << "segcloudsave_ 为：\t" << segcloudsave_ << std::endl;
						}
						if (item1.compare("initraj_") == 0) {
							initraj_ = item2;
							std::cout << "initraj_ 为：\t" << initraj_ << std::endl;
						}
						if (item1.compare("usebufdata_") == 0) {
							usebufdata_ = (item2.compare("true") == 0);
							std::cout << "使用 buffer Data：\t" << usebufdata_ << std::endl;
						}
						if (item1.compare("GrAndOptbufPath_") == 0) {
							GrAndOptbufPath_ = item2;
							std::cout << "Gr And Opt 文件路径为：\t" << GrAndOptbufPath_ << std::endl;
						}
						if (item1.compare("segnum_") == 0) {
							segnum_ = boost::lexical_cast< int >(item2);
							std::cout << "fragments 个数为：\t" << segnum_ << std::endl;
						}
						if (item1.compare("normal_radius_") == 0) {
							normal_radius_ = boost::lexical_cast< float >(item2);
							std::cout << "normal_radius_ 为：\t" << normal_radius_ << std::endl;
						}
						if (item1.compare("path_voc_") == 0) {
							path_voc_ = item2;
							std::cout << "词袋文件路径为：\t" << path_voc_ << std::endl;
						}
						if (item1.compare("path_settings_") == 0) {
							path_settings_ = item2;
							std::cout << "SLAM参数设置文件为：\t" << path_settings_ << std::endl;
						}
						if (item1.compare("path_sequence_") == 0) {
							path_sequence_ = item2;
							std::cout << "RGB D sequence文件夹为：\t" << path_sequence_ << std::endl;
						}
						if (item1.compare("path_association_") == 0) {
							path_association_ = item2;
							std::cout << "association.txt 文件路径为：\t" << path_association_ << std::endl;
						}
						if (item1.compare("path_oni_") == 0) {
							path_oni_ = item2;
							std::cout << "oni 文件路径为：\t" << path_oni_ << std::endl;
						}
						if (item1.compare("path_traj_") == 0) {
							path_traj_ = item2;
							std::cout << "转移矩阵[向量]保存文件夹为：\t" << path_traj_ << std::endl;
						}
						if (item1.compare("path_trajRT_") == 0) {
							path_trajRT_ = item2;
							std::cout << "转移矩阵[矩阵]保存文件夹为：\t" << path_trajRT_ << std::endl;
						}
						if (item1.compare("path_KeyPointsRel_") == 0) {
							path_KeyPointsRel_ = item2;
							std::cout << "关键点关系保存文件夹为：\t" << path_KeyPointsRel_ << std::endl;
						}
						if (item1.compare("path_pcdSave_") == 0) {
							path_pcdSave_ = item2;
							std::cout << "点云保存路径为：\t" << path_pcdSave_ << std::endl;
						}
						if (item1.compare("strdatabuf_") == 0) {
							strdatabuf_ = item2;
							std::cout << "buffer 数据保存文件夹为：\t" << strdatabuf_ << std::endl;
						}
						if (item1.compare("path_fusedata_") == 0) {
							path_fusedata_ = item2;
							std::cout << "FuseData 数据保存文件夹为：\t" << path_fusedata_ << std::endl;
						}
						//

						if (item1.compare("max_iteration_") == 0) {
							max_iteration_ = boost::lexical_cast< int >(item2);
							std::cout << "max_iteration_ = " << max_iteration_ << std::endl;
						}

						if (item1.compare("resample_leaf_") == 0) {
							resample_leaf_ = boost::lexical_cast< float >(item2);
							std::cout << "resample_leaf_ = " << resample_leaf_ << std::endl;
						}
					}
				}
			}
			else {
				std::cout << "config.config not found! Use default parameters." << std::endl;
			}
		}
};

struct FramedInformation
{
	int id1_;
	int id2_;
	int frame_;
	InformationMatrix information_;
	FramedInformation(int id1, int id2, int f, InformationMatrix t)
		: id1_(id1), id2_(id2), frame_(f), information_(t)
	{}
};
struct RGBDInformation
{
	std::vector< FramedInformation > data_;

	void LoadFromFile(std::string filename)
	{
		data_.clear();
		int id1, id2, frame;
		InformationMatrix info;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d %d", &id1, &id2, &frame);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(0, 0), &info(0, 1), &info(0, 2), &info(0, 3), &info(0, 4), &info(0, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(1, 0), &info(1, 1), &info(1, 2), &info(1, 3), &info(1, 4), &info(1, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(2, 0), &info(2, 1), &info(2, 2), &info(2, 3), &info(2, 4), &info(2, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(3, 0), &info(3, 1), &info(3, 2), &info(3, 3), &info(3, 4), &info(3, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(4, 0), &info(4, 1), &info(4, 2), &info(4, 3), &info(4, 4), &info(4, 5));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(5, 0), &info(5, 1), &info(5, 2), &info(5, 3), &info(5, 4), &info(5, 5));
					data_.push_back(FramedInformation(id1, id2, frame, info));
				}
			}
			fclose(f);
		}
	}
	void SaveToFile(std::string filename)
	{
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++)
		{
			InformationMatrix & info = data_[i].information_;
			fprintf(f, "%d\t%d\t%d\n", data_[i].id1_, data_[i].id2_, data_[i].frame_);
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0, 0), info(0, 1), info(0, 2), info(0, 3), info(0, 4), info(0, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1, 0), info(1, 1), info(1, 2), info(1, 3), info(1, 4), info(1, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2, 0), info(2, 1), info(2, 2), info(2, 3), info(2, 4), info(2, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3, 0), info(3, 1), info(3, 2), info(3, 3), info(3, 4), info(3, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4, 0), info(4, 1), info(4, 2), info(4, 3), info(4, 4), info(4, 5));
			fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5, 0), info(5, 1), info(5, 2), info(5, 3), info(5, 4), info(5, 5));
		}
		fclose(f);
	}
};
struct FramedTransformation
{
	int id1_;
	int id2_;
	int frame_;
	Eigen::Matrix4d transformation_;
	FramedTransformation(int id1, int id2, int f, Eigen::Matrix4d t)
		: id1_(id1), id2_(id2), frame_(f), transformation_(t)
	{}
};
struct RGBDTrajectory
{
	std::vector< FramedTransformation > data_;
	int index_;

	void LoadFromFile(std::string filename)
	{
		data_.clear();
		index_ = 0;
		int id1, id2, frame;
		Eigen::Matrix4d trans;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL)
		{
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL)
			{
				if (strlen(buffer) > 0 && buffer[0] != '#')
				{
					sscanf(buffer, "%d %d %d", &id1, &id2, &frame);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));
					data_.push_back(FramedTransformation(id1, id2, frame, trans));
				}
			}
			fclose(f);
		}
	}
	void SaveToFile(std::string filename)
	{
		FILE * f = fopen(filename.c_str(), "w");
		for (int i = 0; i < (int)data_.size(); i++)
		{
			Eigen::Matrix4d & trans = data_[i].transformation_;
			fprintf(f, "%d\t%d\t%d\n", data_[i].id1_, data_[i].id2_, data_[i].frame_);
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
			fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
		}
		fclose(f);
	}
};
struct BufferData
{
	void GetBuffer(std::vector<RGBDTrajectory> &vbufferTraj, std::vector<PointCloudT> &vPointCloudBuffer, std::string &strbuf, int &sizeNum)
	{
		int numsize = sizeNum;
		for (size_t i = 0; i < numsize; i++)
		{
			char bufTraj[1024];
			sprintf(bufTraj, "%s/traj/%d_TrajSeg.txt", strbuf.c_str(), i);
			RGBDTrajectory traj;
			traj.LoadFromFile(std::string(bufTraj));
			vbufferTraj.push_back(traj);
			std::cout << "Load Fragments Traj To --->" << std::string(bufTraj) << std::endl;
			char bufCloud[1024];
			sprintf(bufCloud, "%s/PointCloud/%d_TrajSeg.pcd", strbuf.c_str(), i);
			PointCloudT pointcloud;
			pcl::io::loadPCDFile(bufCloud, pointcloud);
			vPointCloudBuffer.push_back(pointcloud);
			std::cout << "Load Fragments PointCloud To --->" << std::string(bufCloud) << std::endl;
		}
		std::cout << "Load Done !" << std::endl;
	}
	void SaveBuffer(std::vector<RGBDTrajectory> &vbufferTraj, std::vector<PointCloudT> &vPointCloudBuffer, std::string &strbuf)
	{
		int numsize = vbufferTraj.size();
		for (size_t i = 0; i < numsize; i++)
		{
			char bufTraj[1024];
			sprintf(bufTraj, "%s/traj/%d_TrajSeg.txt", strbuf.c_str(), i);
			vbufferTraj[i].SaveToFile(std::string(bufTraj));
			std::cout << "Saved Fragments Traj To --->" << std::string(bufTraj) << std::endl;
			char bufCloud[1024];
			sprintf(bufCloud, "%s/PointCloud/%d_TrajSeg.pcd", strbuf.c_str(), i);
			pcl::io::savePCDFile(std::string(bufCloud), vPointCloudBuffer[i], true);
			std::cout << "Saved Fragments PointCloud To --->" << std::string(bufCloud) << std::endl;
		}
		std::cout << "Saved Done !" << std::endl;
	}
};
#endif
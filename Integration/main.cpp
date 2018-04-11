
#include "helper.h"
#include "TSDF.hpp"
#include <opencv2/core/eigen.hpp>
#include "TSDF.hpp"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <boost\timer.hpp>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

using namespace std;

void LoadImages(const string &strSequence, const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
	ifstream fAssociation;
	fAssociation.open(strAssociationFilename.c_str());
	while (!fAssociation.eof())
	{
		string s;
		getline(fAssociation, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenamesRGB.push_back(string(strSequence + "/" + sRGB));
			ss >> t;
			ss >> sD;
			vstrImageFilenamesD.push_back(string(strSequence + "/" + sD));

		}
	}
}


int main()
{

	Configuration config;

	string path_sequence = config.path_sequence_;
	string path_association = config.path_association_;
	string path_oni = config.path_oni_;
	string finaltraj = config.finaltraj_;
	string finalmodelname = config.finalmodel_;
	string cameraparam = config.cameraparam_;
	RGBDTrajectory final_pose;
	final_pose.LoadFromFile(finaltraj);
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps;

	LoadImages(path_sequence, path_association, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
	cv::Mat imD = cv::imread(vstrImageFilenamesD[final_pose.data_[0].id1_], CV_LOAD_IMAGE_UNCHANGED);
	TSDF_Volume volumeFinal(imD.cols, imD.rows, 2048, 3.0 / 512);
	volumeFinal.camera_.LoadFromFile(cameraparam);
	//volumeFinal.uint_resolution = 1;
	//volumeFinal.voxel = 512 * 64;
	//volumeFinal.unit_length_ = 3.0 / 1024;
	volumeFinal.tsdf_trunc_ = 0.003;
	volumeFinal.scale_ = 1000.0;

	for (int i = 0; i < final_pose.data_.size(); i++)
	{
		PCL_INFO("Fuse --->  %d/%d...\n", final_pose.data_[i].id1_, final_pose.data_[i].id2_);

		cv::Mat imRGB = cv::imread(vstrImageFilenamesRGB[final_pose.data_[i].id1_], CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat imD = cv::imread(vstrImageFilenamesD[final_pose.data_[i].id1_], CV_LOAD_IMAGE_UNCHANGED);
		std::vector< unsigned short > vdepthdata(imD.cols*imD.rows);
		std::vector< unsigned int > vrgbdata(imRGB.cols*imRGB.rows);
#pragma omp parallel for num_threads( 8 )
		for (int y = 0; y < imD.rows; y++)
		{
			for (int x = 0; x < imD.cols; x++)
			{
				vdepthdata[y * imD.cols + x] = imD.ptr<ushort>(y)[x];
				unsigned int rgb = imRGB.at<cv::Vec3b>(y, x)[2];
				rgb = rgb << 8;
				rgb = rgb | imRGB.at<cv::Vec3b>(y, x)[1];
				rgb = rgb << 8;
				rgb = rgb | imRGB.at<cv::Vec3b>(y, x)[0];
				vrgbdata[y * imD.cols + x] = rgb;
			}
		}
		volumeFinal.IntegrateWithRGB(vdepthdata, vrgbdata, final_pose.data_[i].transformation_);

	}
	pcl::PointCloud<pcl::PointXYZRGB> pointrgb = volumeFinal.GetPointCloudWithRGB();

	pcl::io::savePCDFileBinary(finalmodelname,pointrgb);
	pcl::console::print_highlight("Saving Final Model --->%s\n", finalmodelname.c_str());
	return 0;
}






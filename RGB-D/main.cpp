
#include "helper.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <boost\timer.hpp>
#include <pcl/console/parse.h>
#include <OpenNI.h>
//#include<System.h>
//#include "../../src/System.h"
#include "System.h"


#include <io.h>  //判断文件夹是否存在
#include <direct.h>  //创建文件夹

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

bool Oni2Sequence(const string &stroni, const string &strSequence, const string &strAssociationFilename)
{
	string path_oni = stroni;
	string sequence_dir = strSequence;
	string sequence_association = strAssociationFilename;
	//string sequence_association = sequence_dir + "/association.txt";
	string sequence_rgb = sequence_dir + "/rgb";
	string sequence_depth = sequence_dir + "/depth";

	//vector<cv::Mat> vrgbMat, vdepthMat;
	//vector<double> vTimestamps;
	pcl::console::print_highlight("开始从 oni 文件中读取数据……\n");
	//ProcessingFromOni(path_oni, vrgbMat, vdepthMat, vTimestamps);

	pcl::console::print_highlight(" 创建文件夹 %s\n", sequence_rgb.c_str());
	if (_access(sequence_rgb.c_str(), 0) == -1)
		_mkdir(sequence_rgb.c_str());
	else
	{
		_rmdir(sequence_rgb.c_str());
		_mkdir(sequence_rgb.c_str());
	}
	pcl::console::print_highlight(" 创建文件夹 %s\n", sequence_depth.c_str());
	if (_access(sequence_depth.c_str(), 0) == -1)
		_mkdir(sequence_depth.c_str());
	else
	{
		_rmdir(sequence_depth.c_str());
		_mkdir(sequence_depth.c_str());
	}
	pcl::console::print_highlight(" 创建文件 %s\n", sequence_association.c_str());

	//
	openni::OpenNI::initialize();
	openni::Device fromOniFile;
	fromOniFile.open(path_oni.c_str());
	openni::PlaybackControl *pController = fromOniFile.getPlaybackControl();
	openni::VideoStream streamColor, streamDepth;
	openni::VideoFrameRef frameColor, frameDepth;

	if (fromOniFile.hasSensor(openni::SENSOR_COLOR) && fromOniFile.hasSensor(openni::SENSOR_DEPTH))
	{
		if (streamColor.create(fromOniFile, openni::SENSOR_COLOR) == openni::STATUS_OK&&streamDepth.create(fromOniFile, openni::SENSOR_DEPTH) == openni::STATUS_OK)
		{
			pcl::console::print_highlight("Create Video Stream Success !\n");
		}
		else
		{
			pcl::console::print_highlight("Create Video Stream Error !\n");
			std::cin.get();
			return false;
		}
	}
	else
	{
		pcl::console::print_error("Device do not have color sensor or depth sensor !\n");
		std::cin.get();
		return false;
	}
	int totalcolor = pController->getNumberOfFrames(streamColor);
	int totaldepth = pController->getNumberOfFrames(streamDepth);
	int total = totalcolor < totaldepth ? totalcolor : totaldepth;
	pcl::console::print_highlight("Images in the sequence: %d\n\n ", total);

	//
	ofstream outf;
	outf.open(sequence_association);
	streamColor.start();
	streamDepth.start();
	for (int i = 0; i < total; i++)
	{
		streamColor.readFrame(&frameColor);
		streamDepth.readFrame(&frameDepth);
		cv::Mat rgbmat(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
		cv::cvtColor(rgbmat, rgbmat, CV_RGB2BGR);

		cv::Mat depthMat(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());

		char imgname[1024];
		sprintf_s(imgname, "%05d.png", i);
		string rgbpath = sequence_rgb + "/" + imgname;
		string depthpath = sequence_depth + "/" + imgname;
		cv::imwrite(rgbpath, rgbmat.clone());
		pcl::console::print_highlight(" write %s\t", rgbpath.c_str());
		cv::imwrite(depthpath, depthMat.clone());
		pcl::console::print_highlight("  %s\t", depthpath.c_str());
		char associations[1024];
		sprintf_s(associations, "%05d\trgb/%05d.png\t%05d\tdepth/%05d.png", i, i, i, i);
		pcl::console::print_highlight(" %s\n", associations);
		//std::cout << "write --->" << associations << std::endl;
		outf << associations << std::endl;
	}
	outf.close();
	pcl::console::print_highlight("Done !!!\n");
	return true;
}

int print_help()
{
	std::cout << "\nApplication parameters:" << endl;
	std::cout << "\n配置 config.config文件" << endl;
	std::cout << "    --help, -h							 : print this message" << endl;
	std::cout << "................................................................................." << endl;
	std::cout << "    --path_voc <path_to_vocabulary>     : GPUSIFT 词袋路径 (必选)				  " << endl;
	std::cout << "    --path_setting <path_to_settings>   : SLAM 参数设置路径(必选)				  " << endl;
	std::cout << "    --path_sequence <path_to_sequence>  : 数据图片 文件夹(A,B 二选1，A)		  " << endl;
	std::cout << "    --path_association <resolution>     : associations 文件路径(A,B 二选1，A)	  " << endl;
	std::cout << "    --path_oni <length>                 : oni 数据文件路径(A,B 二选1，B)		  " << endl;
	std::cout << "    																			  " << endl;
	std::cout << "    (path_sequence,path_association)与(path_oni)两个种数据调用格式可以任选一个	  " << endl;
	std::cout << "    																			  " << endl;
	std::cout << "    --path_traj <path_to_traj>			 : 四元组 转移矩阵保存路径 (必选)	      " << endl;
	std::cout << "    --path_trajRT <path_to_trajRT>		 : 4*4 转移矩阵保存路径(必选)			  " << endl;
	std::cout << "    --path_KeyPointsRel <interval>      : 帧间匹配点保存文件夹(必选)			  " << endl;
	std::cout << "    --path_pcdSave <param_file>         : 点云文件保存路径(必选)				  " << endl;
	return 0;
}

//#define TOOLARGE

int main(int argc, char **argv)
{
	using namespace pcl::console;

	if (find_switch(argc, argv, "--help") || find_switch(argc, argv, "-h")) {
		return print_help();
	}

	Configuration config("tracking.config");
	string path_voc = config.path_voc_;
	string path_settings = config.path_settings_;
	string path_sequence = config.path_sequence_;
	string path_association = config.path_association_;
	string path_oni = config.path_oni_;
	string path_traj = config.path_traj_;
	string path_trajRT = config.path_trajRT_;
	string path_KeyPointsRel = config.path_KeyPointsRel_;
	string path_pcdSave = config.path_pcdSave_;
	string path_fusedata = config.path_fusedata_;
	bool useBufData = config.usebufdata_;
	int Segnum = config.segnum_;
	string strdatabuf = config.strdatabuf_;
	string strGrAndOptdatabuf = config.strdatabuf_;

	if (path_voc.empty() || path_settings.empty() || path_traj.empty() || path_trajRT.empty() || path_KeyPointsRel.empty())
	{
		std::cout << "--path_voc" << path_voc << std::endl;
		std::cout << "--path_setting" << path_settings << std::endl;
		std::cout << "--path_traj" << path_traj << std::endl;
		std::cout << "--path_trajRT" << path_trajRT << std::endl;
		std::cout << "--path_KeyPointsRel" << path_KeyPointsRel << std::endl;
		print_help();
		std::cin.get();
		return 0;
	}
	if ((path_sequence.empty() || path_association.empty()) && path_oni.empty())
	{
		std::cout << "--path_sequence" << path_sequence << std::endl;
		std::cout << "--path_association" << path_association << std::endl;
		std::cout << "--path_oni" << path_oni << std::endl;
		print_help();
		std::cin.get();
		return 0;
	}

	if (path_sequence.empty() || path_association.empty() && path_oni.empty())
	{
		print_error("Error !!!\n");
		print_help();
	}
	if (path_association.empty() && !path_oni.empty())
	{
		path_association = path_sequence + "/associations.txt";
		if (!Oni2Sequence(path_oni, path_sequence, path_association))
		{
			print_error("Error !!!\n");
			print_help();
			return 0;
		}
	}
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps;
	LoadImages(path_sequence, path_association, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

	std::vector<PointCloudT> vpointcloudRGB;
	std::vector<RGBDTrajectory> vvframetransformation;
	BufferData buffdata;

	int volume_cols = 0, volume_rows = 0;
	cv::Mat rgbMat = cv::imread(vstrImageFilenamesRGB[0], CV_LOAD_IMAGE_UNCHANGED);
	cv::Mat depthMat = cv::imread(vstrImageFilenamesRGB[0], CV_LOAD_IMAGE_UNCHANGED);
	volume_cols = depthMat.cols;
	volume_rows = depthMat.rows;

		// Create SLAM system. It initializes all system threads and gets ready to process frames.
		SIFT_SLAM::System SLAM(path_voc, path_settings, SIFT_SLAM::System::RGBD, true);

		// Vector for tracking time statistics

		std::cout << endl << "-------" << endl;
		std::cout << "Start processing sequence ..." << endl;

		int nImages = vstrImageFilenamesRGB.size();
		vector<double> vTimesTrack;
		vTimesTrack.resize(nImages);

		boost::timer timer;
		for (int ni = 0; ni < nImages; ni++)
		{
			// Read image and depthmap from file
			cv::Mat imRGB = cv::imread(vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
			cv::Mat imD = cv::imread(vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

			double tframe = vTimestamps[ni];
			printf("%d\n", ni);
			// Pass the image to the SLAM system
			timer.restart();
			if (SLAM.TrackRGBD(imRGB, imD, tframe, ni, nImages))
			{
				RGBDTrajectory tv = SLAM.HandleRestAndGetFrameTransformation();
				vvframetransformation.push_back(tv);
				SLAM.SetSatartFrameId(ni);//设置ID号

				ni--;//跟踪失败的那一阵作为新片段的第一帧重新跟踪
			}
			cv::waitKey(30);
		}
		//将最后的一个fragment保存
		RGBDTrajectory tv = SLAM.GetLastFrameTransformation();
		vvframetransformation.push_back(tv);
		// Stop all threads
		SLAM.Shutdown();

		std::cout << "-------" << endl << endl;
		std::cout << "Tracking Done ! " << endl;
		SLAM.SaveKeyFrameTrajectoryTUM(path_traj);
		SLAM.SaveTrajectoryTUMRT(path_trajRT);
		SLAM.SaveKeyPointsRelative(config.path_KeyPointsRel_);
		
		SLAM.SaveMat("E:\\Experiment\\code\\Sift_SLAM_v4\\dataCache\\mat");
		SLAM.SaveTestMat("E:\\Experiment\\code\\Sift_SLAM_v4\\dataCache\\mat");
		
		//SLAM.SaveKeyPointsRelative(path_KeyPointsRel);
		std::cout << "Saving Done ! " << endl;

		for (size_t i = 0; i < vvframetransformation.size(); i++)
		{
			char bufTraj[1024];
			sprintf(bufTraj, "%s/traj/%d_TrajSeg.txt", strdatabuf.c_str(), i);
			vvframetransformation[i].SaveToFile(std::string(bufTraj));
		}

	std::cin.get();
	return 0;
}

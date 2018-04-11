/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>


#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
//#include "pointcloudmapping.h"
#include "Tracking.h"
#include "helper.h"
//#include "helper.h"
// From http://stackoverflow.com/questions/5801813/c-usleep-is-obsolete-workarounds-for-windows-mingw
void DLL_API usleep(__int64 usec);

namespace SIFT_SLAM
{
	//class PointCloudMapping;
	class Viewer;//画图	
	class FrameDrawer;//画每一帧	
	class Map;//对map进行操作	
	class Tracking;	//追踪过程
	class LocalMapping;	//局部构图	
	class LoopClosing;	//闭环检测

	//class DLL_API FramedTransformation
	//{
	//public:
	//	int mid1;
	//	int mid2;
	//	int mframe;
	//	Eigen::Matrix4d  mtransformation;
	//	FramedTransformation(){};
	//	FramedTransformation(int id1, int id2, int f, Eigen::Matrix4d t) :mid1(id1), mid2(id2), mframe(f), mtransformation(t){};
	//	~FramedTransformation(){};
	//};

	class DLL_API System
	{
	public:
		// Input sensor输入设备
		enum eSensor{
			MONOCULAR = 0,
			STEREO = 1,
			RGBD = 2
		};

	public:

		// Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
		//对SLAM系统的初始化，包含局部地图，闭环检测，视图三个线程
		System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

		// Proccess the given stereo frame. Images must be synchronized and rectified.
		// Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
		// Returns the camera pose (empty if tracking fails).
		//处理给定立体帧，图片必须是同步且经过纠正的
		//返回相机姿态，如果为空，表示追踪失败
		cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

		// Process the given rgbd frame. Depthmap must be registered to the RGB frame.
		// Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
		// Input depthmap: Float (CV_32F).
		// Returns the camera pose (empty if tracking fails).
		//处理给定的RGBD帧，深度图必须与RGB图配准，输入深度图为float类型（CV_32F）
		//返回值为相机姿态，为空，表示追踪失败
		bool TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,int i,int total);

		// Proccess the given monocular frame
		// Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
		// Returns the camera pose (empty if tracking fails).
		cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

		// This stops local mapping thread (map building) and performs only camera tracking.
		//暂停局部地图构建，只进行追踪
		void ActivateLocalizationMode();
		// This resumes local mapping thread and performs SLAM again.
		//重新开启局部地图的线程，这里使用的是mutex信号量的多线程编程
		void DeactivateLocalizationMode();

		// Returns true if there have been a big map change (loop closure, global BA)
		// since last call to this function
		//如果在闭环检测或者全局BA中有较大的地图变动，调用此函数
		bool MapChanged();

		// Reset the system (clear map)
		void Reset();

		// All threads will be requested to finish.
		// It waits until all threads have finished.
		// This function must be called before saving the trajectory.
		void Shutdown();
		void SetIniPose(cv::Mat initpose);
		// Save camera trajectory in the TUM RGB-D dataset format.
		// Only for stereo and RGB-D. This method does not work for monocular.
		// Call first Shutdown()
		// See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
		void SaveTrajectoryTUM(const string &filename);
		void SaveTrajectoryInfoTUM(const string &filename);
		void SaveTrajectoryTUMRT(const string &filename);
		void SaveKeyPointsRelative(const string &filename);
		void SaveMat(const std::string & filename);
		void SaveTestMat(const std::string & filename);
		// Save keyframe poses in the TUM RGB-D dataset format.
		// This method works for all sensor input.
		// Call first Shutdown()
		// See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
		void SaveKeyFrameTrajectoryTUM(const string &filename);

		// Save camera trajectory in the KITTI dataset format.
		// Only for stereo and RGB-D. This method does not work for monocular.
		// Call first Shutdown()
		// See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
		void SaveTrajectoryKITTI(const string &filename);

		// TODO: Save/Load functions
		// SaveMap(const string &filename);
		// LoadMap(const string &filename);

		// Information from most recent processed frame
		// You can call this right after TrackMonocular (or stereo or RGBD)
		int GetTrackingState();
		std::vector<MapPoint*> GetTrackedMapPoints();
		std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
		RGBDTrajectory GetLastFrameTransformation()
		{
			computeFrametransformation();
			return mvframetransformation;

		}
		void SetSatartFrameId(int id)
		{
			mframeid_start = id;
		}
		RGBDTrajectory HandleRestAndGetFrameTransformation();

		bool isLost()
		{
			return mbisLost;
		}
	private:
		void computeFrametransformation();
		bool mbisReset;
	private:

		// Input sensor
		eSensor mSensor;
		int mframeid_start;
		// ORB vocabulary used for place recognition and feature matching.
		SIFTVocabulary* mpVocabulary;

		// KeyFrame database for place recognition (relocalization and loop detection).
		//关键帧存储的地方
		KeyFrameDatabase* mpKeyFrameDatabase;

		// Map structure that stores the pointers to all KeyFrames and MapPoints.
		//所有关键点和点云存储的地方
		Map* mpMap;

		// Tracker. It receives a frame and computes the associated camera pose.
		// It also decides when to insert a new keyframe, create some new MapPoints and
		// performs relocalization if tracking fails.
		//追踪器，接受一帧并且计算相机位姿，并决定何时插入关键帧，关键点
		//追踪失败后进行重定位
		Tracking* mpTracker;

		// Local Mapper. It manages the local map and performs local bundle adjustment.
		//构建局部地图，并对局部地图使用光束法约束(BA)
		LocalMapping* mpLocalMapper;

		// Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
		// a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
		//闭环检测，每插入一个关键帧就计算是否有闭环并且进行全局BA
		LoopClosing* mpLoopCloser;

		// The viewer draws the map and the current camera pose. It uses Pangolin.
		//使用Pangolin库查看地图和相机位姿
		Viewer* mpViewer;

		FrameDrawer* mpFrameDrawer;
		MapDrawer* mpMapDrawer;

		// System threads: Local Mapping, Loop Closing, Viewer.
		// The Tracking thread "lives" in the main execution thread that creates the System object.
		//追踪这个线程是咋爱main函数中，这里另外开辟了局部地图、局部闭环检测、显示地图三个线程
		std::thread* mptLocalMapping;
		std::thread* mptLoopClosing;
		std::thread* mptViewer;

		// Reset flag
		//Sestem重置线程锁
		std::mutex mMutexReset;
		bool mbReset, mbisLost;

		// Change mode flags
		std::mutex mMutexMode;//线程锁类型
		bool mbActivateLocalizationMode;//是否激活定位模式
		bool mbDeactivateLocalizationMode;//是否停止定位模式

		// Tracking state
		int mTrackingState;
		std::vector<MapPoint*> mTrackedMapPoints;
		std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

		//跟踪失败后保存之前的信息
		RGBDTrajectory mvframetransformation;
		//System状态线程锁
		std::mutex mMutexState;
		// point cloud mapping
		//shared_ptr<PointCloudMapping>  mpPointCloudMapping;
	};

}// namespace ORB_SLAM

#endif // SYSTEM_H

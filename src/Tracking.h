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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
//#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include <map>
#include <mutex>
#include <set>
#include <unordered_set>

//#include "pointcloudmapping.h"


namespace SIFT_SLAM
{
	class Viewer;
	class FrameDrawer;
	class Map;
	class LocalMapping;
	class LoopClosing;
	class System;
	//class PointCloudMapping;

	class MapBAPoint
	{
	public:
		int mid;
		map<Frame, size_t> mobservations;

	};

	class  Tracking
	{
	public:
		Tracking(System* pSys, SIFTVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
		//Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap, shared_ptr<PointCloudMapping> pPointCloud,KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
		
		// Preprocess the input and call Track(). Extract features and performs stereo matching.
		// 对输入的图片进行处理，提取特征和立体匹配
		cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp,int i,int total);
		// 设置局部地图
		void SetLocalMapper(LocalMapping* pLocalMapper);
		//设置局部闭环检测
		void SetLoopClosing(LoopClosing* pLoopClosing);
		//设置视图
		void SetViewer(Viewer* pViewer);

		// Load new settings
		// The focal lenght should be similar or scale prediction will fail when projecting points
		// TODO: Modify MapPoint::PredictScale to take into account focal lenght
		// 把焦距考虑进去改变MapPoint的scale
		void ChangeCalibration(const string &strSettingPath);

		// Use this function if you have deactivated local mapping and you only want to localize the camera.
		//设置追踪模式，/只对摄像头的位姿进行计算
		void InformOnlyTracking(const bool &flag);


	public:

		// Tracking states
		enum eTrackingState{
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			OK = 2,
			LOST = 3
		};

		eTrackingState mState;
		eTrackingState mLastProcessedState;

		// Input sensor
		// 输入的传感器
		int mSensor;

		// Current Frame
		// 当前帧
		Frame mCurrentFrame;
		//当前图片
		cv::Mat mImGray;

		// Initialization Variables (Monocular)
		//初始化时前两帧的相关变量，单目
		std::vector<int> mvIniLastMatches;
		std::vector<int> mvIniMatches;
		std::vector<cv::Point2f> mvbPrevMatched;
		std::vector<cv::Point3f> mvIniP3D;
		Frame mInitialFrame;

		// Lists used to recover the full camera trajectory at the end of the execution.
		// Basically we store the reference keyframe for each frame and its relative transformation
		// 最后保存下来的整个的摄像头的位姿，当程序结束时，List用来恢复相机所有运动轨迹。对每一帧存储其参考帧和相对变换
		list<cv::Mat> mlRelativeFramePoses;
		std::vector<cv::Mat> mlRelativeFramePosesBA;
		std::vector<cv::Mat> mlRelativeFramePosesNBA;
		std::vector<cv::Mat> mlRelativeKeyFramePosesNBA;
		std::vector<cv::Mat> mvKeyFramePosesNBA;
		//list<std::vector<int>> mlIndex;
		std::vector<std::pair<int, std::vector<int>>> mlIndex;
		std::vector<std::vector<cv::KeyPoint>> mvvpreKP;
		int mpreindex;
		std::vector<cv::KeyPoint> mprekp;
		std::vector<std::vector<cv::KeyPoint>> mvvcurkp;
		//跟踪初始化时前两帧之间的匹配
		list<KeyFrame*> mlpReferences;
		list<double> mlFrameTimes;
		list<bool> mlbLost;
		bool mbisLost;

		bool bcreatekf;
		bool binsertkf;
		// True if local mapping is deactivated and we are performing only localization
		//Ture，同时跟踪与定位，不插入关键帧，地图不工作。由界面确定，默认false
		bool mbOnlyTracking;

		void Reset();
		Tracking *mplasttracking;
		bool mbRGB;
		int testid;
		cv::Mat mIniPose;

		std::vector<Frame> mvFrame;//关键帧之间的frame
		std::vector<std::vector<Frame>> mvvFrame;
		std::vector<cv::Mat > mvFramemat;//关键帧之间的frame
		std::vector<std::vector<cv::Mat>> mvvFramemat;
		std::vector<cv::Mat > fmvFramemat;//关键帧之间的frame
		std::vector<std::vector<cv::Mat>> fmvvFramemat;

		std::vector<MapBAPoint> mvMappoints;
		std::unordered_set<int> touched_unit;

	protected:

		// Main tracking function. It is independent of the input sensor.
		// 与输入传感器无关的追踪函数
		void Track(int iframe, int totalframe);

		// Map initialization for stereo and RGB-D
		// 地图的初始化，针对立体相机和RGBD相机
		void StereoInitialization();

		void CheckReplacedInLastFrame();
		bool TrackReferenceKeyFrame();
		void UpdateLastFrame();
		bool TrackWithMotionModel();
		bool TrackWithLastFrame();
		bool TrackReferenceKeyFrameBA();

		bool Relocalization();

		void UpdateLocalMap();
		void UpdateLocalPoints();
		void UpdateLocalKeyFrames();

		bool TrackLocalMap();
		void SearchLocalPoints();

		bool NeedNewKeyFrame();
		void CreateNewKeyFrame();
		bool InsertFrameAsKey(Frame &f);
		bool InsertLastFrameAsKey(Frame &f);
		// In case of performing only localization, this flag is true when there are no matches to
		// points in the map. Still tracking will continue if there are enough matches with temporal points.
		// In that case we are doing visual odometry. The system will try to do relocalization to recover
		// "zero-drift" localization to the map.
		//当点在map中没有匹配到时，flag为ture，只进行定位。当有临时点匹配时，进行追踪
		//这时，系统尝试重定位，
		bool mbVO;
		cv::Mat mlastTcw;
		//Other Thread Pointers
		LocalMapping* mpLocalMapper;
		LoopClosing* mpLoopClosing;

		Featureextractor* mpSIFTextractor;
		Featureextractor* mpIniSIFTextractor;

		//BoW
		SIFTVocabulary* mpORBVocabulary;
		KeyFrameDatabase* mpKeyFrameDB;

		// Initalization (only for monocular)
		//单目初始器
		Initializer* mpInitializer;

		//Local Map
		//当前关键帧就是参考帧
		KeyFrame* mpReferenceKF;
		std::vector<KeyFrame*> mvpLocalKeyFrames;
		std::vector<MapPoint*> mvpLocalMapPoints;

		// System
		System* mpSystem;

		//Drawers
		Viewer* mpViewer;
		FrameDrawer* mpFrameDrawer;
		MapDrawer* mpMapDrawer;

		//Map
		Map* mpMap;

		//Calibration matrix
		//标定矩阵
		cv::Mat mK;
		cv::Mat mDistCoef;
		float mbf;

		//New KeyFrame rules (according to fps)
		int mMinFrames;
		int mMaxFrames;

		// Threshold close/far points
		// Points seen as close by the stereo/RGBD sensor are considered reliable
		// and inserted from just one frame. Far points requiere a match in two keyframes.
		//近点/远点 阈值。
		//近点认为可靠点，可以依据单帧插入Map，远点需要在两帧中匹配到才插入
		float mThDepth;

		// For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
		float mDepthMapFactor;

		//Current matches in frame
		int mnMatchesInliers;

		double mtimestamp;
		int mpreKeymatch;
		int mcurKeymatch;
		int mmatch;
		//Last Frame, KeyFrame and Relocalisation Info
		KeyFrame* mpLastKeyFrame;
		Frame mLastFrame;
		unsigned int mnLastKeyFrameId;
		unsigned int mnLastRelocFrameId;

		//Motion Model
		cv::Mat mVelocity;

		//Color order (true RGB, false BGR, ignored if grayscale)


		list<MapPoint*> mlpTemporalPoints;

		cv::Mat mImRGB, mImDepth;
		//shared_ptr<PointCloudMapping>  mpPointCloudMapping;

	};

} //namespace ORB_SLAM

#endif // TRACKING_H

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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
//#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
//#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include <DBow\BowVector.h>
#include <DBow\FeatureVector.h>
#include "ORBVocabulary.h"
#include "KeyFrame.h"
//#include "ORBextractor.h"
//#include "SIFTextractor.h"
//#include "Tracking.h"
#include <opencv2/opencv.hpp>


#define USE_GPU_SIFT

#ifdef USE_GPU_SIFT
#include "GPU_SIFTextractor.h"
typedef SIFT_SLAM::GPU_SIFTextractor Featureextractor;
#else
#include "SIFTextractor.h"
typedef SIFT_SLAM::SIFTextractor Featureextractor;
#endif

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif


namespace SIFT_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class DLL_API Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for RGB-D cameras.
    //Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
	Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, Featureextractor* extractor, SIFTVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
	// Constructor for GPU_SIFT.
	Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, SiftGPU* extractor, SIFTVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

	////Kinect 2.0 初始化
	//Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, Featureextractor* extractor, SIFTVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    // Compute Bag of Words representation.
	// 存放在mBowVec中
    void ComputeBoW();

    // Set the camera pose.
	// 用Tcw更新mTcw
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
	// 判断路标点是否在视野中
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:

    //// Vocabulary used for relocalization.
    //ORBVocabulary* mpORBvocabulary;

    //// Feature extractor. The right is used only in the stereo case.
    //ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
	// Vocabulary used for relocalization.
	SIFTVocabulary* mpSIFTvocabulary;

	// Feature extractor. The right is used only in the stereo case.
	Featureextractor* mpSIFTextractor;

	/////////
	cv::Mat mImgray, mImdepth;
	double mtimestamp;
	/////////
    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
	// mvKeys:原始左图像提取出的特征点（未校正）
	// mvKeysRight:原始右图像提取出的特征点（未校正）
	    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
	// mvKeysUn:校正mvKeys后的特征点，对于双目摄像头，一般得到的图像都是校正好的，再校正一次有点多余
    std::vector<cv::KeyPoint> mvKeysUn;
	std::vector<int> mvindex;
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
	// 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标
	
	// 单目摄像头，这两个容器中存的都是-1
	//mvuRight存储了左目像素点在右目中的对应点的横坐标
	// mvDepth对应的深度
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
	// 左目摄像头和右目摄像头特征点对应的描述子
    cv::Mat mDescriptors, mDescriptorsRight;
	std::vector<float> mvGPUDescriptors;
    // MapPoints associated to keypoints, NULL pointer if no association.
	// 每个特征点对应的MapPoint
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
	// 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
	// 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
	// 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
	// FRAME_GRID_ROWS 48
	// FRAME_GRID_COLS 64
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
	///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
	//指针，指向参考关键帧
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
	//图像提金字塔的层数
    int mnScaleLevels;
	//图像提金字塔的尺度因子
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;
	vector<cv::Point2d> mvPrePairKeyPoints;
	vector<cv::Point2d> mvCurPairKeyPoints;
	vector<int> mvKeyPointsoctaves;

    // Undistorted Image Bounds (computed once).
	// 用于确定画格子时的边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
	private:

};

}// namespace ORB_SLAM

#endif // FRAME_H

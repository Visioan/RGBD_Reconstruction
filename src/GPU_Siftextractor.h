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

#ifndef GPU_SIFTEXTRACTOR_H
#define GPU_SIFTEXTRACTOR_H

#include <windows.h>
#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\nonfree\nonfree.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <memory>
// SiftGPU模块
#include "SiftGPU.h"

#ifdef _DEBUG
#pragma comment(lib,"../lib/SIFTGPU_d.lib")
//#pragma comment(lib,"../libgpusift/static_siftgpu_d.lib")
#else
#pragma comment(lib,"../lib/SIFTGPU.lib")
#endif

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif


namespace SIFT_SLAM
{
	class  DLL_API GPU_SIFTextractor
	{
	public:
		GPU_SIFTextractor(int argc,char** argv);

		~GPU_SIFTextractor(){}

		void operator()(cv::Mat image, cv::Mat depth,
			std::vector<cv::KeyPoint>& keypoints,
			cv::Mat &descriptors, std::vector<float> &vGPUdescriptors, std::vector<int> &vKeyPointsLevels);

		int inline GetLevels(){
			return mnOctaveLayers;
		}

		float inline GetScaleFactor(){
			return msigma;
		}

		std::vector<float> inline GetScaleFactors(){
			return mvScaleFactor;
		}

		std::vector<float> inline GetScaleSigmaSquares(){
			return mvLevelSigma2;
		}

		std::vector<float> inline GetInverseScaleSigmaSquares(){
			return mvInvLevelSigma2;
		}

		//std::vector<cv::Mat> mvImagePyramid;

	protected:

		std::vector<cv::Point> pattern;
		//	std::vector<int> mnFeaturesPerLevel;

		//	std::vector<int> umax;
		//GPUSift
		std::shared_ptr<SiftGPU> mpGPUSift;
		bool bsupport;
		//
		std::vector<float> mvScaleFactor;
		std::vector<float> mvLevelSigma2;
		std::vector<float> mvInvLevelSigma2;
		//int mnfeatures;
		int mnOctaveLayers;
		double mcontrastThreshold;
		double medgeThreshold;
		double msigma;
		cv::Mat mrgbimage;
		std::vector<double> sig;
		cv::Mat mdepthimage;
	};

} //namespace ORB_SLAM

#endif


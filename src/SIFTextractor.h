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

#ifndef SIFTEXTRACTOR_H
#define SIFTEXTRACTOR_H



#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\nonfree\nonfree.hpp>
#include <opencv2\features2d\features2d.hpp>

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif

namespace SIFT_SLAM
{
	class DLL_API SIFTextractor
	{
	public:
		SIFTextractor(int _nfeatures,float _sigma,int _nlevels);

		~SIFTextractor(){}

		void operator()(cv::Mat image, cv::Mat depth,
			std::vector<cv::KeyPoint>& keypoints,
			cv::Mat &descriptors, std::vector<int> &mvKeyPointsLevels);

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
		void buildGaussianPyramid(const cv::Mat& base, std::vector<cv::Mat>& pyr, int nOctaves);
		void buildDoGPyramid(const std::vector<cv::Mat>& gpyr, std::vector<cv::Mat>& dogpyr);
		void findScaleSpaceExtrema(const std::vector<cv::Mat>& gauss_pyr, const std::vector<cv::Mat>& dog_pyr, std::vector<cv::KeyPoint>& keypoints);
		void computeKeyPoints(cv::Mat image, cv::Mat depth, std::vector<cv::KeyPoint>& keypoints/*, cv::Mat &_descriptor*/);

		std::vector<cv::Point> pattern;
	//	std::vector<int> mnFeaturesPerLevel;

	//	std::vector<int> umax;

		std::vector<float> mvScaleFactor;
		std::vector<float> mvLevelSigma2;
		std::vector<float> mvInvLevelSigma2;
		int mnfeatures;
		int mnOctaveLayers;
		double mcontrastThreshold;
		double medgeThreshold;
		double msigma;
		int mbins;//直方图块数，10
		int msize;//resize之后的大小20*20
		cv::Mat mrgbimage;
		std::vector<double> sig;
		cv::Mat mdepthimage;
	};

} //namespace ORB_SLAM

#endif


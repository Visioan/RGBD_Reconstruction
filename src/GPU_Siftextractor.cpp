/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
*
* Copyright (C) 2014-2016 Ra¨²l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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
/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include "GPU_SIFTextractor.h"
#include <gl/glew.h>


using namespace cv;
using namespace std;

namespace SIFT_SLAM
{

	GPU_SIFTextractor::GPU_SIFTextractor(int argc, char** argv) : msigma(1.2), mnOctaveLayers(1), mcontrastThreshold(0.04), medgeThreshold(10), bsupport(true)
	{
		mpGPUSift = std::make_shared<SiftGPU>();
		//mpGPUSift = new SiftGPU;
		mpGPUSift->ParseParam(argc,argv);
		int support = mpGPUSift->CreateContextGL();
		if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
		{
			cerr << "SiftGPU is not supported!" << endl;
			bsupport = false;
		}

		mnOctaveLayers = mpGPUSift->_level_num;

		mvScaleFactor.resize(mnOctaveLayers);
		mvLevelSigma2.resize(mnOctaveLayers);
		mvInvLevelSigma2.resize(mnOctaveLayers);

		for (int i = 0; i < mnOctaveLayers; i++)
		{
			mvScaleFactor[i] = mpGPUSift->_sigma0 * powf(2.0f, float(i) / float(mpGPUSift->_dog_level_num));
			mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
			mvInvLevelSigma2[i] = 1 / mvLevelSigma2[i];
		}
		//mvScaleFactor.resize(numOctaveLayers);
		//mvLevelSigma2.resize(numOctaveLayers);
		//mvInvLevelSigma2.resize(numOctaveLayers);
		////mvScaleFactor[0] = 1.0f;
		////mvLevelSigma2[0] = 1.0f;
		//for (int i = 0; i<numOctaveLayers; i++)
		//{
		//	mvScaleFactor[i] = pow(2, i);
		//	mvInvLevelSigma2[i] = msigma * msigma;
		//	//mvLevelSigma2[i] = 1.0f / mvInvLevelSigma2[i]*2;
		//	mvLevelSigma2[i] = 4 * 2;
		//}
	}
	void GPU_SIFTextractor::operator()(cv::Mat image, cv::Mat depth, std::vector<cv::KeyPoint>& keypoints, cv::Mat &descriptors, std::vector<float> &vGPUdescriptors, std::vector<int> &vKeyPointsLevels)
	{
		if (!bsupport)
		{
			cerr << "SiftGPU is not supported!" << endl;
			return;
		}
		cv::Mat img;
		if (image.channels() < 3) //this should be always true
			cvtColor(image, img, CV_GRAY2BGR);
		else
			image.copyTo(img);

		mpGPUSift->RunSIFT(img.cols, img.rows, img.data, GL_BGR, GL_UNSIGNED_BYTE);
		int num = mpGPUSift->GetFeatureNum();
		vector<float> des(128 * num);
		vector<float> vdes; vdes.reserve(128*num);
		vector<SiftGPU::SiftKeypoint> keys(num);
		mpGPUSift->GetFeatureVector(&keys[0], &des[0]);
		keypoints.reserve(num);
		vKeyPointsLevels.reserve(num);
		//vector<float> des(128 * num);

		for (size_t i = 0; i < num; i++)
		{
			if (keys[i].y >= img.rows || keys[i].x >= img.cols)
				continue;
			float kpdepth = depth.at<float>(keys[i].y,keys[i].x);
			if (kpdepth <= 0)
				continue;
			cv::KeyPoint kp;
			kp.pt = cv::Point2f(keys[i].x, keys[i].y);
			kp.angle = keys[i].o;
			kp.size = keys[i].s;
			keypoints.push_back(kp);
			vKeyPointsLevels.push_back(1);
			vdes.insert(vdes.end(), des.begin() + i * 128, des.begin() + (i + 1) * 128);
		}
		int fn = vdes.size() / 128;
		cv::Mat matdes = cv::Mat(cv::Size(128, fn), CV_32FC1, vdes.data());
		descriptors = matdes.clone();
		vGPUdescriptors = vdes;

	}
} //namespace ORB_SLAM

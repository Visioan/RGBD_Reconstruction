﻿/**
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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H


#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif
namespace SIFT_SLAM
{

class KeyFrame;
class Frame;


class DLL_API KeyFrameDatabase
{
public:

	//初始化关键帧仓库
    KeyFrameDatabase(const SIFTVocabulary &voc);

	//添加关键帧
   void add(KeyFrame* pKF);
   //删除关键帧
   void erase(KeyFrame* pKF);
   //清理关键帧仓库
   void clear();

   // Loop Detection
   //闭环检测
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   //追踪丢失后重定位
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
	//所需字典
  const SIFTVocabulary* mpVoc;

  // Inverted file
  //存储关键帧
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  //KeyFrameDatabase线程锁
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif

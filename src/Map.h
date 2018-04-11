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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

#ifdef SLAM_DLL_EXPORT
#define DLL_API __declspec(dllexport)//_declspec(dllexport)：导出标志
#else
#define DLL_API __declspec(dllimport)
#endif


namespace SIFT_SLAM
{

class MapPoint;
class KeyFrame;

class DLL_API Map
{
public:
    Map();

	//添加关键帧
    void AddKeyFrame(KeyFrame* pKF);
	//添加点云
    void AddMapPoint(MapPoint* pMP);
	//删除点云
    void EraseMapPoint(MapPoint* pMP);
	//删除关键帧
    void EraseKeyFrame(KeyFrame* pKF);
	//设置参考点云
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
	//
    void InformNewBigChange();
    int GetLastBigChangeIdx();

	//获取所有关键帧
    std::vector<KeyFrame*> GetAllKeyFrames();
	//获取所有点云
    std::vector<MapPoint*> GetAllMapPoints();
	//获取所有参考点云
    std::vector<MapPoint*> GetReferenceMapPoints();

	//Map中的点云数目
    long unsigned int MapPointsInMap();
	//Map中的关键帧数目
    long unsigned  KeyFramesInMap();
	//最大关键帧ID
    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

	//Map更新线程锁
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
	//点云创建线程锁，避免在单独的线程中同时创建两个点
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;
	//参考点云
    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
	//在闭环检测和全局BA中较大变换map的ID
    int mnBigChangeIdx;
	//Map线程锁
    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
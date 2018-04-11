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


 
#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

//Linux挂起线程函数的移植
void usleep(__int64 usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}

namespace SIFT_SLAM
{

	System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer) :mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbisLost(false), mframeid_start(0)
	{
		// Output welcome message
		//cout << endl <<
		//	"ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
		//	"This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
		//	"This is free software, and you are welcome to redistribute it" << endl <<
		//	"under certain conditions. See LICENSE.txt." << endl << endl;
		cout << "..........................................................." << endl;
		cout << "..........................................................." << endl;
		cout << "..........................................................." << endl;
		cout << "Input sensor was set to: ";
		//判断传感器类型
		if (mSensor == MONOCULAR)
			cout << "Monocular" << endl;
		else if (mSensor == STEREO)
			cout << "Stereo" << endl;
		else if (mSensor == RGBD)
			cout << "RGB-D" << endl;

		//Check settings file
		//检查配置文件是否存在
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}

		float resolution = fsSettings["PointCloudMapping.Resolution"];
		float scale = fsSettings["PointCloudMapping.Scale"];
		//Load ORB Vocabulary
		cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

		mpVocabulary = new SIFTVocabulary();
		bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
		if (!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

		//Create KeyFrame Database
		mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

		//Create the Map
		mpMap = new Map();

		//Create Drawers. These are used by the Viewer
		//创建视图和画图器
		mpFrameDrawer = new FrameDrawer(mpMap);
		mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		//初始化追踪线程，在main函数中调用
		//mpPointCloudMapping = make_shared<PointCloudMapping>(resolution, scale);
		//mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpPointCloudMapping,mpKeyFrameDatabase, strSettingsFile, mSensor);
//
		mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
			mpMap,  mpKeyFrameDatabase, strSettingsFile, mSensor);

		//Initialize the Local Mapping thread and launch
		//初始化局部图，并开辟线程
		mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
		mptLocalMapping = new thread(&SIFT_SLAM::LocalMapping::Run, mpLocalMapper);

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
		mptLoopClosing = new thread(&SIFT_SLAM::LoopClosing::Run, mpLoopCloser);

		//Initialize the Viewer thread and launch
		if (bUseViewer)
		{
			mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
			mptViewer = new thread(&Viewer::Run, mpViewer);
			mpTracker->SetViewer(mpViewer);
		}

		//Set pointers between threads
		//设置各个线程指针
		//Tracking线程需要设置：另外三个线程
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);
		//局部图线程设置：Tracking线程和LoopCloser线程
		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);
		//LoopCloser线程设置：Tracking线程和局部图线程
		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	RGBDTrajectory System::HandleRestAndGetFrameTransformation()
	{
		unique_lock<mutex> lock(mMutexReset);
		computeFrametransformation();

		mpTracker->Reset();
		mbReset = false;
		mpTracker->mState = SIFT_SLAM::Tracking::NOT_INITIALIZED;//跟踪失败后需要初始化
		//mpTracker->mState=mp
		return mvframetransformation;
	}

	bool System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,int i,int total)
	{
		if (mSensor != RGBD)
		{
			cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
			exit(-1);
		}

		// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if (mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			//等待，知道局部映射完成
			while (!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}

			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if (mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	//{
	//	unique_lock<mutex> lock(mMutexReset);
	//	if (mbReset)
	//	{
	//		//computeFrametransformation();
	//		mpTracker->Reset();
	//		mbReset = false;
	//	}
	//}

	cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp,i,total);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
	mbisLost = mpTracker->mbisLost;

	return mbisLost;
	}
	void System::SetIniPose(cv::Mat initpose)
	{
		mpTracker->mIniPose = initpose.clone();
	}

	void System::ActivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbActivateLocalizationMode = true;
	}

	void System::DeactivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbDeactivateLocalizationMode = true;
	}

	bool System::MapChanged()
	{
		static int n = 0;
		int curn = mpMap->GetLastBigChangeIdx();
		if (n<curn)
		{
			n = curn;
			return true;
		}
		else
			return false;
	}

	void System::Reset()
	{
		unique_lock<mutex> lock(mMutexReset);
		mbReset = true;
	}

	void System::Shutdown()
	{
		mpLocalMapper->RequestFinish();
		mpLoopCloser->RequestFinish();
		if (mpViewer)
		{
			mpViewer->RequestFinish();
			while (!mpViewer->isFinished())
				usleep(5000);
		}
		//mpPointCloudMapping->shutdown();
		// Wait until all thread have effectively stopped
		while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
		{
			usleep(5000);
		}

		if (mpViewer)
			pangolin::BindToContext("ORB-SLAM2: Map Viewer");
	}
	
	void System::computeFrametransformation()
	{
		cout << endl << "compute Frametransformation  ..." << endl;
		mvframetransformation.data_.clear();
		mbisReset = true;
		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		if (mbisLost)
			mpTracker->mlRelativeFramePoses.pop_back();//将跟踪失败的那一帧弹出
		int frameidx = mframeid_start, totalframe = mpTracker->mlpReferences.size();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame* pKF = *lRit;
			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			while (pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			cv::Mat cvframe_t = cv::Mat::eye(4, 4, CV_32F);
			//Eigen::Matrix4d  frame_t;
			cv::Mat submat = cvframe_t.rowRange(0, 3).colRange(0, 3);
			Rwc.copyTo(submat);
			submat = cvframe_t.rowRange(0, 3).col(3);
			twc.copyTo(submat);
			Eigen::Matrix4d eigentraj;
			cv2eigen(cvframe_t, eigentraj);
			mvframetransformation.data_.push_back(FramedTransformation(frameidx, frameidx, frameidx + 1, eigentraj));

			frameidx++;
		}
		cout << endl << "Frametransformation saved!" << endl;
	}

	//生成Information矩阵, 在ElasticReconstruction中使用
	void System::SaveTrajectoryTUMRT(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();

		int frameidx = 0, totalframe = mpTracker->mlpReferences.size();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while (pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			f << frameidx << "\t" << frameidx << "\t" << frameidx + 1<<"\n";
			f << setprecision(9) << Rwc.at<float>(0, 0) << "\t" << setprecision(9) << Rwc.at<float>(0, 1) << "\t" << setprecision(9) << Rwc.at<float>(0, 2) << "\t" << setprecision(9) << twc.at<float>(0) << "\n";
			f << setprecision(9) << Rwc.at<float>(1, 0) << "\t" << setprecision(9) << Rwc.at<float>(1, 1) << "\t" << setprecision(9) << Rwc.at<float>(1, 2) << "\t" << setprecision(9) << twc.at<float>(1) << "\n";
			f << setprecision(9) << Rwc.at<float>(2, 0) << "\t" << setprecision(9) << Rwc.at<float>(2, 1) << "\t" << setprecision(9) << Rwc.at<float>(2, 2) << "\t" << setprecision(9) << twc.at<float>(2) << "\n";
			f << setprecision(9) <<0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0;
			if (frameidx < totalframe-1)
				f << "\n";
			frameidx++;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveKeyPointsRelative(const string &filename)
	{
		cout << endl << "Saving keyPoints Relative to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		ofstream f;

		f << fixed;

		int frameidx = 0, totalframe = mpTracker->mlIndex.size();

		for (size_t j = 0; j < mpTracker->mlIndex.size(); j++)
		{

			std::vector<cv::KeyPoint> curkp = mpTracker->mvvcurkp[j];
			std::vector<cv::KeyPoint> prekp = mpTracker->mvvpreKP[j];
			std::pair<int, std::vector<int>> index = mpTracker->mlIndex[j];
			if (index.first == -1)
				continue;
			char name[1024];
			sprintf(name, "%d_%d.txt", index.first, j);
			string relafile = filename + "/" + name;
			f.open(relafile.c_str());
			f << "preFrame Index preFrameKeyPoints curentFrame curentFrameKeyPoints\n";
			for (size_t i = 0; i < index.second.size(); i++)
			{
				if (index.second[i] == -1)
					continue;

				f << index.first << "\t" << prekp[index.second[i]].pt.x << "\t" << prekp[index.second[i]].pt.y << "\t" << j << "\t" << curkp[i].pt.x << "\t" << curkp[i].pt.y << "\n";
			}
			f.close();
		}

		cout << endl << "keyPoints Relative saved!" << endl;
	}
	
	void System::SaveTestMat(const string &filename)
	{
		cout << endl << "Saving test camera trajectory to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		std::string filename_pose_KF = filename + "/pose_KF.txt";
		std::string filename_pose_BAKF_BAF = filename + "/pose_BAKF_BAF.txt";
		std::string filename_pose_BAKF_BAFq = filename + "/pose_BAKF_BAFq.txt";
		std::string filename_pose_KF_F = filename + "/pose_KF_F.txt";
		std::string filename_pose_KF_BAF = filename + "/pose_KF_BAF.txt";
		std::string filename_pose_BAKF = filename + "/pose_BAKF.txt";
		SaveKeyFrameTrajectoryTUM(filename_pose_BAKF);
		ofstream f_pose_KF, f_pose_BAKF_BAF, f_pose_KF_F, f_pose_KF_BAF, f_pose_BAKF_BAFq;
		f_pose_KF.open(filename_pose_KF.c_str());
		f_pose_BAKF_BAF.open(filename_pose_BAKF_BAF.c_str());
		f_pose_BAKF_BAFq.open(filename_pose_BAKF_BAFq.c_str());
		f_pose_KF_F.open(filename_pose_KF_F.c_str());
		f_pose_KF_BAF.open(filename_pose_KF_BAF.c_str());
		f_pose_KF << fixed;		f_pose_BAKF_BAF << fixed;
		f_pose_KF_F << fixed;		f_pose_KF_BAF << fixed;
		
		list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();

		int frameidx = 0, totalframe = mpTracker->mlpReferences.size();
		for (size_t i=0;i< mpTracker->mlRelativeFramePoses.size();i++,lRit++, lT++, lbL++)
		{
			{
				if (*lbL)
					continue;
				KeyFrame* pKF = *lRit;
				cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);
				while (pKF->isBad())
				{
					Trw = Trw*pKF->mTcp; pKF = pKF->GetParent();
				}

				Trw = Trw*pKF->GetPose()*Two;

				cv::Mat poseBAKFBAF = mpTracker->mlRelativeFramePosesBA[i] * Trw;
				cv::Mat BARwc = poseBAKFBAF.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat BAtwc = -BARwc*poseBAKFBAF.rowRange(0, 3).col(3);

				f_pose_BAKF_BAF << i << "\t" << i << "\t" << i + 1 << "\n";
				f_pose_BAKF_BAF << setprecision(9) << BARwc.at<float>(0, 0) << "\t" << setprecision(9) << BARwc.at<float>(0, 1) << "\t" << setprecision(9) << BARwc.at<float>(0, 2) << "\t" << setprecision(9) << BAtwc.at<float>(0) << "\n";
				f_pose_BAKF_BAF << setprecision(9) << BARwc.at<float>(1, 0) << "\t" << setprecision(9) << BARwc.at<float>(1, 1) << "\t" << setprecision(9) << BARwc.at<float>(1, 2) << "\t" << setprecision(9) << BAtwc.at<float>(1) << "\n";
				f_pose_BAKF_BAF << setprecision(9) << BARwc.at<float>(2, 0) << "\t" << setprecision(9) << BARwc.at<float>(2, 1) << "\t" << setprecision(9) << BARwc.at<float>(2, 2) << "\t" << setprecision(9) << BAtwc.at<float>(2) << "\n";
				f_pose_BAKF_BAF << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0;
				if (i < totalframe - 1)
					f_pose_BAKF_BAF << "\n";
			}
			{
				cv::Mat TrwKFF = cv::Mat::eye(4, 4, CV_32F);

				TrwKFF = TrwKFF*mpTracker->mlRelativeKeyFramePosesNBA[i] * Two;

				cv::Mat poseKFBAF = mpTracker->mlRelativeFramePosesBA[i] * TrwKFF;
				cv::Mat BARwc = poseKFBAF.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat BAtwc = -BARwc*poseKFBAF.rowRange(0, 3).col(3);

				f_pose_KF_BAF << i << "\t" << i << "\t" << i + 1 << "\n";
				f_pose_KF_BAF << setprecision(9) << BARwc.at<float>(0, 0) << "\t" << setprecision(9) << BARwc.at<float>(0, 1) << "\t" << setprecision(9) << BARwc.at<float>(0, 2) << "\t" << setprecision(9) << BAtwc.at<float>(0) << "\n";
				f_pose_KF_BAF << setprecision(9) << BARwc.at<float>(1, 0) << "\t" << setprecision(9) << BARwc.at<float>(1, 1) << "\t" << setprecision(9) << BARwc.at<float>(1, 2) << "\t" << setprecision(9) << BAtwc.at<float>(1) << "\n";
				f_pose_KF_BAF << setprecision(9) << BARwc.at<float>(2, 0) << "\t" << setprecision(9) << BARwc.at<float>(2, 1) << "\t" << setprecision(9) << BARwc.at<float>(2, 2) << "\t" << setprecision(9) << BAtwc.at<float>(2) << "\n";
				f_pose_KF_BAF << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0;
				if (i < totalframe - 1)
					f_pose_KF_BAF << "\n";

				cv::Mat Rwc = poseKFBAF.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat twc = -Rwc*poseKFBAF.rowRange(0, 3).col(3);

				vector<float> q = Converter::toQuaternion(Rwc);

				f_pose_BAKF_BAFq << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
				

			}
			{
				cv::Mat TrwKFF = cv::Mat::eye(4, 4, CV_32F);

				TrwKFF = TrwKFF*mpTracker->mlRelativeKeyFramePosesNBA[i] * Two;

				cv::Mat poseKFBAF = mpTracker->mlRelativeFramePosesNBA[i] * TrwKFF;
				cv::Mat BARwc = poseKFBAF.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat BAtwc = -BARwc*poseKFBAF.rowRange(0, 3).col(3);

				f_pose_KF_F << i << "\t" << i << "\t" << i + 1 << "\n";
				f_pose_KF_F << setprecision(9) << BARwc.at<float>(0, 0) << "\t" << setprecision(9) << BARwc.at<float>(0, 1) << "\t" << setprecision(9) << BARwc.at<float>(0, 2) << "\t" << setprecision(9) << BAtwc.at<float>(0) << "\n";
				f_pose_KF_F << setprecision(9) << BARwc.at<float>(1, 0) << "\t" << setprecision(9) << BARwc.at<float>(1, 1) << "\t" << setprecision(9) << BARwc.at<float>(1, 2) << "\t" << setprecision(9) << BAtwc.at<float>(1) << "\n";
				f_pose_KF_F << setprecision(9) << BARwc.at<float>(2, 0) << "\t" << setprecision(9) << BARwc.at<float>(2, 1) << "\t" << setprecision(9) << BARwc.at<float>(2, 2) << "\t" << setprecision(9) << BAtwc.at<float>(2) << "\n";
				f_pose_KF_F << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0;
				if (i < totalframe - 1)
					f_pose_KF_F << "\n";
			}
		}
		for (size_t i = 0; i<vpKFs.size(); i++)	
		{
			KeyFrame* pKF = vpKFs[i];
			// pKF->SetPose(pKF->GetPose()*Two);

			if (pKF->isBad())
				continue;

			cv::Mat TrwKF = mpTracker->mvKeyFramePosesNBA[i];

			cv::Mat BARwc = TrwKF.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat BAtwc = TrwKF.rowRange(0, 3).col(3);

		//	cv::Mat R = pKF->GetRotation().t();
			vector<float> q = Converter::toQuaternion(BARwc);
			//cv::Mat t = pKF->GetCameraCenter();
			f_pose_KF << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << BAtwc.at<float>(0) << " " << BAtwc.at<float>(1) << " " << BAtwc.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f_pose_KF.close();
		f_pose_BAKF_BAF.close();
		f_pose_BAKF_BAFq.close();
		f_pose_KF_F.close();
		f_pose_KF_BAF.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveMat(const string &filename)
	{
		cout << endl << "Saving Mat to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		std::vector<std::vector<cv::Mat>> mvvFramemat=mpTracker->mvvFramemat;
		std::vector<std::vector<cv::Mat>> fmvvFramemat = mpTracker->fmvvFramemat;

		std::vector<std::vector<Frame>> mvvFrame = mpTracker->mvvFrame;


		ofstream srcf,baf;

		srcf << fixed;
		baf << fixed;
		int frameidx = 0, totalKframe = fmvvFramemat.size();

		for (size_t j = 0; j < totalKframe; j++)
		{
			char srcfname[1024];
			sprintf(srcfname, "%dsrc_.txt", j);
			char baname[1024];
			sprintf(baname, "%dbaf_.txt", j);
			string srcfile = filename + "/" + srcfname;
			string baile = filename + "/" + baname;
			srcf.open(srcfile.c_str());
			baf.open(baile.c_str());

			for (size_t i = 0; i < mvvFramemat[j].size(); i++)
			{

				srcf << mvvFrame[j][i].mnId << "\t" << mvvFrame[j][i].mnId << "\t" << mvvFrame[j][i].mnId + 1 << "\n";
				srcf << setprecision(9) << mvvFramemat[j][i].at<float>(0, 0) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(0, 1) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(0, 2) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(0,3) << "\n";
				srcf << setprecision(9) << mvvFramemat[j][i].at<float>(1, 0) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(1, 1) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(1, 2) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(1,3) << "\n";
				srcf << setprecision(9) << mvvFramemat[j][i].at<float>(2, 0) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(2, 1) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(2, 2) << "\t" << setprecision(9) << mvvFramemat[j][i].at<float>(2,3) << "\n";
				srcf << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0<<"\n";

				baf << mvvFrame[j][i].mnId << "\t" << mvvFrame[j][i].mnId << "\t" << mvvFrame[j][i].mnId + 1 << "\n";
				baf << setprecision(9) << fmvvFramemat[j][i].at<float>(0, 0) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(0, 1) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(0, 2) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(0, 3) << "\n";
				baf << setprecision(9) << fmvvFramemat[j][i].at<float>(1, 0) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(1, 1) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(1, 2) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(1, 3) << "\n";
				baf << setprecision(9) << fmvvFramemat[j][i].at<float>(2, 0) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(2, 1) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(2, 2) << "\t" << setprecision(9) << fmvvFramemat[j][i].at<float>(2, 3) << "\n";
				baf << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 0.0 << "\t" << setprecision(9) << 1.0<<"\n";
			}
			srcf.close();
			baf.close();

		}

		cout << endl << "keyPoints Relative saved!" << endl;
	}
	
	void System::SaveTrajectoryInfoTUM(const string &filename)
	{
		cout << endl << "Saving camera trajectory Information to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryInfoTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		int allnum = mpTracker->mlRelativeFramePoses.size();
		int i = 0;
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while (pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			Eigen::Matrix< double, 6, 6 > information;
			std::set<MapPoint*> setMapPs = pKF->GetMapPoints();
			std::set<MapPoint*>::iterator setMpsbegin = setMapPs.begin();
			std::set<MapPoint*>::iterator setMpsend = setMapPs.end();
			for (; setMpsbegin != setMpsend; setMpsbegin++)
			{
				MapPoint* mapPoints = *setMpsbegin;
				cv::Mat worldPoint = mapPoints->GetWorldPos();
				float sx = (float)(worldPoint.at<double>(0));
				float sy = (float)(worldPoint.at<double>(1));
				float sz = (float)(worldPoint.at<double>(2));
				Eigen::Matrix< double, 3, 6 > A;
				A << 1, 0, 0, 0, 2 * sz, -2 * sy,
					0, 1, 0, -2 * sz, 0, 2 * sx,
					0, 0, 1, 2 * sy, -2 * sx, 0;
				information += A.transpose() * A;
			}

			f << setprecision(6) << i << " " << setprecision(6) << i + 1 << " " << setprecision(6) << allnum << endl;
			f << setprecision(6) << information(0, 0) << " " << setprecision(6) << information(0, 1) << " " << setprecision(6) << information(0, 2) << " " << setprecision(6) << information(0, 3) << " " << setprecision(6) << information(0, 4) << " " << setprecision(6) << information(0, 5) << endl;
			f << setprecision(6) << information(1, 0) << " " << setprecision(6) << information(1,1) << " " << setprecision(6) << information(1,2) << " " << setprecision(6) << information(1,3) << " " << setprecision(6) << information(1,4) << " " << setprecision(6) << information(1,5) << endl;
			f << setprecision(6) << information(2, 0) << " " << setprecision(6) << information(2,1) << " " << setprecision(6) << information(2,2) << " " << setprecision(6) << information(2,3) << " " << setprecision(6) << information(2,4) << " " << setprecision(6) << information(2,5) << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while (pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}
	
	void System::SaveKeyFrameTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		//cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for (size_t i = 0; i<vpKFs.size(); i++)
		{
			KeyFrame* pKF = vpKFs[i];

			// pKF->SetPose(pKF->GetPose()*Two);

			if (pKF->isBad())
				continue;

			cv::Mat R = pKF->GetRotation().t();
			vector<float> q = Converter::toQuaternion(R);
			cv::Mat t = pKF->GetCameraCenter();
			f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		}

		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveTrajectoryKITTI(const string &filename)
	{
	    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
	    if(mSensor==MONOCULAR)
	    {
	        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
	        return;
	    }
	
	    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
	
	    // Transform all keyframes so that the first keyframe is at the origin.
	    // After a loop closure the first keyframe might not be at the origin.
	    cv::Mat Two = vpKFs[0]->GetPoseInverse();
	
	    ofstream f;
	    f.open(filename.c_str());
	    f << fixed;
	
	    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
	    // We need to get first the keyframe pose and then concatenate the relative transformation.
	    // Frames not localized (tracking failure) are not saved.
	
	    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
	    // which is true when tracking failed (lbL).
	    list<SIFT_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
	    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
	    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
	    {
	        SIFT_SLAM::KeyFrame* pKF = *lRit;
	
	        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
	
	        while(pKF->isBad())
	        {
	          //  cout << "bad parent" << endl;
	            Trw = Trw*pKF->mTcp;
	            pKF = pKF->GetParent();
	        }
	
	        Trw = Trw*pKF->GetPose()*Two;
	
	        cv::Mat Tcw = (*lit)*Trw;
	        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
	        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
	
	        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
	             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
	             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
	    }
	    f.close();
	    cout << endl << "trajectory saved!" << endl;
	}

	int System::GetTrackingState()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackingState;
	}

	vector<MapPoint*> System::GetTrackedMapPoints()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedMapPoints;
	}

	vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedKeyPointsUn;
	}

} //namespace ORB_SLAM

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
 
#include "Frame.h"
#include "Converter.h"
#include "SIFTmatcher.h"
#include <thread>

namespace SIFT_SLAM
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
/**
* @brief Copy constructor
*
* 拷贝构造函数, mLastFrame = Frame(mCurrentFrame)
*/
Frame::Frame(const Frame &frame) :mpSIFTvocabulary(frame.mpSIFTvocabulary), mpSIFTextractor(frame.mpSIFTextractor), mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()), mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys), mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight), mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
mDescriptors(frame.mDescriptors.clone()),
mDescriptorsRight(frame.mDescriptorsRight.clone()), mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels), mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor), mvScaleFactors(frame.mvScaleFactors), mvLevelSigma2(frame.mvLevelSigma2), mImgray(frame.mImgray), mImdepth(frame.mImdepth), mtimestamp(frame.mtimestamp), mvKeyPointsoctaves(frame.mvKeyPointsoctaves), mvGPUDescriptors(frame.mvGPUDescriptors), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];
	mvindex = frame.mvindex;
    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

// RGBD初始化
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, Featureextractor* extractor, SIFTVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)	:mpSIFTvocabulary(voc), mpSIFTextractor(extractor), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
	mImgray = imGray;
	mImdepth = imDepth;
	mtimestamp = timeStamp;
    // Frame ID
    mnId=nNextId++;
	//mnId = timeStamp;
    // Scale Level Info
	mnScaleLevels = mpSIFTextractor->GetLevels();
	mfScaleFactor = mpSIFTextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
	mvScaleFactors = mpSIFTextractor->GetScaleFactors();
	mvLevelSigma2 = mpSIFTextractor->GetScaleSigmaSquares();
	mvInvLevelSigma2 = mpSIFTextractor->GetInverseScaleSigmaSquares();


    // SIFT extraction
	(*mpSIFTextractor)(imGray, imDepth, mvKeys, mDescriptors, mvGPUDescriptors, mvKeyPointsoctaves);
    N = mvKeys.size();
	mvindex = std::vector<int>(N,-1);
    if(mvKeys.empty())
        return;

	//计算未失真的关键点
	// 调用OpenCV的矫正函数矫正orb提取的特征点
    //UndistortKeyPoints();
	mvKeysUn = mvKeys;
    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

//Kinect 2.0 初始化

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);
	// 在mGrid中记录了各特征点
    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

/**
* @brief Set the camera pose.
*
* 设置相机姿态，随后会调用 UpdatePoseMatrices() 来改变mRcw,mRwc等变量的值
* @param Tcw Transformation from world to camera
*/
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}
/**
* @brief Computes rotation, translation and camera center matrices from the camera pose.
*
* 根据Tcw计算mRcw、mtcw和mRwc、mOw
*/
void Frame::UpdatePoseMatrices()
{ 
	// [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
	// x_camera = R*x_world + t
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
	// mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
	// mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    mOw = -mRcw.t()*mtcw;
}
/**
* @brief 判断一个点是否在视野内
*
* 计算了重投影坐标，观测方向夹角，预测在当前帧的尺度
* @param  pMP             MapPoint
* @param  viewingCosLimit 视角和平均视角的方向阈值
* @return                 true if is in view
* @see SearchLocalPoints()
*/
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
	// 3D点P在相机坐标系下的坐标
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
	// V-D 1) 将MapPoint投影到当前帧, 并判断是否在图像内
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
	// V-D 3) 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
	// 世界坐标系下，相机到3D点P的向量, 向量方向由相机指向3D点P
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
	// V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
	// V-D 4) 根据深度预测尺度（对应特征点在一层）
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
	// 标记该点将来要被投影
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
	//该3D点投影到双目右侧相机上的横坐标
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}
/**
* @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点
* @param x        图像坐标u
* @param y        图像坐标v
* @param r        边长
* @param minLevel 最小尺度
* @param maxLevel 最大尺度
* @return         满足条件的特征点的序号
*/
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}
/**
* @brief Bag of Words Representation
*
* 计算词包mBowVec和mFeatVec，其中mFeatVec记录了属于第i个node（在第4层）的ni个描述子
* @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
*/

void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        //vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
		vector<vector<float>> vCurrentDesc = Converter::sifttoDescriptorVector(mDescriptors);
        mpSIFTvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}
// 调用OpenCV的矫正函数矫正orb提取的特征点
void Frame::UndistortKeyPoints()
{
	// 如果畸变量为空，则不矫正
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
	// N为提取的特征点数量，将N个特征点保存在N*2的mat中
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
	// 调整mat的通道为2，矩阵的行列形状不变
    mat=mat.reshape(2);
	// 用cv的函数进行失真校正
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
	// 存储校正后的特征点
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
		// 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4,2,CV_32F);
		//左上
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
		//右上
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
		//左下
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
		//右下
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

		//左上和左下横坐标最小的
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
		//右上和右下横坐标最大的
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
		//左上和右上纵坐标最小的
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
		//左下和右下纵坐标最小的
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

// mvDepth直接由depth图像读取
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}
/**
* @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
* @param  i 第i个keypoint
* @return   3D点（相对于世界坐标系）
*/
cv::Mat Frame::UnprojectStereo(const int &i)
{
	// KeyFrame::UnprojectStereo
	// mvDepth是在ComputeStereoMatches函数中求取的
	// mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
	// KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM

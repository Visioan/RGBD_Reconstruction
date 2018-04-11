/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
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

#include "SIFTextractor.h"


using namespace cv;
using namespace std;

namespace SIFT_SLAM
{
	static float _sift_init_sigma = 0.5f;//输入图像的高斯模糊，0.5f
	static int _sift_img_border = 5;//忽略关键点的边界阈值。5
	static int _sift_max_interp_setps = 5;//关键点插值失败之前的最大步长。5
	static int _sift_ori_hist_bins = 36;//方向分配直方图的最大bin。36
	static float _sift_ori_sig_fctr = 1.5f;//方向分量的高斯sigma。1.5f
	static float _sift_ori_radius = 3 * _sift_ori_sig_fctr;//3*_sift_ori_sig_fctr
	static float _sift_ori_peak_ratio = 0.8f;//0.8f
	static int _sift_fixpt_scale = 1;// 1

	typedef float sift_wt;

	static inline void unpackOctave(const KeyPoint& kpt, int& octave, int& layer, float& scale)
	{
		octave = kpt.octave & 255;
		layer = (kpt.octave >> 8) & 255;
		octave = octave < 128 ? octave : (-128 | octave);
		scale = octave >= 0 ? 1.f / (1 << octave) : (float)(1 << -octave);
	}

	static void calcSIFTDescriptor(const Mat& img, Point2f ptf, float ori, float scl,
		int d, int n, float* dst)
	{
		Point pt(cvRound(ptf.x), cvRound(ptf.y));
		float cos_t = cosf(ori*(float)(CV_PI / 180));
		float sin_t = sinf(ori*(float)(CV_PI / 180));
		float bins_per_rad = n / 360.f;
		float exp_scale = -1.f / (d * d * 0.5f);
		float hist_width = 3.0f * scl;
		int radius = cvRound(hist_width * 1.4142135623730951f * (d + 1) * 0.5f);
		// Clip the radius to the diagonal of the image to avoid autobuffer too large exception
		radius = std::min(radius, (int)sqrt((double)img.cols*img.cols + img.rows*img.rows));
		cos_t /= hist_width;
		sin_t /= hist_width;

		int i, j, k, len = (radius * 2 + 1)*(radius * 2 + 1), histlen = (d + 2)*(d + 2)*(n + 2);
		int rows = img.rows, cols = img.cols;

		AutoBuffer<float> buf(len * 6 + histlen);
		float *X = buf, *Y = X + len, *Mag = Y, *Ori = Mag + len, *W = Ori + len;
		float *RBin = W + len, *CBin = RBin + len, *hist = CBin + len;

		for (i = 0; i < d + 2; i++)
		{
			for (j = 0; j < d + 2; j++)
				for (k = 0; k < n + 2; k++)
					hist[(i*(d + 2) + j)*(n + 2) + k] = 0.;
		}

		for (i = -radius, k = 0; i <= radius; i++)
			for (j = -radius; j <= radius; j++)
			{
				// Calculate sample's histogram array coords rotated relative to ori.
				// Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
				// r_rot = 1.5) have full weight placed in row 1 after interpolation.
				float c_rot = j * cos_t - i * sin_t;
				float r_rot = j * sin_t + i * cos_t;
				float rbin = r_rot + d / 2 - 0.5f;
				float cbin = c_rot + d / 2 - 0.5f;
				int r = pt.y + i, c = pt.x + j;

				if (rbin > -1 && rbin < d && cbin > -1 && cbin < d &&
					r > 0 && r < rows - 1 && c > 0 && c < cols - 1)
				{
					float dx = (float)(img.at<sift_wt>(r, c + 1) - img.at<sift_wt>(r, c - 1));
					float dy = (float)(img.at<sift_wt>(r - 1, c) - img.at<sift_wt>(r + 1, c));
					X[k] = dx; Y[k] = dy; RBin[k] = rbin; CBin[k] = cbin;
					W[k] = (c_rot * c_rot + r_rot * r_rot)*exp_scale;
					k++;
				}
			}

		len = k;
		fastAtan2(Y, X, Ori, len, true);
		magnitude(X, Y, Mag, len);
		exp(W, W, len);

		for (k = 0; k < len; k++)
		{
			float rbin = RBin[k], cbin = CBin[k];
			float obin = (Ori[k] - ori)*bins_per_rad;
			float mag = Mag[k] * W[k];

			int r0 = cvFloor(rbin);
			int c0 = cvFloor(cbin);
			int o0 = cvFloor(obin);
			rbin -= r0;
			cbin -= c0;
			obin -= o0;

			if (o0 < 0)
				o0 += n;
			if (o0 >= n)
				o0 -= n;

			// histogram update using tri-linear interpolation
			float v_r1 = mag*rbin, v_r0 = mag - v_r1;
			float v_rc11 = v_r1*cbin, v_rc10 = v_r1 - v_rc11;
			float v_rc01 = v_r0*cbin, v_rc00 = v_r0 - v_rc01;
			float v_rco111 = v_rc11*obin, v_rco110 = v_rc11 - v_rco111;
			float v_rco101 = v_rc10*obin, v_rco100 = v_rc10 - v_rco101;
			float v_rco011 = v_rc01*obin, v_rco010 = v_rc01 - v_rco011;
			float v_rco001 = v_rc00*obin, v_rco000 = v_rc00 - v_rco001;

			int idx = ((r0 + 1)*(d + 2) + c0 + 1)*(n + 2) + o0;
			hist[idx] += v_rco000;
			hist[idx + 1] += v_rco001;
			hist[idx + (n + 2)] += v_rco010;
			hist[idx + (n + 3)] += v_rco011;
			hist[idx + (d + 2)*(n + 2)] += v_rco100;
			hist[idx + (d + 2)*(n + 2) + 1] += v_rco101;
			hist[idx + (d + 3)*(n + 2)] += v_rco110;
			hist[idx + (d + 3)*(n + 2) + 1] += v_rco111;
		}

		// finalize histogram, since the orientation histograms are circular
		for (i = 0; i < d; i++)
			for (j = 0; j < d; j++)
			{
				int idx = ((i + 1)*(d + 2) + (j + 1))*(n + 2);
				hist[idx] += hist[idx + n];
				hist[idx + 1] += hist[idx + n + 1];
				for (k = 0; k < n; k++)
					dst[(i*d + j)*n + k] = hist[idx + k];
			}
		// copy histogram to the descriptor,
		// apply hysteresis thresholding
		// and scale the result, so that it can be easily converted
		// to byte array
		float nrm2 = 0;
		len = d*d*n;
		for (k = 0; k < len; k++)
			nrm2 += dst[k] * dst[k];
		float thr = std::sqrt(nrm2)*0.2f;
		for (i = 0, nrm2 = 0; i < k; i++)
		{
			float val = std::min(dst[i], thr);
			dst[i] = val;
			nrm2 += val*val;
		}
		nrm2 = 512.f / std::max(std::sqrt(nrm2), FLT_EPSILON);

#if 1
		for (k = 0; k < len; k++)
		{
			dst[k] = saturate_cast<uchar>(dst[k] * nrm2);
		}
#else
		float nrm1 = 0;
		for (k = 0; k < len; k++)
		{
			dst[k] *= nrm2;
			nrm1 += dst[k];
		}
		nrm1 = 1.f / std::max(nrm1, FLT_EPSILON);
		for (k = 0; k < len; k++)
		{
			dst[k] = std::sqrt(dst[k] * nrm1);//saturate_cast<uchar>(std::sqrt(dst[k] * nrm1)*SIFT_INT_DESCR_FCTR);
		}
#endif
	}

	cv::Mat createInitialImage(const cv::Mat& img, bool doubleImageSize, float msigma)
	{
		cv::Mat gray, gray_fpt;
		if (img.channels() == 3 || img.channels() == 4)
			cvtColor(img, gray, cv::COLOR_BGR2GRAY);
		else
			img.copyTo(gray);
		gray.convertTo(gray_fpt, cv::DataType<float>::type, _sift_fixpt_scale, 0);

		float sig_diff;

		if (doubleImageSize)
		{
			sig_diff = sqrtf(std::max(msigma * msigma - _sift_init_sigma * _sift_init_sigma * 4, 0.01f));
			cv::Mat dbl;
			cv::resize(gray_fpt, dbl, cv::Size(gray.cols * 2, gray.rows * 2), 0, 0, cv::INTER_LINEAR);
			cv::GaussianBlur(dbl, dbl, cv::Size(), sig_diff, sig_diff);
			return dbl;
		}
		else
		{
			sig_diff = sqrtf(std::max(msigma * msigma - _sift_init_sigma * _sift_init_sigma, 0.01f));
			cv::GaussianBlur(gray_fpt, gray_fpt, cv::Size(), sig_diff, sig_diff);
			return gray_fpt;
		}
	}

	float calcOrientationHist(const cv::Mat& img, cv::Point pt, int radius, float msigma, float* hist, int n)
	{
		int i, j, k, len = (radius * 2 + 1)*(radius * 2 + 1);

		float expf_scale = -1.f / (2.f * msigma * msigma);
		cv::AutoBuffer<float> buf(len * 4 + n + 4);
		float *X = buf, *Y = X + len, *Mag = X, *Ori = Y + len, *W = Ori + len;
		float* temphist = W + len + 2;

		for (i = 0; i < n; i++)
			temphist[i] = 0.f;

		for (i = -radius, k = 0; i <= radius; i++)
		{
			int y = pt.y + i;
			if (y <= 0 || y >= img.rows - 1)
				continue;
			for (j = -radius; j <= radius; j++)
			{
				int x = pt.x + j;
				if (x <= 0 || x >= img.cols - 1)
					continue;

				float dx = (float)(img.at<float>(y, x + 1) - img.at<float>(y, x - 1));
				float dy = (float)(img.at<float>(y - 1, x) - img.at<float>(y + 1, x));

				X[k] = dx; Y[k] = dy; W[k] = (i*i + j*j)*expf_scale;
				k++;
			}
		}

		len = k;

		// compute gradient values, orientations and the weights over the pixel neighborhood
		cv::exp(W, W, len);
		cv::fastAtan2(Y, X, Ori, len, true);
		cv::magnitude(X, Y, Mag, len);

		for (k = 0; k < len; k++)
		{
			int bin = cvRound((n / 360.f)*Ori[k]);
			if (bin >= n)
				bin -= n;
			if (bin < 0)
				bin += n;
			temphist[bin] += W[k] * Mag[k];
		}

		// smooth the histogram
		temphist[-1] = temphist[n - 1];
		temphist[-2] = temphist[n - 2];
		temphist[n] = temphist[0];
		temphist[n + 1] = temphist[1];
		for (i = 0; i < n; i++)
		{
			hist[i] = (temphist[i - 2] + temphist[i + 2])*(1.f / 16.f) +
				(temphist[i - 1] + temphist[i + 1])*(4.f / 16.f) +
				temphist[i] * (6.f / 16.f);
		}

		float maxval = hist[0];
		for (i = 1; i < n; i++)
			maxval = std::max(maxval, hist[i]);

		return maxval;
	}

	bool adjustLocalExtrema(const std::vector<cv::Mat>& dog_pyr, cv::KeyPoint& kpt, int octv, int& layer, int& r, int& c, int mnOctaveLayers, float mcontrastThreshold, float edgeThreshold, float msigma)
	{
		const float img_scale = 1.f / (255 * _sift_fixpt_scale);
		const float deriv_scale = img_scale*0.5f;
		const float second_deriv_scale = img_scale;
		const float cross_deriv_scale = img_scale*0.25f;

		float xi = 0, xr = 0, xc = 0, contr = 0;
		int i = 0;

		for (; i < _sift_max_interp_setps; i++)
		{
			int idx = octv*(mnOctaveLayers + 2) + layer;
			const cv::Mat& img = dog_pyr[idx];
			const cv::Mat& prev = dog_pyr[idx - 1];
			const cv::Mat& next = dog_pyr[idx + 1];

			cv::Vec3f dD((img.at<float>(r, c + 1) - img.at<float>(r, c - 1))*deriv_scale,
				(img.at<float>(r + 1, c) - img.at<float>(r - 1, c))*deriv_scale,
				(next.at<float>(r, c) - prev.at<float>(r, c))*deriv_scale);

			float v2 = (float)img.at<float>(r, c) * 2;
			float dxx = (img.at<float>(r, c + 1) + img.at<float>(r, c - 1) - v2)*second_deriv_scale;
			float dyy = (img.at<float>(r + 1, c) + img.at<float>(r - 1, c) - v2)*second_deriv_scale;
			float dss = (next.at<float>(r, c) + prev.at<float>(r, c) - v2)*second_deriv_scale;
			float dxy = (img.at<float>(r + 1, c + 1) - img.at<float>(r + 1, c - 1) -
				img.at<float>(r - 1, c + 1) + img.at<float>(r - 1, c - 1))*cross_deriv_scale;
			float dxs = (next.at<float>(r, c + 1) - next.at<float>(r, c - 1) -
				prev.at<float>(r, c + 1) + prev.at<float>(r, c - 1))*cross_deriv_scale;
			float dys = (next.at<float>(r + 1, c) - next.at<float>(r - 1, c) -
				prev.at<float>(r + 1, c) + prev.at<float>(r - 1, c))*cross_deriv_scale;

			cv::Matx33f H(dxx, dxy, dxs,
				dxy, dyy, dys,
				dxs, dys, dss);

			cv::Vec3f X = H.solve(dD, cv::DECOMP_LU);

			xi = -X[2];
			xr = -X[1];
			xc = -X[0];

			if (std::abs(xi) < 0.5f && std::abs(xr) < 0.5f && std::abs(xc) < 0.5f)
				break;

			if (std::abs(xi) > (float)(INT_MAX / 3) ||
				std::abs(xr) > (float)(INT_MAX / 3) ||
				std::abs(xc) > (float)(INT_MAX / 3))
				return false;

			c += cvRound(xc);
			r += cvRound(xr);
			layer += cvRound(xi);

			if (layer < 1 || layer > mnOctaveLayers ||
				c < _sift_img_border || c >= img.cols - _sift_img_border ||
				r < _sift_img_border || r >= img.rows - _sift_img_border)
				return false;
		}

		// ensure convergence of interpolation
		if (i >= _sift_max_interp_setps)
			return false;

		{
			int idx = octv*(mnOctaveLayers + 2) + layer;
			const cv::Mat& img = dog_pyr[idx];
			const cv::Mat& prev = dog_pyr[idx - 1];
			const cv::Mat& next = dog_pyr[idx + 1];
			cv::Matx31f dD((img.at<float>(r, c + 1) - img.at<float>(r, c - 1))*deriv_scale,
				(img.at<float>(r + 1, c) - img.at<float>(r - 1, c))*deriv_scale,
				(next.at<float>(r, c) - prev.at<float>(r, c))*deriv_scale);
			float t = dD.dot(cv::Matx31f(xc, xr, xi));

			contr = img.at<float>(r, c)*img_scale + t * 0.5f;
			if (std::abs(contr) * mnOctaveLayers < mcontrastThreshold)
				return false;

			// principal curvatures are computed using the trace and det of Hessian
			float v2 = img.at<float>(r, c)*2.f;
			float dxx = (img.at<float>(r, c + 1) + img.at<float>(r, c - 1) - v2)*second_deriv_scale;
			float dyy = (img.at<float>(r + 1, c) + img.at<float>(r - 1, c) - v2)*second_deriv_scale;
			float dxy = (img.at<float>(r + 1, c + 1) - img.at<float>(r + 1, c - 1) -
				img.at<float>(r - 1, c + 1) + img.at<float>(r - 1, c - 1)) * cross_deriv_scale;
			float tr = dxx + dyy;
			float det = dxx * dyy - dxy * dxy;

			if (det <= 0 || tr*tr*edgeThreshold >= (edgeThreshold + 1)*(edgeThreshold + 1)*det)
				return false;
		}

		kpt.pt.x = (c + xc) * (1 << octv);
		kpt.pt.y = (r + xr) * (1 << octv);
		kpt.octave = octv + (layer << 8) + (cvRound((xi + 0.5) * 255) << 16);
		kpt.size = msigma*powf(2.f, (layer + xi) / mnOctaveLayers)*(1 << octv) * 2;
		kpt.response = std::abs(contr);

		return true;
	}

	SIFTextractor::SIFTextractor(int _nfeatures, float _sigma, int _nlevels) : mnfeatures(_nfeatures), msigma(1.6), mnOctaveLayers(3), mcontrastThreshold(0.04), medgeThreshold(10)
	{
		//int numOctaveLayers = mnOctaveLayers + 3;
		int numOctaveLayers = 9;
		sig.resize(numOctaveLayers);
		sig[0] = msigma;
		double k = pow(2., 1. / mnOctaveLayers);
		for (int i = 1; i < mnOctaveLayers + 3; i++)
		{
			double sig_prev = pow(k, (double)(i - 1))*msigma;
			double sig_total = sig_prev*k;
			sig[i] = std::sqrt(sig_total*sig_total - sig_prev*sig_prev);
		}

		mvScaleFactor.resize(numOctaveLayers);
		mvLevelSigma2.resize(numOctaveLayers);
		mvInvLevelSigma2.resize(numOctaveLayers);
		//mvScaleFactor[0] = 1.0f;
		//mvLevelSigma2[0] = 1.0f;
		for (int i = 0; i<numOctaveLayers; i++)
		{
			mvScaleFactor[i] = pow(2,i);
			//mvLevelSigma2[i] = msigma * msigma;
			//mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
			mvInvLevelSigma2[i] = msigma * msigma;
			mvLevelSigma2[i] = 1.0f / mvInvLevelSigma2[i];
		}
	}

	void SIFTextractor::buildGaussianPyramid(const cv::Mat& base, std::vector<cv::Mat>& pyr, int nOctaves)
	{
		//std::vector<double> sig(mnOctaveLayers + 3);
		pyr.resize(nOctaves*(mnOctaveLayers + 3));


		for (int o = 0; o < nOctaves; o++)
		{
			for (int i = 0; i < mnOctaveLayers + 3; i++)
			{
				cv::Mat& dst = pyr[o*(mnOctaveLayers + 3) + i];
				if (o == 0 && i == 0)
					dst = base;
				// base of new octave is halved image from end of previous octave
				else if (i == 0)
				{
					const cv::Mat& src = pyr[(o - 1)*(mnOctaveLayers + 3) + mnOctaveLayers];
					cv::resize(src, dst, cv::Size(src.cols / 2, src.rows / 2),
						0, 0, cv::INTER_NEAREST);
				}
				else
				{
					const cv::Mat& src = pyr[o*(mnOctaveLayers + 3) + i - 1];
					cv::GaussianBlur(src, dst, cv::Size(), sig[i], sig[i]);
				}
			}
		}
	}

	void SIFTextractor::buildDoGPyramid(const std::vector<cv::Mat>& gpyr, std::vector<cv::Mat>& dogpyr)
	{
		int nOctaves = (int)gpyr.size() / (mnOctaveLayers + 3);
		dogpyr.resize(nOctaves*(mnOctaveLayers + 2));

		for (int o = 0; o < nOctaves; o++)
		{
			for (int i = 0; i < mnOctaveLayers + 2; i++)
			{
				const cv::Mat& src1 = gpyr[o*(mnOctaveLayers + 3) + i];
				const cv::Mat& src2 = gpyr[o*(mnOctaveLayers + 3) + i + 1];
				cv::Mat& dst = dogpyr[o*(mnOctaveLayers + 2) + i];
				subtract(src2, src1, dst, cv::noArray(), cv::DataType<float>::type);
			}
		}
	}

	void SIFTextractor::findScaleSpaceExtrema(const std::vector<cv::Mat>& gauss_pyr, const std::vector<cv::Mat>& dog_pyr, std::vector<cv::KeyPoint>& keypoints)
	{
		int nOctaves = (int)gauss_pyr.size() / (mnOctaveLayers + 3);
		int threshold = cvFloor(0.5 * mcontrastThreshold / mnOctaveLayers * 255 * _sift_fixpt_scale);
		const int n = _sift_ori_hist_bins;
		float hist[36];
		cv::KeyPoint kpt;

		keypoints.clear();

		for (int o = 0; o < nOctaves; o++)
			for (int i = 1; i <= mnOctaveLayers; i++)
			{
				int idx = o*(mnOctaveLayers + 2) + i;
				const cv::Mat& img = dog_pyr[idx];
				const cv::Mat& prev = dog_pyr[idx - 1];
				const cv::Mat& next = dog_pyr[idx + 1];
				int step = (int)img.step1();
				int rows = img.rows, cols = img.cols;

				for (int r = _sift_img_border; r < rows - _sift_img_border; r++)
				{
					const float* currptr = img.ptr<float>(r);
					const float* prevptr = prev.ptr<float>(r);
					const float* nextptr = next.ptr<float>(r);

					for (int c = _sift_img_border; c < cols - _sift_img_border; c++)
					{
						float val = currptr[c];

						// find local extrema with pixel accuracy
						if (std::abs(val) > threshold &&
							((val > 0 && val >= currptr[c - 1] && val >= currptr[c + 1] &&
							val >= currptr[c - step - 1] && val >= currptr[c - step] && val >= currptr[c - step + 1] &&
							val >= currptr[c + step - 1] && val >= currptr[c + step] && val >= currptr[c + step + 1] &&
							val >= nextptr[c] && val >= nextptr[c - 1] && val >= nextptr[c + 1] &&
							val >= nextptr[c - step - 1] && val >= nextptr[c - step] && val >= nextptr[c - step + 1] &&
							val >= nextptr[c + step - 1] && val >= nextptr[c + step] && val >= nextptr[c + step + 1] &&
							val >= prevptr[c] && val >= prevptr[c - 1] && val >= prevptr[c + 1] &&
							val >= prevptr[c - step - 1] && val >= prevptr[c - step] && val >= prevptr[c - step + 1] &&
							val >= prevptr[c + step - 1] && val >= prevptr[c + step] && val >= prevptr[c + step + 1]) ||
							(val < 0 && val <= currptr[c - 1] && val <= currptr[c + 1] &&
							val <= currptr[c - step - 1] && val <= currptr[c - step] && val <= currptr[c - step + 1] &&
							val <= currptr[c + step - 1] && val <= currptr[c + step] && val <= currptr[c + step + 1] &&
							val <= nextptr[c] && val <= nextptr[c - 1] && val <= nextptr[c + 1] &&
							val <= nextptr[c - step - 1] && val <= nextptr[c - step] && val <= nextptr[c - step + 1] &&
							val <= nextptr[c + step - 1] && val <= nextptr[c + step] && val <= nextptr[c + step + 1] &&
							val <= prevptr[c] && val <= prevptr[c - 1] && val <= prevptr[c + 1] &&
							val <= prevptr[c - step - 1] && val <= prevptr[c - step] && val <= prevptr[c - step + 1] &&
							val <= prevptr[c + step - 1] && val <= prevptr[c + step] && val <= prevptr[c + step + 1])))
						{
							int r1 = r, c1 = c, layer = i;
							if (!adjustLocalExtrema(dog_pyr, kpt, o, layer, r1, c1,
								mnOctaveLayers, (float)mcontrastThreshold,
								(float)medgeThreshold, (float)msigma))
								continue;
							float scl_octv = kpt.size*0.5f / (1 << o);
							float omax = calcOrientationHist(gauss_pyr[o*(mnOctaveLayers + 3) + layer],
								cv::Point(c1, r1),
								cvRound(_sift_ori_radius * scl_octv),
								_sift_ori_sig_fctr * scl_octv,
								hist, n);
							float mag_thr = (float)(omax * _sift_ori_peak_ratio);
							for (int j = 0; j < n; j++)
							{
								int l = j > 0 ? j - 1 : n - 1;
								int r2 = j < n - 1 ? j + 1 : 0;

								if (hist[j] > hist[l] && hist[j] > hist[r2] && hist[j] >= mag_thr)
								{
									float bin = j + 0.5f * (hist[l] - hist[r2]) / (hist[l] - 2 * hist[j] + hist[r2]);
									bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
									kpt.angle = 360.f - (float)((360.f / n) * bin);
									if (std::abs(kpt.angle - 360.f) < FLT_EPSILON)
										kpt.angle = 0.f;
									keypoints.push_back(kpt);
								}
							}
						}
					}
				}
			}
	}

	static void calcDescriptors(const vector<Mat>& gpyr, const vector<KeyPoint>& keypoints,
		Mat& descriptors, int nOctaveLayers, int firstOctave)
	{
		int d = 4, n = 4;

		for (size_t i = 0; i < keypoints.size(); i++)
		{
			KeyPoint kpt = keypoints[i];
			int octave, layer;
			float scale;
			unpackOctave(kpt, octave, layer, scale);
			CV_Assert(octave >= firstOctave && layer <= nOctaveLayers + 2);
			float size = kpt.size*scale;
			Point2f ptf(kpt.pt.x*scale, kpt.pt.y*scale);
			const Mat& img = gpyr[(octave - firstOctave)*(nOctaveLayers + 3) + layer];

			float angle = 360.f - kpt.angle;
			if (std::abs(angle - 360.f) < FLT_EPSILON)
				angle = 0.f;
			calcSIFTDescriptor(img, ptf, angle, size*0.5f, d, n, descriptors.ptr<float>((int)i));
		}
	}

	void SIFTextractor::computeKeyPoints(cv::Mat image, cv::Mat depth, std::vector<cv::KeyPoint>& keypoints/*, cv::Mat &_descriptor*/)
	{
		int firstOctave = 0, actualNOctaves = 0, actualNLayers = 0;

		if (image.empty() || image.depth() != CV_8U)
			CV_Error(CV_StsBadArg, "image is empty or has incorrect depth (!=CV_8U)");

		if (depth.empty())
			CV_Error(CV_StsBadArg, "depth has incorrect type ");

		cv::Mat base = createInitialImage(image, firstOctave < 0, (float)msigma);
		std::vector<cv::Mat> gpyr, dogpyr;
		int nOctaves = actualNOctaves > 0 ? actualNOctaves : cvRound(log((double)std::min(base.cols, base.rows)) / log(2.) - 2) - firstOctave;
		buildGaussianPyramid(base, gpyr, nOctaves);
		buildDoGPyramid(gpyr, dogpyr);

		findScaleSpaceExtrema(gpyr, dogpyr, keypoints);
		cv::KeyPointsFilter::removeDuplicated(keypoints);

		std::vector<cv::KeyPoint> tmpkeypoints = keypoints;
		keypoints.clear();
		for (size_t i = 0; i < tmpkeypoints.size(); i++)
		{
			cv::KeyPoint kpt = tmpkeypoints[i];
			float scale = 1.f / (float)(1 << -firstOctave);
			kpt.octave = (kpt.octave & ~255) | ((kpt.octave + firstOctave) & 255);
			kpt.pt *= scale;
			kpt.pt.x = cvRound(kpt.pt.x);
			kpt.pt.y = cvRound(kpt.pt.y);
			kpt.size *= scale;
			//if (depth.at<float>(kpt.pt.y, kpt.pt.x) > 0)
			//{
				//cout << depth.ptr<ushort>(cvRound(kpt.pt.y))[cvRound(kpt.pt.x)] << endl;
				//cout << depth.at<float>(cvRound(kpt.pt.y), cvRound(kpt.pt.x)) << endl;

				keypoints.push_back(kpt);
			//}
		}
		//if (mnfeatures > 0)
		//	KeyPointsFilter::retainBest(keypoints, mnfeatures*1.5);
		//_descriptor.create((int)keypoints.size(),128,cv_32);
		//calcDescriptors(gpyr, keypoints, _descriptor, mnOctaveLayers,firstOctave);
	}

	void SIFTextractor::operator()(cv::Mat image, cv::Mat depth, std::vector<cv::KeyPoint>& keypoints, cv::Mat &descriptors, vector<int> &mvKeyPointsoctaves)
	{
		//computeKeyPoints(image,depth,keypoints,descriptors);
		computeKeyPoints(image, depth, keypoints);
		cv::SIFT sift(mnfeatures,mnOctaveLayers);
		sift(image, cv::Mat(), keypoints, descriptors, true);
		std::map<int, float> vmap;
		std::map<int, float>::iterator iter = vmap.begin();
		mvKeyPointsoctaves.resize(keypoints.size());

		int firstOctave = 0;
		int maxOctave = INT_MIN;
		int actualNLayers = 0, actualNOctaves = 0;

		for (size_t i = 0; i < keypoints.size(); i++)
		{
			int octave, layer;
			float scale;
			unpackOctave(keypoints[i], octave, layer, scale);
			keypoints[i].octave = octave+1;
			firstOctave = std::min(firstOctave, octave);
			maxOctave = std::max(maxOctave, octave);
			actualNLayers = std::max(actualNLayers, layer - 2);
			mvKeyPointsoctaves[i] = keypoints[i].octave;
			//if (i==0)
			//	vmap.insert(std::make_pair(octave, scale));
			//else
			//{
			//	for (iter = vmap.begin(); iter != vmap.end(); iter++)
			//	{
			//		if (iter->first != octave&&iter->second != scale)
			//		{
			//			vmap.insert(std::make_pair(octave, scale));
			//		}
			//	}
			//}

		}
		if (mnfeatures > 0)
			KeyPointsFilter::retainBest(keypoints, mnfeatures*1.5);
		firstOctave = std::min(firstOctave, 0);
		assert(firstOctave >= -1 && actualNLayers <= mnOctaveLayers);
		//actualNOctaves = maxOctave - firstOctave + 1;

		//cv::SIFT sift(mnfeatures,mnOctaveLayers);
		//sift(image, cv::Mat(), keypoints,descriptors, true);
		//cv::initModule_nonfree();
		//cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = cv::DescriptorExtractor::create("SIFT");
		//descriptor_extractor->compute(image,keypoints,descriptors);
	}
} //namespace ORB_SLAM

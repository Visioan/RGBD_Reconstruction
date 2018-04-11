#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "../RGBD_Reconstruction/helper.h"
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/time.h>

#include "TSDFVolumeUnit.h"

class VolumeUnit
{
public:
	VolumeUnit(float sdf, float weigth, int xi, int yi, int zi, unsigned rgb) :
		sdf_(sdf), weight_(weigth), xi_(xi), yi_(yi), zi_(zi), rgb_(rgb)
	{};
	VolumeUnit(){};
	~VolumeUnit(){};
	float sdf_;
	float weight_;
	int xi_, yi_, zi_;
	unsigned int rgb_;
};

class TSDF_Volume
{
public:
	TSDF_Volume(int cols, int rows, int resolution, float uint);
	~TSDF_Volume(void);

public:
	void IntegrateWithRGB(const std::vector< unsigned short > & depth, const std::vector< unsigned int > & color, const Eigen::Matrix4d & transformation);

	pcl::PointCloud<pcl::PointXYZRGB> GetPointCloudWithRGB();
	pcl::PointCloud<pcl::PointXYZI> GetPointCloud();
	CameraParam camera_;
	int cols_, rows_;

	int resolution_;
	int half_voxel;
	double unit_length_;
	double tsdf_trunc_;
	float scale_;
	std::unordered_map< int, VolumeUnit> data_;
private:

	int round(double x) {
		return static_cast< int >(floor(x + 0.5));
	}
	bool UVD2XYZ(int u, int v, unsigned short d, double & x, double & y, double & z) {
		if (d > 0) {
			z = d / scale_;
			x = (u - camera_.cx_) * z / camera_.fx_;
			y = (v - camera_.cy_) * z / camera_.fy_;
			return true;
		}
		else {
			return false;
		}
	}

	bool XYZ2UVD(double x, double y, double z, int & u, int & v, unsigned short & d) {
		if (z > 0) {
			u = round(x * camera_.fx_ / z + camera_.cx_);
			v = round(y * camera_.fy_ / z + camera_.cy_);
			d = static_cast< unsigned short >(round(z * scale_));
			return (u >= 0 && u < 640 && v >= 0 && v < 480);
		}
		else {
			return false;
		}
	}
	int hash_key(int x, int y, int z) {
		return x * resolution_ * resolution_ + y * resolution_ + z;
	}





};

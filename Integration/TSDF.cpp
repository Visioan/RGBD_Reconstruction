#include "TSDF.hpp"

TSDF_Volume::TSDF_Volume(int cols, int rows, int resolution,float uint)
	: cols_(cols)
	, rows_(rows)
	, unit_length_(uint)
	, tsdf_trunc_(0.03)
	, scale_(1000.0), resolution_(resolution)
{
	half_voxel = resolution_ / 2;
}

TSDF_Volume::~TSDF_Volume(void)
{
}

void TSDF_Volume::IntegrateWithRGB(const std::vector< unsigned short > & depth, const std::vector< unsigned int > & color, const Eigen::Matrix4d & transformation)
{
	Eigen::Matrix4d trans_inv = transformation.inverse();
	std::unordered_set< int > touched_unit;

	double x, y, z;
	int xi, yi, zi, key;
	for (int v = 0; v < rows_; v += 1) {
		for (int u = 0; u < cols_; u += 1) {
			unsigned short d = depth[v * cols_ + u];
			if (UVD2XYZ(u, v, d, x, y, z)) {
				Eigen::Vector4d vv = transformation * Eigen::Vector4d(x, y, z, 1);
				xi = ((int)floor(vv(0) / unit_length_ + 0.5) );
				yi = ((int)floor(vv(1) / unit_length_ + 0.5) );
				zi = ((int)floor(vv(2) / unit_length_ + 0.5) );

				float dp_scaled;
				unsigned short d = depth[v * cols_ + u];
				float xl = (u - camera_.cx_) / camera_.fx_;
				float yl = (v - camera_.cy_) / camera_.fy_;
				float lambda = sqrtf(xl * xl + yl * yl + 1);
				float res = d * lambda / 1000.f;
				if (res > camera_.integration_trunc_) 
					dp_scaled = 0.0f;				
				else 
					dp_scaled = depth[v * cols_ + u] * lambda / 1000.f;
				
				Eigen::Vector4f gridv(xi*unit_length_, yi*unit_length_, zi*unit_length_, 1);

				if (dp_scaled < 0.001f)
					continue;
				float rx = gridv(0) - transformation(0, 3);
				float ry = gridv(1) - transformation(1, 3);
				float rz = gridv(2) - transformation(2, 3);
				float sdf = dp_scaled - sqrtf(rx * rx + ry * ry + rz * rz);
				if (sdf < -tsdf_trunc_)
					continue;
				float tsdf = std::min< float >(1.0f, sdf / tsdf_trunc_);
				// need to change weight
				float w = 1.0f;

				key = hash_key(xi, yi, zi);
				if (touched_unit.find(key) == touched_unit.end())
				{
					touched_unit.insert(key);
					data_[key] = VolumeUnit((w * tsdf) / w, w, xi, yi, zi, color[v * cols_ + u]);
				}
				else
				{
					data_[key].sdf_ = (data_[key].sdf_ * data_[key].weight_ + w * tsdf) / (data_[key].weight_+ w);
					data_[key].weight_ += w;
					data_[key].rgb_ = color[v * cols_ + u];
				}
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGB> TSDF_Volume::GetPointCloudWithRGB()
{
	//PointCloud< PointXYZRGB >::Ptr world(new PointCloud< PointXYZRGB >);
	pcl::PointCloud<pcl::PointXYZRGB>  pointcloud;

	for (std::unordered_map< int, VolumeUnit >::iterator it = data_.begin(); it != data_.end(); it++)
	{
		VolumeUnit unit = it->second;
		float sdf = unit.sdf_;
		float w = unit.weight_;
		unsigned int rgb = unit.rgb_;
		{
			if (w != 0.0f && sdf < 0.98f && sdf >= -0.98f) {
				pcl::PointXYZRGB p;
				p.x = unit.xi_;
				p.y = unit.yi_;
				p.z = unit.zi_;
				p.r = unsigned char(rgb & 0xff0000 >> 16);
				rgb = rgb >> 8;
				p.g = unsigned char(rgb & 0xff00 >> 8);
				rgb = rgb >> 8;
				p.b = unsigned char(rgb);
				pointcloud.push_back(p);
			}
		}
	}
	PCL_INFO("%d voxel rgb points .\n", pointcloud.size());
	return pointcloud;

}

pcl::PointCloud<pcl::PointXYZI> TSDF_Volume::GetPointCloud()
{
	//PointCloud< PointXYZRGB >::Ptr world(new PointCloud< PointXYZRGB >);
	pcl::PointCloud<pcl::PointXYZI>  pointcloud;

	for (std::unordered_map< int, VolumeUnit >::iterator it = data_.begin(); it != data_.end(); it++)
	{
		VolumeUnit unit = it->second;
		float sdf = unit.sdf_;
		float w = unit.weight_;
		unsigned int rgb = unit.rgb_;
		{
			if (w != 0.0f && sdf < 0.98f && sdf >= -0.98f) {
				pcl::PointXYZI p;
				p.x = unit.xi_;
				p.y = unit.yi_;
				p.z = unit.zi_;
				p.intensity = *sdf;
				pointcloud.push_back(p);
			}
		}
	}
	PCL_INFO("%d voxel rgb points .\n", pointcloud.size());
	return pointcloud;

}


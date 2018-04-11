#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <string>
#include<conio.h>//内含有kbhit()函数    在VC里面有这个头文件
#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <time.h>
#include <stdlib.h>
#include <thread>
#include<mutex>
#include <stdio.h>

#include <pcl/io/ply_io.h>
#include <pcl\io\pcd_io.h>
#include <pcl/io/grabber.h>
//#include <pcl/io/openni2_grabber.h>
//#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/vtk_lib_io.h> 
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl\console\print.h>
#include <pcl\console\parse.h>
#include <pcl\console\time.h>
#include <pcl/common/time.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>  

//#include <opencv2\core.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//boost库用来正则化string
#include <boost\format.hpp>
#include <boost/tokenizer.hpp>     
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>//

#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#endif    // _COMMON_H_
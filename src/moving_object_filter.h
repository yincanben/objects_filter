/*************************************************************************
	> File Name: moving_object_filter.h
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时05分32秒
 ************************************************************************/

#ifndef _MOVING_OBJECT_FILTER_H
#define _MOVING_OBJECT_FILTER_H
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>

typedef pcl::PointXYZRGBA point_type ;
typedef pcl::PointCloud<point_type> cloud_type ;

#define threshod_binary 30
class MovingObjectFilter{
    public:
        MovingObjectFilter() ;
        void ExtractObject( cloud_type::ConstPtr cloud, cv::Mat& gray_img ) ;
        cloud_type::ConstPtr get_filter_cloud();
        void clear_object_cloud(  );
    private:
        void image_diff(const cv::Mat &gray_img);
        void image_separate( cloud_type::ConstPtr cloud ) ;
        void pcl_segmentation( cloud_type::ConstPtr cloud ) ;
        bool image_extract_cluster( cloud_type::ConstPtr cloud ) ;
        void transform_coordinate(cloud_type::ConstPtr cloud) ;
        cv::Mat last_image ;
        cv::Mat binary_image ;
        cv::Mat previous_frame ;
        cv::Mat current_frame ;
        cloud_type last_cloud ;
        cloud_type object_cloud ; //save the moving object
        cloud_type filter_cloud ; //save the rest cloud(i.e don't include moving object)
        int frame_count  ;//set the interval of frame_count
        std::vector<Eigen::Vector3f> previous_coordinate ;
        std::vector<Eigen::Vector3f> current_coordinate ;
        double previous_z , current_z ;
        //double deta_x , deta_y ;
        int count ;

};
#endif

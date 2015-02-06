/*************************************************************************
	> File Name: main.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时04分34秒
 ************************************************************************/

#include <pcl/io/openni_grabber.h>
#include "moving_object_filter.h"
//typedef pcl::PointXYZRGBA point_type;
//typedef pcl::PointCloud<point_type> cloud_type;


void convert_to_img(const cloud_type& cloud, cv::Mat& gray_img){
    gray_img.create(cloud.height, cloud.width, CV_8UC1);
    for(int row = 0; row < cloud.height;row++){
        for(int col = 0; col < cloud.width;col++){
            const point_type& pt = cloud.at(col, row);
            gray_img.at<unsigned char>(row, col) = 0.3*pt.r + 0.6*pt.g + 0.1*pt.b;
        }//for col
    }//for row
}

void cloud_callback(cloud_type::ConstPtr cloud, MovingObjectFilter* mvObjFil, pcl::visualization::CloudViewer* cloud_viewer ){
    cv::Mat gray_img; //Create a grayscale image for feature extraction
    convert_to_img(*cloud, gray_img); //Extract 2D information
    static double last = pcl::getTime ();
    mvObjFil->ExtractObject( cloud , gray_img ) ;
    double now = pcl::getTime ();
    std::cout << "time = " << now - last << std::endl ;
    last = now ;

    //cv::imshow("Current View", gray_img);
    if(cv::waitKey(1) > 0)
        exit(0);
    
    if(!cloud_viewer->wasStopped()){// && !nicv->get_world_cloud()->empty())
        cloud_viewer->showCloud( mvObjFil->get_filter_cloud() );
        mvObjFil->clear_object_cloud();
    }
}


int main (int argc, const char** argv){
    MovingObjectFilter mvObjFilter ;
    pcl::visualization::CloudViewer cloud_viewer("Moving Object Viewer") ;

    boost::function<void (const cloud_type::ConstPtr&)> f = boost::bind( cloud_callback, _1, &mvObjFilter, &cloud_viewer );
    pcl::OpenNIGrabber interface;
    interface.registerCallback (f);
    interface.start();
    while(true)
        sleep(1);
    interface.stop();
    return 0;
}

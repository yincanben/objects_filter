/*************************************************************************
	> File Name: moving_object_filter.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时05分54秒
 ************************************************************************/
 #include "moving_object_filter.h"
 
 MovingObjectFilter::MovingObjectFilter():frame_count(0) 
 {
    std::cout << "constructor" << std::endl ;
    //previous_frame.create(640,480,)
 }
 
 void MovingObjectFilter::ExtractObject( cloud_type::ConstPtr cloud, cv::Mat &gray_image ){
    //std::cout << "ExtractObject" << std::endl ;
    image_diff(gray_image) ;
    //transform_coordinate(cloud) ;
    image_separate( cloud ) ;
    double last1 = pcl::getTime() ;
    pcl_segmentation( cloud ) ;
    double now1 = pcl::getTime() ;
     std::cout << "Time = " << now1-last1 << std::endl ;
    
 }

void MovingObjectFilter::image_diff(const cv::Mat &gray_image){
    //std::cout << "Processing the image" << std::endl ;
    cv::Mat BlurImage1, BlurImage2, diff_image ;
    if(last_image.empty()){//for first frame
        last_image = gray_image ;
        cv::GaussianBlur( last_image, BlurImage2, cv::Size( 11, 11 ), 0, 0 );
    }else{// if(  frame_count%5 == 0 ){
        cv::GaussianBlur( gray_image, BlurImage1, cv::Size( 11, 11 ), 0, 0 );
        cv::GaussianBlur( last_image, BlurImage2, cv::Size( 11, 11 ), 0, 0 );
        last_image = gray_image ;
        cv::absdiff( BlurImage1, BlurImage2, diff_image ) ;
        //cv::imshow( "Binary Image", diff_image );
        cv::threshold( diff_image, binary_image, threshod_binary, 255, cv::THRESH_BINARY ); 
        //cv::imshow( "Binary Image", binary_image );
    /*}else{
        frame_count ++ ;
        if( frame_count > 50000 )    frame_count = 0 ;
        */
    }
}
void MovingObjectFilter::image_separate( cloud_type::ConstPtr cloud ){
    previous_frame.create( cloud->height, cloud->width, CV_8UC1 ) ;
    current_frame.create( cloud->height, cloud->width, CV_8UC1 ) ;
    previous_z = 0.0 ;
    current_z = 0.0 ;
    //int previous_count = 0;
    //int current_count = 0 ;
    Eigen::Vector3f v1;
    if(last_cloud.size() == 0){ //for first frame
        std::cout << "first cloud" << std::endl ;
        last_cloud = *cloud ;
    }else{

        for( int row = 0; row < cloud->height; row++ ){
            for(int col = 0; col < cloud->width; col++ ){
                previous_z = isnan( last_cloud.at(col,row).z) ? 20 : last_cloud.at(col,row).z ;
                current_z = isnan( cloud->at(col,row).z) ? 20 : cloud->at(col,row).z ;
                if(binary_image.at<unsigned char>(row,col) == 255){
                    //std::cout << last_cloud.at(col,row).z << std::endl ; 
                    if( previous_z < current_z ){
                        //std::cout << "test" << std::endl ;
                        previous_frame.at<unsigned char>(row,col) = 255 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;
                        v1 << last_cloud.at(col,row).x , last_cloud.at(col,row).y , last_cloud.at(col,row).z ;
                        previous_coordinate.push_back(v1) ;
                        //previous_coordinate[previous_count].x = last_cloud.at(col,row).x ;
                        /*previous_coordinate[previous_count].x = last_cloud.at(col,row).x ;
                        previous_coordinate[previous_count].y = last_cloud.at(col,row).y ;
                        previous_coordinate[previous_count].z = last_cloud.at(col,row).z ;*/
                        //previous_count++ ;
                        //std::cout << "previous_count = " << previous_count << std::endl ;
                        
                    }else if( previous_z > current_z ){
                        previous_frame.at<unsigned char>(row,col) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 255 ;
                        v1 << cloud->at(col,row).x , cloud->at(col,row).y , cloud->at(col,row).z ;
                        current_coordinate.push_back(v1) ;
                        /*
                        current_coordinate[current_count].x() = cloud->at(col,row).x ;
                        current_coordinate[current_count].y() = cloud->at(col,row).y ;
                        current_coordinate[current_count].z() = cloud->at(col,row).z ;
                        current_count++ ;
                        std::cout << "current_count = " << current_count << std::endl ;
                        */
                    }else{
                        previous_frame.at<unsigned char>( row, col ) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;   
                    }
                }else{
                    previous_frame.at<unsigned char>(row,col) = 0 ;
                    current_frame.at<unsigned char>(row,col) = 0 ;
                }
            }//for col
        }//for row
        last_cloud = *cloud ;
        //cv::imshow("previous_frame",previous_frame) ;
        //cv::imshow("current_frame",current_frame) ;
        //std::cout << "other cloud" << std::endl ;
    }
}

void MovingObjectFilter::pcl_segmentation( cloud_type::ConstPtr cloud ){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>); 
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGBA>); 
        
    // Step 1: Filter out NaNs from data
    pcl::PassThrough<pcl::PointXYZRGBA> pass; 
    pass.setInputCloud (cloud); 
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (0.5, 6.0); 
    pass.filter (*cloud_filtered); 
    /*
    pass.setInputCloud (cloud_filtered); 
    pass.setFilterFieldName ("x"); 
    pass.setFilterLimits (-4.0, 4.0); 
    //pass.setFilterLimitsNegative (true); 
    pass.filter (*cloud_filtered); 

    pass.setInputCloud (cloud_filtered); 
    pass.setFilterFieldName ("y"); 
    pass.setFilterLimits (-4.0, 4.0); 
    pass.filter (*cloud_filtered);
    */

    // Step 2: Filter out statistical outliers*****************influence the speed
    /*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor ;
    sor.setInputCloud(cloud_filtered) ;
    sor.setMeanK(50) ;
    sor.setStddevMulThresh(1.0) ;
    sor.filter(*cloud_filtered) ;
    */

    // Step 3: Downsample the point cloud (to save time in the next step)
    
    pcl::VoxelGrid<pcl::PointXYZRGBA> downSampler; 
    downSampler.setInputCloud (cloud_filtered); 
    downSampler.setLeafSize (0.01f, 0.01f, 0.01f); 
    downSampler.filter (*cloud_filtered2);
    
        
    // Step 4: Remove the ground plane using RANSAC 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg; 
    seg.setOptimizeCoefficients (true); // Optional 
    //seg.setMaxIteractions(100) ; //Optional,maybe can be lower
    // Mandatory 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    //seg.setMethodType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE) ;
    seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
    seg.setEpsAngle (pcl::deg2rad (10.0));
    //seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC); 
    seg.setDistanceThreshold (0.01); //1cm
    seg.setInputCloud (cloud_filtered2->makeShared()); 
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ()) ;
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (cloud_filtered2);
    // Step 4.1: Extract the points that lie in the ground plane
    extract.setIndices (inliers);
    if( inliers->indices.size() > 4000 ){
        cout << "indices size =  " << inliers->indices.size() << endl ;
        extract.setNegative (false);
        extract.filter (*cloud_plane);
    }
    // Step 4.2: Extract the points that are objects(i.e. are not in the ground plane)
    extract.setNegative (true);
    extract.filter ( *cloud_filtered2 ) ;
        
    if (inliers->indices.size () == 0) { 
        PCL_ERROR ("Could not estimate a planar model for the given dataset."); 
        //exit(0); 
    } 
    // PAINT SURFACE
    /*
    for (unsigned int i = 0; i < inliers->indices.size(); i++){ 
        int idx = inliers->indices[i]; 
        cloud_filtered2->points[idx].r = 255; 
        cloud_filtered2->points[idx].g = 0; 
        cloud_filtered2->points[idx].b = 0; 
    }*/ 
        
    // Step 5: EuclideanCluster Extract the moving objects
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (cloud_filtered2);

    std::vector<pcl::PointIndices> cluster_indices ;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec ;
    ec.setClusterTolerance (0.02) ; // 2cm
    ec.setMinClusterSize (1000) ;
    ec.setMaxClusterSize (25000) ;
    ec.setSearchMethod (tree) ;
    ec.setInputCloud (cloud_filtered2) ;
    ec.extract (cluster_indices) ;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    float minX(0.0), minY(0.0), minZ(0.0), maxX(0.0), maxY(0.0), maxZ(0.0) ;
    point_type cluster_point ;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cluster_point = cloud_filtered2->points[*pit] ;
            cloud_cluster->points.push_back (cluster_point); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        //cloud_cluster->width = 640 ;
        //cloud_cluster->height= 480 ;
        //cloud_cluster->resize(cloud_cluster->width * cloud_cluster->height) ;
        cloud_cluster->is_dense = true;
        
        if(image_extract_cluster(cloud_cluster)) {
            object_cloud += *cloud_cluster ;
            current_coordinate.clear();
            //std::cout << "true" << std::endl ;
            //break ;
        }else{
            //cloud_cluster->clear() ;
            continue ;
        }
            
        //hull.setInputCloud(cloud_cluster) ;
        //hull.setAlpha(0.1);
        //hull.setDimension( 3 );
        //hull.reconstruct( *concaveHull, polygons );
        //for(int i = 0; i < polygons.size(); i++)
        //std::cout << polygons[i] << std::endl;
            
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
        

    /*
    //viewer.showCloud(cloud_cluster); 
    //cloud_viewer->showCloud(cloud_plane) ;
    */
}

bool MovingObjectFilter::image_extract_cluster( cloud_type::ConstPtr cloud ){
    double minX(10.0), minY(10.0), minZ(10.0), maxX(0.0), maxY(0.0), maxZ(0.0), averageZ(0.0) ;
    count = 0 ;
    for(int i = 0; i<cloud->points.size(); i++){
        // for min  
        if ( cloud->points[i].x < minX )
            minX = cloud->points[i].x ;
        if ( cloud->points[i].y < minY )
            minY = cloud->points[i].y ;
        if (cloud->points[i].z < minZ)
            minZ = cloud->points[i].z ;
        //for max
        if (cloud->points[i].x > maxX)
            maxX = cloud->points[i].x;
        if (cloud->points[i].y > maxY)
            maxY = cloud->points[i].y;
        if (cloud->points[i].z > maxZ)
            maxZ = cloud->points[i].z;
        averageZ += cloud->points[i].z ;
    }
    averageZ = averageZ / cloud->points.size() ;
    for (std::vector<Eigen::Vector3f>::const_iterator it = current_coordinate.begin(); it != current_coordinate.end(); ++it){
        //Eigen::Vector3f v = *it ;
        if( it->x()>minX && it->x()<maxX && it->y()>minY && it->y()<maxY && abs((it->z()) - averageZ )<0.5){
            if(it->z()<minZ || it->z()>maxZ)    continue ;
            count ++ ;
        }
    }
    //std::cout << "size = "  << current_coordinate.size << std::endl ;
    /*
    int count = 0 ;
    minX = (minX + deta_x*320) / deta_x ;
    maxX = (maxX + deta_x*320) / deta_x ;
    minY = (minY + deta_y*240) / deta_y ;
    maxY = (maxY + deta_y*240) / deta_y ;
    std::cout << "minX = " << minX << ";maxX = " << maxX << "; miny= " << minY << "; maxY = " << maxY << std::endl ; 
    //std::cout << "height= " << last_cloud.height << "; width= " << last_cloud.width << std::endl ;
    
    for(int i=0 ; i<last_cloud.height; i++){
        for(int j=0; j<last_cloud.width; j++){
            if( current_frame.at<unsigned char>(i,j) == 255 ){
                if(i>minX && i<maxX && j>minY && j<maxY)    count ++ ;
                std::cout << "count = " << count << std::endl ;
            }
        }
    }*/

    /*
    cloud_type plane_cloud ;
    pcl::ConvexHull<pcl::PointXYZRGBA> chull;
    std::vector<pcl::Vertices> polygons ;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
    plane_cloud = *cloud ;
    for( int i=0 ; i<cloud->points.size(); i++ ){
        plane_cloud.points[i].z = 4.0 ;    
    }
    chull.setInputCloud ( plane_cloud ); 
    chull.setAlpha(0.2);
    chull.setDimension(2); 
    chull.reconstruct ( *convexhull, polygons ); 
    */
    if(count > 1500)
        std::cout << "count = " << count << std::endl ;
    if(count > 1500){
        //count = 0 ;
        return true ;
    }else{
        return false ;
    }
}

cloud_type::ConstPtr MovingObjectFilter::get_filter_cloud(){
    return object_cloud.makeShared() ;
}

void MovingObjectFilter::clear_object_cloud(){
    object_cloud.clear();
}

/*
void MovingObjectFilter::transform_coordinate(cloud_type::ConstPtr cloud){
    double min_x = 10.0, min_y = 10.0 , min_z = 10.0, max_x = 0.0, max_y = 0.0 , max_z = 0.0 ;
    deta_x = 0.0 ;
    deta_y = 0.0 ;
    for( int row=0 ; row<cloud->height; row++ ){
        for(int col=0; col<cloud->width; col++){
            const point_type& pt = cloud->at(col, row) ;
            if(pt.x < min_x)    min_x = pt.x ;
            if(pt.y < min_y)    min_y = pt.y ;
            if(pt.z < min_z)    min_z = pt.z ;
            if(pt.x > max_x)    max_x = pt.x ;
            if(pt.y > max_y)    max_y = pt.y ;
            if(pt.z > max_z)    max_z = pt.z ;
        }
    }
    deta_x = max_x - min_x ;
    deta_y = max_y - min_y ;
    std::cout << "deta_x/640 = " << deta_x/640 << ";deta_y/480 = " << deta_y/480 << std::endl ;
}*/

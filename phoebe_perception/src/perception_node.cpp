#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

//Service headers
#include <phoebe_perception/target_position.h>
//Message Publisher header: Uncomment for publishing
// #include <phoebe_perception/target_pose.h>




class GetHandlePosition
{
public:
  GetHandlePosition()
  {

    // object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
    // position_pub = nh.advertise<phoebe_perception::target_pose>("handles_position", 10);    
    cloud_topic = nh.param<std::string>("cloud_topic", "/xtion/depth/points");
    // point_cloud_sub_ = nh.subscribe(cloud_topic, 1, &GetHandlePosition::processPointCloudCb, this);
    position_server_ = nh.advertiseService("get_position", &GetHandlePosition::getPositionCb, this);



    world_frame = nh.param<std::string>("world_frame", "base_footprint");
    camera_frame = nh.param<std::string>("camera_frame", "xtion_optical_frame");
    voxel_leaf_size = nh.param<float>("voxel_leaf_size", 0.01);
    x_filter_min = nh.param<float>("x_filter_min", -1.0);
    x_filter_max = nh.param<float>("x_filter_max",  3.5);
    y_filter_min = nh.param<float>("y_filter_min", -1.0);
    y_filter_max = nh.param<float>("y_filter_max",  1.0);
    z_filter_min = nh.param<float>("z_filter_min", 0.1);
    z_filter_max = nh.param<float>("z_filter_max",  3.5);
    plane_max_iter = nh.param<int>("plane_max_iterations", 200);
    plane_dist_thresh = nh.param<float>("plane_distance_threshold", 0.02);
    cluster_tol = nh.param<float>("cluster_tolerance", 0.025);
    cluster_min_size = nh.param<int>("cluster_min_size", 200);
    cluster_max_size = nh.param<int>("cluster_max_size", 50000);
    //wait_on_transform_ = true;
  }


  bool getPositionCb(phoebe_perception::target_position::Request& req,
                     phoebe_perception::target_position::Response& res)
  {

    ROS_INFO("Request Received");
    processPointCloudCb();
    geometry_msgs::Point left_handle, right_handle; 
   
    left_handle.x = left.x;
    left_handle.y = left.y;
    left_handle.z = left.z;
   
    right_handle.x = right.x;
    right_handle.y = right.y;
    right_handle.z = right.z;

    res.points[0] = left_handle;
    res.points[1] = right_handle;
    return true;
  }



 // void processPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg) // Uncomment for subscriber and comment for service
  void processPointCloudCb(void)  // Comment for Subscriber and uncomment for service
  {        
      /*
       * LISTEN FOR POINTCLOUD For service call
       */
      std::string topic = nh.resolveName(cloud_topic);
      ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
      sensor_msgs::PointCloud2::ConstPtr recent_cloud =
                   ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh, ros::Duration(5.0));


       // For subscirber comment above lines and uncomment following
       // sensor_msgs::PointCloud2::ConstPtr recent_cloud = msg;            

      /*
      * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
      */
      tf::TransformListener listener;
      tf::StampedTransform stransform;
      try
      {
        listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(1.0));
        listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
      sensor_msgs::PointCloud2 transformed_cloud;    
      pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

      /*
       * CONVERT POINTCLOUD ROS->PCL
       */
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg (transformed_cloud, cloud);      
      
      // Apply VoxelGrid Filter to down sample
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
      applyVoxelGridFilter(cloud_ptr,cloud_filtered);

      //Apply Pass through Filters for x y and z
      applyPassthroughFilter(cloud_filtered);

      // Apply Planner Segmentation
      applyPlaneSegmentation(cloud_filtered);

      // Exctract Euclidean Cluster
      extractEuclideanCluster(cloud_filtered);


      // Remove Statistical Outlier and Get Handles Position
      getPositions(cloud_filtered); 
      
      // Convert to ROS sensor msg and publish
      // publishPointCloud(cloud_filtered);


  }

    

  
  ~GetHandlePosition(void)
  {

  }


private:

  ros::NodeHandle nh;
  //ros::NodeHandle priv_nh_("~");
  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string cloud_topic, world_frame, camera_frame;
  float voxel_leaf_size;
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  int plane_max_iter;
  float plane_dist_thresh;
  float cluster_tol;
  int cluster_min_size;
  int cluster_max_size;
  double cluster_max_x_;
  double cluster_min_x_;
  double cluster_max_y_;
  double cluster_min_y_;
  pcl::PointXYZ centroid , left, right;
  double orientation;
  //bool wait_on_transform_;
  /*
   * SETUP PUBLISHERS
   */
  // ros::Publisher object_pub;
  // ros::Publisher position_pub;

  // ROS Subscriber
  ros::Subscriber point_cloud_sub_;

  // ROS Server for giving handles position
  ros::ServiceServer position_server_ ;


  /* ========================================
   *  VOXEL GRID
   * ========================================*/
  void applyVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
  {

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;  
    voxel_filter.setInputCloud (input_cloud);
    voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter (*cloud_filtered);

    ROS_INFO("VoxelGrid Filter Applied");
    // ROS_INFO_STREAM("Original cloud  had " << input_cloud->size() << " points");
    // ROS_INFO_STREAM("Downsampled cloud  with " << cloud_filtered->size() << " points");

  }

  /* ========================================
     *  PASSTHROUGH FILTER(S)
     * ========================================*/
  void applyPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered )
  {
      
    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min, x_filter_max);
    pass_x.filter(xf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min, y_filter_max);
    pass_y.filter(yf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_filter_min, z_filter_max);
    pass_z.filter(zf_cloud);

    *cloud_filtered = zf_cloud;
    ROS_INFO("PassThrough Filter Applied");

  }

  /* ========================================
     *  PLANE SEGEMENTATION
     * ========================================*/
  void applyPlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
  {      
    
    pcl::PointCloud<pcl::PointXYZ> cloud_f ;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Optional
    seg.setOptimizeCoefficients (true);
    //Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (plane_max_iter);
    seg.setDistanceThreshold (plane_dist_thresh);
    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
      //break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (cloud_f);
    *cloud_filtered = cloud_f;
    //ROS_INFO_STREAM("PointCloud after removal of planar inliers: " << cloud_f->points.size () << " data points." );

  }


  /* ===================================
   * EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
   * ========================================*/
  void extractEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) 
  {

  // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);    
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (cluster_min_size);
    ec.setMaxClusterSize (cluster_max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
             cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
     
    
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;      

      ROS_INFO_STREAM("EUCLIDEAN Cluster has " << cloud_cluster->points.size() << " points." );
      clusters.push_back(cloud_cluster);

      cloud_filtered->swap(*(clusters.at(0)));    
      
    }
  }



  // Remove Statistical Outlier and Get Handles Position

  void getPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) 
  {

    /* ========================================
     *  STATISTICAL OUTLIER REMOVAL (OPTIONAL)
     * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr = input_cloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud (cluster_cloud_ptr);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (*sor_cloud_filtered);

    // input_cloud =   sor_cloud_filtered; 
    
    
    
    if(cluster_cloud_ptr->points.size () > 0)
    {  
      
       cluster_max_x_ = cluster_cloud_ptr->points[0].x;
       cluster_min_x_ = cluster_cloud_ptr->points[0].x;
       cluster_max_y_ = cluster_cloud_ptr->points[0].y;
       cluster_min_y_ = cluster_cloud_ptr->points[0].y;
        
       for (int i = 0; i < cluster_cloud_ptr->points.size(); i++) 
       {
          
       
         // if (cloud_cluster->points[i].x > cluster_max_x_) {
         // cluster_max_x_ = cloud_cluster.points[i].x;
         // }
         // if (cloud_cluster->points[i].x < cluster_min_x_) {        
         // cluster_min_x_ = cloud_cluster.points[i].x;
       
            // }
           
           if (cluster_cloud_ptr->points[i].y > cluster_max_y_) 
           {
              cluster_max_y_ = cluster_cloud_ptr->points[i].y;
              left = cluster_cloud_ptr->points[i];
              //ROS_INFO_STREAM("Left x: " << left.x << " y: " << left.y << " z: " << left.z  << "." );
                
           }
           if (cluster_cloud_ptr->points[i].y < cluster_min_y_) 
           {
              cluster_min_y_ = cluster_cloud_ptr->points[i].y;
              right = cluster_cloud_ptr->points[i];
              //ROS_INFO_STREAM("Right x: " << right.x << " y: " << right.y << " z: " << right.z  << "." );
           }          
        }  
     } 


  }



  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD 
   * ========================================*/
  /*
  void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
  {
    sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*input_cloud , *pc2_cloud);  
    pc2_cloud->header.frame_id=world_frame;
    pc2_cloud->header.stamp=ros::Time::now();
    object_pub.publish(pc2_cloud);


    // ========================================
    // BROADCAST TRANSFORM (OPTIONAL)
    // ========================================


    static tf::TransformBroadcaster br, lf, rt;
    tf::Transform center_transform, left_transform, right_transform;    
    
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    
    
    ROS_INFO_STREAM("Left x: " << left.x << " y: " << left.y << " z: " << left.z  << "." );
    left_transform.setOrigin( tf::Vector3(left.x,  left.y,   left.z) );  
    left_transform.setRotation(q);
    lf.sendTransform(tf::StampedTransform(left_transform, ros::Time::now(), world_frame, "left"));
    
    ROS_INFO_STREAM("Right x: " << right.x << " y: " << right.y << " z: " << right.z  << "." );
    right_transform.setOrigin( tf::Vector3(right.x,  right.y,   right.z) );  
    right_transform.setRotation(q);
    rt.sendTransform(tf::StampedTransform(right_transform, ros::Time::now(), world_frame, "right"));


    //Publish handles position: Uncomment for publishing 
    
    geometry_msgs::Point left_handle, right_handle;
    phoebe_perception::target_pose position_msg;

    position_msg.header.frame_id = world_frame;
    position_msg.header.stamp = ros::Time::now(); 
   
    left_handle.x = left.x;
    left_handle.y = left.y;
    left_handle.z = left.z;
   
    right_handle.x = right.x;
    right_handle.y = right.y;
    right_handle.z = right.z;

    position_msg.points[0] = left_handle;
    position_msg.points[1] = right_handle;

    position_pub.publish(position_msg); 
   
  }

  */
};


// ==========================================================
int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");

  GetHandlePosition handles_position;  

  ros::spin();

  // ros::Duration d(0.5);
  // while (ros::ok()) {

  //   ros::spinOnce();
  //   d.sleep();
  // }
  

  return 0;
}  
 
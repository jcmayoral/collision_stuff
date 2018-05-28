#include <footprint_checker/base_collision_checker.h>

BaseCollisionChecker::BaseCollisionChecker(ros::NodeHandle &nh):
        nh_(nh), is_point_cloud_received_(false), collision_threshold_(20.0),
        is_footprint_received(false), footprint_extender_(nh_),distance_to_obstacle_(0.06),
        static_collision_threshold_(20)
{
    orientations_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collisions_orientations", 1);
    //From Local_planner
    footprint_sub_ = nh.subscribe("/move_base/local_costmap/footprint",4, &BaseCollisionChecker::footprintCB, this);
    //PointCloud contains costs fo the lcoal planner DWA
    point_cloud_sub_ = nh_.subscribe("/move_base/DWAPlannerROS/cost_cloud",1, &BaseCollisionChecker::pointCloudCB, this);
    //Publisher of modified version of cost_cloud
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("overlap_costmap",2);
    //Publisher of collision Contact Points...Subscriber is on the Layer
    collision_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("collision_contact_point",1);

    nh.param("distance_to_obstacle_", distance_to_obstacle_,0.06);
    nh.param("collision_checker_threshold", collision_threshold_,30.0);
    nh.param("static_collision_threshold", static_collision_threshold_,25);
    ROS_INFO_STREAM("Distance to obstacle set to " << distance_to_obstacle_);
    ROS_INFO_STREAM("Static Collision Thrshold " << static_collision_threshold_);
    ROS_INFO("State: INIT");

    //define the servicee
    service = nh.advertiseService("/collision_checker",
        &BaseCollisionChecker::runService, this);

    ROS_INFO("Ready to start...");

    ros::spin();

}

BaseCollisionChecker::~BaseCollisionChecker()
{
}

bool BaseCollisionChecker::runService(footprint_checker::CollisionCheckerMsg::Request  &req,
         footprint_checker::CollisionCheckerMsg::Response &resp)
{
    resp.success = false;
    double threshold = 0.2;//how close should the orientation given should be from the orientations of the footprints to match a point of collision

    if (is_point_cloud_received_ && is_footprint_received){ // footprint and point_cloud must be received
        ROS_INFO_STREAM("Request Received");
        //extend footprint
        footprint_extender_.getIntermediateFootprint(footprint_);
        //update PointCloud
        updatePointCloud();
        ROS_INFO("Service Finished Correctly");
        resp.success = true;

        //get Yaw from input quaternion
        double collision_yaw = tf::getYaw(req.collision_orientation);
        ROS_INFO_STREAM("Measured orientation " << collision_yaw);
        resp.is_static_collision = false;

        tf::TransformListener tf_listener;
        tf_listener.waitForTransform(footprint_extender_.goal_frame_, footprint_extender_.base_frame_, ros::Time(0), ros::Duration(1));


        //Iterator initialization
        std::vector<std::pair<double,double> >::iterator it = footprint_extender_.footprint_extended_vector_.begin(); //init iterator
        std::vector<double>::iterator cost_it = footprint_costs_.begin(); //init footprint_costs_ iterator

        //iterate on footprint and costs
        for ( ; it != footprint_extender_.footprint_extended_vector_.end(); ++it, ++cost_it){

          geometry_msgs::PoseStamped pose_in, pose_out;
          pose_in.header.frame_id = footprint_extender_.base_frame_;
          pose_in.pose.position.x = it->first;
          pose_in.pose.position.y = it->second;
          pose_in.pose.orientation.w = 1;

          //transform pose on footprint to base_footprint
          tf_listener.transformPose (footprint_extender_.goal_frame_, ros::Time(0), pose_in, footprint_extender_.base_frame_, pose_out);

          //Angle on local frame of footprint
          ROS_DEBUG_STREAM("ANGLE " << atan2( pose_out.pose.position.y, pose_out.pose.position.x));

          //If diff of provided yaw and footprint quaternion is lees than the threshold then the point of collision is found
          if (fabs(collision_yaw - atan2( pose_out.pose.position.y, pose_out.pose.position.x)) < threshold){
            geometry_msgs::PointStamped msg;
            msg.header.frame_id =  footprint_extender_.base_frame_;

            /*If point of collision is added dirrectly over the footprint the robot might be stuck
            * A common resolution on costamp is 0.05 cm per pixel... a modified point of collision will be set on the next pixel
            * This is done taken advantage on the local coordinates
            * TODO Not magic number it can be taken from the costmap + an error
            */

            //Front
            if (it->first > 0){
                msg.point.x = it->first + distance_to_obstacle_;
            }
            else{//back
                msg.point.x = it->first - distance_to_obstacle_;
            }

            if (it->second > 0){//right
                msg.point.y = it->second + distance_to_obstacle_;
            }
            else{//left
                msg.point.y = it->second - distance_to_obstacle_;
            }

            ROS_WARN_STREAM("Cost of Collision "<< *cost_it);
            //Add poitn to collision
            collision_point_pub_.publish(msg);
            if (*cost_it < static_collision_threshold_){
              //If cost of the selected footprint point is less than a threshold then the collision happens again an obstacle on the map
              resp.is_static_collision = true;
            }

          }
        }
        //return collisions_poses
        resp.potential_collisions = collided_poses_array_;
        return true;
    }
    else{
        ROS_WARN("Subscribing Topics Missing Not Received");
        return false;
    }
}

void BaseCollisionChecker::footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg){
    geometry_msgs::PolygonStamped tmp = *msg;
    footprint_ = tmp.polygon;
    is_footprint_received = true;
    ROS_INFO_ONCE("FootprintCB Received");
}

void BaseCollisionChecker::pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    point_cloud_ = *msg;
    is_point_cloud_received_ = true;
    ROS_INFO_ONCE("PointCloud Received");
}


void BaseCollisionChecker::updatePointCloud(){

    //clear store arrays
    collided_poses_.clear();
    footprint_costs_.clear();

    std::mutex mtx;           // mutex for critical section
    mtx.lock();

    //needed for add colors to the pointcloud //Approach not used
    //sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_);
    //pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    //sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_, "x");
    //sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_, "y");
    //sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_, "z");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_, "r");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_, "g");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_, "b");

    //cost_cloud provides a field named total_cost
    sensor_msgs::PointCloud2Iterator<float> iter_tc(point_cloud_, "total_cost");
    //conversion for native pcl approach
    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(point_cloud_,pcl_point_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_point_cloud,*temp_cloud);
    //end conversion

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (temp_cloud);

    int K = 5;

    //iterates on the extended footprint
    for (std::vector<std::pair<double,double> >::iterator it = footprint_extender_.footprint_extended_vector_.begin() ;
              it != footprint_extender_.footprint_extended_vector_.end(); ++it){
        //Search
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = it->first;
        searchPoint.y = it->second;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        double partial_cost = 0.0;

        //search for the nearest points of the footprin
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
              //sum cost of the neighbour
              partial_cost += *(iter_tc + pointIdxNKNSearch[i]);
              temp_cloud->points[ pointIdxNKNSearch[i] ].r = 255;
            }

            //mean_cost
            partial_cost /= pointIdxNKNSearch.size();
            ROS_DEBUG_STREAM("costs " << partial_cost);
            //if(partial_cost<min_cost){
            footprint_costs_.push_back(partial_cost);

            //TODO a probability approach can be develope on this point taken current speed against cost on the costmap
            if(partial_cost>= collision_threshold_){ //potential_collision found if the collision threshold is less
              ROS_DEBUG_STREAM("Potential Collision Found in " << searchPoint.x << " , " << searchPoint.y);
              geometry_msgs::Pose tmp_pose;
              tmp_pose.position.x = searchPoint.x;
              tmp_pose.position.y = searchPoint.y;
              tmp_pose.orientation.w = 1.0;
              collided_poses_.push_back(tmp_pose);

              for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){//set color to visualize clusters
                temp_cloud->points[ pointIdxNKNSearch[i] ].b = 255;
                temp_cloud->points[ pointIdxNKNSearch[i] ].g = 255;
              }
            }
        }
        // End search
    }
    transformAndPublishPoints();
    pcl::toROSMsg(*temp_cloud, point_cloud_);
    point_cloud_pub_.publish(point_cloud_);
    mtx.unlock();
}

void BaseCollisionChecker::transformAndPublishPoints(){

  collided_poses_array_.poses.clear();
  collided_poses_array_.header.frame_id = footprint_extender_.goal_frame_;

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  tf_listener.waitForTransform(footprint_extender_.goal_frame_, footprint_extender_.base_frame_, ros::Time(0), ros::Duration(1));

  for (std::vector<geometry_msgs::Pose>::iterator it = collided_poses_.begin() ; it != collided_poses_.end(); ++it){
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header.frame_id = footprint_extender_.base_frame_;
    pose_in.pose = *it;
    tf_listener.transformPose (footprint_extender_.goal_frame_, ros::Time(0), pose_in, footprint_extender_.base_frame_, pose_out);

    tf::Quaternion quat = tf::createQuaternionFromYaw(atan2(pose_out.pose.position.y,pose_out.pose.position.x));
    tf::quaternionTFToMsg(quat,pose_out.pose.orientation);
    collided_poses_array_.poses.push_back(pose_out.pose);
    ROS_DEBUG_STREAM("Collision in base_footprint " << pose_out);
  }

  orientations_pub_.publish(collided_poses_array_);//publish orientations
}

#include "static_obstacles_recovery/static_obstacles_recovery.h"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(static_obstacles_recovery, StaticObstaclesCollisionRecovery,
                        static_obstacles_recovery::StaticObstaclesCollisionRecovery,
                        fault_core::FaultRecoveryBehavior)

using namespace fault_core;
namespace static_obstacles_recovery
{

  StaticObstaclesCollisionRecovery::StaticObstaclesCollisionRecovery(): amcl_pose_(), is_pose_received_(false), threshold_(4.0), max_iterations_(100)
  {
    fault_cause_ = FaultTopology::STATIC_OBSTACLE;
    ros::NodeHandle n;
    global_client_ = n.serviceClient<std_srvs::Empty>("/global_localization");
    amcl_client_ = n.serviceClient<std_srvs::Empty>("/request_nomotion_update");
    clear_costmaps_client_ = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    amcl_sub_ = n.subscribe("/amcl_pose", 10, &StaticObstaclesCollisionRecovery::amclCB,this);

    n.getParam("/mislocalization_threshold", threshold_);
    ROS_INFO("Constructor StaticObstaclesCollisionRecovery");
    ros::spinOnce(); // the missing call
  }

  void StaticObstaclesCollisionRecovery::amclCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
    amcl_pose_ = *msg;
    is_pose_received_ = true;
  }

  StaticObstaclesCollisionRecovery::~StaticObstaclesCollisionRecovery()
  {

  }

  void StaticObstaclesCollisionRecovery::initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
  {

  }

  bool StaticObstaclesCollisionRecovery::runFaultBehavior()
  {
    if (!is_pose_received_){
      ROS_ERROR_ONCE("Localization not received");
      return false;
    }

    std_srvs::Empty s;


    //clear_costmaps
    if (ros::service::waitForService ("/move_base/clear_costmaps", 100)) {
      if(!clear_costmaps_client_.call(s)) {
        ROS_ERROR ("Error clearing costmap service");
	      return false;
      }
    }

    //reset Localtization
    if (ros::service::waitForService ("/global_localization", 100)) {
      if(!global_client_.call(s)) {
        ROS_ERROR ("Error calling service");
	      return false;
      }

      int step = 0;
      bool ready = false;      //Force update of the particle filter
      ros::service::waitForService ("/request_nomotion_update", 100);

      while((!ready) && step < max_iterations_){ //TODO
        if(!amcl_client_.call(s)){
          ROS_ERROR("Resample Error");
          return false;
        }
        step++;

        ros::Duration(0.1).sleep();
        ROS_INFO_STREAM(ready);
        ready = fabs(amcl_pose_.pose.covariance[0]) < threshold_ &&
                fabs(amcl_pose_.pose.covariance[7]) < threshold_ &&
                fabs(amcl_pose_.pose.covariance[35]) < threshold_;
      }
      if (step == max_iterations_){
        ROS_ERROR("Max Number of Iterations Reached");
      }
    }
    return true;
  }

}  // namespace static_obstacles_recovery
#include "dynamic_obstacles_recovery/dynamic_obstacles_recovery.h"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(dynamic_obstacles_recovery, DynamicObstaclesCollisionRecovery,
                        dynamic_obstacles_recovery::DynamicObstaclesCollisionRecovery,
                        fault_core::FaultRecoveryBehavior)

using namespace fault_core;
namespace dynamic_obstacles_recovery
{

  DynamicObstaclesCollisionRecovery::DynamicObstaclesCollisionRecovery()
  {
    fault_cause_ = FaultTopology::DYNAMIC_OBSTACLE;
    ros::NodeHandle n;
    clear_costmaps_client_ = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    recovery_srv_client_ = n.serviceClient<mcr_recovery_behaviors::ForceFieldMsg>("force_field_service");
    ROS_INFO("Constructor DynamicObstaclesCollisionRecovery");
    ros::spinOnce(); // the missing call
  }

  DynamicObstaclesCollisionRecovery::~DynamicObstaclesCollisionRecovery()
  {

  }

  void DynamicObstaclesCollisionRecovery::initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
  {

  }

  bool DynamicObstaclesCollisionRecovery::runFaultBehavior()
  {

    std_srvs::Empty s;
    mcr_recovery_behaviors::ForceFieldMsg recovery_srv;

    ROS_INFO("Running Dynamic Recovery");
    //recover from collision force_field_service
    if(recovery_srv_client_.call(recovery_srv)){
      ROS_INFO("FORCE FIELD RECOVERY SUCCESS");
    }
    else{
      ROS_WARN("FORCE FIELD RECOVERY FAILED");
    }

    //clear costmap
    if (ros::service::waitForService ("/move_base/clear_costmaps", 10)) {
      if(!clear_costmaps_client_.call(s)) {
        ROS_ERROR ("Error clearing costmap service");
	      return false;
      }
    }
    return true;
  }

}  // namespace static_obstacles_recovery

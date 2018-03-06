#include "static_obstacles_recovery/static_obstacles_recovery.h"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(static_obstacles_recovery, StaticObstaclesCollisionRecovery,
                        static_obstacles_recovery::StaticObstaclesCollisionRecovery,
                        fault_core::FaultRecoveryBehavior)

using namespace fault_core;
namespace static_obstacles_recovery
{

  StaticObstaclesCollisionRecovery::StaticObstaclesCollisionRecovery()
  {
    fault_cause_ = FaultTopology::STATIC_OBSTACLE;
    ros::NodeHandle n;
    clear_costmaps_client_ = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    ROS_INFO("Constructor StaticObstaclesCollisionRecovery");
    ros::spinOnce(); // the missing call
  }


  StaticObstaclesCollisionRecovery::~StaticObstaclesCollisionRecovery()
  {

  }

  void StaticObstaclesCollisionRecovery::initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
  {

  }

  bool StaticObstaclesCollisionRecovery::runFaultBehavior()
  {
    ROS_INFO("Runningg Static Collision Recovery");

    std_srvs::Empty s;
    //clear_costmaps
    /*
    if (ros::service::waitForService ("/move_base/clear_costmaps", 10)) {
      if(!clear_costmaps_client_.call(s)) {
        ROS_ERROR ("Error clearing costmap service");
	      return false;
      }
    }*/
    return true;
  }

}  // namespace static_obstacles_recovery

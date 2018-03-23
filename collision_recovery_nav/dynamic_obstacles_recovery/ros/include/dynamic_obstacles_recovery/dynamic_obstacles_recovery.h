#ifndef DYNAMIC_OBSTACLES_RECOVERY_H
#define DYNAMIC_OBSTACLES_RECOVERY_H

#include <math.h>
#include <ros/ros.h>
#include <fault_core/fault_recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <mcr_recovery_behaviors/ForceFieldMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace dynamic_obstacles_recovery
{

  class DynamicObstaclesCollisionRecovery : public fault_core::FaultRecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      DynamicObstaclesCollisionRecovery();

      /**
       * @brief Destructor
       */
      ~DynamicObstaclesCollisionRecovery();

      /**
       * @brief Initializes plugin
       */
      void initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      bool runFaultBehavior();


    private:
      //It is useful to clear costmap
      ros::ServiceClient clear_costmaps_client_;
      //force_field_service
      ros::ServiceClient recovery_srv_client_;


  };

}  // namespace mislocalization_collision_recovery

#endif  // STATIC_OBSTACLES_RECOVERY_H

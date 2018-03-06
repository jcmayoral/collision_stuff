#ifndef STATIC_OBSTACLES_RECOVERY_H
#define STATIC_OBSTACLES_RECOVERY_H

#include <math.h>
#include <ros/ros.h>
#include <fault_core/fault_recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace static_obstacles_recovery
{

  class StaticObstaclesCollisionRecovery : public fault_core::FaultRecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      StaticObstaclesCollisionRecovery();

      /**
       * @brief Destructor
       */
      ~StaticObstaclesCollisionRecovery();

      /**
       * @brief Initializes plugin
       */
      void initialize(costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      bool runFaultBehavior();


    private:
      ros::ServiceClient clear_costmaps_client_;
  
  };

}  // namespace mislocalization_collision_recovery

#endif  // STATIC_OBSTACLES_RECOVERY_H

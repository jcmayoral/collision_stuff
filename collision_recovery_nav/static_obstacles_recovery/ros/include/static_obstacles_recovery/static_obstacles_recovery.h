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
      void amclCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);


    private:
      geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
      ros::ServiceClient global_client_;
      ros::Subscriber amcl_sub_;
      ros::ServiceClient amcl_client_;
      ros::ServiceClient clear_costmaps_client_;
      bool is_pose_received_;
      double threshold_;
      double max_iterations_;

  };

}  // namespace mislocalization_collision_recovery

#endif  // STATIC_OBSTACLES_RECOVERY_H
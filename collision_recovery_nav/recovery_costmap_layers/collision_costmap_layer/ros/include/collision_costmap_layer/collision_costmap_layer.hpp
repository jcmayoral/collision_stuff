#ifndef COLLISION_COSTMAP_LAYER_H
#define COLLISION_COSTMAP_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace collision_costmap_layer
{

class CollisionCostmapLayer : public costmap_2d::Layer
{
public:
  CollisionCostmapLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void pointCB(geometry_msgs::PointStamped msg);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  std::vector<double> mark_x_;
  std::vector<double> mark_y_;
  bool is_pose_received_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  ros::Subscriber point_sub_;
  tf::TransformListener listener_;


};
}
#endif

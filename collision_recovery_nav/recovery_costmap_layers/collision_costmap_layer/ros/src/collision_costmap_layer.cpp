#include<collision_costmap_layer/collision_costmap_layer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(collision_costmap_layer::CollisionCostmapLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace collision_costmap_layer
{

CollisionCostmapLayer::CollisionCostmapLayer(): is_pose_received_(false), mark_x_(), mark_y_() {}

void CollisionCostmapLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  ros::NodeHandle private_n("~");

  current_ = true;
  point_sub_ = private_n.subscribe("/base_collision_checker/collision_contact_point", 10, &CollisionCostmapLayer::pointCB,this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CollisionCostmapLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void CollisionCostmapLayer::pointCB(geometry_msgs::PointStamped msg){
  ROS_INFO_STREAM("Point in " << msg.header.frame_id << " Received");

  geometry_msgs::PointStamped map_point;

  listener_.waitForTransform("/base_link", "/map", ros::Time::now(), ros::Duration(10.0));

  try{
       listener_.transformPoint("map", msg, map_point);

   }
   catch(tf::TransformException& ex){
       ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
   }

  mark_x_.push_back(map_point.point.x);
  mark_y_.push_back(map_point.point.y);

  is_pose_received_ = true;
}


void CollisionCostmapLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CollisionCostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (!is_pose_received_)
    return;

  //mark_x_ = robot_x + cos(robot_yaw);//puntos a cambiar
  //mark_y_ = robot_y + sin(robot_yaw);
  //ROS_INFO_STREAM("pose " << robot_x << robot_y << robot_yaw);
  ROS_INFO("Update Bounds");
  *min_x = std::min(*min_x, *std::min_element(mark_x_.begin(),mark_x_.end()));
  *min_y = std::min(*min_y, *std::min_element(mark_y_.begin(),mark_y_.end()));
  *max_x = std::max(*max_x, *std::max_element(mark_x_.begin(),mark_x_.end()));
  *max_y = std::max(*max_y, *std::max_element(mark_y_.begin(),mark_y_.end()));
}

void CollisionCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  if (!is_pose_received_)
    return;

  unsigned int mx;
  unsigned int my;

  for (int i = 0; i< mark_x_.size(); i++){
    if(master_grid.worldToMap(mark_x_.at(i), mark_y_.at(i), mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }

  //is_pose_received_ = false;

}

} // end namespace

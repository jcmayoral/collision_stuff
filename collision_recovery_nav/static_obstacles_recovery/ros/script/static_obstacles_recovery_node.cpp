#include "static_obstacles_recovery/static_obstacles_recovery.h"
#include <ros/ros.h>

using namespace static_obstacles_recovery;

int main(int argc,char** argv){

  ros::init(argc, argv, "static_obstacles_recovery_node");

  StaticObstaclesCollisionRecovery* strategy = new StaticObstaclesCollisionRecovery();

  while(ros::ok()){

    if(strategy->runFaultBehavior()){
      ROS_INFO("DONE");
      return 1;
    }
    ros::spinOnce(); // the missing call
    //diagnoser_.isolateFault();
  }

  return 1;
}

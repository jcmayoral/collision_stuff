#include "simple_collision_detector/simple_collision_detector.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(simple_collision_detector, SimpleCollisionDetector,
                        simple_collision_detector::SimpleCollisionDetector,
                        fault_core::FaultDetector)

using namespace fault_core;
using namespace message_filters;
namespace simple_collision_detector
{

  void SimpleCollisionDetector::instantiateServices(ros::NodeHandle nh){


  }

  SimpleCollisionDetector::SimpleCollisionDetector(): isCollisionDetected(false)
  {
    //Define Fault Type as Unknown
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    //Define Fault Cause as Unknown
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor SimpleCollisionDetector");
  }


  SimpleCollisionDetector::~SimpleCollisionDetector()
  {

  }

  fault_core::FaultTopology SimpleCollisionDetector::getFault()
  {
     return fault_;
  }


  // This callback receives topics from all observers and triggers the isolation if any of them return collision
  void SimpleCollisionDetector::mainCallBack(const fusion_msgs::sensorFusionMsg msg){
    ROS_DEBUG_STREAM("Message received " << msg.window_size);
    if (msg.msg == fusion_msgs::sensorFusionMsg::ERROR){
      isCollisionDetected = true;
    }
    else{
      isCollisionDetected = false;
    }
  }

  //This function is called on the navigation_manager, register n number of subscribers
  void SimpleCollisionDetector::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
    for (int i = 0; i< sensor_number;i++){
      ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &SimpleCollisionDetector::mainCallBack, this);
      array_subcribers_.push_back(sub);
    }

  }

  bool SimpleCollisionDetector::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    if (isCollisionDetected){
      isolateFault();
    }
    return isCollisionDetected;
  }

  void SimpleCollisionDetector::isolateFault(){
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void SimpleCollisionDetector::diagnoseFault(){
    fault_.cause_ = FaultTopology::MISLOCALIZATION; // By default run MisLocalization Recovery Strategy
    fault_.type_ = FaultTopology::COLLISION; // Classify the fault as a Collision
    ROS_ERROR_ONCE("Collision FOUND");
  }
}  // namespace simple_collision_detector

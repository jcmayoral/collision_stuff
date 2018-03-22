#ifndef SIMPLE_COLLISION_DETECTOR_H
#define SIMPLE_COLLISION_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>

namespace simple_collision_detector
{

  class SimpleCollisionDetector : public fault_core::FaultDetector
  {
    public:

      /**
       * @brief Constructor
       */
      SimpleCollisionDetector();

      /**
       * @brief Destructor
       */
      ~SimpleCollisionDetector();


      void instantiateServices(ros::NodeHandle nh);

      /**
       * @brief Initializes plugin
       * @param number of sensors
       */
      void initialize(int sensor_number);

      /**
       * @brief Executes the detection of teh fault
       */
      bool detectFault();

      /**
       * @brief Executes the detection of teh fault
       */
      void isolateFault();

      /**
       * @brief   diagnose Fault
       */
      void diagnoseFault();

      /**
       * @brief   detect Fault
       */
      fault_core::FaultTopology getFault();

      /**
       * @brief default CallBack for any collision observer
       */
      void mainCallBack(const fusion_msgs::sensorFusionMsg msg);

    private:

      /**
       * @brief stores n number of collision observers subcribers
       */

      std::vector<ros::Subscriber> array_subcribers_;

      /*
      * @brief isCollisionDetected is the flag used by navigation_manager to trigger isolation and recovery procedures
      */
      bool isCollisionDetected;
  };

}  // namespace simple_collision_detector

#endif  // SIMPLE_COLLISION_DETECTOR_H

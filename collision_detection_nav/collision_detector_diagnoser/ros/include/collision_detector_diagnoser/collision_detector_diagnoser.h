#ifndef COLLISION_DETECTOR_DIAGNOSER_H
#define COLLISION_DETECTOR_DIAGNOSER_H

#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <fusion_msgs/monitorStatusMsg.h>

#include <kinetic_energy_monitor/KineticEnergyMonitorMsg.h>
#include <footprint_checker/CollisionCheckerMsg.h>

#include <dynamic_reconfigure/server.h>
#include <collision_detector_diagnoser/diagnoserConfig.h>
#include <collision_detector_diagnoser/sync_policies.h>
#include <collision_detector_diagnoser/sensor_fusion_methods.hpp>
#include <list>
#include <chrono>
#include <collision_detector_diagnoser/custom_filter.hpp>
#include <math.h>

namespace collision_detector_diagnoser
{

  class CollisionDetectorDiagnoser : public fault_core::FaultDetector, public CustomMessageFilter
  {
    public:

      /**
       * @brief Function Thread from CustomMessageFilter
       */
      void listenTime();

      /**
       * @brief Function Thread from CustomMessageFilter
       */
      void timeoutReset();

      /**
       * @brief Constructor used  by the navigation_manager
       */
      CollisionDetectorDiagnoser();

      /**
       * @brief Constructor used  by the collision_detector_diagnoser node
       */
      CollisionDetectorDiagnoser(int sensor_number);

      /**
       * @brief Destructor
       */
      ~CollisionDetectorDiagnoser();

      /*
      * @brief Function used for Dynamic Recofigure set up on navigation_manager
      */
      void instantiateServices(ros::NodeHandle nh);

      /**
       * @brief Initializes plugin
       * @param number of sensors
       */
      void initialize(int sensor_number);

      /**
       * @brief Executes the detection of the fault
       */
      bool detectFault();

      /**
       * @brief Executes the isolation of the fault
       */
      void isolateFault();

      /**
       * @brief   diagnose Fault
       */
      void diagnoseFault();

      /**
       * @brief   getter for FaultTopology
       */
      fault_core::FaultTopology getFault();

      /*
      * @brief Message Filter CallBack for Two Sensors
      */
      void twoSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                              const fusion_msgs::sensorFusionMsgConstPtr& detector_2);

      /*
      * @brief Message Filter CallBack for Three Sensors
      */
      void threeSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3);

      /*
      * @brief Message Filter CallBack for Four Sensors
      */
      void fourSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_4);

      /*
      * @brief Message Filter CallBack for Five Sensors
      */
      void fiveSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_4,
                                const fusion_msgs::sensorFusionMsgConstPtr& detector_5);
      /*
      * @brief Message General CallBack for n Collision Observers
      */
      void simpleCallBack(const fusion_msgs::sensorFusionMsg msg);

      /*
      * @brief Dynamic Reconfigure CallBack Function
      */
      void dyn_reconfigureCB(collision_detector_diagnoser::diagnoserConfig &config, uint32_t level);

      /*
      * @brief Sensor Fusion Mode Selector
      */
      void selectMode();

      /*
      * @brief Plot Collision Orientations coming from sensors
      */
      void plotOrientation(list<fusion_msgs::sensorFusionMsg> v);

    private:

      /*
      * @brief Unregister CB for Syncronization
      */
      void unregisterCallbackForSyncronizers();

      /*
      * @brief Register CB for Syncronization
      */
      void registerCallbackForSyncronizers(int sensor_number);

      /*
      * @brief Unregister CB for Non Filter Observers
      */
      void setUnfilteredSubscribers(int sensor_number, ros::NodeHandle nh);

      /*
      * @brief Create array of subscribers for Filtered Messages
      */
      void setFilteredSubscribers(int sensor_number, ros::NodeHandle nh);

      /*
      * @brief Clear Subscribers to Filtered Observers
      */
      void resetFilteredSubscribers();

      /*
      * @brief Clear Subscribers to Unfiltered Observers
      */
      void resetUnFilteredSubscribers();

      /*
      * @brief Link Dynamic Reconfigure Server with CB
      */
      void setDynamicReconfigureServer();


      // Vector of unfiltered subscribers
      std::vector<ros::Subscriber> array_subscribers_;

      // Vector of filtered subscribers
      std::vector<message_filters::Subscriber<fusion_msgs::sensorFusionMsg>*> filtered_subscribers_;

      // Collision Flag
      bool isCollisionDetected;

      // Time of Collision
      std_msgs::Header time_of_collision_;

      // Publisher for Sound Feedback
      ros::Publisher speak_pub_;

      // Publisher for Visualization Purposes
      ros::Publisher collision_pub_;

      // Diagnostic message Publisher
      fusion_msgs::monitorStatusMsg status_output_msg_;

      // Policies for Syncronization of Messagers
      message_filters::Synchronizer<MySyncPolicy2>*syncronizer_for_two_;
      message_filters::Synchronizer<MySyncPolicy3>*syncronizer_for_three_;
      message_filters::Synchronizer<MySyncPolicy4>*syncronizer_for_four_;
      message_filters::Synchronizer<MySyncPolicy5>*syncronizer_for_five_;


      // Handler of Syncronization
      message_filters::Connection main_connection;

      // Services Clients
      ros::ServiceClient strength_srv_client_;
      ros::ServiceClient orientations_srv_client_;

      //Dynamic Reconfigure
      dynamic_reconfigure::Server<collision_detector_diagnoser::diagnoserConfig>* dyn_server;
      dynamic_reconfigure::Server<collision_detector_diagnoser::diagnoserConfig>::CallbackType dyn_server_cb;

      // SF mode selected
      int mode_;

      // Filter/Unfilter Selected
      bool filter_;

      // Number of Sources
      int sensor_number_;

      // Collision Threshold
      double percentage_threshold_;


      //SF methods Classes (sensor_fusion_methods.hpp)
      SensorFusionApproach default_approach_;
      VotingApproach voting_approach_;
      WeightedApproach weighted_approach_;

      // Parent SF Method used for overwriting on the run
      SensorFusionApproach* fusion_approach_;

      //Collision Orientations from sensors
      ros::Publisher orientation_pub_;

      // Filter Params
      int queue_size_;
      double age_penalty_;
      double max_interval_;

      // Filter through Custom Filter
      bool is_custom_filter_requested_;

      // Allow publishing feedback
      bool debug_mode_;


      // Stores mean collision orientation from observers
      tf::Quaternion mean_collision_orientation_;

      // Publish just one collision_messages
      bool is_collision_published_;

  };

}  // namespace collision_detector_diagnoser

#endif  // COLLISION_DETECTOR_DIAGNOSER_H

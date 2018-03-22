#include <list>
#include <ros/ros.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <std_msgs/Float32.h>

/*
* This file provides several sensor fusion approaches
* for collsion detection including voting, weigthed_voting
*/

using namespace std;


// parent class
class SensorFusionApproach {
  public:
    // detect function must be overwritten
    virtual bool detect(list<fusion_msgs::sensorFusionMsg> v) {
      ROS_DEBUG("Default");
      collision_observers_ids_.clear();
      //Look on all sensors
      for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
        if(it->msg == fusion_msgs::sensorFusionMsg::ERROR){ // If any is detecting collision return True
          ROS_WARN_STREAM("Collision DETECTED on " << it->sensor_id);
          collision_observers_ids_.push_back(it->sensor_id.data);
          return true;
        }
      }
      return false;
    };

    /*
    * @brief setter of threshold
    */
    void setThreshold(double thr){
      ROS_DEBUG("Threshold update");
      threshold = thr;
    }

    /*
    * @brief getter of collsion observers
    */

    const vector<string>& getCollisionObservers() const{
      return collision_observers_ids_;
    };

  protected:
    //threshold
    double threshold = 0.5;
    //observers which detect collision
    std::vector<string> collision_observers_ids_;
};

class VotingApproach : public SensorFusionApproach {
    public:
      //detect function is overwritted
      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        ROS_DEBUG("Consensus");
        int counter = 0; // init conter
        collision_observers_ids_.clear();
        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          if(it->msg == fusion_msgs::sensorFusionMsg::ERROR){ //sum one to counter if collision is detected
            collision_observers_ids_.push_back(it->sensor_id.data);
            ROS_WARN_STREAM("Collision DETECTED on " << it->sensor_id);
            counter ++;
          }
        }
        if (counter >= v.size()*threshold){ // if counter > percentage then the voting is acceepted
          ROS_ERROR_STREAM("Voting Collision Detected");
          ROS_DEBUG_STREAM("Counter " << counter);
          return true;
        }

        return false; //otherwise return fall
      };
};

class WeightedApproach : public SensorFusionApproach {
    public:

      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        ROS_DEBUG("Weighted");
        double max_value = 0; //TODO
        double count = 0;
        collision_observers_ids_.clear();

        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          max_value+= fusion_msgs::sensorFusionMsg::ERROR * it->weight; //weight * status
          collision_observers_ids_.push_back(it->sensor_id.data);
          count += it->msg * it->weight;
        }

        ROS_DEBUG_STREAM("Count " << count);
        if ((count/max_value) >= threshold){ // check if the counte is enough to detect collision
          ROS_ERROR_STREAM("Weighted Collision Detected: " << count << " Of " << max_value << " Percentage:" << (count/max_value));
          return true;

        }

        return false;
      };
};

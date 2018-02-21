#include <ros/ros.h>
#include <thread>

namespace collision_detector_diagnoser
{
  class CustomMessageFilter   {
    public:
    CustomMessageFilter(){

    };
    ~CustomMessageFilter(){

    };

    void stop(){
      delete [] collision_flags_;
    }

    void reset(){
      stop();
      start(input_number_);
    }

    virtual void listenTime(){

    }

    virtual void timeoutReset(){

    }

    void start(int observers_number){
       collision_flags_ = new bool[observers_number];
       registerCallback(observers_number);
       monitoring_thread_ = new std::thread(&CustomMessageFilter::listenTime,this);
       monitoring_thread_->detach();                // pauses until first finishes
       timeout_reset_thread_ = new std::thread(&CustomMessageFilter::timeoutReset,this);
       timeout_reset_thread_->detach();
       input_number_ = observers_number;
    }

    void setTimeOut(double new_timeout){
      timeout_ = new_timeout;
    }

    int getTimeOut(){
      return timeout_;
    }

    int getInputNumber(){
      return input_number_;
    }

    void setCustomThreshold(double new_threshold){
      custom_threshold_ = new_threshold;
    }

    double getCustomThrehold(){
      return custom_threshold_;
    }

    bool getCollisionFlag(int index){
      return collision_flags_[index];
    }

    void registerCallback(int input_number){
      for (int i=0; i < input_number; ++i){
        my_subscriber_ = nh_.subscribe<fusion_msgs::sensorFusionMsg> ("/collisions_" +std::to_string(i), 10,boost::bind(&CustomMessageFilter::subscribeCB,this, _1, i));
        }

    }

    void subscribeCB(const fusion_msgs::sensorFusionMsgConstPtr& detector, int index){
      ROS_INFO("Custom CB");
      if (detector->msg == 2){
        collision_flags_[index] = true;
      }
    }

  private:
    int timeout_;
    int input_number_;
    ros::Subscriber my_subscriber_;
    ros::NodeHandle nh_;
    bool *collision_flags_;
    std::thread *monitoring_thread_;
    std::thread *timeout_reset_thread_;
    double custom_threshold_;
  };
};//end namespace

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
      int input_number = (sizeof(collision_flags_)/sizeof(*collision_flags_));
      stop();
      start(input_number);
    }

    virtual void listenTime(){
      std::cout<<"original listen";

    }

    void start(int observers_number){
       collision_flags_ = new bool[observers_number];
       registerCallback(observers_number);
       monitoring_thread_ = new std::thread(&CustomMessageFilter::listenTime,this);
       monitoring_thread_->join();                // pauses until first finishes
    }

    void setTimeOut(double new_timeout){
      timeout_ = new_timeout;
    }

    double getTimeOut(){
      return timeout_;
    }

    void registerCallback(int input_number){
      for (int i=0; i != input_number; ++i){
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
    double timeout_;
    ros::Subscriber my_subscriber_;
    ros::NodeHandle nh_;
    bool *collision_flags_;
    std::thread *monitoring_thread_;
  };
};//end namespace

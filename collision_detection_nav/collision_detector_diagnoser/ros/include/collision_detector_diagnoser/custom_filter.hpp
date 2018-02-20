#include <ros/ros.h>


class CustomMessageFilter{
  CustomMessageFilter();
  ~CustomMessageFilter();

  void stop(){
    delete [] collision_flags_;
  }

  void start(int observers_number){
     collision_flags_ = new bool[observers_number];
     registerCallback(observers_number);
  }

  void setTimeOut(double new_timeout){
    timeout_ = new_timeout;
  }

  double getTimeOut(){
    return timeout_;
  }

  void registerCallback(int input_number){

    for (int i=0; i != input_number; ++i){
      ros::Subscriber tmp_subscribe;  //blabla
      syncronizer_ = new boost::bind(&subscribe,sub, i);
    }

  }

  void subscribe(const fusion_msgs::sensorFusionMsgConstPtr& detectors, int index){
  }

private:
  double timeout_;
  bool *collision_flags_;
  boost::function<fusion_msgs::sensorFusionMsg, int> syncronizer_;
}

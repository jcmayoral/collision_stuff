#include <ros/ros.h>


class CustomMessageFilter{
  CustomMessageFilter();
  ~CustomMessageFilter();
public:

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
      ros::Subscriber tmp_subscribe = nh_.subscribe<fusion_msgs::sensorFusionMsg> ("/collisions" +std::to_string(i), 10,boost::bind(&CustomMessageFilter::subscribeCB,this, _1, i));
    }

  }

  void subscribeCB(const fusion_msgs::sensorFusionMsgConstPtr& detector, int index){
  }

private:
  double timeout_;
  ros::NodeHandle nh_;
  bool *collision_flags_;
};

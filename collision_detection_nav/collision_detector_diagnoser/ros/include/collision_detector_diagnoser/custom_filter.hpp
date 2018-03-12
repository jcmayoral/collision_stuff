#include <ros/ros.h>
#include <thread>
#include <mutex>

namespace collision_detector_diagnoser
{
  class CustomMessageFilter   {
    public:
    CustomMessageFilter(): input_number_(0), my_subscribers_(),custom_collision_observers_ids_(){
      ROS_INFO("CustomMessageFilter Constructor");
    };
    ~CustomMessageFilter(){

    };

    void stop(){
      removeSubscribers();
      resetCollisionFlags();
      //delete [] collision_flags_;
    }

    void removeSubscribers(){
      for (int i = 0; i< my_subscribers_.size();i++){//remove normal subscribers
        my_subscribers_[i].shutdown();
      }//endFor
      my_subscribers_.clear();
    }

    void reset(){
      stop();
      start(input_number_);
    }

    virtual void listenTime(){

    }

    virtual void timeoutReset(){

    }

    void resetCollisionFlags(){
      //std::cout << "RCF";

      for (int b = 0; b < input_number_;++b){
        if (collision_flags_[b]){
            ROS_WARN_STREAM("Collision Found in topic number " << b);
        }
        collision_flags_[b] = false;
        flags_ids_[b] = false;
      }
    }

    void start(int observers_number){
      input_number_ = observers_number;
      collision_flags_ = new bool[observers_number];
      //init flags
      for( int i = 0; i < input_number_; ++i ){
        collision_flags_[i] = false;
      }

      flags_ids_ = new bool[observers_number];

      resetCollisionFlags();
      registerCallback(observers_number);
      monitoring_thread_ = new std::thread(&CustomMessageFilter::listenTime,this);
      monitoring_thread_->detach();                // pauses until first finishes
      timeout_reset_thread_ = new std::thread(&CustomMessageFilter::timeoutReset,this);
      timeout_reset_thread_->detach();
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
        ros::Subscriber sub = nh_.subscribe<fusion_msgs::sensorFusionMsg> ("/collisions_" +std::to_string(i), 10,boost::bind(&CustomMessageFilter::subscribeCB,this, _1, i));
        my_subscribers_.push_back(sub);
        }

    }

    void subscribeCB(const fusion_msgs::sensorFusionMsgConstPtr& detector, int index){
      //mutex mu;
      //mu.lock();
      if (detector->msg == 2){
        collision_flags_[index] = true;

        if (!flags_ids_[index]){
          custom_collision_observers_ids_.push_back(string(detector->sensor_id.data));
          flags_ids_[index] = true;
        }
      }
      //mu.unlock();
    }

    void clearCustomCollisionObserversIDS(){
      ROS_DEBUG_STREAM("Deleting " << custom_collision_observers_ids_.size());
      custom_collision_observers_ids_.clear();
    }

    const vector<string>& getCustomCollisionObservers() const{
      return custom_collision_observers_ids_;
    };

  private:
    int timeout_;
    int input_number_;
    std::vector<ros::Subscriber> my_subscribers_;
    std::vector<string> custom_collision_observers_ids_;

    ros::NodeHandle nh_;
    bool *collision_flags_;
    bool *flags_ids_;

    std::thread *monitoring_thread_;
    std::thread *timeout_reset_thread_;
    double custom_threshold_;
  };
};//end namespace

#include <ros/ros.h>
#include <thread>
#include <mutex>

/*
* This class provide a Simple Message Filter based on time.
* It needs a time window, the number of observers and the collsion threshold
* Note Flags_ids_ array is just used for
*/

namespace collision_detector_diagnoser
{
  class CustomMessageFilter   {
    public:
    CustomMessageFilter(): input_number_(0), my_subscribers_(),custom_collision_observers_ids_(){
      ROS_INFO("CustomMessageFilter Constructor");
    };
    ~CustomMessageFilter(){

    };


    /*
    * @brief stop the matching
    */
    void stop(){
      //unregister subscribers cb
      removeSubscribers();
      //reset flags
      resetCollisionFlags();
    }

    void removeSubscribers(){
      for (int i = 0; i< my_subscribers_.size();i++){
        //shutdown subscriber
        my_subscribers_[i].shutdown();
      }//endFor
      my_subscribers_.clear();
    }

    void reset(){
      stop(); //Stop Filter
      start(input_number_); // Start Filter
    }

    virtual void listenTime(){ // Monitors Collisions over the observers

    }

    virtual void timeoutReset(){ //reset Flags when timeout is reached

    }

    void resetCollisionFlags(){
      for (int b = 0; b < input_number_;++b){
        if (collision_flags_[b]){ //Plot All Detections during a time period
            ROS_WARN_STREAM("Collision Found in topic number " << b);
        }
        collision_flags_[b] = false; // clear Flag of b observer
        flags_ids_[b] = false; //clear flag_id of b observer
      }
    }

    void start(int observers_number){
      //stores number of observers
      input_number_ = observers_number;
      //init flags array
      collision_flags_ = new bool[observers_number];
      //init all flags in false
      for( int i = 0; i < input_number_; ++i ){
        collision_flags_[i] = false;
      }

      //init flags_ids_
      flags_ids_ = new bool[observers_number];

      //clear CollisionFlags
      resetCollisionFlags();
      //registerCallbacks
      registerCallback(observers_number);

      //Init monitor Thread
      monitoring_thread_ = new std::thread(&CustomMessageFilter::listenTime,this);
      monitoring_thread_->detach(); // run on back
      //Init Timeout reset thread
      timeout_reset_thread_ = new std::thread(&CustomMessageFilter::timeoutReset,this);
      timeout_reset_thread_->detach(); //run on back
    }

    /*
    * @brief setter of timeout
    */
    void setTimeOut(double new_timeout){
      timeout_ = new_timeout;
    }

    /*
    * @brief getter of timeout
    */
    int getTimeOut(){
      return timeout_;
    }

    /*
    * @brief setter of number of Observers
    */
    int getInputNumber(){
      return input_number_;
    }

    /*
    * @brief setter of collision threshold
    */
    void setCustomThreshold(double new_threshold){
      custom_threshold_ = new_threshold;
    }

    /*
    * @brief getter of collision threshold
    */
    double getCustomThrehold(){
      return custom_threshold_;
    }

    /*
    * @brief get flag status of index source
    */
    bool getCollisionFlag(int index){
      return collision_flags_[index];
    }

    /*
    * @brief registerCallback function for input_number of observers
    */
    void registerCallback(int input_number){
      for (int i=0; i < input_number; ++i){
        ros::Subscriber sub = nh_.subscribe<fusion_msgs::sensorFusionMsg> ("/collisions_" +std::to_string(i), 10,boost::bind(&CustomMessageFilter::subscribeCB,this, _1, i));
        my_subscribers_.push_back(sub); // store subscriber
        }

    }

    /*
    * @brief proposed callback for all observers
    */
    void subscribeCB(const fusion_msgs::sensorFusionMsgConstPtr& detector, int index){
      if (detector->msg == 2){ // If colliison set flag
        collision_flags_[index] = true;

        if (!flags_ids_[index]){ // if collision does not happen before then add sensor_id
          custom_collision_observers_ids_.push_back(string(detector->sensor_id.data)); //store sensor_id
          flags_ids_[index] = true; //flags_id collision already detected on this time period
        }
      }
    }

    /*
    * @brief clear vector od obserser ids
    */
    void clearCustomCollisionObserversIDS(){
      ROS_DEBUG_STREAM("Deleting " << custom_collision_observers_ids_.size());
      custom_collision_observers_ids_.clear();
    }

    /*
    * @brief get ids of observers which detected collision
    */
    const vector<string>& getCustomCollisionObservers() const{
      return custom_collision_observers_ids_;
    };

  protected:
    //must be protected accesing directly from collision_detector_diagnoser
    bool *collision_flags_;

  private:
    //Window of time on milliseconds
    int timeout_;
    //Number of observers
    int input_number_;

    //vector of subscribers
    std::vector<ros::Subscriber> my_subscribers_;
    //vector of sensor_ids
    std::vector<string> custom_collision_observers_ids_;

    ros::NodeHandle nh_;
    //array of flags_ids
    bool *flags_ids_;

    //thread declarations
    std::thread *monitoring_thread_;
    std::thread *timeout_reset_thread_;

    //threshold
    double custom_threshold_;
  };
};//end namespace

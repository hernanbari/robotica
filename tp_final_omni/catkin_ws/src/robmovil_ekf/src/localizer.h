#ifndef __ROBMOVIL_EKF_LOCALIZER_H__
#define __ROBMOVIL_EKF_LOCALIZER_H__

#include <ros/ros.h>
#include <robmovil_msgs/LandmarkArray.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>

#include "localizer_ekf.h"

namespace robmovil_ekf {
  class Localizer
  {
    public:
      Localizer(ros::NodeHandle& n);

      void on_landmark_array(const robmovil_msgs::LandmarkArrayConstPtr& msg);
      // void on_imu(const sensor_msgs::ImuConstPtr& msg);
      void on_odometry(const nav_msgs::OdometryConstPtr& msg);

      void on_posts_array(const geometry_msgs::PoseArrayConstPtr& msg);

    private:
      LocalizerEKF ekf;
      // Dejo esta var para que imprima la primera vez las posiciones de los posts
      bool set_map;
      bool only_prediction;
      
      //ros::Subscriber landmark_sub, imu_sub, odo_sub;
      ros::Subscriber landmark_sub, odo_sub, posts_sub;
      ros::Publisher pose_pub;
      std::string base_frame_, map_frame_, laser_frame_;
      
      ros::Timer prediction_timer;
      tf2_ros::TransformBroadcaster transform_broadcaster_;  


      tf::TransformListener listener2;
      tf::StampedTransform map_transform;

      
      void prediction_event(const ros::TimerEvent& event);
      void advance_time(const ros::Time& now);
      void publish_estimate(const ros::Time& now);
      
      void prediction(const ros::Time& time);
      
      /* ultimo comando de control recibido */
      LocalizerEKF::Vector u;
      ros::Time t;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZER_H__

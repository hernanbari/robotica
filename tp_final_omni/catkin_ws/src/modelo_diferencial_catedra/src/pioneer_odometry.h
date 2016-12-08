#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robmovil_msgs/MultiEncoderTicks.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace robmovil
{

class PioneerOdometry
{
  public:

    PioneerOdometry(ros::NodeHandle& nh);

    void on_velocity_cmd(const geometry_msgs::Twist& twist);

    void on_encoder_ticks(const robmovil_msgs::MultiEncoderTicks& encoder);

  private:

    ros::NodeHandle& nh_;

    ros::Subscriber twist_sub_, encoder_sub_;

    ros::Publisher vel_pub_front_left_, vel_pub_front_right_, vel_pub_rear_left_, vel_pub_rear_right_, pub_odometry_;

  // Acá pueden agregar las variables de instancia que necesiten
  // ...

    double x_, y_, theta_;

    bool ticks_initialized_;
    int32_t last_ticks_front_left_, last_ticks_front_right_, last_ticks_rear_left_, last_ticks_rear_right_;
    ros::Time last_ticks_time;

    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
};

}

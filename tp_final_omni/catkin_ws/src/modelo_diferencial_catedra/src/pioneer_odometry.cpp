#include "pioneer_odometry.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace robmovil;

// CHECKEAR UNIDADES DE CONSTANTES
#define WHEEL_RADIUS 0.050
#define ENCODER_TICKS 500.0
#define LX 0.175
#define LY 0.175



inline double wrapAngle( double angle )
{
  double twoPi = 2.0 * M_PI;
  return angle - twoPi * floor( angle / twoPi );
}

PioneerOdometry::PioneerOdometry(ros::NodeHandle& nh)
  : nh_(nh), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = nh.subscribe("/robot/cmd_vel", 1, &PioneerOdometry::on_velocity_cmd, this);

  vel_pub_front_left_ = nh.advertise<std_msgs::Float64>("/robot/front_left_wheel/cmd_vel", 1);
  vel_pub_front_right_ = nh.advertise<std_msgs::Float64>("/robot/front_right_wheel/cmd_vel", 1);
  vel_pub_rear_left_ = nh.advertise<std_msgs::Float64>("/robot/rear_left_wheel/cmd_vel", 1);
  vel_pub_rear_right_ = nh.advertise<std_msgs::Float64>("/robot/rear_right_wheel/cmd_vel", 1);

  encoder_sub_ = nh.subscribe("/robot/encoders", 1, &PioneerOdometry::on_encoder_ticks, this);

  pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/robot/odometry", 1);

  tf_broadcaster = boost::make_shared<tf::TransformBroadcaster>();
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::Twist& twist)
{
  double xLinearVel = twist.linear.x;
  double yLinearVel = twist.linear.y;
  double angularVel = twist.angular.z;
  
  double vFrontLeft 	= (1/WHEEL_RADIUS) * (xLinearVel-yLinearVel-(LX+LY)*angularVel);
  double vFrontRight 	= (1/WHEEL_RADIUS) * (xLinearVel+yLinearVel+(LX+LY)*angularVel);
  double vRearLeft 		= (1/WHEEL_RADIUS) * (xLinearVel+yLinearVel-(LX+LY)*angularVel);
  double vRearRight 	= (1/WHEEL_RADIUS) * (xLinearVel-yLinearVel+(LX+LY)*angularVel);

  // publish front left velocity
  {
    std_msgs::Float64 msg;
    msg.data = vFrontLeft;

    vel_pub_front_left_.publish( msg );
  }

  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vFrontRight

    vel_pub_front_right_.publish( msg );
  }

  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRearLeft;

    vel_pub_rear_left_.publish( msg );
  }
  
  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRearRight;

    vel_pub_rear_right_.publish( msg );
  }
  
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::EncoderTicks& encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (not ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_time = encoder.header.stamp;
    
    last_ticks_front_left_ 	= encoder.ticks[0];
    last_ticks_front_right_ = encoder.ticks[1];
    last_ticks_rear_left_ 	= encoder.ticks[2];
    last_ticks_rear_right_ 	= encoder.ticks[3];
    return;
  }

  int32_t delta_ticks_front_left 	= encoder.ticks_front_left.data - last_ticks_front_left_;
  int32_t delta_ticks_front_right = encoder.ticks_front_right.data - last_ticks_front_right_;
  int32_t delta_ticks_rear_left 	= encoder.ticks_rear_left.data - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right 	= encoder.ticks_rear_right.data - last_ticks_rear_right_;

  // calculo el desplazamiento relativo

  double delta_front_left 	= M_PI * 2 * WHEEL_RADIUS * delta_ticks_front_left / ENCODER_TICKS;
  double delta_front_right 	= M_PI * 2 * WHEEL_RADIUS * delta_ticks_front_right / ENCODER_TICKS;
	double delta_rear_left 		= M_PI * 2 * WHEEL_RADIUS * delta_ticks_rear_left / ENCODER_TICKS;
  double delta_rear_right 	= M_PI * 2 * WHEEL_RADIUS * delta_ticks_rear_right / ENCODER_TICKS;

  double delta_x = (delta_front_left+delta_front_right+delta_rear_left+delta_rear_left) * WHEEL_RADIUS/4;
  double delta_y = (-delta_front_left+delta_front_right+delta_rear_left-delta_rear_left) * WHEEL_RADIUS/4;
  double delta_theta = (-delta_front_left+delta_front_right-delta_rear_left+delta_rear_left) * WHEEL_RADIUS/(4*(LX+LY));

  // double delta_theta 		= (delta_right - delta_left) / WHEEL_BASELINE;
  // double delta_distance = (delta_left + delta_right) / 2;

  // double delta_x = delta_distance * cos( theta_ );
  // double delta_y = delta_distance * sin( theta_ );

  // double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  // actualizo el estado local

  // ROS_DEBUG_STREAM("theta " << theta_);
  // ROS_DEBUG_STREAM("delta theta " << delta_theta);

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // normalizo el angulo
  //theta_ = wrapAngle( theta_ );

  // ROS_DEBUG_STREAM("theta " << theta_ << std::endl);

  // Armo el mensaje de odometría

  nav_msgs::Odometry msg;

  msg.header.stamp = encoder.header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
  
  //msg.pose.covariance = ...
	
	double delta_t = (encoder.header.stamp - last_ticks_time).toSec();
  
  // NO ESTOY SEGURO SI ESTO ESTA BIEN
  msg.twist.twist.linear.x = delta_x / delta_t;
  msg.twist.twist.linear.y = delta_y / delta_t;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = delta_theta / delta_t;

  //msg.twist.covariance = ...

  pub_odometry_.publish( msg );

  // Actualizo las variables de estado

  last_ticks_front_left_ = encoder.ticks[0];
	last_ticks_front_right_ = encoder.ticks[1];
	last_ticks_rear_left_ = encoder.ticks[2];
	last_ticks_rear_right_ = encoder.ticks[3];
	last_ticks_time = encoder.header.stamp;


  /* Mando tambien un transform usando TF */
  tf::Transform t;
  tf::poseMsgToTF(msg.pose.pose, t);
  tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));

}

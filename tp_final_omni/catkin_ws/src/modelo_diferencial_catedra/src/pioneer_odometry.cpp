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

  ROS_DEBUG_STREAM("xLinearVel: " << xLinearVel << "    yLinearVel: " << yLinearVel << "    angularVel: " << angularVel << std::endl);

  
  double vFrontLeft 	= (1/WHEEL_RADIUS) * (xLinearVel - yLinearVel - (LX+LY) * angularVel);
  double vFrontRight 	= (1/WHEEL_RADIUS) * (xLinearVel + yLinearVel + (LX+LY) * angularVel);
  double vRearLeft 		= (1/WHEEL_RADIUS) * (xLinearVel + yLinearVel - (LX+LY) * angularVel);
  double vRearRight 	= (1/WHEEL_RADIUS) * (xLinearVel - yLinearVel + (LX+LY) * angularVel);

  // publish front left velocity
  ROS_DEBUG_STREAM("vFrontLeft: " << vFrontLeft << "    vFrontRight: " << vFrontRight << "    vRearLeft: " << vRearLeft << "    vRearRight: " << vFrontRight << std::endl);
  
  {
    std_msgs::Float64 msg;
    msg.data = vFrontLeft;

    vel_pub_front_left_.publish( msg );
  }

  // publish front right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vFrontRight;

    vel_pub_front_right_.publish( msg );
  }

  // publish rear left velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRearLeft;

    vel_pub_rear_left_.publish( msg );
  }
  
  // publish rear right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRearRight;

    vel_pub_rear_right_.publish( msg );
  }
  
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::MultiEncoderTicks& encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (not ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_time = encoder.header.stamp;
    
    last_ticks_front_left_ 	= encoder.ticks[0].data;
    last_ticks_front_right_ = encoder.ticks[1].data;
    last_ticks_rear_left_ 	= encoder.ticks[2].data;
    last_ticks_rear_right_ 	= encoder.ticks[3].data;
    return;
  }

  int32_t delta_ticks_front_left 	= encoder.ticks[0].data - last_ticks_front_left_;
  int32_t delta_ticks_front_right = encoder.ticks[1].data - last_ticks_front_right_;
  int32_t delta_ticks_rear_left 	= encoder.ticks[2].data - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right 	= encoder.ticks[3].data - last_ticks_rear_right_;

  // CHECKEAR FORMULAS, SI PONES VEL_X = 0.1, ODOMETRY DA 0.13, VEL_ANG = 0.3, ODOM = 0.4
  // PUEDE SER QUE SEA UN ERROR ACEPTABLE/ESPERABLE O TAL VEA HAY ALGO MAL
  // NUNCA USO QUE EL ENCODER DA 500 VUELTAS

  double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  double delta_front_left 	= delta_ticks_front_left  * (2*M_PI/ENCODER_TICKS) / delta_t;
  double delta_front_right 	= delta_ticks_front_right * (2*M_PI/ENCODER_TICKS) / delta_t;
	double delta_rear_left 		= delta_ticks_rear_left   * (2*M_PI/ENCODER_TICKS) / delta_t;
  double delta_rear_right 	= delta_ticks_rear_right  * (2*M_PI/ENCODER_TICKS) / delta_t;

  
  double vel_x      = ( delta_front_left + delta_front_right + delta_rear_left + delta_rear_right) * WHEEL_RADIUS/4;
  double vel_y      = (-delta_front_left + delta_front_right + delta_rear_left - delta_rear_right) * WHEEL_RADIUS/4;
  double vel_theta  = (-delta_front_left + delta_front_right - delta_rear_left + delta_rear_right) * WHEEL_RADIUS/(4*(LX+LY));

  
  // double vel_dir = atan2(vel_y, vel_x);
  // double vel_norm = pow(pow(vel_x,2) + pow(vel_y,2), 0.5);

  // CHECKEAR TODO ESTO DE ABAJO, LAS FORMULAS DE ARRIBA ESTAN BIEN; EL TEMA ES CÖMO SE USAN
  // calculo el desplazamiento relativo
  // double vel_distance = vel_norm*delta_t;

  // double delta_theta 		= (delta_right - delta_left) / WHEEL_BASELINE;
  // double vel_distance = (delta_left + delta_right) / 2;

  // ESTA FORMULA LA HICE QUEMADO; PARECE FUNCIONAR PERO NI IDEA POR QUE
  // MAS QUE NADA LA SUMA DE LOS ANGULOS, FUE MAGIA PROBANDO,
  // AL PPIO ERA COS (ALGUNO DE LOS ANGULOS), DESPUES COS * COS
  // double delta_x = vel_distance * cos( vel_dir + theta_);
  // double delta_y = vel_distance * sin( vel_dir + theta_);

  // double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  double vel_x_rot = cos(theta_) * vel_x - sin(theta_) * vel_y;
  double vel_y_rot = sin(theta_) * vel_x  + cos(theta_) * vel_y;

  // actualizo el estado local

  // ROS_DEBUG_STREAM("theta " << theta_);
  // ROS_DEBUG_STREAM("delta theta " << delta_theta);

  x_      += vel_x_rot * delta_t;
  y_      += vel_y_rot * delta_t;
  theta_  += vel_theta * delta_t;

  // normalizo el angulo
  //theta_ = wrapAngle( theta_ );

  // ROS_INFO_STREAM("x: " << x_ << "    y: " << y_ << "    theta: " << theta_ << std::endl);
  // ROS_INFO_STREAM("delta_x: " << delta_x << "    delta_y: " << delta_y << std::endl);

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
	
  // NO ESTOY SEGURO SI ESTO ESTA BIEN
  msg.twist.twist.linear.x = vel_x;
  msg.twist.twist.linear.y = vel_y;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = vel_theta;

  //msg.twist.covariance = ...

  ROS_DEBUG_STREAM("vel_x: " << msg.twist.twist.linear.x << "    vel_y: " << msg.twist.twist.linear.y << "    vel_theta: " << msg.twist.twist.angular.z << std::endl);

  pub_odometry_.publish( msg );

  // Actualizo las variables de estado

  last_ticks_front_left_ = encoder.ticks[0].data;
	last_ticks_front_right_ = encoder.ticks[1].data;
	last_ticks_rear_left_ = encoder.ticks[2].data;
	last_ticks_rear_right_ = encoder.ticks[3].data;
	last_ticks_time = encoder.header.stamp;


  /* Mando tambien un transform usando TF */
  tf::Transform t;
  tf::poseMsgToTF(msg.pose.pose, t);
  tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));

}

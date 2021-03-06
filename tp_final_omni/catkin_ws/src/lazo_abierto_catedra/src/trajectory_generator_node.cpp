#include <ros/ros.h>
#include <robmovil_msgs/Trajectory.h>
#include <robmovil_msgs/TrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

void build_sin_trajectory(double, double, double, double, robmovil_msgs::Trajectory&, nav_msgs::Path&);
void build_box_trajectory(double, double, robmovil_msgs::Trajectory&, nav_msgs::Path&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Publisher trajectory_publisher = nh.advertise<robmovil_msgs::Trajectory>("/robot/trajectory", 1, true);
  
  // Path descripto en poses para visualizacion en RViz
  ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/ground_truth/target_path", 1, true);

  robmovil_msgs::Trajectory trajectory_msg;
  nav_msgs::Path path_msg;

  trajectory_msg.header.seq = 0;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "odom";

  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";
  
  double stepping;
  double total_time;
  double amplitude;
  double cycles;
  
  nhp.param<double>("stepping", stepping, 0.1);
  nhp.param<double>("total_time", total_time, 20); // 20: da masomenos, 50: lo sigue muy bien el pioneer
  nhp.param<double>("amplitude", amplitude, 1);
  nhp.param<double>("cycles", cycles, 1);
  
  //build_sin_trajectory(stepping, total_time, amplitude, cycles, trajectory_msg, path_msg);

  build_box_trajectory(stepping, total_time, trajectory_msg, path_msg);
  trajectory_publisher.publish( trajectory_msg );
  path_publisher.publish( path_msg );

  ros::spin();

  return 0;
}

void build_box_trajectory(double stepping, double total_time, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg)
{
  // Estaria bueno esto de hacerlo en funcion del tiempo, pero paja
  /*
  for (double t = 0; t <= total_time; t = t + stepping)
  {
  
  }
  */

  //double x[] = {	  0,    0,    0,  0, -0.5, -1, -1.5, -2,   -2, -2,   -2, -2, -1.5, -1.0, -0.5, 0}; //-2.5, -3, 0};
  //double y[] = {   -0.5, -1.0, -1.5, -2,   -2, -2,   -2, -2, -1.5, -1, -0.5,  0,    0,    0,    0, 0}; //, -2.5, 0, 0};
  /*double x[] = { 0,  0,  0,  0,
  				-1, -2, -3, -4, 
  				-4, -4, -4, -4,
  				-3, -2, -1,  0};
  double y[] = {-1, -2, -3, -4,
  				-4, -4, -4, -4,
  				-3, -2, -1,  0,
  				 0,  0,  0,  0,};
*/
  // Cómo hago para que siempre este orientado hacia afuera?
  // Supuse que con angulos negativos deberia andar
  double x[] = {0, -4, -4, 0};
  double y[] = {-4, -4, 0, 0};
  double a[] = {-M_PI*0.5, -M_PI, -M_PI*1.5, 0}; //-90.0, -180.0, -270.0, -0.0};
  
  /*
  double x[] = {0, 0, 0, 0,
                -1, -2, -3, -4,
                -4, -4, -4, -4,
                -3, -2, -1, 0};
  double y[] = {-1, -2, -3, -4, 
                -4, -4, -4, -4, 
                -3, -2, -1, 0, 
                0, 0, 0, 0};

  double a[] = {-M_PI*0.125, -M_PI*0.250, -M_PI*0.375, -M_PI*0.5,
                -M_PI*0.625, -M_PI*0.750, -M_PI*0.875, -M_PI,  
                -M_PI*1.125, -M_PI*1.250, -M_PI*1.375, -M_PI*1.5,
                -M_PI*1.625, -M_PI*1.750, -M_PI*1.875, 0};
*/

  /*double a[] = {-M_PI/4, -M_PI/2, 
                -M_PI*0.75, -M_PI, 
                -M_PI*1.25, -M_PI*3/2, 
                -M_PI*1.75, 0}; //-45.0, -90.0, -135.0, -180.0, -225.0, -270.0, -315.0, -0.0};
  */

  /*double a[] = {-M_PI/ 8,  -M_PI/ 4,  -M_PI*(3/8),  -M_PI/2 ,
  				-M_PI/2 * (5/4),  -M_PI/2 * (6/4),  -M_PI/2 * (7/4),  -M_PI/2 * (8/4),
  				-M_PI/2 * (9/4),  -M_PI/2 * (10/4), -M_PI/2 * (11/4), -M_PI/2 * (12/4), 
  				-M_PI/2 * (13/4), -M_PI/2 * (14/4), -M_PI/2 * (15/4),        -0}; //-90.0, -180.0, -270.0, -0.0};
	*/
  /*
  // Ni idea estas velocidades, tire random
  double vx = 0.5;
  double vy = 0.5;
  double va = 1.0;
  */

  //for (int t = 0; t <= 15; t = t + 1)
  for (int t = 0; t <= 3; t = t + 1)
  {

    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.time_from_start = ros::Duration( 0 );

    point_msg.transform.translation.x = x[int(t)];
    point_msg.transform.translation.y = y[int(t)];
    point_msg.transform.translation.z = 0;

    point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( a[int(t)] );

    point_msg.velocity.linear.x = 0; //vx;
    point_msg.velocity.linear.y = 0; //vy;
    point_msg.velocity.linear.z = 0;

    point_msg.velocity.angular.x = 0;
    point_msg.velocity.angular.y = 0;
    point_msg.velocity.angular.z = 0; //va;

    trajectory_msg.points.push_back( point_msg );
    
    geometry_msgs::PoseStamped stamped_pose_msg;
    
    stamped_pose_msg.header.stamp = path_msg.header.stamp;
    stamped_pose_msg.header.frame_id = path_msg.header.frame_id;
    
    stamped_pose_msg.pose.position.x = x[int(t)];
    stamped_pose_msg.pose.position.y = y[int(t)];
    stamped_pose_msg.pose.position.z = 0;
    
    stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw( a[int(t)] );
    
    path_msg.poses.push_back(stamped_pose_msg);
  }
}

void build_sin_trajectory(double stepping, double total_time, double amplitude, double cycles, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg)
{
  // atan2(vy(0), vx(0)) = orientacion inicial
  double initial_orientation = atan2( amplitude * (cycles * 2*M_PI * 1/total_time), cycles * 2*M_PI * 1/total_time );
  
  for (double t = 0; t <= total_time; t = t + stepping)
  {
    // X se extiende lo suficiente para dar varias vueltas en el tiempo determinado
    double x = cycles * 2*M_PI * t * 1/total_time;
    // Y funcion seno con determinada amplitud
    double y = amplitude * sin( x );

    // derivadas primeras
    double vx = cycles * 2*M_PI * 1/total_time;
    double vy = amplitude * cos(x) * vx;

    // derivadas segundas
    double vvx = 0;
    double vvy = amplitude * (-sin(x) * vx * vx + cos(x) * vvx);
    
    /* dado que la funcion esta construida pensada con Y "hacia arriba", X "hacia derecha" 
     * y la orientacion inicial puede no ser 0 -> entonces aplicamos una rotacion de manera de 
     * alinear la primera orientacion al eje X del robot y todo vector direccion de manera acorde */
    double x_rot = cos(-initial_orientation) * x + -sin(-initial_orientation) * y;
    double y_rot = sin(-initial_orientation) * x + cos(-initial_orientation) * y;
    double vx_rot = cos(-initial_orientation) * vx + -sin(-initial_orientation) * vy;
    double vy_rot = sin(-initial_orientation) * vx + cos(-initial_orientation) * vy;
    double vvx_rot = cos(-initial_orientation) * vvx + -sin(-initial_orientation) * vvy;
    double vvy_rot = sin(-initial_orientation) * vvx + cos(-initial_orientation) * vvy;
    
    x = x_rot; y = y_rot; vx = vx_rot; vy = vy_rot; vvx = vvx_rot; vvy = vvy_rot;
    
    // calculo del angulo en cada momento y la derivada del angulo
    double a = atan2( vy, vx );
    double va = (vvy*vx-vvx*vy)/(vx*vx+vy*vy);

    // se crean los waypoints de la trajectoria
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.time_from_start = ros::Duration( t );

    point_msg.transform.translation.x = x;
    point_msg.transform.translation.y = y;
    point_msg.transform.translation.z = 0;

    point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( a );

    point_msg.velocity.linear.x = vx;
    point_msg.velocity.linear.y = vy;
    point_msg.velocity.linear.z = 0;

    point_msg.velocity.angular.x = 0;
    point_msg.velocity.angular.y = 0;
    point_msg.velocity.angular.z = va;

    trajectory_msg.points.push_back( point_msg );
    
    geometry_msgs::PoseStamped stamped_pose_msg;
    
    stamped_pose_msg.header.stamp = path_msg.header.stamp;
    stamped_pose_msg.header.frame_id = path_msg.header.frame_id;
    
    stamped_pose_msg.pose.position.x = x;
    stamped_pose_msg.pose.position.y = y;
    stamped_pose_msg.pose.position.z = 0;
    
    stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(a);
    
    path_msg.poses.push_back(stamped_pose_msg);
  }
}

#include <fstream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

std::string formatTime(const boost::posix_time::ptime& time, const char* format)
{
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format( format );

  std::stringstream stream;
  stream.str("");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << time;

  return stream.str();
}

std::string timestamp()
{
  return formatTime(boost::posix_time::second_clock::local_time(), "%Y-%m-%d_%H:%M:%S");
}

class Logger
{
  public:

    Logger(ros::NodeHandle& nh);

  private:

    ros::Subscriber robot_pose_sub_, ground_truth_sub_, goal_poses_sub_, ekf_pose_sub_, goal_vel_sub_, ground_truth_vel_sub_;

    std::ofstream robot_logfile_, ground_truth_logfile_, goal_poses_logfile_, ekf_poses_logfile_, goal_vel_logfile_, ground_truth_vel_logfile_;

  // funciones auxiliares

    void handleRobotPose(const nav_msgs::Odometry& msg);

    void handleGroundTruthPose(const nav_msgs::Odometry& msg);

    void handleGoalPose(const geometry_msgs::PoseStamped& msg);

    void handleEkfPose(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void handleGoalVel(const geometry_msgs::TwistStamped& msg);

    void handleGroundTruthVel(const nav_msgs::Odometry& msg);
};

Logger::Logger(ros::NodeHandle& nh)
  : robot_logfile_( timestamp() + "_odom_poses.log" ), ground_truth_logfile_( timestamp() + "_ground-truth.log" ), goal_poses_logfile_( timestamp() + "_goals_poses.log" ),
  ekf_poses_logfile_( timestamp() + "_ekf_poses.log"), goal_vel_logfile_(timestamp() + "_goals_vel.log")
{
  robot_pose_sub_ = nh.subscribe("/robot/odometry", 1, &Logger::handleRobotPose, this);
  ground_truth_sub_ = nh.subscribe("/robot/ground_truth", 1, &Logger::handleGroundTruthPose, this);
  goal_poses_sub_ = nh.subscribe("/goal_pose", 1, &Logger::handleGoalPose, this);
  ekf_pose_sub_ = nh.subscribe("/localizer/pose", 1, &Logger::handleEkfPose, this);
  goal_vel_sub_ = nh.subscribe("/robot/cmd_vel", 1, &Logger::handleGoalVel, this);
  ground_truth_vel_sub_ = nh.subscribe("/robot/ground_truth", 1, &Logger::handleGroundTruthVel, this);
}

void Logger::handleRobotPose(const nav_msgs::Odometry& msg)
{
  ROS_INFO("LOG ODOM POSE");
  robot_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

void Logger::handleGroundTruthPose(const nav_msgs::Odometry& msg)
{

  ROS_INFO("LOG GT POSE");
  ground_truth_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

void Logger::handleGoalPose(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("LOG GOAL POSE");
  goal_poses_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.position.x << " " << msg.pose.position.y << " " << tf2::getYaw( msg.pose.orientation ) << std::endl;
}

void Logger::handleEkfPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  ROS_INFO("LOG EKF POSE");
 ekf_poses_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

void Logger::handleGoalVel(const geometry_msgs::TwistStamped& msg)
{
  ROS_INFO("LOG GOAL VELOCITIES");
 goal_vel_logfile_ << msg.header.stamp.toSec() << " " << msg.twist.linear.x << " " << msg.twist.linear.y << " " << msg.twist.angular.z << std::endl;
}

void Logger::handleGroundTruthVel(const nav_msgs::Odometry& msg)
{

  ROS_INFO("LOG GT VELOCITIES");
  ground_truth_vel_logfile_ << msg.header.stamp.toSec() << " " << msg.twist.twist.linear.x << " " << msg.twist.twist.linear.y << " " << msg.twist.twist.angular.z << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");
  ros::NodeHandle nh;

  Logger logger( nh );

  ROS_INFO("LOGGER LAUNCHED");

  ros::spin();

  return 0;
}

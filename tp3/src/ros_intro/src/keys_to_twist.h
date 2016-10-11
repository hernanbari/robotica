#pragma once

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <geometry_msgs/Twist.h>

namespace robmovil
{

class KeysToTwist
{
  public:

    KeysToTwist(ros::NodeHandle& nh);

    void on_key_up(const keyboard::Key& key_event);
    void on_key_down(const keyboard::Key& key_event);

  private:

    ros::NodeHandle& nh_;

    ros::Subscriber key_up_sub_, key_down_sub_;

    ros::Publisher twist_pub_;

  // Acá pueden agregar las variables de instancia que necesiten
  // ...
};

}

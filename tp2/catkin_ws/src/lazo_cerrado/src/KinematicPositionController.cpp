#include <angles/angles.h>
#include "KinematicPositionController.h"
#include "tf_utils.hpp"
#include "vector"
#include "math.h"

KinematicPositionController::KinematicPositionController(ros::NodeHandle& nh) :
  TrajectoryFollower(nh), transform_listener_( tfBuffer_ )
{
    expected_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
    
    ros::NodeHandle nhp("~");
    
    std::string goal_selection;
    nhp.param<std::string>("goal_selection", goal_selection, "FIXED_GOAL");
    nhp.param<double>("fixed_goal_x", fixed_goal_x_, 3);     
    nhp.param<double>("fixed_goal_y", fixed_goal_y_, 0);     
    nhp.param<double>("fixed_goal_a", fixed_goal_a_, -M_PI_2);     
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const ros::Time& t0, const ros::Time& t1, double y0, double y1, const ros::Time& t)
{
  return y0 + (t - t0).toSec() * (y1 - y0) / (t1 - t0).toSec();
}

bool KinematicPositionController::getCurrentPose(const ros::Time& t, double& x, double& y, double& a)
{
  tf2::Transform odom_to_robot;
  if (not lookupTransformSafe(tfBuffer_, "odom", "base_link", t, odom_to_robot))
    return false;

  x = odom_to_robot.getOrigin().getX();
  y = odom_to_robot.getOrigin().getY();

  a = tf2::getYaw(odom_to_robot.getRotation());

  return true;
}

/**
 * NOTA: Para un sistema estable mantener:
 * - 0 < K_RHO
 * - K_RHO < K_ALPHA
 * - K_BETA < 0
 */
#define K_RHO .15
#define K_ALPHA .15 
#define K_BETA -.30

bool KinematicPositionController::control(const ros::Time& t, double& v, double& w)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  /** EJERCICIO 1: COMPLETAR: Aqui deberan realizar las cuentas necesarias para determinar:
   *             - la velocidad lineal: asignando la variable v
   *             - la velocidad angular: asignando la variable w 
   *  
   *  RECORDAR: cambiar el marco de referencia en que se encuentran dx, dy y theta */
  
// cos -theta -sin -theta
// sin -theta cos -theta

  double dx = (goal_x*cos(-current_a)+goal_y*-sin(-current_a))-(current_x*cos(-current_a)+current_y*-sin(-current_a));
  double dy = (goal_x*sin(-current_a)+goal_y*cos(-current_a))-(current_x*sin(-current_a)+current_y*cos(-current_a));
  double theta = goal_a-current_a;
  
  // Computar variables del sistema de control
  double rho = pow(pow(dx,2)+pow(dy,2),0.5);
  double alpha = angles::normalize_angle(atan2(dy,dx)-theta); // Normalizes the angle to be -M_PI circle to +M_PI circle It takes and returns radians. 
  double beta =  angles::normalize_angle(-theta-alpha); // Realizar el calculo dentro del metodo de normalizacion

  /* Calcular velocidad lineal y angular* 
   * Existen constantes definidas al comienzo del archivo para
   * K_RHO, K_ALPHA, K_BETA */
  v = K_RHO*rho;
  w = K_ALPHA*alpha+K_BETA*beta;

  //ROS_INFO_STREAM("atan2: " << atan2(dy, dx) << " theta siegwart: " << theta << " expected_atheta: "  << current_a << " rho: " << rho << " alpha: " << alpha << " beta: " << beta << " v: " << v << " w: " << w);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

// Lookahead distance
#define LKAHD_DIST 1
std::vector<unsigned int> prev_wpoints;

bool KinematicPositionController::getPursuitBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::Trajectory& trajectory = getTrajectory();
  
  /** EJERCICIO 3:
   * Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
   * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */
  
  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */  

  if (trajectory.points.size() == 0){
    return false;
  }
  unsigned int nearest_wpoint = 0;
  double nearest_wpoint_x = -1;
  double nearest_wpoint_y = -1;
  double nearest_wpoint_a = -1;
  double nearest_dist = INT_MAX;
  
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);
    
    //...
  
    double dist_to_wpoint = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (find(prev_wpoints.begin(), prev_wpoints.end(),i) ==  prev_wpoints.end() && dist_to_wpoint < nearest_dist){
      nearest_wpoint = i;
      nearest_wpoint_x = wpoint_x;
      nearest_wpoint_y = wpoint_y;
      nearest_wpoint_a = wpoint_a;
      nearest_dist = dist_to_wpoint;
    }
  }

  if (nearest_dist > LKAHD_DIST || nearest_wpoint == trajectory.points.size()-1){
    x = nearest_wpoint_x;
    y = nearest_wpoint_y;
    a = nearest_wpoint_a;
  }else {
    
    prev_wpoints.push_back(nearest_wpoint);

    // const robmovil_msgs::TrajectoryPoint& near_wpoint = trajectory.points[nearest_wpoint];
    // double x0 = near_wpoint.transform.translation.x;
    // double y0 = near_wpoint.transform.translation.y;

    const robmovil_msgs::TrajectoryPoint& next_wpoint = trajectory.points[nearest_wpoint+1];
    double x1 = next_wpoint.transform.translation.x;
    double y1 = next_wpoint.transform.translation.y;
    double a1 = tf2::getYaw(next_wpoint.transform.rotation);
    
   //  // Recta entre dos puntos:
   // // m = (y1-y0)/(x1-x0)
   // // c = m*(-x0)+y0

   // // Interseccion circulo y recta:
   // // y = mx+c
   // // r^2 = (x-p)^2+(y-q)^2
   
   // // A = m^2+1
   // // B = 2(mc-mq-p)
   // // C = q^2-r^2+p^2-2cq+c^2

   // // x = (-B+-sqrt(B^2-4*A*C))/2*A
   // // y = m*x+c
  
   //  // Recta entre dos puntos:  
   //  double m = (y1-y0)/(x1-x0);
   //  double c = m*(-x0)+y0;
   //  double p = current_x;
   //  double q = current_y;


   //  // Interseccion circulo y recta:
   //  double A = pow(m,2)+1;
   //  double B = 2*(m*c-m*q-p);
   //  double C = pow(q,2)-pow(LKAHD_DIST,2)+pow(p,2)-2*c*q+pow(c,2);



   //  x = (-B+sqrt(pow(B,2)-4*A*C))/2*A;
   //  y = m*x+c;

   //  // La orientacion a seguir es la pendiente de la recta (positiva o negativa???)
   //  a = m;

    x = x1;
    y = y1;
    a = a1;
   

  }
  // ROS_INFO_STREAM("wpoints: " << trajectory);

  
  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return true;
}

bool KinematicPositionController::getTimeBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  ROS_INFO_STREAM("processing index: " << next_point_idx);

  const robmovil_msgs::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const ros::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const ros::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}

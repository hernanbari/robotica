#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>
#include <queue>

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner(ros::NodeHandle& nh)
: RRTPlanner(nh, 0, 0)
{ 
  nh.param<double>("goal_bias", goal_bias_, 0.6);
  int it_tmp;
  nh.param<int>("max_iterations", it_tmp, 20000);
  max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
  nh.param<double>("linear_velocity_stepping", Vx_step_, 0.05);
  nh.param<double>("angular_velocity_stepping", Wz_step_, 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
  /* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
  /* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{  
  
    /* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
     * 
     * ATENCION: - Tener encuenta el valor de la variable global goal_bias_ 
     *           - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
     *           - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
     *                grid_->info.width, grid_->info.height, grid_->info.resolution */  
    
    double chance = randBetween(0, 1);
    double close_area = 0;
    double close_area_distance = 1;
    double close_area_theta = 1;

    if(chance < goal_bias_){
        close_area = 1;
        close_area_distance = 0.1;
        close_area_theta = 0.25;
    }


    // PARA QUE SE DEBERIA USAR getOriginOfCell() ????


    double x =      randBetween(0, close_area_distance*grid_->info.height);
    double y =      randBetween(0, close_area_distance*grid_->info.width);
    double theta =  randBetween(-M_PI*close_area_theta,M_PI*close_area_theta);

    double x_orig;
    double y_orig;
    getOriginOfCell((uint)0,(uint)0, x_orig,y_orig);


    SpaceConfiguration rand( {x_orig + x + goal_config_.get(0)*close_area, y_orig + y + goal_config_.get(1)*close_area, angles::normalize_angle(theta+goal_config_.get(2)*close_area) } );

    return rand;
}

double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  /* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */
  
  double K_dist=1, K_ori=0.5;

  double dist_ori = abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) );
  double dist_euc = pow( pow(c1.get(0)-c2.get(0),2) + pow(c1.get(1)-c2.get(1),2) , 0.5);

  return K_dist*dist_euc  + K_ori*dist_ori;
}

double dist_eucl(double x0,double y0,double x1,double y1){
  return pow( pow(x0-x1,2)+pow(y0-y1,2), 0.5);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
  /* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
   * 
   * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
   *             generado hasta el momento como CLAVES DEL DICCIONARIO
   *           - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween() 
   *             y utilizar esa funcion como auxiliar */
  
  SpaceConfiguration nearest;
  double min_distance = std::numeric_limits<double>::max(); 
  

  // PREGUNTA: LOS VECINOS DE UN NODO SIN HIJOS LIBRES SON:
  //           - EL "PREVIO" 
  //           - LOS 3 ADONDE PUEDE LLEGAR 
  //           ????????
  //           => SI TIENE HIJOS LIBRES, EL VALUE DEL DICT TIENE QUE SER MENOR A 4 ó A < 3 ?????

  for(const auto& config : graph_ )
  {
    double d = distancesBetween(config.first, rand_config_);

    if(d < min_distance && config.second.size()<3){
        min_distance = d;
        nearest = config.first;
    }
  }

  return nearest;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{  
  /* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
   *            La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
   * 
   * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
   *           - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
   *           - Utilizar angles::normalize_angle() */
  
  SpaceConfiguration steer;
  
  /* Conjunto de steers ya ocupados en la configuracion near_config_ */
  const std::list<SpaceConfiguration> occupied_steerings = graph_[near_config_];
  std::vector<SpaceConfiguration> free_steerings; // NO LO USAMOS
  double min_dist = std::numeric_limits<double>::max();
  
  /* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */
  
  for(int i = -1; i<2; i++){

    double x_posible = near_config_.get(0) + Vx_step_*cos(angles::normalize_angle(near_config_.get(2)+ i*Wz_step_));
    double y_posible = near_config_.get(1) + Vx_step_*sin(angles::normalize_angle(near_config_.get(2)+ i*Wz_step_));
    double theta_posible = angles::normalize_angle(near_config_.get(2)+ i*Wz_step_);

    SpaceConfiguration s_posible({ x_posible, y_posible, theta_posible });

    double d = distancesBetween(s_posible, rand_config_);
    if(d < min_dist && find(occupied_steerings.begin(), occupied_steerings.end(), s_posible) == occupied_steerings.end()){
      min_dist = d;
      steer = s_posible;
    }

  }

  return steer;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
  /* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */

  for(int i=-1; i<2; i++){
    for(int j=-1; j<2; j++){
      double x = new_config_.get(0)+i*grid_->info.resolution;
      double y = new_config_.get(1)+j*grid_->info.resolution;
      if(isPositionOccupy(x,y))
        return false;
    }
  }  
  return true;
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
  
  /* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */

   double x0 = new_config_.get(0);
   double y0 = new_config_.get(1);
   double x1 = goal_config_.get(0);
   double y1 = goal_config_.get(1);
   
  if(dist_eucl(x0,y0,x1,y1) < 0.1 && abs( angles::shortest_angular_distance(new_config_.get(2), goal_config_.get(2)) ) < M_PI/2)
    return true;

  return false;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory,
                                                            const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                                            std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  std::vector<SpaceConfiguration> path;
  SpaceConfiguration current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }
  
  result_trajectory.header.stamp = ros::Time::now();
  result_trajectory.header.frame_id = "odom";
  
  ros::Duration t_from_start = ros::Duration(0);
  ros::Duration delta_t = ros::Duration(1);

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {    
    double config_x = it->get(0);
    double config_y = it->get(1);
    double config_theta = it->get(2);
    
    tf::Transform wp_odom_ref;
    wp_odom_ref.getOrigin().setX(config_x);
    wp_odom_ref.getOrigin().setY(config_y);
    wp_odom_ref.getOrigin().setZ(0);
    wp_odom_ref.setRotation(tf::createQuaternionFromYaw(config_theta));
    
    wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
    
    // Se crean los waypoints de la trayectoria
    robmovil_msgs::TrajectoryPoint point_msg;
    
    transformTFToMsg(wp_odom_ref, point_msg.transform);
    
    if(it != path.rend()-1) {
      double config_dx = (it+1)->get(0) - config_x;
      double config_dy = (it+1)->get(1) - config_y;
      double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
      point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
      point_msg.velocity.angular.z = config_dtheta;
    }else{
      point_msg.velocity.linear.x = 0;
      point_msg.velocity.angular.z = 0;
    }
    
    point_msg.time_from_start = t_from_start;
    
    result_trajectory.points.push_back( point_msg );
    
    t_from_start += delta_t;
  }
}

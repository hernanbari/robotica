#include "AStarPlanner.h"
#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>

#define COST_BETWEEN_CELLS 1

robmovil_planning::AStarPlanner::AStarPlanner(ros::NodeHandle& nh)
: GridPlanner(nh)
{ }

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.height)
   *             y aquellas celdas ocupadas */
  
  std::vector<Cell> neighbors;
  for(int i = -1; i<2; i++){
    for(int j = -1; j<2; j++){
      int n_i = i+c.i;
      int n_j = j+c.j;
      if (isCellOccupy(uint(n_i), uint(n_j)))
        return std::vector<Cell>();
      else if (n_i < 0 || n_j < 0 || uint(n_i) > grid_->info.width || uint(n_j) > grid_->info.height 
          || (uint(n_i)==c.i && uint(n_j) == c.j))
        continue;
      neighbors.push_back(Cell(uint(n_i), uint(n_j)));
    }
  }
  return neighbors;
}

double robmovil_planning::AStarPlanner::heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
{  
  /* COMPLETAR: Funcion de heuristica de costo */
  return abs(current.i-goal.i)+abs(current.j-goal.j);
}

bool robmovil_planning::AStarPlanner::do_planning(robmovil_msgs::Trajectory& result_trajectory)
{
  uint start_i, start_j;
  uint goal_i, goal_j;
  
  getCellOfPosition(starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), start_i, start_j);
  getCellOfPosition(goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), goal_i, goal_j);
  
  /* Celdas de inicio y destino */
  Cell start = Cell(start_i, start_j);
  Cell goal = Cell(goal_i, goal_j);
  
  /* Contenedores auxiliares recomendados para completar el algoritmo */
  std::priority_queue<CellWithPriority, std::vector<CellWithPriority>, PriorityCompare> frontier;
  std::map<Cell, Cell> came_from;
  std::map<Cell, double> cost_so_far;
  
  bool path_found = false;
  
  /* Inicializacion de los contenedores (start comienza con costo 0) */
  frontier.push(CellWithPriority(start, 0));
  cost_so_far[start] = 0;
  
  /* COMPLETAR: Utilizar los contenedores auxiliares para la implementacion del algoritmo A*
   * NOTA: Pueden utilizar las funciones neighbors(const Cell& c) y heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
   *       para la resolucion */
  
   while(!frontier.empty()){
    CellWithPriority current = frontier.top();
    if (current == goal){
      path_found = true;
      break;
    }
    frontier.pop();
    std::vector<robmovil_planning::AStarPlanner::Cell> neighbor_vector = neighbors(current);
    for(std::vector<robmovil_planning::AStarPlanner::Cell>::iterator it = neighbor_vector.begin(); it != neighbor_vector.end(); it++){
      if(cost_so_far.find(*it) != cost_so_far.end())
        continue;
      double cost = cost_so_far[current] + COST_BETWEEN_CELLS;
      // if(find(frontier.begin(), frontier.end(), *it) == frontier.end())
      //   frontier.push_back(*it);
      cost_so_far[*it] = cost;
      // if(cost >= cost_so_far[*it])
      //   continue;
      came_from[*it] = current;
      frontier.push(CellWithPriority(*it, cost+heuristic_cost(start, goal, *it)));
        
    }

   }


  if(not path_found)
    return false;
  
  /* Construccion y notificacion de la trajectoria.
   * NOTA: Se espera que came_from sea un diccionario representando un grafo de forma que:
   *       goal -> intermedio2 -> intermedio1 -> start */
  notifyTrajectory(result_trajectory, start, goal, came_from);

  return true;
}

void robmovil_planning::AStarPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                                                       std::map<Cell, Cell>& came_from)
{
  std::vector<Cell> path;
  Cell current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    ROS_INFO_STREAM("Path " << it->i << ", " << it->j);
    
    double cell_x, cell_y;
    
    getCenterOfCell(it->i, it->j, cell_x, cell_y);
    
    // Se crean los waypoints de la trajectoria
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.transform.translation.x = cell_x;
    point_msg.transform.translation.y = cell_y;
    point_msg.transform.translation.z = 0;
    
    if(it != path.rend()-1){
      double delta_x, delta_y;
      getCenterOfCell((it+1)->i, (it+1)->j, delta_x, delta_y);
      delta_x = delta_x - cell_x;
      delta_y = delta_y - cell_y;
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(delta_y, delta_x)) );
    } else
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(cell_y, cell_x)) );
    
    result_trajectory.points.push_back( point_msg );
  }
}

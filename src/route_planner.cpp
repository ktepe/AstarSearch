#include "route_planner.h"
#include <algorithm>
#include <iostream>
#include <vector>
//ket_dbg.h defines debugging output switches.
// 
#include "ket_dbg.h"

using std::sort;
using std::vector;

  
  
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    //this->start_node = &(model).FindClosestNode(start_x, start_y);
    //this->end_node = &(model).FindClosestNode(end_x, end_y);

  start_node = &(model.FindClosestNode(start_x, start_y));
  end_node = &(model.FindClosestNode(end_x, end_y));
  
#if true
  std::cout <<"in Route Planner: start_node x "<< start_node->x << " start node y "<< start_node->y << std::endl;
  std::cout <<"in Route Planner: end_node x "<< end_node->x << " end node y "<< end_node->y << std::endl;
  std::cout <<"in Route Planner: distance between start and end "<< start_node->distance(*(end_node)) << std::endl;
#endif
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
// float distance(Node other) const {
//            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
//        }

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
   return node->distance(*(end_node));   
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  current_node->FindNeighbors();
  current_node->visited = true;
  #if DBG_
  std::cout << "in AddNeighbors " << current_node->neighbors.size() << std::endl;
  #endif

  #if DBG_
  std::cout << "in AddNeighbors current node (x,y) ( " << current_node->x  << ", "<< current_node->y << " )" << std::endl;
#endif

  for(RouteModel::Node *nodex : current_node->neighbors)
    {
      // for each nodex in neighbors do the following 
      //	nodex_hvalue=CalculateHValue(node);
      //	nodex_gvalue=current_node_gvalue+this->CalculateHValue(node)
      //	nodex.visited = true
      //	nodex_parent_node = current_node
      //	open_list.add(nodex)
		// check this again
      	nodex->h_value = CalculateHValue(nodex);
        nodex->g_value = current_node->g_value + nodex->distance(*current_node);
      	nodex->parent = current_node;
      	nodex->visited = true;
      	open_list.emplace_back(nodex);
    }
}

/**
 * Compare the F values of two cells.
 */
bool Compare(RouteModel::Node *n1, RouteModel::Node *n2) {
  return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
RouteModel::Node *RoutePlanner::NextNode() {
//need our own comparator based on h and g values.
	//std::vector<Node *> neighbors;
  	RouteModel::Node *current;
  #if DBG
  std::cout << "in NextNode open_list size " << open_list.size() << std::endl;
  #endif
  	sort(open_list.begin(), open_list.end(), Compare);
    current = open_list.back();
  	open_list.pop_back();

#if DBG
  std::cout << "in NextNode : current (x,y) "<< current->x << " , "<< current->y << std::endl;
#endif

  	return current;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // TODO: Implement your solution here.
  #if DBG
  std::cout << "in RoutePlanner : current (x,y) "<< current_node->x << " , "<< current_node->y << std::endl;
  std::cout << "in RoutePlanner : current->parent (x,y) "<< current_node->parent->x << " , "<< current_node->parent->y << std::endl;
  #endif
  
	RouteModel::Node *temp_node;
  	temp_node = current_node;
 #if DBG
  std::cout << "in RoutePlanner : temp node  (x,y) "<< temp_node->x << " , "<< temp_node->y << std::endl;
  std::cout << "in RoutePlanner : temp->parent (x,y) "<< temp_node->parent->x << " , "<< temp_node->parent->y << std::endl;
  #endif

  while (!((temp_node->parent)==nullptr)){
  		path_found.emplace_back(*temp_node);
    	distance += temp_node->distance(*(temp_node->parent));
      	temp_node  = temp_node->parent;
  }
  path_found.emplace_back(*start_node);
  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  reverse(path_found.begin(), path_found.end());
  return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
/*
For this function you shall first:
1- mark start node as visited
2- add the start node to the open list
3-loop until the open list is empty
4-inside the loop you shall first pick the next node.
5-check if it is the intended node via checking against the end node
6-if yes then construct the final path then break
7-if no shall add another neighbor then iterate again.
*/

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // TODO: Implement your solution here.
	start_node->visited = true;
	open_list.emplace_back(start_node);
  	current_node = start_node;
  
 #if DBG
  std::cout << "in Astar : start node " << current_node << " (x,y) "<< current_node->x << " , "<< current_node->y << std::endl;   
  #endif
   
  
  while(open_list.size() >0 )
 {

    #if DBG
    std::cout << "in Astar  while loop current node " << current_node << " (x,y) "<< current_node->x << " , "<< current_node->y << std::endl;   
    #endif

    AddNeighbors(current_node);
    current_node = NextNode();
	if ((current_node == end_node))
    {
	  std::vector<RouteModel::Node> path_found = ConstructFinalPath(end_node);
 	 #if true
  	std::cout << "in Astar : path_found size " << path_found.size()  << std::endl;   
  	#endif
  	m_Model.path = path_found; 
  
      break;
    }
 }
  
}




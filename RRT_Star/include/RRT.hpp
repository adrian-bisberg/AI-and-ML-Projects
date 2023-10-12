#include "Math.hpp"
#include "Obstacles.hpp"
#include "DrawUtils.hpp"

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <exception>

// A custom exception for any config errors.
class RRTStartConfigExcption : public std::exception {
public:
    RRTStartConfigExcption(std::string message) : message(message){}
    std::string message;
const char * what () {
    return message.c_str();
}
};

/// @brief Node structure for storing auxillary information for each RRT* tree vertex.
struct Node{
    Vector2f vertex;             //< Point in space.
    int parentIndex;             //< Index in RRT vector of the parent Node.
    std::vector<int> children;   //< Indicies in RRT vector of children nodes. 
    float cost;                  //< Distance traveled from start along each ancestor.
};

class RRTStar{
public:

/// @brief Construct an object to store and use for RRT* computation.
/// @param xMax Max coordinate value in x direction of space.
/// @param yMax Max coordinate value in y direction of space.
/// @param obs Reference to the obstacles to test for collision.
/// @param start Point cooridnates to start search at.
/// @param goal Point coorindates to center goal area in.
/// @param goalRadius Radius from goal to consider completed goal.
/// @param neighbordoodRadius Optional tuning parameter for neighborhood retrieval.
/// @param stepSizeRho Optional tuning parameter for sample step size from tree.
/// @param maxIterations Optional tuning parameter for maximum iterations before algorithm reports
///                      goal notf found.
RRTStar(int xMax, 
            int yMax, 
            Obstacles& obs, 
            const Vector2f& start, 
            const Vector2f& goal, 
            int goalRadius, 
            int neighbordoodRadius = 50, 
            int stepSizeRho = 30, 
            int maxIterations = 3000);

/// @brief Find the best path from the set start to goal region.
/// @return List of waypoints to travel between.
std::vector<Vector2f> findBestPath();

/// @brief Draw to the renderer the best path if found.
void drawPath(SDL_Renderer* renderer);

/// @brief Draw the entire tree constructed in the process.
void drawTree(SDL_Renderer* renderer);

/// @brief Retrieve the final cost of the path that was found.
int getCost(){
    return m_pathCost;
}

private:
    
std::vector<Node> m_tree;      //< Track the verticies of the tree.
Vector2f m_start;              //< Starting location.
Vector2f m_goal;               //< Goal region center.
int m_goalRadius;              //< Radius of the goal region.
Obstacles* m_obs;              //< Obstacles in the region.
std::vector<Vector2f> m_path;  //< Retrieved path found.
int m_pathCost = 0;            //< Cost of the path found.

struct RRTStarConfig{
    int xmax; //< Max value for x coordinates according to given state space
    int ymax; //< Max value for y coordinates according to given state space
    int neighborhoodRadius; //< Radius to aquire neighborhood of closest vertices
    int maxIterations; //< Maximum number of iterations to perfrom before reporting failure to find path.
    int rho; //< Stepping size for steering function.
}config;

// Report if the provided point is in the goal region.
bool reachedGoal(const Vector2f& point);

// Retrieve the index of the nearest node in the tree based on the provided point.
int findNearest(const Vector2f& point);

// Find the index of all nodes within the neighborhood radius of the provided point.
std::vector<int> findNeighborhood(const Vector2f& point);

// Choose the parent node based on which point in the neighborhood would lead to the new point 
// with the lowest total cost.
int chooseParentNode(const std::vector<int>& neighborhood, int nearest, const Vector2f& newPoint);

// Revise the tree by checking if any neighbors can be improved in cost by passing through the newly added point.
void rewire(const std::vector<int>& neighborhood, int newPoint);

//Choose a random coordinate in the free space.
Vector2f freeRandomCoordinate();

// Steer the random coordinate to a new coordinate within rho distance of the nearest point.
Vector2f steer(const Vector2f& newPoint, const Vector2f& nearestPoint);

// Update the cost of all children of this parent with the new cost of the parent plus the cost 
// of the parent to the child.
void updateChildrenCosts(const Node& parent);

// Reconstruct the final path found to the last node by tracing back throught the parents.
std::vector<Vector2f> reconstructPath(const Node& last);
};
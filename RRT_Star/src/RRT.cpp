#include "RRT.hpp"

#include <algorithm>

RRTStar::RRTStar(int gridXMax, 
            int gridYMax, 
            Obstacles& obs, 
            const Vector2f& start, 
            const Vector2f& goal, 
            int goalRadius, 
            int neighbordoodRadius, 
            int stepSizeRho, 
            int maxIterations)
{
        // Set variables
        config.xmax = gridXMax;
        config.ymax = gridYMax;
        config.neighborhoodRadius = neighbordoodRadius;
        config.maxIterations = maxIterations;
        config.rho = stepSizeRho;
        m_obs = &obs;
        m_path = {};

        // Make sure the start is not in the obstacles
        if(!m_obs->inObstacles(start)){
            m_start = start;
        }else{
            throw RRTStartConfigExcption("Cannot set start location within an obstacle.");
        }

        // Set goal either way, if it is in obstacle path will just not be found.
        m_goal = goal;
        m_goalRadius = goalRadius;
        
        // Seed the random number generation.
        std::srand(std::time(0));
}

void RRTStar::drawPath(SDL_Renderer* renderer)
{
    for(int i = 0; i < m_path.size(); i++){
        if(i != m_path.size() - 1){
            // Draw edge from point to next in path.
            SDL_SetRenderDrawColor(renderer,0,100,100,250);
            SDL_RenderDrawLine(renderer,m_path.at(i).x, m_path.at(i).y, m_path.at(i+1).x, m_path.at(i+1).y);
        }

        SDL_SetRenderDrawColor(renderer,0,150,40,250);
        DrawPointScaled(renderer, m_path.at(i).x, m_path.at(i).y,3);
    }
}

bool RRTStar::reachedGoal(const Vector2f& point)
{
    return Distance(point, m_goal) <= m_goalRadius;
}

std::vector<Vector2f> RRTStar::reconstructPath(const Node& last)
{
    std::vector<Vector2f> path;
    path.push_back(last.vertex);
    int p = last.parentIndex;

    while(p != -1){
        path.push_back(m_tree.at(p).vertex);
        p = m_tree.at(p).parentIndex;
    }
    return path;
}

void RRTStar::drawTree(SDL_Renderer* renderer)
{
    for(int i = 0; i < m_tree.size(); i++){
        Node n = m_tree.at(i);
        if(n.parentIndex != -1){
            Node parent = m_tree.at(n.parentIndex);
            SDL_SetRenderDrawColor(renderer,240,240,240,70);
            SDL_RenderDrawLine(renderer,n.vertex.x, n.vertex.y, parent.vertex.x, parent.vertex.y);
        }

        SDL_SetRenderDrawColor(renderer,230,230,230,70);
        DrawPointScaled(renderer, m_tree.at(i).vertex.x, m_tree.at(i).vertex.y);
    }
}   

std::vector<Vector2f> RRTStar::findBestPath()
{
    // reset tree in case running multiple times
    m_tree.clear();
    m_path.clear();
    m_pathCost = 0;

    // add start vertex to tree
    m_tree.push_back({m_start, -1, {}, 0});

    // Run for up to the maximum specified iterations.
    for(int i = 0; i < config.maxIterations; i++){

        // Find a new cooridnate to try from random sample the steering towards the 
        // nearest coordinate in the tree to a new point.
        Vector2f randSample = freeRandomCoordinate();
        int nearest = findNearest(randSample);
        Vector2f newPoint = steer(randSample, m_tree.at(nearest).vertex);

        // Ensure that the new point is not in an obstacle, otherwise try again
        // in next iteration
        if(!m_obs->inObstacles(newPoint)){

            // Get the neighborhood and choose the best cost parent form it 
            // for the new point.
            std::vector<int> neighbors = findNeighborhood(newPoint);
            int parent = chooseParentNode(neighbors, nearest, newPoint);

            if(parent == -1){
                //skipping iteration, only could find paths through obstacles
                continue;
            }

            // Add the new index to the tree via the chosen parent
            int newIndex = m_tree.size();
            m_tree.push_back({newPoint, parent, {}, m_tree.at(parent).cost + Distance(m_tree.at(parent).vertex, newPoint)});
            m_tree.at(parent).children.push_back(newIndex);

            // Rewire the tree to check for shorter cost paths
            rewire(neighbors, newIndex);

            // Check if the new point found was in the goal region and return the reocnstructed path if so.
            if(reachedGoal(newPoint)){
                m_path = reconstructPath(m_tree.at(newIndex));
                m_pathCost = m_tree.at(newIndex).cost;
                return m_path;
            }
        }
    }
    // No path found after max iterations
    return {};
}

Vector2f RRTStar::steer(const Vector2f& randPoint, const Vector2f& nearestPoint)
{
    // Get the direction between the random point and the nearest using the normal
    Vector2f direction = NormalizedRetrieve(randPoint - nearestPoint);
    int dist = Distance(randPoint, nearestPoint);

    // check if the random point is already close enough
    if(dist <= config.rho){
        return randPoint;
    }else{
        // If not close enough take a step of length rho in the direction of the random point.
        Vector2f step = {(float)trunc(direction.x * config.rho), (float)trunc(direction.y * config.rho)};
        return nearestPoint + step;
    }
}

int RRTStar::chooseParentNode(const std::vector<int>& neighborhood, int nearest, const Vector2f& newPoint)
{
    // Start with the nearest node as best partent index
    int bestParent = nearest;

    // Using the cost to reach the node, plus the distance from this node to the new point
    int bestCost = m_tree.at(nearest).cost + Distance(m_tree.at(nearest).vertex, newPoint);

    // Check against all the neigbors to find the best path parent
    for(int i = 0; i < neighborhood.size(); i++){
        int nIndex = neighborhood.at(i);
        Node& n = m_tree.at(nIndex);

        // Found a better parent if the cost to the parent plus the cost
        // to the new point is less than the best found so far AND
        // there is no obsatcle obstructing the path to the new point
        if(n.cost + Distance(n.vertex, newPoint) < bestCost
           && !this->m_obs->segmentInObstacles(n.vertex, newPoint)){
            bestParent = nIndex;
            bestCost = n.cost + Distance(n.vertex, newPoint);
        }
    }

    // special check if the nearest node was chosen and there is an obstacle in the way, need
    // to indacate this point cannot be used
    if(bestParent == nearest && m_obs->segmentInObstacles(m_tree.at(bestParent).vertex, newPoint)){
        return -1;
    }

    return bestParent;
}

void RRTStar::rewire(const std::vector<int>& neighborhood, int newPoint)
{
    for(int i = 0; i < neighborhood.size(); i++){
        int nIndex = neighborhood.at(i);
        Node& n = m_tree.at(nIndex);

        // If cost to new point plus distance from new point to neighbor is less than the 
        // neighbors current cost ...
        // AND there are no obstacles between the new vertex and the neighbor
        if(m_tree.at(newPoint).cost + Distance(m_tree.at(newPoint).vertex, n.vertex) < n.cost
           && !m_obs->segmentInObstacles(n.vertex, m_tree.at(newPoint).vertex)){

            // Remove this node from its parents children list
            std::vector<int>& children = m_tree.at(n.parentIndex).children;
            children.erase(std::remove(children.begin(), children.end(), nIndex), children.end());

            // Update the new cost and parent index
            n.cost = m_tree.at(newPoint).cost + Distance(m_tree.at(newPoint).vertex, n.vertex);
            n.parentIndex = newPoint;

            // Add as a child to the new parent
            m_tree.at(newPoint).children.push_back(nIndex);

            // Recursively update any children costs with the new connection cost
            updateChildrenCosts(n);
        }
    }
}

void RRTStar::updateChildrenCosts(const Node& parent)
{
    // End condition of recursion.
    if (parent.children.empty()){
        return;
    }

    for(int i = 0; i < parent.children.size(); i++){
        int childIndex = parent.children.at(i);
        m_tree.at(childIndex).cost = parent.cost + Distance(parent.vertex, m_tree.at(childIndex).vertex);
        updateChildrenCosts(m_tree.at(childIndex));
    }
}

int RRTStar::findNearest(const Vector2f& point)
{
    int min_dist = INT_MAX;
    int min_index = -1;

    for(int i = 0; i < m_tree.size(); i++){
        if (Distance(point, m_tree.at(i).vertex) < min_dist){
            min_dist = Distance(point, m_tree.at(i).vertex);
            min_index = i;
        }
    }
    return min_index;
}

Vector2f RRTStar::freeRandomCoordinate()
{
    bool needsRandom = true;

    while(needsRandom){
        Vector2f p = {(float)trunc(rand() % config.xmax), (float)trunc(rand() % config.ymax)};

        if(!m_obs->inObstacles(p)){
            return p;
        }
    }
    return {};
}

std::vector<int> RRTStar::findNeighborhood(const Vector2f& point)
{
    std::vector<int> points;
    for(int i = 0; i < m_tree.size(); i++){
        if (Distance(point, m_tree.at(i).vertex) <= config.neighborhoodRadius){
            points.push_back(i);
        }
    }
    return points;
}

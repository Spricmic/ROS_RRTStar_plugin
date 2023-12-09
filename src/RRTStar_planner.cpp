#include "RRTStar_planner.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <vector>
#include <algorithm>


class Node {
public:
    // Default constructor
    Node() : x(0), y(0), parent(nullptr), cost(0) {}

    // Parameterized constructor
    Node(double x, double y) : x(x), y(y), parent(nullptr), cost(0) {}

    // Member variables
    double x;
    double y;
    Node* parent;
    double cost;
};


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrtstar_planner::RRTStar, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrtstar_planner {

    RRTStar::RRTStar (){
    }

    RRTStar::RRTStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);

        kd_tree_ = std::make_shared<planner::KDTreeNodeList>(2);  // create an KDTree structure for two dimensional space.
    }


    void RRTStar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    }

    bool RRTStar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        ROS_INFO("RRTstar.makePlan() called");

        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized, please call RRTStar.initialize() to use the planner");
            return false;
        }
        // clear Path to make sure its an empty array.
        plan.clear();

        plan.push_back(start);
        plan.push_back(goal);
         
        return true;
    }

    /**
     * @brief Checks if a given cell with coordinates (x, y) is an obstacle in the costmap
     * 
     * @param costmap The costmap to check
     * @param x x-coordinate of the cell
     * @param y y-coordinate of the cell
     * @return true if the cell is an obstacle, false otherwise
     */
    bool RRTStar::isCellObstacle(const costmap_2d::Costmap2D& costmap, unsigned int x, unsigned int y) {
        unsigned char cost = costmap.getCost(x, y);

        // Assuming LETHAL_OBSTACLE represents obstacles
        // TODO check if value set in LETHAL_OBSTACLE is really an obstacle.
        return cost == costmap_2d::LETHAL_OBSTACLE;
    }

    /**
    * @brief Checks if a given cell with coordinates (x, y) is within the bounds of the costmap.
    *
    * @param costmap The costmap to check against.
    * @param x The x-coordinate of the cell.
    * @param y The y-coordinate of the cell.
    * @return True if the cell is within the bounds, false otherwise.
    */
    bool RRTStar::isCellOnMap(const costmap_2d::Costmap2D& costmap, unsigned int x, unsigned int y){
        // Get the size of the costmap in cells
        unsigned int size_x = costmap.getSizeInCellsX();
        unsigned int size_y = costmap.getSizeInCellsY();

        // Check if the given cell coordinates are within the bounds of the costmap
        return x >= 0 && x < static_cast<int>(size_x) && y >= 0 && y < static_cast<int>(size_y);
    }       

    /**
     * @brief generates a random point on the map.
     * 
     * @param costmap The costmap to check
     * @return Node a randomly generated node with (x,y) values.
     */
    Node RRTStar::generateRandomPoint(const costmap_2d::Costmap2D& costmap) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> x_dist(0, costmap.getSizeInCellsX() - 1);
        std::uniform_int_distribution<int> y_dist(0, costmap.getSizeInCellsY() - 1);

        Node randomPoint;
        do {
            randomPoint.x = x_dist(gen);
            randomPoint.y = y_dist(gen);
        } while !RRTStar::isCellOnMap(costmap, randomPoint.x, randomPoint.y);

        return randomPoint; 
    }

    /**
     * @brief add a new point to the path that the robot needs to follow.
     * 
     * @param plan the current path plan of the robot to which to add the new waypoint
     * @param x x coordinates of the point in meters
     * @param y y coordinates of the point in meters
     */
    void RRTStar::addPointToPath(std::vector<geometry_msgs::PoseStamped>& plan, double x, double y) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }

    // overhaul this with the use of KDTree as this is really really unefficent.
    /**
     * @brief findes all nodes in a radius near the specified node.
     * 
     * @param newNode the new node around which should be searched for other nodes
     * @param nodes a pointer to where the found nodes should be stored.
     * @param searchRadius the search radius in which shoudl be searched for other nodes
     * @return std::vector<Node> a vector containing all the found nodes
     */
    std::vector<Node> RRTStar::findNearNodes(const Node& newNode, const std::vector<Node>& nodes, double searchRadius) {
        std::vector<Node> nearNodes;

        for (const auto& node : nodes) {
            double distance = std::sqrt(std::pow(node.x - newNode.x, 2) + std::pow(node.y - newNode.y, 2));
            if (distance <= searchRadius) {
                nearNodes.push_back(node);
            }
        }

        return nearNodes;
    }

    /**
     * @brief calculates the distance between two nodes.
     * 
     * @param node1 the node on point 1 to check.
     * @param node2 the node on point 2 to check.
     * @return double return the distance between the two nodes.
     */
    double RRTStar::calculateDistance(const Node& node1, const Node& node2) {
        return std::sqrt(std::pow(node2.x - node1.x, 2) + std::pow(node2.y - node1.y, 2));
    }



};

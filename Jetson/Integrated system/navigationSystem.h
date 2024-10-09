// #ifndef NAVIGATION_SYSTEM_H
// #define NAVIGATION_SYSTEM_H

// #include <vector>
// #include <cmath>
// #include <iostream>

// #include "positionController.h"

// // in meters
// #define FORWRD_MOVE_DISTANCE 0.3

// #define OBSTACLE_DETECTION_DISTANCE 0.3

// enum class NavigationState
// {
//     FORWARD,
//     BACKWARD,
//     STOP,
//     AVOID_OBSTACLE,
//     MOVE_AWAY_FROM_OBSTACLE,
//     ATTEMPT_TO_PASS_OBSTACLE,
// };

// enum class TurnDirection
// {
//     LEFT,
//     RIGHT,
// };

// class navigationSystem
// {
// public:
//     navigationSystem();
//     ~navigationSystem();

//     std::vector<float> explore(std::vector<float> robotPosition, std::vector<float> distMeasurements);

//     // Function to get the navigation state
//     NavigationState getNavigationState();

//     // Function to set the position controller
//     void setPositionController(positionController *posController);

// private:
//     // Functions for the state machine
//     std::vector<float> forwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
//     std::vector<float> backwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
//     std::vector<float> stopState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
//     std::vector<float> avoidObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
//     std::vector<float> moveAwayFromObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
//     std::vector<float> attemptToPassObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);

//     // Function to check if forward motion is possible using distance sensor readings
//     bool isForwardMotionPossible(std::vector<float> distMeasurements);

//     // Function to convert turn direction to real world angle
//     float turnDirectionToAngle(TurnDirection turnDirection, std::vector<float> robotPosition);

//     void alternateTurnDirection();

//     // Navigation state variable
//     NavigationState navigationState;

//     // Turn direction variable
//     // Thist variable indicates the next turn direction after the robot has detected an obstacle
//     // It cycles between LEFT and RIGHT after each detection and avoidance of an obstacle
//     TurnDirection turnDirection;

//     // Distance traveled since the last obstacle was detected
//     // When this exceeds some threshold, the robot turns back
//     float distanceSinceLastObstacle;

//     // Position controller object
//     positionController *posController;
// };
// #endif // NAVIGATION_SYSTEM_H

#ifndef NAVIGATION_SYSTEM_H
#define NAVIGATION_SYSTEM_H

#include <vector>
#include <stack>
#include <set>
#include <utility>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <climits>
#include <iostream>

// Maximum depth for DFS in grid cells
#define MAX_DFS_DEPTH 100

// Grid size in meters
#define MAP_GRID_SIZE 0.4

// Node for the graph representing the operational area
class Node
{
public:
    std::pair<int, int> coordinates; // Grid cell coordinates
    bool isFree;                     // True if the cell is free space
    bool isVisited;                  // True if the node has been visited
    bool isObstacle;                 // True if the cell is an obstacle
    bool inStack;                    // True if the node is in the exploration stack

    // Directional neighbors
    std::vector<Node *> neighbors; // Stored in consistent order: North, East, South, West. nullptr if no neighbor in that direction

    Node(std::pair<int, int> coords)
        : coordinates(coords), isFree(true), isVisited(false), isObstacle(false), inStack(false)
    {
        neighbors = std::vector<Node *>(4, nullptr);
    }
};

// Graph representing the operational area
namespace std
{
    template <>
    struct hash<std::pair<int, int>>
    {
        size_t operator()(const std::pair<int, int> &p) const
        {
            // Combine the hashes of the two integers
            size_t h1 = std::hash<int>{}(p.first);
            size_t h2 = std::hash<int>{}(p.second);
            return h1 ^ (h2 << 1); // Or use any other hash combination method
        }
    };
}

class NavigationGraph
{
public:
    NavigationGraph() {}
    ~NavigationGraph() {}

    std::unordered_map<std::pair<int, int>, Node *> nodes;

    // Get the node at the specified coordinates
    Node *getNode(const std::pair<int, int> &coords)
    {
        auto it = nodes.find(coords);
        if (it != nodes.end())
        {
            return it->second;
        }
        return nullptr;
    }

    // Add a node to the graph
    void addNode(Node *node)
    {
        nodes[node->coordinates] = node;
    }

private:
    // Private members...
};

enum class Direction
{
    North,
    East,
    South,
    West
};

enum class NavigationState
{
    MAIN_EXPLORE,
    DIJKSTRA,
    FINISHED
};

class navigationSystem
{
public:
    navigationSystem();
    ~navigationSystem();

    // Function to explore the operational area
    // Recieves real world coordinates and distance sensor data
    // Returns the next set of coordinates for the robot to move to
    std::vector<float> explore(std::vector<float> worldCoords, std::vector<float> distSensorData);

    // Function to get the state of the navigation system
    NavigationState getState() { return this->state; }

private:
    // State machine functions
    std::vector<float> mainExplore(std::vector<float> worldCoords, std::vector<float> distSensorData);
    std::vector<float> dijkstraExplore(std::vector<float> worldCoords, std::vector<float> distSensorData);

    // Visits the current node and creates its neighbors if necessary
    void visitCurrentNode();

    bool isAdjacent(Node *node1, Node *node2);

    void createNeighbors();

    Direction getOppositeDirection(Direction direction);

    bool isCellInfrontFree(std::vector<float> distSensorData);

    Direction getDirectionFromAToB(Node *nodeA, Node *nodeB);

    float getRealWorldAngle(Direction direction);

    std::pair<int, int> getGridCoordinates(std::vector<float> worldCoords);
    std::vector<float> getRealWorldCoordinates(std::pair<int, int> coords, Direction dir);

    // Function to get the next coordinates of the robot given the current direction
    std::pair<int, int> getNextCoordinates(Direction direction);

    // Dijkstra's algorithm to find the shortest path to a location
    // This algorithm is used when switching to BFS to find the closest unvisited node
    std::vector<std::pair<int, int>> dijkstra(Node *start, Node *end);

    // Exploration stack
    std::stack<Node *> explorationStack;

    // Current direction of the robot
    Direction currentDirection;

    // Current node the robot is at
    Node *currentNode;

    // Graph representing the operational area
    NavigationGraph graph;

    // State of the navigation system
    NavigationState state;

    // Path found by Dijkstra's algorithm
    std::vector<std::pair<int, int>> dijkstraPath;

    Node *targetNode; // Node for dijkstra target
};
#endif // NAVIGATION_SYSTEM_H
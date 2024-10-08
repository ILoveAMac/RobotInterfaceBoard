#include "navigationSystem.h"

navigationSystem::navigationSystem()
{
    this->explorationStack = std::stack<Node *>();

    // Create a start node at the origin
    Node *start = new Node(std::pair<int, int>(0, 0));
    this->graph.addNode(start);
    this->currentNode = start;

    this->explorationStack.push(start);

    this->state = NavigationState::MAIN_EXPLORE;

    this->currentDirection = Direction::North;
}

navigationSystem::~navigationSystem()
{
}

std::vector<float> navigationSystem::explore(std::vector<float> worldCoords, std::vector<float> distSensorData)
{

    // State machine here
    switch (this->state)
    {
    case NavigationState::MAIN_EXPLORE:
        return this->mainExplore(worldCoords, distSensorData);
    case NavigationState::DIJKSTRA:
        return this->dijkstraExplore(worldCoords, distSensorData);
    case NavigationState::FINISHED:
        return worldCoords;
    default:
        return worldCoords;
    }
}

std::vector<float> navigationSystem::mainExplore(std::vector<float> worldCoords, std::vector<float> distSensorData)
{
    // Check if the exploration stack is empty
    if (this->explorationStack.empty())
    {
        // Exploration is complete
        this->state = NavigationState::FINISHED;
        return worldCoords;
    }

    // Get the next node to explore
    Node *nextNode = this->explorationStack.top();
    this->explorationStack.pop();

    if (nextNode->isVisited)
    {
        // Node has already been visited
        return worldCoords;
    }

    // Check if the node has at least two visited neighbors
    int visitedNeighbors = 0;
    for (Node *neighbor : nextNode->neighbors)
    {
        if (neighbor != nullptr)
        {
            if (neighbor->isVisited)
            {
                visitedNeighbors++;
            }
        }
    }
    if (visitedNeighbors >= 2)
    {
        // Node has at least two visited neighbors
        // We skip this node
        return worldCoords;
    }

    // ========== Exploration logic ==========

    // Check if the current node is the next node
    if (this->currentNode == nextNode)
    {
        // Visit the current node
        std::cout << "Next node: " << nextNode->coordinates.first << ", " << nextNode->coordinates.second << std::endl;
        this->visitCurrentNode();

        return worldCoords; // We don't need to move
    }

    // Check if the next node is adjacent to the current node
    if (this->isAdjacent(this->currentNode, nextNode))
    {
        // Get the direction from the current node to the next node
        Direction direction = this->getDirectionFromAToB(this->currentNode, nextNode);

        // Check if we are facing in the same direction
        if (direction == this->currentDirection)
        {
            // Check if the cell in front is free
            if (this->isCellInfrontFree(distSensorData))
            {
                // Move the robot to the next node
                std::pair<int, int> nextCoords = nextNode->coordinates;
                std::vector<float> nextWorldCoords = this->getRealWorldCoordinates(nextCoords);

                // Update the current node
                this->currentNode = nextNode;

                return nextWorldCoords;
            }
            else
            {
                // Mark the next node as visited and not free and as an obstacle
                nextNode->isVisited = true;
                nextNode->isFree = false;
                nextNode->isObstacle = true;

                // return the current world coordinates
                return worldCoords;
            }
        }
        else
        {
            // Rotate the robot to face the correct direction
            float angle = this->getRealWorldAngle(direction);
            this->currentDirection = direction;

            // Push the next node back to the stack
            this->explorationStack.push(nextNode);

            return std::vector<float>{worldCoords[0], worldCoords[1], angle};
        }
    }

    // The node is not adjacent to the current node so we should use dijkstraExplore to attempt to navigate to the node
    this->state = NavigationState::DIJKSTRA;
    this->targetNode = nextNode;
    return this->dijkstraExplore(worldCoords, distSensorData);
}

std::vector<float> navigationSystem::dijkstraExplore(std::vector<float> worldCoords, std::vector<float> distSensorData)
{
    // Check if we have a path to follow
    if (this->dijkstraPath.empty())
    {
        // No path available, switch back to main exploration
        this->state = NavigationState::MAIN_EXPLORE;
        return worldCoords;
    }

    // Get the next coordinates in the path
    std::pair<int, int> nextCoords = this->dijkstraPath.front();
    this->dijkstraPath.erase(this->dijkstraPath.begin());

    // Get the node corresponding to nextCoords
    Node *nextNode = this->graph.getNode(nextCoords);

    if (nextNode == nullptr)
    {
        // Node does not exist, switch back to main exploration
        this->state = NavigationState::MAIN_EXPLORE;
        return worldCoords;
    }

    // Check if the next node is adjacent
    if (!this->isAdjacent(this->currentNode, nextNode))
    {
        // Nodes are not adjacent, re-plan path
        this->dijkstraPath = this->dijkstra(this->currentNode, this->targetNode);
        return this->dijkstraExplore(worldCoords, distSensorData);
    }

    // Get the direction from the current node to the next node
    Direction direction = this->getDirectionFromAToB(this->currentNode, nextNode);

    // Check if we need to rotate
    if (direction != this->currentDirection)
    {
        // Rotate the robot to face the correct direction
        float angle = this->getRealWorldAngle(direction);
        this->currentDirection = direction;

        // Put the nextCoords back to the front of the path
        this->dijkstraPath.insert(this->dijkstraPath.begin(), nextCoords);

        // Return the current position with the new angle for rotation
        return std::vector<float>{worldCoords[0], worldCoords[1], angle};
    }

    // Check if the cell in front is free
    if (this->isCellInfrontFree(distSensorData))
    {
        // Move the robot to the next node
        std::vector<float> nextWorldCoords = this->getRealWorldCoordinates(nextCoords);

        // Update the current node
        this->currentNode = nextNode;

        // Visit the current node
        this->visitCurrentNode();

        // Continue along the path if there are more nodes
        if (!this->dijkstraPath.empty())
        {
            // Proceed to the next node in the path
            return nextWorldCoords;
        }
        else
        {
            // Path complete, switch back to main exploration
            this->state = NavigationState::MAIN_EXPLORE;
            return nextWorldCoords;
        }
    }
    else
    {
        // Obstacle encountered, mark the node accordingly
        nextNode->isVisited = true;
        nextNode->isFree = false;
        nextNode->isObstacle = true;

        // Re-plan the path from the current node
        this->dijkstraPath = this->dijkstra(this->currentNode, this->targetNode);

        if (this->dijkstraPath.empty())
        {
            // No path found, switch back to main exploration
            this->state = NavigationState::MAIN_EXPLORE;
            return worldCoords;
        }
        else
        {
            // Continue with the new path
            return this->dijkstraExplore(worldCoords, distSensorData);
        }
    }
}

void navigationSystem::visitCurrentNode()
{
    // If the node has already been visited, return
    if (this->currentNode->isVisited)
    {
        return;
    }

    // Mark the node as visited
    this->currentNode->isVisited = true;

    // Create the neighbors of the current node
    this->createNeighbors();

    // Add the unexplored neighbors to the exploration stack
    for (Node *neighbor : this->currentNode->neighbors)
    {
        if (neighbor != nullptr && !neighbor->isVisited && !neighbor->inStack)
        {
            neighbor->inStack = true;
            this->explorationStack.push(neighbor);
        }
    }
}

bool navigationSystem::isAdjacent(Node *node1, Node *node2)
{
    std::pair<int, int> coords1 = node1->coordinates;
    std::pair<int, int> coords2 = node2->coordinates;

    return abs(coords1.first - coords2.first) + abs(coords1.second - coords2.second) == 1;
}

void navigationSystem::createNeighbors()
{
    // Create the neighbors of the current node in all directions
    for (int i = 0; i < 4; i++)
    {
        Direction direction = static_cast<Direction>(i);
        std::pair<int, int> coords = this->getNextCoordinates(direction);
        Node *neighbor = this->graph.getNode(coords);

        if (neighbor == nullptr)
        {
            neighbor = new Node(coords);
            this->graph.addNode(neighbor);
        }

        // Add the node to the neighbors list of the current node
        this->currentNode->neighbors[i] = neighbor;

        // Add the reverse neighbor relationship
        neighbor->neighbors[static_cast<int>(this->getOppositeDirection(direction))] = this->currentNode;
    }
}

Direction navigationSystem::getOppositeDirection(Direction direction)
{
    switch (direction)
    {
    case Direction::North:
        return Direction::South;
    case Direction::East:
        return Direction::West;
    case Direction::South:
        return Direction::North;
    case Direction::West:
        return Direction::East;
    }

    return Direction::North; // Default case
}

bool navigationSystem::isCellInfrontFree(std::vector<float> distSensorData)
{
    bool isFree = false;
    for (int i = 0; i < static_cast<int>(distSensorData.size()); i++)
    {
        if (distSensorData[i] > 0.4 && distSensorData[i] != -1)
        {
            isFree = true;
            break;
        }
    }

    return isFree;
}

Direction navigationSystem::getDirectionFromAToB(Node *nodeA, Node *nodeB)
{
    std::pair<int, int> coordsA = nodeA->coordinates;
    std::pair<int, int> coordsB = nodeB->coordinates;

    if (coordsA.first == coordsB.first)
    {
        if (coordsA.second > coordsB.second)
        {
            return Direction::East; // Moving East decreases y
        }
        else if (coordsA.second < coordsB.second)
        {
            return Direction::West; // Moving West increases y
        }
        return Direction::North; // Nodes are the same
    }
    else if (coordsA.second == coordsB.second)
    {
        if (coordsA.first < coordsB.first)
        {
            return Direction::North; // Moving North increases x
        }
        else if (coordsA.first > coordsB.first)
        {
            return Direction::South; // Moving South decreases x
        }
        return Direction::North; // Nodes are the same
    }
    else
    {
        // Nodes are not adjacent or not aligned; handle as needed
        throw std::runtime_error("Nodes are not adjacent in getDirectionFromAToB");
    }
}

float navigationSystem::getRealWorldAngle(Direction direction)
{
    // Angle in radians
    float angle = 0.0;
    switch (direction)
    {
    case Direction::North:
        angle = 0.0;
        break;
    case Direction::East:
        angle = M_PI / 2;
        break;
    case Direction::South:
        angle = M_PI;
        break;
    case Direction::West:
        angle = -M_PI / 2;
        break;
    }

    return angle;
}

std::pair<int, int> navigationSystem::getGridCoordinates(std::vector<float> worldCoords)
{
    // Convert the real world coordinates to grid coordinates
    int x = static_cast<int>(worldCoords[0] / MAP_GRID_SIZE);
    int y = static_cast<int>(worldCoords[1] / MAP_GRID_SIZE);

    return std::pair<int, int>(x, y);
}

std::vector<float> navigationSystem::getRealWorldCoordinates(std::pair<int, int> coords)
{
    // convert the grid coordinates to real world coordinates
    float x = coords.first * MAP_GRID_SIZE;
    float y = coords.second * MAP_GRID_SIZE;

    return std::vector<float>{x, y};
}

std::pair<int, int> navigationSystem::getNextCoordinates(Direction direction)
{
    int dx = 0;
    int dy = 0;
    switch (direction)
    {
    case Direction::North:
        dx = 1;
        dy = 0;
        break;
    case Direction::East:
        dx = 0;
        dy = -1;
        break;
    case Direction::South:
        dx = -1;
        dy = 0;
        break;
    case Direction::West:
        dx = 0;
        dy = 1;
        break;
    }

    std::pair<int, int> currentCoords = this->currentNode->coordinates;
    return std::pair<int, int>(currentCoords.first + dx, currentCoords.second + dy);
}

std::vector<std::pair<int, int>> navigationSystem::dijkstra(Node *start, Node *end)
{
    // Priority queue for the open set (min-heap based on distance)
    typedef std::pair<int, Node *> QueueElement;
    auto compare = [](QueueElement a, QueueElement b)
    { return a.first > b.first; };
    std::priority_queue<QueueElement, std::vector<QueueElement>, decltype(compare)> openSet(compare);

    // Maps to store distances and previous nodes
    std::unordered_map<Node *, int> distances;
    std::unordered_map<Node *, Node *> previous;

    // Initialize distances to infinity
    for (auto &pair : this->graph.nodes)
    {
        distances[pair.second] = INT_MAX;
    }
    distances[start] = 0;

    openSet.push(std::make_pair(0, start));

    while (!openSet.empty())
    {
        Node *current = openSet.top().second;
        openSet.pop();

        if (current == end)
        {
            // Reconstruct the path from start to end
            std::vector<std::pair<int, int>> path;
            Node *node = end;
            while (node != nullptr)
            {
                path.insert(path.begin(), node->coordinates);
                node = previous[node];
            }
            return path;
        }

        for (Node *neighbor : current->neighbors)
        {
            if (neighbor != nullptr && neighbor->isFree != false) // Consider free or unknown nodes
            {
                int tentativeDistance = distances[current] + 1;

                if (tentativeDistance < distances[neighbor])
                {
                    distances[neighbor] = tentativeDistance;
                    previous[neighbor] = current;
                    openSet.push(std::make_pair(distances[neighbor], neighbor));
                }
            }
        }
    }

    // No path found
    return std::vector<std::pair<int, int>>();
}

// Standard Library Headers
#include <chrono>  // For timing
#include <cmath>   // For math functions
#include <cstdio>  // For printf
#include <cstring> // For memcpy
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// OpenCV Headers
#include <opencv2/opencv.hpp>

// Boost Headers
#include <boost/asio.hpp>

// Project-Specific Headers
#include "robotController.cuh"

int main()
{
    // Create a robot controller
    robotController robotController;

    // Set the controller state
    robotController.setRobotState(RobotState::DETECTION_ALLIGNMENT);
    // Main loop
    while (true)
    {
        // Update the robot controller
        robotController.update();
    }
    return 0;
}
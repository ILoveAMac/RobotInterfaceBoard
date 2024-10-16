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
    robotController rController;

    // Set the controller state
    rController.setRobotState(RobotState::MARKER_SETUP);
    // Main loop
    while (true)
    {
        // Update the robot controller
        rController.update();
    }
    return 0;
}
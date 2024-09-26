#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <cstring> // For memcpy
#include <cstdio>  // For printf
#include <thread>
#include <chrono>
#include <math.h>

#include "serialHelper.h"
#include "positionController.h"

int main()
{
    serialHelper serial("/dev/ttyUSB0", 9600);

    // Create a position controller
    positionController controller(0.5, 10, 0.1, 0.05);

    // Set the goal position
    controller.setGoal(0, 0, 0);

    // reset the position of the robot
    serial.resetPosition();

    // request poop pickup
    serial.requestAndWaitForPoopPickup();

    // in a loop update the velocities until the robot is at the goal
    while (true)
    {
        // receive and print the position
        std::vector<float> position = serial.receivePosition();
        // std::cout << "Position: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
        // Check if the robot is at the goal
        // if (controller.atGoalPosition() && controller.atGoalOrientation())
        // {
        //     break;
        // }

        // Update the velocities
        std::vector<float> velocities = controller.updateVelocities(position[0], position[1], position[2]);
        // print the velocities
        // std::cout << "Velocities: " << velocities[0] << ", " << velocities[1] << std::endl;

        // Send the motor speeds
        serial.sendSpeeds(velocities[1], velocities[1], velocities[0], velocities[0]);

        // Print distance sensor measurements for sensor 5
        float distance = serial.receiveDistanceSensorMeasurement(SENSE_1);
        std::cout << "Distance sensor: " << distance << std::endl;
        if (distance < 0.2 && distance != -1)
        {
            break;
        }

        // delay for 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // set the motor speeds to 0
    serial.sendSpeeds(0, 0, 0, 0);

    return 0;
}
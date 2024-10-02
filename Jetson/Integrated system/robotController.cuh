#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "serialHelper.h"
#include "visualServoing.h"
#include "positionController.h"
#include "aiHelperUtils.h"
#include "yolo.cuh"

// Delay Time (ms)
#define DELAY_TIME 50

// Ai Model Path
#define MODEL_PATH "/home/wihan/model/"

// Image Size
#define IMAGE_WIDTH 448
#define IMAGE_HEIGHT 448

// Image capture device
#define IMAGE_CAPTURE_DEVICE 0

// Position Controller Parameters
#define C_KP 0.5
#define C_KA 0.5
#define C_GOAL_TOLERANCE 0.1
#define C_ANGLE_TOLERANCE 0.05

// Serial connection parameters
#define USB_CONNECTION "/dev/ttyUSB0"
#define BAUD_RATE 9600

// Enum for robot states
enum class RobotState
{
    IDLE,                                // In this state the robot does nothing
    MOVE_AND_DETECT,                     // In this state the robot uses a navigation algorithm to cover an area, it also detects poop
    DETECTION_ALLIGNMENT,                // In this state the robot alligns itself to the poop
    PICKUP,                              // In this state the robot preforms the poop pickup procedure
    MOVE_BACK_TO_POSITION_BEFORE_PICKUP, // In this state the robot moves back to the position before the pickup,
                                         // such that the navigation algorithm can continue can resume
    SEARCH_FOR_MARKER,                   // In this state the robot searches for a marker
    NAVIGATE_TO_MARKER,                  // In this state the robot navigates to the marker
    ALLIGN_TO_MARKER,                    // In this state the robot alligns itself to the marker
    DROP_OFF                             // In this state the robot drops off the poop
};

class robotController
{
public:
    robotController();
    ~robotController();

    // State machine update function
    // Call this function in a loop to update the robot state
    void update();

    // Function to set the robot state
    void setRobotState(RobotState state);

private:
    RobotState robotState;

    // === Functions for each state ===
    void idle();
    void moveAndDetect();
    void detectionAllignment();
    void pickup();
    void moveBackToPositionBeforePickup();
    void searchForMarker();
    void navigateToMarker();
    void allignToMarker();
    void dropOff();

    // === Helper Functions ===
    std::vector<float> getRobotPosition(); // Function uses the serial helper to get the robot position

    void captureAndPreProcessImage(); // Function captures an image from the camera and pre-processes it

    void updateRobotPosition(); // Function updates the robot position using the controller

    std::vector<std::vector<float>> getBoundingBoxesAndDraw(); // Function gets bounding boxes from the YOLO model and draws them on the image

    void delay(int ms);

    int image_counter;

    // === Storage for Captured Images ===
    float *input_image;
    float *host_image;

    cv::Mat frame;
    cv::Mat resized_frame;
    std::vector<cv::Mat> channels;

    // === Variables ===
    std::vector<float> robotPosition;

    // === Objects ===

    // AI Helper object
    aiHelperUtils aiHelper;

    // YOLO object
    yolo yolo;

    // Visual servoing object
    visualServoing visualServoing;

    // Position controller object
    positionController positionController;

    // Serial helper object
    serialHelper serial;

    // Open CV Video Capture object
    cv::VideoCapture cap;
};

#endif // ROBOT_CONTROLLER_H
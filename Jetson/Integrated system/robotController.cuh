#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

// Threading related includes
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <tuple>

#include "serialHelper.h"
#include "visualServoing.h"
#include "positionController.h"
#include "aiHelperUtils.h"
#include "yolo.cuh"
#include "navigationSystem.h"
#include "markerSystem.cuh"

// Max Poops that the robot can collect
#define MAX_POOPS 2

// Delay Time (ms)
#define DELAY_TIME 25

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
#define C_GOAL_TOLERANCE 0.2
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
    WAIT_FOR_PICKUP_VERIFICATION,        // In this state the robot checks again with the AI if the poop is still there
    MOVE_BACK_TO_POSITION_BEFORE_PICKUP, // In this state the robot moves back to the position before the pickup,
                                         // such that the navigation algorithm can continue can resume
    SEARCH_FOR_MARKER,                   // In this state the robot searches for a marker
    NAVIGATE_TO_MARKER,                  // In this state the robot navigates to the marker
    ALLIGN_TO_MARKER,                    // In this state the robot alligns itself to the marker
    DROP_OFF,                            // In this state the robot drops off the poop
    ROTATE_AWAY_FROM_MARKER,             // In this state the robot rotates away from the marker
    AI_SETUP,                            // In this state the robot sets up the AI (Starting the AI thread)
    MARKER_SETUP,                        // In this state the robot sets up the marker system (Starting the marker thread)
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
    void waitForPickupVerification();
    void moveBackToPositionBeforePickup();
    void searchForMarker();
    void navigateToMarker();
    void allignToMarker();
    void dropOff();
    void rotateAwayFromMarker();
    void aiSetup();
    void markerSetup();

    // === Helper Functions ===
    std::vector<float> getRobotPosition(); // Function uses the serial helper to get the robot position

    void captureAndPreProcessImage(); // Function captures an image from the camera and pre-processes it

    void updateRobotPosition(); // Function updates the robot position using the controller

    std::vector<std::vector<float>> getBoundingBoxesAndDraw(); // Function gets bounding boxes from the YOLO model and draws them on the image

    std::vector<float> getDistanceMeasurements(); // Function gets distance measurements from the onboard sensors
    bool isThereFreeSpaceForPickup();             // Function checks if there is free space for the robot to pickup the poop
    bool canMoveForwards();                       // Function checks if the robot can move forwards

    void openBucket(); // Function opens the bucket for 5 seconds then closes it

    void updateSystemStateString(); // Function updates the system state string

    void delay(int ms);

    int image_counter;

    // === Storage for Captured Images ===
    float *input_image;
    float *host_image;

    cv::Mat frame;
    cv::Mat resized_frame;
    std::vector<cv::Mat> channels;

    // Threading-related members
    std::thread aiThread;
    std::thread markerThread;
    std::atomic<bool> aiThreadRunning;
    std::atomic<bool> markerThreadRunning;
    std::atomic<bool> poopDetected;
    std::atomic<bool> markerDetected;
    std::atomic<bool> newDetectionAvailable;
    std::atomic<bool> newMarkerAvailable;
    std::mutex dataMutex;
    cv::Mat latestFrame;
    std::vector<std::vector<float>> detectedBboxes;
    std::tuple<std::vector<double>, std::vector<double>> detectedMarker;

    // Threading functions
    void aiProcessingLoop();
    void markerLoop();
    cv::Mat preprocessFrame(const cv::Mat &frame);
    cv::Mat preprocessFrameForMarker(const cv::Mat &frame);

    // === Variables ===
    std::vector<float> robotPosition;
    std::vector<float> robotPositionBeforePickup;
    std::vector<float> distanceMeasurements;

    int numPoopsCollected;

    std::string stateString;

    // === Objects ===

    // Marker System
    markerSystem markerSystem;

    // Navigation algorithm object
    navigationSystem navigation;

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
#include "robotController.cuh"

void printVector(const std::vector<double> &vec)
{
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        std::cout << vec[i];
        if (i < vec.size() - 1)
        {
            std::cout << ", "; // Add comma for all elements except the last one
        }
    }
    std::cout << "]";
}

robotController::robotController() : aiHelper(),
                                     yolo(MODEL_PATH),
                                     visualServoing(IMAGE_HEIGHT, IMAGE_WIDTH),
                                     positionController(C_KP, C_KA, C_GOAL_TOLERANCE, C_ANGLE_TOLERANCE),
                                     serial(USB_CONNECTION, BAUD_RATE),
                                     cap(IMAGE_CAPTURE_DEVICE)
{
    // Set robot to idle state
    this->robotState = RobotState::IDLE;

    // Alocate device memory for the input image (3 channels, 448x448)
    cudaMalloc(&this->input_image, 3 * 448 * 448 * sizeof(float));

    // Allocate host memory for the input image (use standard malloc or new)
    this->host_image = static_cast<float *>(malloc(3 * 448 * 448 * sizeof(float)));

    // Alocate memory for channels
    this->channels = std::vector<cv::Mat>(3);

    // Reset the position of the robot
    this->serial.resetPosition();

    // Set the camera angle
    this->serial.setCameraAngle(160);

    this->robotPosition = {0, 0, 0};

    this->robotPositionBeforePickup = {0, 0, 0};

    // Set the goal position
    this->positionController.setGoal(0, 0, 0);

    // Check if the webcam opened successfully
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the webcam" << std::endl;
        throw std::runtime_error("Error: Could not open the webcam");
    }

    // Set camera parameters
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Create a window to display the results
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);

    this->image_counter = 0;

    this->distanceMeasurements = {0, 0, 0, 0, 0};

    this->numPoopsCollected = 0;

    navigation.setPositionController(&this->positionController);

    // Initialize atomic flags
    aiThreadRunning.store(false);
    markerThreadRunning.store(false);
    poopDetected.store(false);

    delay(1000);
}

robotController::~robotController()
{
    // Free device memory
    cudaFree(this->input_image);

    // Free host memory
    free(this->host_image);

    // Signal the AI thread to stop
    aiThreadRunning = false;

    // Wait for the AI thread to finish
    if (aiThread.joinable())
    {
        aiThread.join();
    }
}

void robotController::update()
{
    // === START: Area to put code that should run every loop iteration ===
    // get the current robot position
    this->robotPosition = getRobotPosition();

    // get the distance measurements
    this->distanceMeasurements = getDistanceMeasurements();
    // === END: Area to put code that should run every loop iteration ===

    switch (this->robotState)
    {
    case RobotState::IDLE:
        idle();
        break;
    case RobotState::MOVE_AND_DETECT:
        moveAndDetect();
        break;
    case RobotState::DETECTION_ALLIGNMENT:
        detectionAllignment();
        break;
    case RobotState::PICKUP:
        pickup();
        break;
    case RobotState::WAIT_FOR_PICKUP_VERIFICATION:
        waitForPickupVerification();
        break;
    case RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP:
        moveBackToPositionBeforePickup();
        break;
    case RobotState::SEARCH_FOR_MARKER:
        searchForMarker();
        break;
    case RobotState::NAVIGATE_TO_MARKER:
        navigateToMarker();
        break;
    case RobotState::ALLIGN_TO_MARKER:
        allignToMarker();
        break;
    case RobotState::DROP_OFF:
        dropOff();
        break;
    case RobotState::ROTATE_AWAY_FROM_MARKER:
        rotateAwayFromMarker();
        break;
    case RobotState::AI_SETUP:
        aiSetup();
        break;
    case RobotState::MARKER_SETUP:
        markerSetup();
        break;
    case RobotState::MOVE_TO_DROP_POSITION:
        moveToDropPosition();
        break;
    case RobotState::ROTATE_FOR_TRANSLATION:
        rotateForTranslation();
        break;
    case RobotState::MARKER_TRANSLATION:
        markerTranslation();
        break;
    case RobotState::ROTATE_TO_FACE_MARKER:
        rotateToFaceMarker();
        break;
    default:
        break;
    }

    // START: Area to put code that should run after every loop iteration
    // Fetch the latest frame and detection results
    cv::Mat displayFrame;
    std::vector<std::vector<float>> bboxes;
    std::tuple<std::vector<double>, std::vector<double>> markerVectors;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (!latestFrame.empty())
        {
            displayFrame = latestFrame.clone();
            bboxes = detectedBboxes;
            markerVectors = detectedMarker;
        }
    }

    // Draw bounding boxes and other info
    if (!displayFrame.empty())
    {
        displayFrame = aiHelper.drawBoundingBoxes(displayFrame, bboxes);
        aiHelperUtils::drawSensorReadingsOnFrame(displayFrame, this->distanceMeasurements);

        // Update the system state string
        updateSystemStateString();
        aiHelperUtils::drawSystemStateOnFrame(displayFrame, this->stateString);

        // Display the frame
        cv::imshow("Detection", displayFrame);
    }

    // Print the detected marker vectors
    // if (!std::get<0>(markerVectors).empty())
    // {
    //     std::cout << "Translation: ";
    //     printVector(std::get<0>(markerVectors));
    //     std::cout << std::endl;

    //     // Printing Euler Angles Vector
    //     std::cout << "Euler Angles: ";
    //     printVector(std::get<1>(markerVectors));
    //     std::cout << std::endl;
    // }

    // Handle keyboard input or other events
    char key = cv::waitKey(1);
    if (key == 'c')
    {
        // Save the current image
        std::string filename = "cal/captured_image_" + std::to_string(this->image_counter) + ".png";
        if (cv::imwrite(filename, displayFrame))
        {
            std::cout << "Image saved: " << filename << std::endl;
            image_counter++;
        }
        else
        {
            std::cerr << "Error: Could not save image" << std::endl;
        }
    }
    else if (key == 'o')
    {
        // Open the bucket
        std::cout << "Bucket opened" << std::endl;
        openBucket(); // opens the bucket, waits for 5 seconds then closes it
        std::cout << "Bucket closed" << std::endl;
    }
    else if (key == 's')
    {
        // Start the robot
        this->setRobotState(RobotState::MOVE_AND_DETECT);
    }

    // END: Area to put code that should run after every loop iteration
}

void robotController::setRobotState(RobotState state)
{
    this->robotState = state;
}

void robotController::idle()
{
    // Set robot speed to zero continuously in the idle state
    this->serial.sendSpeeds(0, 0, 0, 0);

    // Delay to prevent spamming the serial connection
    this->delay(DELAY_TIME);

    // === TRANSITION ===
    if (cv::waitKey(1) == 's')
    {
        // Set the robot state to move and detect
        this->setRobotState(RobotState::MOVE_AND_DETECT);
    }
}

void robotController::moveAndDetect()
{
    // If the position controller is rotating the robot, skip AI detection
    State pcState = positionController.getState();
    if (pcState == State::ROTATE_TO_GOAL || pcState == State::ROTATE_TO_GOAL_ORIENTATION)
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
        return;
    }

    // Check if poop has been detected by the AI thread
    if (poopDetected.load())
    {
        // Transition to the detection alignment state
        this->setRobotState(RobotState::DETECTION_ALLIGNMENT);

        // Set the goal to the current position
        this->updateRobotPosition();
        // Store the current robot position before the pickup
        this->robotPositionBeforePickup = this->robotPosition;

        this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], this->robotPosition[2]);
        this->positionController.setState(State::IDLE);
        return;
    }

    // if the robot is still moving dont get a new goal position
    if (pcState == State::MOVE_TO_GOAL)
    {
        this->updateRobotPosition();
        // Check if an obstacle has been detected, if so we need to call the explore function to get a new setpoint
        if (!canMoveForwards())
        {
            std::vector<float> goalPosition = this->navigation.explore(this->robotPosition, this->distanceMeasurements);
            this->positionController.setGoal(goalPosition[0], goalPosition[1], goalPosition[2]);
            this->positionController.setState(State::ROTATE_TO_GOAL_ORIENTATION);
        }

        this->delay(DELAY_TIME);
        return;
    }

    this->updateRobotPosition();

    // No poop detected, proceed with navigation
    std::vector<float> goalPosition = this->navigation.explore(this->robotPosition, this->distanceMeasurements);
    this->positionController.setGoal(goalPosition[0], goalPosition[1], goalPosition[2]);

    delay(DELAY_TIME);
}

void robotController::detectionAllignment()
{
    // Check if the position controller is busy with a rotation
    if (positionController.getState() != State::ROTATE_TO_GOAL &&
        positionController.getState() != State::ROTATE_TO_GOAL_ORIENTATION &&
        positionController.getState() != State::MOVE_TO_GOAL)
    {
        // Check if new AI detection results are available
        bool detectionAvailable = false;
        std::vector<std::vector<float>> bboxes;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            detectionAvailable = newDetectionAvailable; // A flag set by the AI thread
            if (detectionAvailable)
            {
                bboxes = detectedBboxes;
                newDetectionAvailable = false; // Reset the flag
            }
        }

        if (!detectionAvailable)
        {
            // No new detection results yet, wait briefly
            this->delay(DELAY_TIME);
            this->updateRobotPosition();
            return;
        }

        if (!bboxes.empty())
        {
            std::vector<float> bbox = aiHelperUtils::getBoindingBoxWithLargestArea(bboxes);

            // Use the visual servoing algorithm to compute the updated desired robot position and orientation
            std::vector<float> updatedPosition = this->visualServoing.calculateControlPosition(bbox, this->robotPosition, this->positionController);

            if (this->visualServoing.getCurrentState() == servoingState::STOP)
            {
                this->visualServoing.resetState();

                // Move to the pickup state
                this->setRobotState(RobotState::PICKUP);
                return;
            }

            // Set the setpoint for the position controller
            this->positionController.setGoal(updatedPosition[0], updatedPosition[1], updatedPosition[2]);
        }
        else
        {
            // Go back to search pattern, it seems we have lost the poop
            std::cout << "Lost poop, going back to search pattern" << std::endl;
            // Set the goal position to the position before pickup
            this->positionController.setGoal(this->robotPositionBeforePickup[0],
                                             this->robotPositionBeforePickup[1],
                                             this->robotPositionBeforePickup[2]);

            this->setRobotState(RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP);
        }

        // Update the robot position
        this->updateRobotPosition();
    }
    else
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
    }
}

void robotController::pickup()
{
    // Check if there is free space for the pickup
    if (!isThereFreeSpaceForPickup())
    {
        // Set the goal position to the position before pickup
        this->positionController.setGoal(this->robotPositionBeforePickup[0], this->robotPositionBeforePickup[1], this->robotPositionBeforePickup[2]);
        this->setRobotState(RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP);
        return;
    }

    std::cout << "Picking up poop" << std::endl;
    this->serial.requestAndWaitForPoopPickup();

    this->delay(100);
    serial.requestAndWaitForArmPosition(STEPPER_2, CLOCKWISE, 35);
    this->delay(100);

    serial.requestAndWaitForArmPosition(STEPPER_3, CLOCKWISE, 150);

    // Move the robot slightly forward linearly
    this->delay(100);
    this->serial.sendSpeeds(0.2, 0.2, 0.2, 0.2);
    this->delay(700);
    this->serial.sendSpeeds(0, 0, 0, 0);
    this->delay(100);

    this->delay(100);
    this->serial.sendSpeeds(0.1, 0.1, 0.1, 0.1);
    this->delay(300);
    this->serial.sendSpeeds(0, 0, 0, 0);
    this->delay(100);

    this->delay(300);
    serial.requestAndWaitForArmPosition(STEPPER_2, CLOCKWISE, 5);
    this->delay(300);

    serial.requestAndWaitForArmPosition(STEPPER_3, COUNTERCLOCKWISE, 40);
    this->delay(100);

    serial.requestAndWaitForArmPosition(STEPPER_3, COUNTERCLOCKWISE, 55);
    this->delay(100);

    serial.requestAndWaitForArmPosition(STEPPER_2, CLOCKWISE, 35);
    this->delay(100);

    // Get the current robot position
    std::vector<float> position = this->getRobotPosition();
    this->delay(100);

    // Set the goal position to the current position
    this->positionController.setGoal(position[0], position[1], position[2]);

    this->serial.requestAndWaitForPickupLift();

    this->setRobotState(RobotState::WAIT_FOR_PICKUP_VERIFICATION);
}

void robotController::waitForPickupVerification()
{
    // Check if new AI detection results are available
    bool detectionAvailable = false;
    std::vector<std::vector<float>> bboxes;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        detectionAvailable = newDetectionAvailable; // A flag set by the AI thread
        if (detectionAvailable)
        {
            bboxes = detectedBboxes;
            newDetectionAvailable = false; // Reset the flag
        }
    }

    if (detectionAvailable)
    {
        // Update distance measurements
        this->distanceMeasurements = getDistanceMeasurements();

        // Check if there are still poop in the frame and if space permits
        if (!bboxes.empty() && isThereFreeSpaceForPickup())
        {
            // Transition back to DETECTION_ALLIGNMENT state
            this->setRobotState(RobotState::DETECTION_ALLIGNMENT);
        }
        else
        {
            // Set goal position to the position before pickup
            this->positionController.setGoal(
                this->robotPositionBeforePickup[0],
                this->robotPositionBeforePickup[1],
                this->robotPositionBeforePickup[2]);
            this->setRobotState(RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP);

            // Increase the number of poops picked up
            this->numPoopsCollected++;
        }
    }
    else
    {
        // No new detection results yet, wait briefly
        this->delay(10);
    }
}

void robotController::moveBackToPositionBeforePickup()
{
    // Move the robot back to the position before the pickup
    // The setpoint for the position controller is already set
    if (this->positionController.getState() == State::IDLE)
    {
        // disable reverse mode
        this->positionController.setReverseMode(false);

        // Set the robot state to move and detect if the maximum number of poops have not been collected
        if (this->numPoopsCollected < MAX_POOP)
        {
            this->setRobotState(RobotState::MOVE_AND_DETECT);
        }
        else
        {
            // Set the robot state to marker setup, it will then search for the marker
            this->setRobotState(RobotState::MARKER_SETUP);
        }
    }
    else
    {
        // enable reverse mode
        this->positionController.setReverseMode(true);

        // Update the robot position
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
    }
}

// This state will be very similar to the move and detect state
// The robot will navigate using the navigation algorithm to cover an area
// The Robot will not detect poop but rather search for the marker
// Once the marker is found it will transition to the navigate to marker state
void robotController::searchForMarker()
{
    // If the position controller is rotating the robot, skip marker detection
    State pcState = positionController.getState();
    if (pcState == State::ROTATE_TO_GOAL || pcState == State::ROTATE_TO_GOAL_ORIENTATION)
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
        return;
    }

    // Check if marker has been detected by the marker thread
    if (markerDetected.load())
    {
        // Transition to the navigate to marker state
        this->setRobotState(RobotState::NAVIGATE_TO_MARKER);

        // Set the goal to the current position
        this->updateRobotPosition();

        this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], this->robotPosition[2]);
        this->positionController.setState(State::IDLE);
        return;
    }

    // if the robot is still moving dont get a new goal position
    if (pcState == State::MOVE_TO_GOAL)
    {
        this->updateRobotPosition();
        // Check if an obstacle has been detected, if so we need to call the explore function to get a new setpoint
        if (!canMoveForwards())
        {
            std::vector<float> goalPosition = this->navigation.explore(this->robotPosition, this->distanceMeasurements);
            this->positionController.setGoal(goalPosition[0], goalPosition[1], goalPosition[2]);
            this->positionController.setState(State::ROTATE_TO_GOAL_ORIENTATION);
        }

        this->delay(DELAY_TIME);
        return;
    }

    this->updateRobotPosition();

    // No marker detected, proceed with navigation
    std::vector<float> goalPosition = this->navigation.explore(this->robotPosition, this->distanceMeasurements);
    this->positionController.setGoal(goalPosition[0], goalPosition[1], goalPosition[2]);

    delay(DELAY_TIME);
}

// This state will be similar to the detection allignment state for the AI
// The robot will allign itself to the marker
void robotController::navigateToMarker()
{
    // Check if the position controller is busy with a rotation
    if (positionController.getState() != State::ROTATE_TO_GOAL &&
        positionController.getState() != State::ROTATE_TO_GOAL_ORIENTATION &&
        positionController.getState() != State::MOVE_TO_GOAL)
    {
        // Check if new marker detection results are available
        bool detectionAvailable = false;
        std::tuple<std::vector<double>, std::vector<double>> markerVectors;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            detectionAvailable = newMarkerAvailable; // A flag set by the marker thread
            if (detectionAvailable)
            {
                markerVectors = detectedMarker;
                newMarkerAvailable = false; // Reset the flag
            }
        }

        if (!detectionAvailable)
        {
            // No new detection results yet, wait briefly
            this->delay(DELAY_TIME);
            this->updateRobotPosition();
            return;
        }

        if (!std::get<0>(markerVectors).empty())
        {
            // Get the distance from the marker, if the distance is greater than 1 meter, continue moving forwards

            float distance = std::get<0>(markerVectors)[2];
            std::cout << "Distance to marker: " << distance << std::endl;
            // calculate the distance to move forwards
            float distanceToMove = distance - 1.0f;
            if (distance > 0.5f && distanceToMove > 0.3)
            {
                // Move the robot forwards in it current orientation
                this->updateRobotPosition();
                std::vector<float> newRobotPosition = this->robotPosition;
                newRobotPosition[0] += distanceToMove * std::cos(newRobotPosition[2]);
                newRobotPosition[1] += distanceToMove * std::sin(newRobotPosition[2]);
                this->positionController.setGoal(newRobotPosition[0], newRobotPosition[1], newRobotPosition[2]);
                this->delay(DELAY_TIME);
                this->updateRobotPosition();
                return;
            }
            else
            {
                // We are close to the marker, transition to the allign to marker state
                this->setRobotState(RobotState::ALLIGN_TO_MARKER);
                this->updateRobotPosition();
                this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], this->robotPosition[2]);
                this->updateRobotPosition();
                return;
            }
        }
        else
        {
            // Go back to search pattern, it seems we have lost the marker
            std::cout << "Lost marker, going back to search pattern" << std::endl;
            // Set the goal position to the position before pickup
            this->updateRobotPosition();
            this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], this->robotPosition[2]);
            this->updateRobotPosition();

            this->setRobotState(RobotState::SEARCH_FOR_MARKER);
        }
    }
    else
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
    }
}

void robotController::allignToMarker()
{
    // 1. Get the lastest marker detection
    // Check if new marker detection results are available
    bool detectionAvailable = false;
    std::tuple<std::vector<double>, std::vector<double>> markerVectors;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        detectionAvailable = newMarkerAvailable; // A flag set by the marker thread
        if (detectionAvailable)
        {
            markerVectors = detectedMarker;
            newMarkerAvailable = false; // Reset the flag
        }
    }

    if (!detectionAvailable)
    {
        // No new detection results yet, wait briefly
        this->delay(DELAY_TIME);
        this->updateRobotPosition();
        return;
    }

    if (std::get<0>(markerVectors).empty())
    {
        this->delay(DELAY_TIME);
        this->updateRobotPosition();
        return;
    }

    // 2. Check if we are within allignment
    //    -- The yaw axes of the marker must be within say 10 degrees with respect to the robot
    //    -- If we are in allignment, we go to a state where we move the robot forwards untill we can drop off
    //    -- 10 degrees in radians is 0.1745
    float yaw = 0;
    if (!std::get<0>(markerVectors).empty())
    {
        // Get the yaw
        yaw = std::get<1>(markerVectors)[2];
    }

    if (yaw < 0.1745 && yaw > -0.1745)
    {
        // Transition to the drop off state
        this->setRobotState(RobotState::MOVE_TO_DROP_POSITION);
        return;
    }

    this->calculatedYaw = yaw;
    this->distanceFromMarker = std::get<0>(markerVectors)[2];
    this->distanceToTranslate = this->distanceFromMarker * std::tan(yaw);

    // 3. If we are not in allignment, we have to translate the robot horozontally
    //    -- Translate by distanceFromMarker * tan(yaw)
    //    -- after translation rotate by 90 - yaw degrees
    // Translation direction is dependent on the sign of yaw
    // Rotation direction is also dependent on sign of yaw
    // Positive: left ccw
    // Negative: right cw

    // Transition to the rotate for translation state
    this->setRobotState(RobotState::ROTATE_FOR_TRANSLATION);
}

void robotController::dropOff()
{
    delay(100);
    serial.requestAndWaitForDropoff();
    delay(100);

    // Reset the number of poops picked up
    this->numPoopsCollected = 0;

    // Set the robot to reverse for 0.4 meters linearly and do a 180 degree rotation
    float new_x = this->robotPosition[0] - 0.4 * std::cos(this->robotPosition[2]);
    float new_y = this->robotPosition[1] - 0.4 * std::sin(this->robotPosition[2]);
    this->positionController.setGoal(new_x, new_y, this->robotPosition[2] + M_PI);

    // enable reverse mode
    this->positionController.setReverseMode(true);

    // Transition to the rotate away from marker state
    this->setRobotState(RobotState::ROTATE_AWAY_FROM_MARKER);
}

void robotController::rotateAwayFromMarker()
{
    // If the position controller is in the idle state, transition to the move and detect state
    if (this->positionController.getState() == State::IDLE)
    {
        // disable reverse mode
        this->positionController.setReverseMode(false);

        // reset the robot position
        this->serial.resetPosition();

        // Set the robot state to move and detect
        this->setRobotState(RobotState::MOVE_AND_DETECT);
    }
    else
    {
        // Update the robot position
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
    }
}

// This state simply sets the AI thread to run and the marker thread to stop
// It then transitions to the move and detect state
void robotController::aiSetup()
{
    // Set the camera angle to 160 degrees
    this->serial.setCameraAngle(160);

    // Stop the marker processing thread if it is running
    if (markerThreadRunning.load())
    {
        markerThreadRunning.store(false);
        if (markerThread.joinable())
        {
            markerThread.join(); // Wait for the marker thread to finish
        }
    }

    // Start the AI processing thread
    aiThreadRunning.store(true);
    aiThread = std::thread(&robotController::aiProcessingLoop, this);

    // Set the robot state to move and detect
    this->setRobotState(RobotState::MOVE_AND_DETECT);
}

// This state simply sets the marker thread to run and the AI thread to stop
// It then transitions to the search for marker state
void robotController::markerSetup()
{
    // Set the camera angle to 10 degrees
    delay(100);
    this->serial.setCameraAngle(-10);
    delay(100);

    // Stop the AI processing thread if it is running
    if (aiThreadRunning.load())
    {
        aiThreadRunning.store(false);
        if (aiThread.joinable())
        {
            aiThread.join(); // Wait for the AI thread to finish
        }
    }

    // Start the marker processing thread
    markerThreadRunning.store(true);
    markerThread = std::thread(&robotController::markerLoop, this);

    // Set the robot state to search for marker
    this->setRobotState(RobotState::SEARCH_FOR_MARKER);
}

void robotController::moveToDropPosition()
{
    if (this->positionController.getState() == State::MOVE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL_ORIENTATION)
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
        return;
    }
}

void robotController::rotateForTranslation()
{
    std::cout << "Rotating for translation" << std::endl;
    // Check the sign of the yaw
    // Rotate the robot in the direction of the sign of the yaw
    if (this->calculatedYaw > 0)
    {
        float newAngle = this->robotPosition[2] - (M_PI / 2) - this->calculatedYaw;
        // Normalize the angle
        newAngle = fmod(newAngle + M_PI, 2 * M_PI) - M_PI;

        this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], newAngle);
    }
    else
    {
        float newAngle = this->robotPosition[2] + (M_PI / 2) - this->calculatedYaw;
        // Normalize the angle
        newAngle = fmod(newAngle + M_PI, 2 * M_PI) - M_PI;

        this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], newAngle);
    }

    // Transition to the marker translation state
    this->setRobotState(RobotState::MARKER_TRANSLATION);

    this->delay(DELAY_TIME);
    this->updateRobotPosition();
}

void robotController::markerTranslation()
{
    std::cout << "Distance to translate: " << this->distanceToTranslate << std::endl;
    // While the robot is moving to the goal or rotating, just update the robot position
    if (this->positionController.getState() == State::MOVE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL_ORIENTATION)
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
        return;
    }

    this->updateRobotPosition();

    // Move the robot forwards by the distance calculated in the direction that the robot is facing
    float new_x = this->robotPosition[0] + fabs(this->distanceToTranslate) * std::cos(this->robotPosition[2]);
    float new_y = this->robotPosition[1] + fabs(this->distanceToTranslate) * std::sin(this->robotPosition[2]);

    this->positionController.setGoal(new_x, new_y, this->robotPosition[2]);

    // Transition to the rotate to face marker state
    this->setRobotState(RobotState::ROTATE_TO_FACE_MARKER);

    this->delay(DELAY_TIME);
    this->updateRobotPosition();
}

void robotController::rotateToFaceMarker()
{
    std::cout << "Rotating to face marker" << std::endl;
    // While the robot is moving to the goal or rotating, just update the robot position
    if (this->positionController.getState() == State::MOVE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL || this->positionController.getState() == State::ROTATE_TO_GOAL_ORIENTATION)
    {
        this->updateRobotPosition();
        this->delay(DELAY_TIME);
        return;
    }

    this->updateRobotPosition();

    // Rotate the robot to face the marker
    // 90 - yaw
    float newAngle = this->robotPosition[2] - (M_PI / 2);

    // Normalize the angle, between -pi and pi
    newAngle = fmod(newAngle + M_PI, 2 * M_PI) - M_PI;

    this->positionController.setGoal(this->robotPosition[0], this->robotPosition[1], newAngle);

    // Transition to the move to drop position state
    this->setRobotState(RobotState::MOVE_TO_DROP_POSITION);

    this->delay(DELAY_TIME);
    this->updateRobotPosition();
}

std::vector<float> robotController::getRobotPosition()
{
    return serial.receivePosition();
}

void robotController::captureAndPreProcessImage()
{
    // Capture an image from the camera
    this->cap >> this->frame;

    // Check if the frame is empty
    if (frame.empty())
    {
        std::cerr << "Error: Captured empty frame" << std::endl;
        throw std::runtime_error("Error: Captured empty frame");
    }

    // Resize the image
    cv::resize(this->frame, this->resized_frame, cv::Size(448, 448));

    // Resize the image to 448x448 (input size for YOLOv1)
    cv::resize(this->frame, this->resized_frame, cv::Size(448, 448));

    // Convert the image from BGR to RGB
    cv::cvtColor(this->resized_frame, this->resized_frame, cv::COLOR_BGR2RGB);

    // Convert image to float and normalize
    resized_frame.convertTo(this->resized_frame, CV_32F, 1.0 / 255.0);

    // Split channels
    cv::split(resized_frame, this->channels);

    // Copy the data from the OpenCV Mat to the host memory (channels first format)
    for (int c = 0; c < 3; ++c)
    {
        for (int h = 0; h < 448; ++h)
        {
            for (int w = 0; w < 448; ++w)
            {
                this->host_image[c * 448 * 448 + h * 448 + w] = this->channels[c].at<float>(h, w);
            }
        }
    }
    // Transfer the data from host memory to the GPU memory (device)
    cudaMemcpy(this->input_image, this->host_image, 3 * 448 * 448 * sizeof(float), cudaMemcpyHostToDevice);
}

void robotController::updateRobotPosition()
{
    // Update the robot position
    std::vector<float> position = this->robotPosition;
    std::vector<float> velocities = this->positionController.updateVelocities(position[0], position[1], position[2]);
    serial.sendSpeeds(velocities[1], velocities[1], velocities[0], velocities[0]);
}

std::vector<std::vector<float>> robotController::getBoundingBoxesAndDraw()
{
    // Get the bounding boxes
    std::vector<std::vector<float>> bboxes = yolo.getBoxPredictions(this->input_image);
    // Draw the bounding boxes
    cv::cvtColor(this->resized_frame, this->resized_frame, cv::COLOR_RGB2BGR);
    resized_frame = aiHelper.drawBoundingBoxes(resized_frame, bboxes);
    cv::cvtColor(this->resized_frame, this->resized_frame, cv::COLOR_BGR2RGB);
    return bboxes;
}

std::vector<float> robotController::getDistanceMeasurements()
{
    // Measure distance from each sensor with a 5ms delay between each measurement
    std::vector<float> distances;
    distances.push_back(serial.receiveDistanceSensorMeasurement(DistSense::SENSE_1));
    delay(5);
    distances.push_back(serial.receiveDistanceSensorMeasurement(DistSense::SENSE_2));
    delay(5);
    distances.push_back(serial.receiveDistanceSensorMeasurement(DistSense::SENSE_3));
    delay(5);
    distances.push_back(serial.receiveDistanceSensorMeasurement(DistSense::SENSE_4));
    delay(5);
    distances.push_back(serial.receiveDistanceSensorMeasurement(DistSense::SENSE_5));
    delay(5);

    return distances;
}

bool robotController::isThereFreeSpaceForPickup()
{
    bool freeSpace = true;

    // Sensor 1
    if (this->distanceMeasurements[0] < 0.3 && this->distanceMeasurements[0] != -1)
    {
        freeSpace = false;
    }

    // Sensor 2
    if (this->distanceMeasurements[1] < 0.3 && this->distanceMeasurements[1] != -1)
    {
        freeSpace = false;
    }

    // Sensor 3
    if (this->distanceMeasurements[2] < 0.2 && this->distanceMeasurements[2] != -1)
    {
        freeSpace = false;
    }

    // Sensor 4
    if (this->distanceMeasurements[3] < 0.2 && this->distanceMeasurements[3] != -1)
    {
        freeSpace = false;
    }

    // Sensor 5
    if (this->distanceMeasurements[4] < 0.3 && this->distanceMeasurements[4] != -1)
    {
        return false;
    }

    return freeSpace;
}

bool robotController::canMoveForwards()
{
    // If distance sensor 1, 2 or 5 is below 0.3 it is not possible.

    // No detection if measurement is -1

    // Sensor 1
    if (this->distanceMeasurements[0] < 0.30 && this->distanceMeasurements[0] != -1)
    {
        return false;
    }

    // Sensor 2
    if (this->distanceMeasurements[1] < 0.30 && this->distanceMeasurements[1] != -1)
    {
        return false;
    }

    // Senor 3
    if (this->distanceMeasurements[2] < 0.20 && this->distanceMeasurements[2] != -1)
    {
        return false;
    }

    // Sensor 4
    if (this->distanceMeasurements[3] < 0.20 && this->distanceMeasurements[3] != -1)
    {
        return false;
    }

    // Sensor 5
    if (this->distanceMeasurements[4] < 0.30 && this->distanceMeasurements[4] != -1)
    {
        return false;
    }

    return true;
}

void robotController::openBucket()
{
    delay(100);
    serial.requestAndWaitForArmPosition(STEPPER_3, CLOCKWISE, 220);
    delay(5000);
    serial.requestAndWaitForArmPosition(STEPPER_3, COUNTERCLOCKWISE, 220);
    delay(100);
}

void robotController::updateSystemStateString()
{
    std::string mainRobotState;
    switch (this->robotState)
    {
    case RobotState::IDLE:
        mainRobotState = "IDLE";
        break;
    case RobotState::MOVE_AND_DETECT:
        mainRobotState = "MOVE_AND_DETECT";
        break;
    case RobotState::DETECTION_ALLIGNMENT:
        mainRobotState = "DETECTION_ALLIGNMENT";
        break;
    case RobotState::PICKUP:
        mainRobotState = "PICKUP";
        break;
    case RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP:
        mainRobotState = "MOVE_BACK_TO_POSITION_BEFORE_PICKUP";
        break;
    case RobotState::SEARCH_FOR_MARKER:
        mainRobotState = "SEARCH_FOR_MARKER";
        break;
    case RobotState::NAVIGATE_TO_MARKER:
        mainRobotState = "NAVIGATE_TO_MARKER";
        break;
    case RobotState::ALLIGN_TO_MARKER:
        mainRobotState = "ALLIGN_TO_MARKER";
        break;
    case RobotState::DROP_OFF:
        mainRobotState = "DROP_OFF";
        break;
    case RobotState::ROTATE_AWAY_FROM_MARKER:
        mainRobotState = "ROTATE_AWAY_FROM_MARKER";
        break;
    case RobotState::AI_SETUP:
        mainRobotState = "AI_SETUP";
        break;
    case RobotState::MARKER_SETUP:
        mainRobotState = "MARKER_SETUP";
        break;
    case RobotState::MOVE_TO_DROP_POSITION:
        mainRobotState = "MOVE_TO_DROP_POSITION";
        break;
    case RobotState::ROTATE_FOR_TRANSLATION:
        mainRobotState = "ROTATE_FOR_TRANSLATION";
        break;
    case RobotState::MARKER_TRANSLATION:
        mainRobotState = "MARKER_TRANSLATION";
        break;
    case RobotState::ROTATE_TO_FACE_MARKER:
        mainRobotState = "ROTATE_TO_FACE_MARKER";
        break;
    default:
        mainRobotState = "UNKNOWN";
        break;
    }

    std::string positionControllerState;
    switch (this->positionController.getState())
    {
    case State::IDLE:
        positionControllerState = "IDLE";
        break;
    case State::MOVE_TO_GOAL:
        positionControllerState = "MOVE_TO_GOAL";
        break;
    case State::ROTATE_TO_GOAL:
        positionControllerState = "ROTATE_TO_GOAL";
        break;
    case State::ROTATE_TO_GOAL_ORIENTATION:
        positionControllerState = "ROTATE_TO_GOAL_ORIENTATION";
        break;
    default:
        positionControllerState = "UNKNOWN";
        break;
    }

    std::string visualServoingState;
    switch (this->visualServoing.getCurrentState())
    {
    case servoingState::STOP:
        visualServoingState = "STOP";
        break;
    case servoingState::ROTATE:
        visualServoingState = "ROTATE";
        break;
    case servoingState::MOVE_FORWARD:
        visualServoingState = "MOVE_FORWARD";
        break;
    default:
        visualServoingState = "UNKNOWN";
        break;
    }

    this->stateString = "RS: " + mainRobotState + " | PC: " + positionControllerState + " | VS: " + visualServoingState;
}

void robotController::delay(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// This is a thread function that runs the AI processing loop
// It runs in a separate thread to prevent blocking the main thread as the AI is slow
void robotController::aiProcessingLoop()
{
    while (aiThreadRunning.load())
    {
        // Capture an image from the camera
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            cap >> frame; // Ensure exclusive access if cap is used in multiple threads
        }

        if (frame.empty())
        {
            std::cerr << "Error: Captured empty frame" << std::endl;
            continue;
        }

        // Preprocess the image
        cv::Mat preprocessedFrame = preprocessFrame(frame);

        // Run AI detection
        auto bboxes = yolo.getBoxPredictions(this->input_image);

        // Update shared variables safely
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            this->latestFrame = preprocessedFrame.clone(); // Store the original frame
            this->detectedBboxes = bboxes;
            this->poopDetected = !bboxes.empty();
            newDetectionAvailable = true;
        }

        // Sleep briefly to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Sleep briefly to prevent high CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

cv::Mat robotController::preprocessFrame(const cv::Mat &frame)
{
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, cv::Size(448, 448));
    cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2RGB);
    resizedFrame.convertTo(resizedFrame, CV_32F, 1.0 / 255.0);
    cv::split(resizedFrame, this->channels);

    // Copy the data to the host_image
    for (int c = 0; c < 3; ++c)
    {
        for (int h = 0; h < 448; ++h)
        {
            for (int w = 0; w < 448; ++w)
            {
                this->host_image[c * 448 * 448 + h * 448 + w] = this->channels[c].at<float>(h, w);
            }
        }
    }

    // Transfer the data from host to device
    cudaMemcpy(this->input_image, this->host_image, 3 * 448 * 448 * sizeof(float), cudaMemcpyHostToDevice);

    cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_RGB2BGR);

    return resizedFrame;
}

cv::Mat robotController::preprocessFrameForMarker(const cv::Mat &frame)
{
    cv::Mat resizedFrame;

    // Step 1: Resize the frame to 448x448
    cv::resize(frame, resizedFrame, cv::Size(448, 448));

    // Step 2: Convert the frame from BGR to RGB
    cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2RGB);

    // Step 3: Convert to float and normalize to [0, 1]
    resizedFrame.convertTo(resizedFrame, CV_32F, 1.0 / 255.0);

    // Step 4: Copy the data to the host_image array in row-major order
    std::memcpy(this->host_image, resizedFrame.ptr<float>(), 3 * 448 * 448 * sizeof(float));

    // Step 5: Transfer the data from host to device (GPU)
    cudaMemcpy(this->input_image, this->host_image, 3 * 448 * 448 * sizeof(float), cudaMemcpyHostToDevice);

    // Step 6: Convert back to BGR for display if needed
    cv::cvtColor(resizedFrame, resizedFrame, cv::COLOR_RGB2BGR);

    // Return the preprocessed frame for potential display
    return resizedFrame;
}

void robotController::markerLoop()
{
    while (markerThreadRunning.load())
    {
        // Capture an image from the camera
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            cap >> frame; // Ensure exclusive access if cap is used in multiple threads
        }

        if (frame.empty())
        {
            std::cerr << "Error: Captured empty frame" << std::endl;
            continue;
        }

        // Preprocess the image
        cv::Mat preprocessedFrame = preprocessFrameForMarker(frame);

        // Preform marker detection
        std::tuple<std::vector<double>, std::vector<double>> result = this->markerSystem.detectMarkers(this->input_image);
        // Check if the marker was detected, do this by checking if the vectors are empty
        if (!std::get<0>(result).empty() && !std::get<1>(result).empty())
        {
            // Update shared variables safely
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                this->latestFrame = preprocessedFrame.clone(); // Store the original frame
                this->markerDetected = true;
                this->newMarkerAvailable = true;
                this->detectedMarker = result;
            }
        }
        else
        {
            // Update shared variables safely
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                this->latestFrame = preprocessedFrame.clone(); // Store the original frame
                // Set empty vectors to indicate no marker was detected
                this->detectedMarker = std::make_tuple(std::vector<double>(), std::vector<double>());
                this->markerDetected = false;
            }
        }

        // Sleep briefly to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Sleep briefly to prevent high CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

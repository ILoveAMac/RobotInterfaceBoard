#include "robotController.cuh"

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
    aiThreadRunning = true;
    poopDetected = false;

    // Start the AI processing thread
    aiThread = std::thread(&robotController::aiProcessingLoop, this);
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
    default:
        break;
    }

    // START: Area to put code that should run after every loop iteration
    // Fetch the latest frame and detection results
    cv::Mat displayFrame;
    std::vector<std::vector<float>> bboxes;
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (!latestFrame.empty())
        {
            displayFrame = latestFrame.clone();
            bboxes = detectedBboxes;
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

    // Update the robot position
    // this->updateRobotPosition();
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

        // Set the robot state to move and detect
        this->setRobotState(RobotState::MOVE_AND_DETECT);
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

void robotController::searchForMarker()
{
    // TODO! Implement a search algorithm to find the marker
}

void robotController::navigateToMarker()
{
    // TODO! Once the marker has been detected, navigate towards it
}

void robotController::allignToMarker()
{
    // TODO! Once alligned to the marker, transition to the drop off state
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

    // Sensor 5 is not used for this check, as the poop may trigger the sensor

    return freeSpace;
}

bool robotController::canMoveForwards()
{
    // If distance sensor 1, 2 or 5 is below 0.3 it is not possible.
    // if distance sensor 3 or 4 are below 0.2 it is not possible

    // No detection if measurement is -1

    // Sensor 1
    if (this->distanceMeasurements[0] < 0.3 && this->distanceMeasurements[0] != -1)
    {
        return false;
    }

    // Sensor 2
    if (this->distanceMeasurements[1] < 0.3 && this->distanceMeasurements[1] != -1)
    {
        return false;
    }

    // Sensor 3
    if (this->distanceMeasurements[2] < 0.2 && this->distanceMeasurements[2] != -1)
    {
        return false;
    }

    // Sensor 4
    if (this->distanceMeasurements[3] < 0.2 && this->distanceMeasurements[3] != -1)
    {
        return false;
    }

    // Sensor 5
    // if (this->distanceMeasurements[4] < 0.3 && this->distanceMeasurements[4] != -1)
    // {
    //     return false;
    // }

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
    while (aiThreadRunning)
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

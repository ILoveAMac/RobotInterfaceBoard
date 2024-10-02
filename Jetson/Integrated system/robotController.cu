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

    robotPosition = {0, 0, 0};

    // Set the goal position
    this->positionController.setGoal(0, 0, 0);

    // set visual servoing setpoint
    this->visualServoing.setTargetDist(0.2f); // target distance from the poop

    // Check if the webcam opened successfully
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the webcam" << std::endl;
        throw std::runtime_error("Error: Could not open the webcam");
    }

    // Set camera parameters
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // Create a window to display the results
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);
}

robotController::~robotController()
{
    // Free device memory
    cudaFree(this->input_image);

    // Free host memory
    free(this->host_image);
}

void robotController::update()
{
    // === START: Area to put code that should run every loop iteration ===
    // get the current robot position
    this->robotPosition = this->getRobotPosition();
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
    // TODO: Implement a transition to the next state from the idle state
}

void robotController::moveAndDetect()
{
}

// This function will allign the robot with the poop once it has been detected
// The function will use the visual servoing algorithm class
// The function will compute the updated desired robot position and orientation
// The updated position and orientation will be sent to the position controller
// The function will continously use the ai to detect the poop, but during rotations the detection will be disabled
void robotController::detectionAllignment()
{
    // Check if the position controller is busy with a rotation
    if (positionController.getState() != State::ROTATE_TO_GOAL && positionController.getState() != State::ROTATE_TO_GOAL_ORIENTATION)
    {
        // Detect poop with ai
        auto bboxes = getBoundingBoxesAndDraw();

        if (bboxes.size() > 0)
        {
            // TODO: Pick the bounding box with highest confidence and area, for now just pick the first one
            std::vector<float> bbox = bboxes[0];

            // Use the visual servoing algorithm to compute the updated desired robot position and orientation
            std::vector<float> updatedPosition = this->visualServoing.calculateControlPosition(bbox, this->robotPosition);

            // TODO: if the update position is small enough, set the robot state to pickup

            // Set the setpoint for the position controller
            this->positionController.setGoal(updatedPosition[0], updatedPosition[1], updatedPosition[2]);
        }
        else
        {
            // Go back to search pattern
            // this->setRobotState(RobotState::MOVE_BACK_TO_POSITION_BEFORE_PICKUP);
        }
        // updtate the robot position
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
}

void robotController::moveBackToPositionBeforePickup()
{
}

void robotController::searchForMarker()
{
}

void robotController::navigateToMarker()
{
}

void robotController::allignToMarker()
{
}

void robotController::dropOff()
{
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
    std::vector<std::vector<float>> bboxes = yolo.getBoxPredictions(input_image);
    // Draw the bounding boxes
    cv::cvtColor(this->resized_frame, this->resized_frame, cv::COLOR_RGB2BGR);
    resized_frame = aiHelper.drawBoundingBoxes(resized_frame, bboxes);
    // Display the image
    cv::imshow("Detection", resized_frame);

    return bboxes;
}

void robotController::delay(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
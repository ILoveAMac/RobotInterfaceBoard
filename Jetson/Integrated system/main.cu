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
#include "Conv2D.cuh"
#include "FullyConnected.cuh"
#include "Layer.cuh"
#include "MaxPool2D.cuh"
#include "ModelLoadingHelper.h"
#include "aiHelperUtils.h"
#include "positionController.h"
#include "serialHelper.h"
#include "yolo.cuh"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

int main()
{

    serialHelper serial("/dev/ttyUSB0", 9600);
    // Create a position controller
    positionController controller(0.5, 0.5, 0.1, 0.05);
    // Set the goal position
    controller.setGoal(0, 0.5, 0);
    // reset the position of the robot
    serial.resetPosition();

    // Initialize VideoCapture with default camera (index 0)
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // Check if the webcam opened successfully
    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the webcam" << std::endl;
        return -1;
    }

    // Pre-allocate variables
    cv::Mat frame;
    cv::Mat resized_frame;
    std::vector<cv::Mat> channels(3);

    // Allocate device memory for the input image (3 channels, 448x448)
    float *input_image;
    cudaMalloc(&input_image, 3 * 448 * 448 * sizeof(float));

    // Allocate host memory for the input image (use standard malloc or new)
    auto host_image = static_cast<float *>(malloc(3 * 448 * 448 * sizeof(float)));

    // Create a window to display the results
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);

    int image_counter = 0;

    aiHelperUtils aiHelper;

    yolo yolo("/home/wihan/model/");

    // Main loop
    while (true)
    {
        auto t1 = high_resolution_clock::now();
        // Capture a frame from the webcam
        cap >> frame;

        // Check if the frame is empty
        if (frame.empty())
        {
            std::cerr << "Error: Captured empty frame" << std::endl;
            break;
        }

        // Resize the image to 448x448 (input size for YOLOv1)
        cv::resize(frame, resized_frame, cv::Size(448, 448));

        // Convert the image from BGR to RGB
        cv::cvtColor(resized_frame, resized_frame, cv::COLOR_BGR2RGB);

        // Convert image to float and normalize
        resized_frame.convertTo(resized_frame, CV_32F, 1.0 / 255.0);

        // Split channels
        cv::split(resized_frame, channels);

        // Copy the data from the OpenCV Mat to the host memory (channels first format)
        for (int c = 0; c < 3; ++c)
        {
            for (int h = 0; h < 448; ++h)
            {
                for (int w = 0; w < 448; ++w)
                {
                    host_image[c * 448 * 448 + h * 448 + w] = channels[c].at<float>(h, w);
                }
            }
        }

        // Transfer the data from host memory to the GPU memory (device)
        cudaMemcpy(input_image, host_image, 3 * 448 * 448 * sizeof(float), cudaMemcpyHostToDevice);

        // If the current state is not a rotation, use the YOLO model to detect objects
        if (controller.getState() != State::ROTATE_TO_GOAL && controller.getState() != State::ROTATE_TO_POSITION)
        {
            // Get the bounding boxes
            std::vector<std::vector<float>> bboxes = yolo.getBoxPredictions(input_image);

            // Draw the bounding boxes
            cv::cvtColor(resized_frame, resized_frame, cv::COLOR_RGB2BGR);
            resized_frame = aiHelper.drawBoundingBoxes(resized_frame, bboxes);
        }
        else
        {
            // small delay to allow the robot to rotate
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Display the image
        cv::imshow("Detection", resized_frame);

        // Exit if 'q' is pressed
        if (cv::waitKey(1) == 'c')
        {
            // Save the current image
            std::string filename = "img/captured_image_" + std::to_string(image_counter) + ".png";
            if (cv::imwrite(filename, 255 * resized_frame))
            {
                std::cout << "Image saved: " << filename << std::endl;
                image_counter++;
            }
            else
            {
                std::cerr << "Error: Could not save image" << std::endl;
            }
        }
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms\n";

        std::vector<float> position = serial.receivePosition();
        std::vector<float> velocities = controller.updateVelocities(position[0], position[1], position[2]);
        serial.sendSpeeds(velocities[1], velocities[1], velocities[0], velocities[0]);

        // print the current position
        std::cout << "Current position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;

        float distance = serial.receiveDistanceSensorMeasurement(SENSE_1);
        // std::cout << "Distance sensor: " << distance << std::endl;
        if (distance < 0.2 && distance != -1)
        {
            // set velocities to 0
            serial.sendSpeeds(0, 0, 0, 0);
            break;
        }
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();

    // Free allocated memory
    cudaFree(input_image);

    return 0;
}
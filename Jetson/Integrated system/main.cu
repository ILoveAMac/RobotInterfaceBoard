// Standard Library Headers
#include <chrono>  // For timing
#include <cmath>   // For math functions
#include <cstdio>  // For printf
#include <cstring> // For memcpy
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <cuda_fp16.h>

// OpenCV Headers
#include <opencv2/opencv.hpp>

// Boost Headers
#include <boost/asio.hpp>

// Project-Specific Headers
#include "Conv2D.cuh"
#include "FullyConnected.cuh"
#include "Layer.cuh"
#include "MaxPool2D.cuh"
#include "ModelLoadingHelper.cuh"
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
    // Initialize VideoCapture with default camera (index 0)
    cv::VideoCapture cap(0);

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
    __half *input_image;
    cudaMalloc(&input_image, 3 * 448 * 448 * sizeof(__half));

    // Allocate host memory for the input image (use standard malloc or new)
    // auto host_image = static_cast<float *>(malloc(3 * 448 * 448 * sizeof(float)));
    __half *host_image = new __half[3 * 448 * 448];

    // Create a window to display the results
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);

    int image_counter = 0;

    aiHelperUtils aiHelper;

    yolo yolo("/home/wihan/model/");

    // Main loop
    while (true)
    {
        //
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
                    host_image[c * 448 * 448 + h * 448 + w] = __float2half(channels[c].at<float>(h, w));
                }
            }
        }

        // Transfer the data from host memory to the GPU memory (device)
        cudaMemcpy(input_image, host_image, 3 * 448 * 448 * sizeof(__half), cudaMemcpyHostToDevice);

        // Get the bounding boxes
        std::vector<std::vector<float>> bboxes = yolo.getBoxPredictions(input_image);
        auto t2 = high_resolution_clock::now();

        // Draw the bounding boxes
        cv::cvtColor(resized_frame, resized_frame, cv::COLOR_RGB2BGR);
        resized_frame = aiHelper.drawBoundingBoxes(resized_frame, bboxes);

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
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms\n";
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();

    // Free allocated memory
    cudaFree(input_image);

    return 0;
}
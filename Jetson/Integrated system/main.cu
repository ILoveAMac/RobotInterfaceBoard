// Standard Library Headers
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>  // For timing
#include <cstring> // For memcpy
#include <cstdio>  // For printf
#include <cmath>   // For math functions

// OpenCV Headers
#include <opencv2/opencv.hpp>

// Boost Headers
#include <boost/asio.hpp>

// Project-Specific Headers
#include "ModelLoadingHelper.h"
#include "Conv2D.cuh"
#include "Layer.cuh"
#include "MaxPool2D.cuh"
#include "FullyConnected.cuh"
#include "serialHelper.h"
#include "positionController.h"

void drawBoundingBoxes(cv::Mat &image, const float *output, const int grid_size, const int num_bboxes, const int img_width, const int img_height)
{
    const int S = grid_size;  // 7x7 grid in YOLOv1
    const int B = num_bboxes; // Number of bounding boxes per grid cell, typically 2

    for (int i = 0; i < S; i++)
    {
        for (int j = 0; j < S; j++)
        {
            for (int b = 0; b < B; b++)
            { // Loop over each bounding box in the cell
                // Calculate the starting index for the current bounding box (5 values per bounding box)
                const int index = (i * S + j) * (5 * B) + b * 5;

                // Get confidence score for the current bounding box
                float confidence = output[index + 4];

                // If there's an object (skip low-confidence boxes)
                if (confidence > 0.57)
                {
                    // Extract x, y, width, height from the output
                    float x_offset = output[index];           // x relative to the grid cell
                    float y_offset = output[index + 1];       // y relative to the grid cell
                    float w = output[index + 2] * img_width;  // Width relative to image size
                    float h = output[index + 3] * img_height; // Height relative to image size

                    // Convert cell-relative x and y to absolute coordinates in the image
                    float x_center = (j + x_offset) * (img_width / S);  // Absolute x-center
                    float y_center = (i + y_offset) * (img_height / S); // Absolute y-center

                    // Calculate the top-left and bottom-right points of the bounding box
                    int x1 = static_cast<int>(x_center - w / 2);
                    int y1 = static_cast<int>(y_center - h / 2);
                    int x2 = static_cast<int>(x_center + w / 2);
                    int y2 = static_cast<int>(y_center + h / 2);

                    // Draw the bounding box on the image
                    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

                    // Convert confidence to string and format it
                    std::string label = cv::format("%.2f", confidence);

                    // Set the position for the confidence label (above the top-left corner of the bounding box)
                    int baseline = 0;
                    cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                    int label_x = std::max(x1, 0);                     // Ensure the label is inside the image boundaries
                    int label_y = std::max(y1 - label_size.height, 0); // Display above the box

                    // Draw the label background rectangle
                    cv::rectangle(image, cv::Point(label_x, label_y), cv::Point(label_x + label_size.width, label_y + label_size.height + baseline),
                                  cv::Scalar(0, 255, 0), cv::FILLED);

                    // Put the confidence text on the image
                    cv::putText(image, label, cv::Point(label_x, label_y + label_size.height),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                }
            }
        }
    }
}

int main()
{
    const auto MLH = ModelLoadingHelper("/home/wihan/model/");
    std::vector<Layer *> model;

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
    float *input_image;
    cudaMalloc(&input_image, 3 * 448 * 448 * sizeof(float));

    // Allocate host memory for the input image (use standard malloc or new)
    auto host_image = static_cast<float *>(malloc(3 * 448 * 448 * sizeof(float)));

    // model.push_back(new Conv2D(5, 32, 2, 2, "0", MLH, 224, 224, 32, 448, 448, 3));
    // model.push_back(new MaxPool2D(224, 224, 32, 112, 112, 32));
    // model.push_back(new Conv2D(3, 64, 1, 1, "2", MLH, 112, 112, 64, 112, 112, 32));
    // model.push_back(new MaxPool2D(112, 112, 64, 56, 56, 64));
    // model.push_back(new Conv2D(1, 64, 1, 0, "4", MLH, 56, 56, 64, 56, 56, 64));
    // model.push_back(new Conv2D(3, 128, 1, 1, "5", MLH, 56, 56, 128, 56, 56, 64));
    // model.push_back(new Conv2D(1, 128, 1, 0, "6", MLH, 56, 56, 128, 56, 56, 128));
    // model.push_back(new Conv2D(3, 256, 1, 1, "7", MLH, 56, 56, 256, 56, 56, 128));
    // model.push_back(new MaxPool2D(56, 56, 256, 28, 28, 256));
    // model.push_back(new Conv2D(1, 128, 1, 0, "9", MLH, 28, 28, 128, 28, 28, 256));
    // model.push_back(new Conv2D(3, 256, 1, 1, "10", MLH, 28, 28, 256, 28, 28, 128));
    // model.push_back(new Conv2D(1, 256, 1, 0, "11", MLH, 28, 28, 256, 28, 28, 256));
    // model.push_back(new Conv2D(3, 512, 1, 1, "12", MLH, 28, 28, 512, 28, 28, 256));
    // model.push_back(new MaxPool2D(28, 28, 512, 14, 14, 512));
    // model.push_back(new Conv2D(1, 256, 1, 0, "14", MLH, 14, 14, 256, 14, 14, 512));
    // model.push_back(new Conv2D(3, 512, 1, 1, "15", MLH, 14, 14, 512, 14, 14, 256));
    // model.push_back(new Conv2D(3, 256, 2, 1, "16", MLH, 7, 7, 256, 14, 14, 512));
    // model.push_back(new Conv2D(3, 128, 1, 1, "17", MLH, 7, 7, 128, 7, 7, 256));
    // // Fully connected layers
    // model.push_back(new FullyConnected(128 * 7 * 7, 512, MLH, "1", true));
    // model.push_back(new FullyConnected(512, 7 * 7 * 2 * 5, MLH, "4", false));

    model.push_back(new Conv2D(5, 32, 2, 2, "0", MLH, 224, 224, 32, 448, 448, 3));
    model.push_back(new MaxPool2D(224, 224, 32, 112, 112, 32));
    model.push_back(new Conv2D(3, 64, 1, 1, "2", MLH, 112, 112, 64, 112, 112, 32));
    model.push_back(new MaxPool2D(112, 112, 64, 56, 56, 64));
    model.push_back(new Conv2D(1, 64, 1, 0, "4", MLH, 56, 56, 64, 56, 56, 64));
    model.push_back(new Conv2D(3, 128, 1, 1, "5", MLH, 56, 56, 128, 56, 56, 64));
    model.push_back(new Conv2D(1, 128, 1, 0, "6", MLH, 56, 56, 128, 56, 56, 128));
    model.push_back(new Conv2D(3, 256, 1, 1, "7", MLH, 56, 56, 256, 56, 56, 128));
    model.push_back(new MaxPool2D(56, 56, 256, 28, 28, 256));
    model.push_back(new Conv2D(1, 128, 1, 0, "9", MLH, 28, 28, 128, 28, 28, 256));
    model.push_back(new Conv2D(3, 256, 1, 1, "10", MLH, 28, 28, 256, 28, 28, 128));
    model.push_back(new Conv2D(1, 256, 1, 0, "11", MLH, 28, 28, 256, 28, 28, 256));
    model.push_back(new Conv2D(3, 256, 1, 1, "12", MLH, 28, 28, 256, 28, 28, 256));
    model.push_back(new MaxPool2D(28, 28, 256, 14, 14, 256));
    model.push_back(new Conv2D(1, 256, 1, 0, "14", MLH, 14, 14, 256, 14, 14, 256));
    model.push_back(new Conv2D(3, 256, 1, 1, "15", MLH, 14, 14, 256, 14, 14, 256));
    model.push_back(new Conv2D(3, 256, 2, 1, "16", MLH, 7, 7, 256, 14, 14, 256));
    model.push_back(new Conv2D(3, 64, 1, 1, "17", MLH, 7, 7, 64, 7, 7, 256));
    // Fully connected layers
    model.push_back(new FullyConnected(64 * 7 * 7, 512, MLH, "1", true));
    model.push_back(new FullyConnected(512, 7 * 7 * 2 * 5, MLH, "4", false));

    // Load model weights
    for (const auto layer : model)
    {
        layer->loadData();
    }

    // Allocate host memory for output (assuming output size is 7x7x10)
    auto host_output = static_cast<float *>(malloc(7 * 7 * 10 * sizeof(float)));

    // Create a window to display the results
    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);

    int image_counter = 88;

    // Main loop
    while (true)
    {
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

        // ===== Forward Propagation =====
        float *input = input_image; // Input to the first layer
        float *output = nullptr;    // Placeholder for the output

        // Record the start time
        auto start = std::chrono::high_resolution_clock::now();

        for (const auto &layer : model)
        {
            output = layer->forward(input); // Perform forward propagation through each layer
            input = output;                 // Set output of the current layer as the input to the next layer
            cudaDeviceSynchronize();
        }

        // Record the end time
        auto end = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double> duration = end - start;
        std::cout << "Time taken for forward pass: " << duration.count() << " seconds" << std::endl;

        // Copy the data from GPU to CPU
        cudaMemcpy(host_output, output, 7 * 7 * 10 * sizeof(float), cudaMemcpyDeviceToHost);

        // Draw the bounding boxes using the copied output
        // Convert image back to BGR for OpenCV display
        cv::cvtColor(resized_frame, resized_frame, cv::COLOR_RGB2BGR);
        drawBoundingBoxes(resized_frame, host_output, 7, 2, 448, 448);

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
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();

    // Free allocated memory
    cudaFree(input_image);
    free(host_image);
    free(host_output);

    // Clean up layers
    for (auto layer : model)
    {
        delete layer;
    }

    return 0;
}

// #include <iostream>
// #include <chrono> // For timing
// #include <string>
// #include <vector>

// #include "ModelLoadingHelper.h"
// #include "Conv2D.cuh"
// #include "Layer.cuh"
// #include "MaxPool2D.cuh"
// #include "FullyConnected.cuh"
// #include <opencv2/opencv.hpp>

// void drawBoundingBoxes(cv::Mat &image, const float *output, const int grid_size, const int num_bboxes, const int img_width, const int img_height)
// {
//     const int S = grid_size;  // 7x7 grid in YOLOv1
//     const int B = num_bboxes; // Number of bounding boxes per grid cell, typically 2

//     for (int i = 0; i < S; i++)
//     {
//         for (int j = 0; j < S; j++)
//         {
//             for (int b = 0; b < B; b++)
//             { // Loop over each bounding box in the cell
//                 // Calculate the starting index for the current bounding box (5 values per bounding box)
//                 const int index = (i * S + j) * (5 * B) + b * 5;

//                 // Get confidence score for the current bounding box
//                 float confidence = output[index + 4];

//                 // If there's an object (skip low-confidence boxes)
//                 if (confidence > 0.1)
//                 {
//                     // Extract x, y, width, height from the output
//                     float x_offset = output[index];           // x relative to the grid cell
//                     float y_offset = output[index + 1];       // y relative to the grid cell
//                     float w = output[index + 2] * img_width;  // Width relative to image size
//                     float h = output[index + 3] * img_height; // Height relative to image size

//                     // Convert cell-relative x and y to absolute coordinates in the image
//                     float x_center = (j + x_offset) * (img_width / S);  // Absolute x-center
//                     float y_center = (i + y_offset) * (img_height / S); // Absolute y-center

//                     // Calculate the top-left and bottom-right points of the bounding box
//                     int x1 = static_cast<int>(x_center - w / 2);
//                     int y1 = static_cast<int>(y_center - h / 2);
//                     int x2 = static_cast<int>(x_center + w / 2);
//                     int y2 = static_cast<int>(y_center + h / 2);

//                     // Draw the bounding box on the image
//                     cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

//                     // Convert confidence to string and format it
//                     std::string label = cv::format("%.2f", confidence);

//                     // Set the position for the confidence label (above the top-left corner of the bounding box)
//                     int baseline = 0;
//                     cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
//                     int label_x = std::max(x1, 0);                     // Ensure the label is inside the image boundaries
//                     int label_y = std::max(y1 - label_size.height, 0); // Display above the box

//                     // Draw the label background rectangle
//                     cv::rectangle(image, cv::Point(label_x, label_y), cv::Point(label_x + label_size.width, label_y + label_size.height + baseline),
//                                   cv::Scalar(0, 255, 0), cv::FILLED);

//                     // Put the confidence text on the image
//                     cv::putText(image, label, cv::Point(label_x, label_y + label_size.height),
//                                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
//                 }
//             }
//         }
//     }
// }

// int main()
// {
//     const auto MLH = ModelLoadingHelper("/home/wihan/model/");
//     std::vector<Layer *> model;

//     // Initialize VideoCapture with default camera (index 0)
//     cv::VideoCapture cap(0);

//     // Check if the webcam opened successfully
//     if (!cap.isOpened())
//     {
//         std::cerr << "Error: Could not open the webcam" << std::endl;
//         return -1;
//     }

//     // Pre-allocate variables
//     cv::Mat frame;
//     cv::Mat resized_frame;
//     std::vector<cv::Mat> channels(3);

//     // Allocate device memory for the input image (3 channels, 448x448)
//     float *input_image;
//     cudaMalloc(&input_image, 3 * 448 * 448 * sizeof(float));

//     // Allocate host memory for the input image (use standard malloc or new)
//     auto host_image = static_cast<float *>(malloc(3 * 448 * 448 * sizeof(float)));

//     // model.push_back(new Conv2D(5, 32, 2, 2, "0", MLH, 224, 224, 32, 448, 448, 3));
//     // model.push_back(new MaxPool2D(224, 224, 32, 112, 112, 32));
//     // model.push_back(new Conv2D(3, 64, 1, 1, "2", MLH, 112, 112, 64, 112, 112, 32));
//     // model.push_back(new MaxPool2D(112, 112, 64, 56, 56, 64));
//     // model.push_back(new Conv2D(1, 64, 1, 0, "4", MLH, 56, 56, 64, 56, 56, 64));
//     // model.push_back(new Conv2D(3, 128, 1, 1, "5", MLH, 56, 56, 128, 56, 56, 64));
//     // model.push_back(new Conv2D(1, 128, 1, 0, "6", MLH, 56, 56, 128, 56, 56, 128));
//     // model.push_back(new Conv2D(3, 256, 1, 1, "7", MLH, 56, 56, 256, 56, 56, 128));
//     // model.push_back(new MaxPool2D(56, 56, 256, 28, 28, 256));
//     // model.push_back(new Conv2D(1, 128, 1, 0, "9", MLH, 28, 28, 128, 28, 28, 256));
//     // model.push_back(new Conv2D(3, 256, 1, 1, "10", MLH, 28, 28, 256, 28, 28, 128));
//     // model.push_back(new Conv2D(1, 256, 1, 0, "11", MLH, 28, 28, 256, 28, 28, 256));
//     // model.push_back(new Conv2D(3, 512, 1, 1, "12", MLH, 28, 28, 512, 28, 28, 256));
//     // model.push_back(new MaxPool2D(28, 28, 512, 14, 14, 512));
//     // model.push_back(new Conv2D(1, 256, 1, 0, "14", MLH, 14, 14, 256, 14, 14, 512));
//     // model.push_back(new Conv2D(3, 512, 1, 1, "15", MLH, 14, 14, 512, 14, 14, 256));
//     // model.push_back(new Conv2D(3, 256, 2, 1, "16", MLH, 7, 7, 256, 14, 14, 512));
//     // model.push_back(new Conv2D(3, 128, 1, 1, "17", MLH, 7, 7, 128, 7, 7, 256));
//     // // Fully connected layers
//     // model.push_back(new FullyConnected(128 * 7 * 7, 512, MLH, "1", true));
//     // model.push_back(new FullyConnected(512, 7 * 7 * 2 * 5, MLH, "4", false));

//     model.push_back(new Conv2D(5, 32, 2, 2, "0", MLH, 224, 224, 32, 448, 448, 3));
//     model.push_back(new MaxPool2D(224, 224, 32, 112, 112, 32));
//     model.push_back(new Conv2D(3, 64, 1, 1, "2", MLH, 112, 112, 64, 112, 112, 32));
//     model.push_back(new MaxPool2D(112, 112, 64, 56, 56, 64));
//     model.push_back(new Conv2D(1, 64, 1, 0, "4", MLH, 56, 56, 64, 56, 56, 64));
//     model.push_back(new Conv2D(3, 128, 1, 1, "5", MLH, 56, 56, 128, 56, 56, 64));
//     model.push_back(new Conv2D(1, 128, 1, 0, "6", MLH, 56, 56, 128, 56, 56, 128));
//     model.push_back(new Conv2D(3, 256, 1, 1, "7", MLH, 56, 56, 256, 56, 56, 128));
//     model.push_back(new MaxPool2D(56, 56, 256, 28, 28, 256));
//     model.push_back(new Conv2D(1, 128, 1, 0, "9", MLH, 28, 28, 128, 28, 28, 256));
//     model.push_back(new Conv2D(3, 256, 1, 1, "10", MLH, 28, 28, 256, 28, 28, 128));
//     model.push_back(new Conv2D(1, 256, 1, 0, "11", MLH, 28, 28, 256, 28, 28, 256));
//     model.push_back(new Conv2D(3, 256, 1, 1, "12", MLH, 28, 28, 256, 28, 28, 256));
//     model.push_back(new MaxPool2D(28, 28, 256, 14, 14, 256));
//     model.push_back(new Conv2D(1, 256, 1, 0, "14", MLH, 14, 14, 256, 14, 14, 256));
//     model.push_back(new Conv2D(3, 256, 1, 1, "15", MLH, 14, 14, 256, 14, 14, 256));
//     model.push_back(new Conv2D(3, 256, 2, 1, "16", MLH, 7, 7, 256, 14, 14, 256));
//     model.push_back(new Conv2D(3, 64, 1, 1, "17", MLH, 7, 7, 64, 7, 7, 256));
//     // Fully connected layers
//     model.push_back(new FullyConnected(64 * 7 * 7, 512, MLH, "1", true));
//     model.push_back(new FullyConnected(512, 7 * 7 * 2 * 5, MLH, "4", false));

//     // Load model weights
//     for (const auto layer : model)
//     {
//         layer->loadData();
//     }

//     // Allocate host memory for output (assuming output size is 7x7x10)
//     auto host_output = static_cast<float *>(malloc(7 * 7 * 10 * sizeof(float)));

//     // Create a window to display the results
//     cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);

//     int image_counter = 88;

//     // Main loop
//     while (true)
//     {
//         // Capture a frame from the webcam
//         cap >> frame;

//         // Check if the frame is empty
//         if (frame.empty())
//         {
//             std::cerr << "Error: Captured empty frame" << std::endl;
//             break;
//         }

//         // Resize the image to 448x448 (input size for YOLOv1)
//         cv::resize(frame, resized_frame, cv::Size(448, 448));

//         // Convert the image from BGR to RGB
//         cv::cvtColor(resized_frame, resized_frame, cv::COLOR_BGR2RGB);

//         // Convert image to float and normalize
//         resized_frame.convertTo(resized_frame, CV_32F, 1.0 / 255.0);

//         // Split channels
//         cv::split(resized_frame, channels);

//         // Copy the data from the OpenCV Mat to the host memory (channels first format)
//         for (int c = 0; c < 3; ++c)
//         {
//             for (int h = 0; h < 448; ++h)
//             {
//                 for (int w = 0; w < 448; ++w)
//                 {
//                     host_image[c * 448 * 448 + h * 448 + w] = channels[c].at<float>(h, w);
//                 }
//             }
//         }

//         // Transfer the data from host memory to the GPU memory (device)
//         cudaMemcpy(input_image, host_image, 3 * 448 * 448 * sizeof(float), cudaMemcpyHostToDevice);

//         // ===== Forward Propagation =====
//         float *input = input_image; // Input to the first layer
//         float *output = nullptr;    // Placeholder for the output

//         // Record the start time
//         auto start = std::chrono::high_resolution_clock::now();

//         for (const auto &layer : model)
//         {
//             output = layer->forward(input); // Perform forward propagation through each layer
//             input = output;                 // Set output of the current layer as the input to the next layer
//             cudaDeviceSynchronize();
//         }

//         // Record the end time
//         auto end = std::chrono::high_resolution_clock::now();
//         const std::chrono::duration<double> duration = end - start;
//         std::cout << "Time taken for forward pass: " << duration.count() << " seconds" << std::endl;

//         // Copy the data from GPU to CPU
//         cudaMemcpy(host_output, output, 7 * 7 * 10 * sizeof(float), cudaMemcpyDeviceToHost);

//         // Draw the bounding boxes using the copied output
//         // Convert image back to BGR for OpenCV display
//         cv::cvtColor(resized_frame, resized_frame, cv::COLOR_RGB2BGR);
//         // drawBoundingBoxes(resized_frame, host_output, 7, 2, 448, 448);

//         // Display the image
//         cv::imshow("Detection", resized_frame);

//         // Exit if 'q' is pressed
//         if (cv::waitKey(1) == 'c')
//         {
//             // Save the current image
//             std::string filename = "img/captured_image_" + std::to_string(image_counter) + ".png";
//             if (cv::imwrite(filename, 255 * resized_frame))
//             {
//                 std::cout << "Image saved: " << filename << std::endl;
//                 image_counter++;
//             }
//             else
//             {
//                 std::cerr << "Error: Could not save image" << std::endl;
//             }
//         }
//     }

//     // Release resources
//     cap.release();
//     cv::destroyAllWindows();

//     // Free allocated memory
//     cudaFree(input_image);
//     free(host_image);
//     free(host_output);

//     // Clean up layers
//     for (auto layer : model)
//     {
//         delete layer;
//     }

//     return 0;
// }
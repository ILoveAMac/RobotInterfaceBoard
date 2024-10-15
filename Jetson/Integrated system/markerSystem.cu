#include "markerSystem.cuh"

markerSystem::markerSystem() : cannyDetector(IMG_WIDTH, IMG_HEIGHT, SIGMA, KERNEL_SIZE, MIN_THRESHOLD, MAX_THRESHOLD, NMS_LOW),
                               greyConverter(RED_FACTOR, GREEN_FACTOR, BLUE_FACTOR, IMG_WIDTH, IMG_HEIGHT)
{
}

markerSystem::~markerSystem()
{
}

std::tuple<std::vector<double>, std::vector<double>> markerSystem::detectMarkers(float *image)
{
    // First we convert the image to grey
    float *greyImage = greyConverter.imageToGrey(image);

    int width = 448;
    int height = 448;

    // Step 1: Allocate memory on the CPU (host)
    float *hostImage = new float[width * height]; // Allocate space for 448x448 grayscale image on the CPU

    // Step 2: Copy image from GPU to CPU
    cudaMemcpy(hostImage, greyImage, width * height * sizeof(float), cudaMemcpyDeviceToHost);

    // Step 3: Convert the float array to an OpenCV Mat object
    // OpenCV expects 8-bit or 32-bit image types, so if your image is a float,
    // you may need to normalize it first or convert it to an 8-bit format.
    cv::Mat greyMat(height, width, CV_32FC1, hostImage); // Create an OpenCV Mat from the float array

    // Optionally convert to 8-bit for display purposes (since OpenCV displays grayscale images in 8-bit)
    cv::Mat greyMat8U;
    greyMat.convertTo(greyMat8U, CV_8UC1, 255.0); // Scale the float values to 0-255 for display

    // Step 4: Display the image in an OpenCV window
    cv::imshow("Grayscale Image", greyMat8U);
    cv::waitKey(0); // Wait for a key press before closing the window

    // Step 5: Clean up
    delete[] hostImage; // Free the CPU memory

    // Apply canny edge detection
    float *edges = cannyDetector.applyCanny(greyImage);

    // Convert the Canny result back to an OpenCV Mat
    cv::Mat cannyOutput(IMG_WIDTH, IMG_HEIGHT, CV_32FC1, edges);
    cannyOutput.convertTo(cannyOutput, CV_8UC1, 255.0);

    // Display the Canny edge detection result in a separate window
    cv::imshow("Canny Edge Detection Output", cannyOutput);

    // Find contours
    std::vector<std::vector<int>> contours = contourDetector.detect(edges, IMG_WIDTH, IMG_HEIGHT);

    // Use the marker isolator to find the nested pair of quads (markers)
    std::vector<std::vector<std::vector<int>>> markers = markerIsolator.isolateMarkersFromContours(contours);

    for (const auto &marker : markers)
    {
        // Nested pair contains [parentQuad, childQuad]
        std::vector<cv::Point> parentQuad;
        std::vector<cv::Point> childQuad;

        // Extract the parent quad points
        for (size_t i = 0; i < marker[0].size(); i += 2)
        {
            parentQuad.emplace_back(marker[0][i], marker[0][i + 1]);
        }

        // Extract the child quad points
        for (size_t i = 0; i < marker[1].size(); i += 2)
        {
            childQuad.emplace_back(marker[1][i], marker[1][i + 1]);
        }

        // Solve p4p
        // Define world points
        std::vector<cv::Point3d> worldPoints = {
            {0.0, 0.0, 0.0},     // Point 1
            {0.0, 0.130, 0.0},   // Point 2
            {0.130, 0.130, 0.0}, // Point 3
            {0.130, 0.0, 0.0},   // Point 4
        };

        // Prepare world points in the required format
        std::vector<std::vector<double>> worldPointsVec;
        for (const auto &point : worldPoints)
        {
            worldPointsVec.push_back({point.x, point.y, point.z});
        }

        std::vector<std::vector<double>> ImagePoints;
        ImagePoints.push_back({static_cast<double>(parentQuad[0].x), static_cast<double>(parentQuad[0].y)});
        ImagePoints.push_back({static_cast<double>(parentQuad[1].x), static_cast<double>(parentQuad[1].y)});
        ImagePoints.push_back({static_cast<double>(parentQuad[2].x), static_cast<double>(parentQuad[2].y)});
        ImagePoints.push_back({static_cast<double>(parentQuad[3].x), static_cast<double>(parentQuad[3].y)});

        GrunertSolution solution = perspectiveSolver::solveP4P(worldPointsVec, ImagePoints);
        std::vector<double> sideLengths = solution.sideLengths;
        std::vector<double> angles = solution.angles;
        cv::Mat rotationMatrix = solution.rotationMatrix;
        cv::Mat translationVector = solution.translationVector;

        if (translationVector.size().height == 3)
        {
            std::vector<double> eulerAngles = perspectiveSolver::getEulerAngles(rotationMatrix);

            // Store the translationvector in a c++ vector
            std::vector<double> translation;
            for (int i = 0; i < 3; i++)
            {
                translation.push_back(translationVector.at<double>(i));
            }

            // return the translation and euler angles
            return std::make_tuple(translation, eulerAngles);
        }
    }
    // Return empty vectors if no markers were found
    return std::make_tuple(std::vector<double>(), std::vector<double>());
}

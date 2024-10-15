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

        // Step 1: Convert the grayscale cannyOutput (CV_8UC1) to BGR
        cv::Mat colorOutput;
        cv::cvtColor(cannyOutput, colorOutput, cv::COLOR_GRAY2BGR); // Convert 8-bit grayscale to 3-channel BGR

        // Step 2: Draw the parent quad in blue
        cv::polylines(colorOutput, parentQuad, true, cv::Scalar(255, 0, 0), 2); // Blue color

        // Step 3: Draw the child quad in green
        cv::polylines(colorOutput, childQuad, true, cv::Scalar(0, 255, 0), 2); // Green color

        // Step 4: Draw red dots at each corner of the parent quad and label them
        for (size_t i = 0; i < parentQuad.size(); ++i)
        {
            cv::circle(colorOutput, parentQuad[i], 5, cv::Scalar(0, 0, 255), -1); // Red color

            // Label each point with its index
            std::string label = std::to_string(i);
            cv::putText(colorOutput, label, parentQuad[i] + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 1); // Yellow text
        }

        // Step 5: Display the result in a window
        cv::imshow("Marker Detection - Webcam", colorOutput);

        if (!rotationMatrix.empty() && !translationVector.empty())
        {
            // Define camera intrinsic parameters and distortion coefficients
            cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << FX, 0, CX,
                                    0, FY, CY,
                                    0, 0, 1);

            cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << K1, K2, 0, 0, 0); // Adjust if you have more coefficients

            // Convert rotation matrix to rotation vector
            cv::Mat rvec;
            cv::Rodrigues(rotationMatrix, rvec);

            // Define 3D points of the coordinate axes in the object's coordinate system
            std::vector<cv::Point3f> axisPoints;
            axisPoints.push_back(cv::Point3f(0, 0, 0));     // Origin
            axisPoints.push_back(cv::Point3f(0.05f, 0, 0)); // X-axis (5 cm)
            axisPoints.push_back(cv::Point3f(0, 0.05f, 0)); // Y-axis (5 cm)
            axisPoints.push_back(cv::Point3f(0, 0, 0.05f)); // Z-axis (5 cm)

            // Project the 3D points onto the image plane
            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(
                axisPoints,
                rvec,
                translationVector,
                cameraMatrix,
                distCoeffs,
                imagePoints);

            // Draw the axes on the image
            cv::Mat img_with_axes = colorOutput.clone();
            cv::line(img_with_axes, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X-axis in RED
            cv::line(img_with_axes, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y-axis in GREEN
            cv::line(img_with_axes, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z-axis in BLUE

            // Optionally, label the axes
            cv::putText(img_with_axes, "X", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255),
                        2);
            cv::putText(img_with_axes, "Y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0),
                        2);
            cv::putText(img_with_axes, "Z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
                        2);

            // Display the image
            cv::imshow("Pose Visualization", img_with_axes);
        }

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

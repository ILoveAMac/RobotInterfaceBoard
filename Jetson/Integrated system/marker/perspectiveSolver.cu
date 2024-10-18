//
// Created by Wihan on 2024-09-27.
//

#include "perspectiveSolver.cuh"

perspectiveSolver::perspectiveSolver()
{
}

perspectiveSolver::~perspectiveSolver()
{
}

GrunertSolution perspectiveSolver::solveP4P(const std::vector<std::vector<double>> &worldPoints,
                                            const std::vector<std::vector<double>> &imagePoints)
{
    // // Check if there are exactly 4 world points and 4 image points
    // if (worldPoints.size() != 4 || imagePoints.size() != 4)
    // {
    //     throw std::invalid_argument("A unique solution requires exactly 4 points.");
    // }

    // std::vector<std::vector<double>> reorderedPoints = reorderPoints(imagePoints);

    // std::vector<std::vector<double>> undistortedNormalizedPoints = removeDistortion(reorderedPoints);

    // // Compute unit direction vectors from undistorted normalized coordinates
    // std::vector<std::vector<double>> directionVectors;
    // for (const auto &point : undistortedNormalizedPoints)
    // {
    //     double x_u = point.at(0);
    //     double y_u = point.at(1);
    //     double z_u = 1.0;
    //     double norm = std::sqrt(x_u * x_u + y_u * y_u + z_u * z_u);
    //     double j_x = x_u / norm;
    //     double j_y = y_u / norm;
    //     double j_z = z_u / norm;
    //     directionVectors.push_back({j_x, j_y, j_z});
    // }

    // // Generate all combinations of three points out of four
    // std::vector<std::array<int, 3>> combinations = {
    //     {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};

    // std::vector<GrunertSolution> solutionSets[4];

    // for (int i = 0; i < static_cast<int>(combinations.size()); ++i)
    // {
    //     std::vector<std::vector<double>> subsetWorldPoints;
    //     std::vector<std::vector<double>> subsetDirectionVectors;

    //     for (int idx : combinations[i])
    //     {
    //         subsetWorldPoints.push_back(worldPoints[idx]);
    //         subsetDirectionVectors.push_back(directionVectors[idx]);
    //     }

    //     solutionSets[i] = grunertsMethod(subsetWorldPoints, subsetDirectionVectors);
    // }

    // // Now compare all solution sets to find overlapping solutions
    // GrunertSolution overlappingSolution;
    // bool foundOverlap = false;

    // for (int i = 0; i < 4 && !foundOverlap; ++i)
    // {
    //     for (int j = i + 1; j < 4 && !foundOverlap; ++j)
    //     {
    //         for (const auto &sol1 : solutionSets[i])
    //         {
    //             for (const auto &sol2 : solutionSets[j])
    //             {
    //                 if (areSolutionsSimilar(sol1, sol2))
    //                 {
    //                     overlappingSolution = sol1;
    //                     foundOverlap = true;
    //                     break;
    //                 }
    //             }
    //         }
    //     }
    // }

    // if (!foundOverlap)
    // {
    //     // throw std::runtime_error("No overlapping solution found.");
    // }

    // return overlappingSolution;

    // Check if there are exactly 4 world points and 4 image points
    if (worldPoints.size() != 4 || imagePoints.size() != 4)
    {
        throw std::invalid_argument("A unique solution requires exactly 4 points.");
    }

    // Convert world points and image points to OpenCV Mat format
    cv::Mat worldMat(4, 3, CV_64F);
    cv::Mat imageMat(4, 2, CV_64F);

    for (int i = 0; i < 4; ++i)
    {
        worldMat.at<double>(i, 0) = worldPoints[i][0];
        worldMat.at<double>(i, 1) = worldPoints[i][1];
        worldMat.at<double>(i, 2) = worldPoints[i][2];

        imageMat.at<double>(i, 0) = imagePoints[i][0];
        imageMat.at<double>(i, 1) = imagePoints[i][1];
    }

    // Camera intrinsic matrix K
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << FX, 0, CX,
                            0, FY, CY,
                            0, 0, 1);

    // Distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << K1, K2, 0, 0, 0);

    // Output rotation and translation vectors
    cv::Mat rvec, tvec;

    // Use OpenCV solvePnP to find the pose
    bool success = cv::solvePnP(worldMat, imageMat, cameraMatrix, distCoeffs, rvec, tvec);

    if (!success)
    {
        throw std::runtime_error("solvePnP failed to find a solution.");
    }

    // Convert rotation vector to rotation matrix
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    // Populate the GrunertSolution struct
    GrunertSolution solution;
    solution.rotationMatrix = rotationMatrix;
    solution.translationVector = tvec;

    return solution;
}

std::vector<double> perspectiveSolver::getEulerAngles(cv::Mat R)
{
    // std::vector<double> eulerAngles(3, 0.0); // Output: [phi, theta, psi]

    // // Check if gimbal lock condition (R(2, 0) != ±1)
    // if (fabs(R.at<double>(2, 0)) != 1)
    // {
    //     // General case
    //     const double theta1 = -asin(R.at<double>(2, 0)); // theta1 = -asin(R31)
    //     const double theta2 = P_PI - theta1;             // theta2 = pi - theta1

    //     // Compute corresponding phi1 and psi1
    //     const double psi1 = atan2(R.at<double>(2, 1) / cos(theta1), R.at<double>(2, 2) / cos(theta1));
    //     const double phi1 = atan2(R.at<double>(1, 0) / cos(theta1), R.at<double>(0, 0) / cos(theta1));

    //     // Optionally, compute the second solution (phi2, psi2) using theta2
    //     double psi2 = atan2(R.at<double>(2, 1) / cos(theta2), R.at<double>(2, 2) / cos(theta2));
    //     double phi2 = atan2(R.at<double>(1, 0) / cos(theta2), R.at<double>(0, 0) / cos(theta2));

    //     // Select the first set of angles (you could choose to return both solutions if needed)
    //     eulerAngles[0] = phi2;   // phi
    //     eulerAngles[1] = theta2; // theta
    //     eulerAngles[2] = psi2;   // psi
    // }
    // else
    // {
    //     // Gimbal lock condition: R31 = ±1
    //     eulerAngles[0] = 0; // Set phi to 0 as it's arbitrary in this case

    //     if (R.at<double>(2, 0) == -1)
    //     {
    //         eulerAngles[1] = P_PI / 2;                                      // theta = pi/2
    //         eulerAngles[2] = atan2(R.at<double>(0, 1), R.at<double>(0, 2)); // psi = atan2(R12, R13)
    //     }
    //     else
    //     {
    //         eulerAngles[1] = -P_PI / 2;                                       // theta = -pi/2
    //         eulerAngles[2] = atan2(-R.at<double>(0, 1), -R.at<double>(0, 2)); // psi = atan2(-R12, -R13)
    //     }
    // }

    // return eulerAngles;

    cv::Mat rvecMat;

    cv::Rodrigues(R, rvecMat);

    if (rvecMat.type() != CV_64F)
    {
        rvecMat.convertTo(rvecMat, CV_64F);
    }

    std::vector<double> eulerAngles(3, 0.0);
    eulerAngles[0] = rvecMat.at<double>(0, 0);
    eulerAngles[1] = rvecMat.at<double>(1, 0);
    eulerAngles[2] = rvecMat.at<double>(2, 0);

    return eulerAngles;
}

std::vector<GrunertSolution> perspectiveSolver::grunertsMethod(const std::vector<std::vector<double>> &worldPoints,
                                                               const std::vector<std::vector<double>> &imagePoints)
{
    // Check if there are exactly 3 world points and 3 image points
    if (worldPoints.size() != 3 || imagePoints.size() != 3)
    {
        throw std::invalid_argument("Grunert's method requires exactly 3 world points and 3 image points.");
    }

    // Check if each world point has exactly 3 coordinates (X, Y, Z)
    for (const auto &point : worldPoints)
    {
        if (point.size() != 3)
        {
            throw std::invalid_argument("Each world point must have exactly 3 coordinates.");
        }
    }

    // Check if each image point has exactly 3 values
    // Each point in a 3D unit vector
    for (const auto &point : imagePoints)
    {
        if (point.size() != 3)
        {
            throw std::invalid_argument("Each image point must have exactly 3 values.");
        }
    }

    // 1. ===== calculate the angles, alpha, beta and gamma using the known world points =====
    // Calculate the side lengths (distances between world points)
    const double a = euclideanDistance(worldPoints[1], worldPoints[2]); // Distance between P2 and P3
    const double b = euclideanDistance(worldPoints[0], worldPoints[2]); // Distance between P1 and P3
    const double c = euclideanDistance(worldPoints[0], worldPoints[1]); // Distance between P1 and P2

    std::vector<std::vector<double>> j = imagePoints;

    // Calculate the dot products to get cos(alpha), cos(beta), cos(gamma)
    const double cosAlpha = j[1][0] * j[2][0] + j[1][1] * j[2][1] + j[1][2] * j[2][2];
    const double cosBeta = j[0][0] * j[2][0] + j[0][1] * j[2][1] + j[0][2] * j[2][2];
    const double cosGamma = j[0][0] * j[1][0] + j[0][1] * j[1][1] + j[0][2] * j[1][2];

    // Use full intermediate values
    const double a2 = a * a;
    const double b2 = b * b;
    const double c2 = c * c;

    const double cosAlpha2 = cosAlpha * cosAlpha;
    const double cosBeta2 = cosBeta * cosBeta;
    const double cosGamma2 = cosGamma * cosGamma;

    // 2. ===== Setup the 4th order polynomial coefficients =====
    const double A4 = std::pow((a2 - c2) / b2 - 1, 2) - (4 * c2 / b2) * cosAlpha2;

    const double A3 = 4 * ((a2 - c2) / b2 * (1 - (a2 - c2) / b2) * cosBeta - (1 - (a2 + c2) / b2) * cosAlpha * cosGamma + 2 * (c2 / b2) * cosAlpha2 * cosBeta);

    const double A2 = 2 * (std::pow((a2 - c2) / b2, 2) - 1 + 2 * std::pow((a2 - c2) / b2, 2) * cosBeta2 + 2 * ((b2 - c2) / b2) * cosAlpha2 - 4 * ((a2 + c2) / b2) * cosAlpha * cosBeta * cosGamma + 2 * ((b2 - a2) / b2) * cosGamma2);

    const double A1 = 4 * (-(a2 - c2) / b2 * (1 + (a2 - c2) / b2) * cosBeta + 2 * (a2 / b2) * cosGamma2 * cosBeta - (1 - (a2 + c2) / b2) * cosAlpha * cosGamma);

    const double A0 = std::pow(1 + (a2 - c2) / b2, 2) - (4 * a2 / b2) * cosGamma2;

    // 3. ===== Solve for the roots of the polynomial =====
    const std::vector<double> realRoots = quartic(A3 / A4, A2 / A4, A1 / A4, A0 / A4);

    // 4. ===== Populate the solutions with alpha beta and gamma =====
    // There will be up to 4 roots for the polygon

    // Check if there are any valid roots left
    if (realRoots.empty())
    {
        // No roots were found, we return an empty solution vector
        std::vector<GrunertSolution> empty_solution;
        return empty_solution;
    }

    // Create a vector of GrunertSolution with the size of the number of real roots
    // Remove the pre-allocation of the solutions vector
    std::vector<GrunertSolution> solutions;

    // Precompute the angles once, as they remain constant
    const double alpha = std::acos(cosAlpha);
    const double beta = std::acos(cosBeta);
    const double gamma = std::acos(cosGamma);

    // Loop over the real roots
    for (size_t i = 0; i < realRoots.size(); ++i)
    {
        const double v = realRoots[i];

        // Calculate u using the given formula
        const double numerator = ((-1 + (a2 - c2) / b2) * v * v) - (2 * ((a2 - c2) / b2) * cosBeta * v) + (1 + (a2 - c2) / b2);
        const double denominator = 2 * (cosGamma - v * cosAlpha);

        // Check for division by zero
        if (std::abs(denominator) < REAL_TOLERANCE)
        {
            // Skip invalid roots
            continue;
        }

        const double u = numerator / denominator;

        // Calculate S1, S2, and S3
        const double S1 = std::sqrt(c2 / (1 + u * u - 2 * u * cosGamma));
        const double S2 = u * S1;
        const double S3 = v * S1;

        // Create 3D points in the camera frame using the normalized vectors j1, j2, and j3
        cv::Mat P_cam(3, 3, CV_64F);
        P_cam.at<double>(0, 0) = S1 * j[0][0];
        P_cam.at<double>(1, 0) = S1 * j[0][1];
        P_cam.at<double>(2, 0) = S1 * j[0][2];

        P_cam.at<double>(0, 1) = S2 * j[1][0];
        P_cam.at<double>(1, 1) = S2 * j[1][1];
        P_cam.at<double>(2, 1) = S2 * j[1][2];

        P_cam.at<double>(0, 2) = S3 * j[2][0];
        P_cam.at<double>(1, 2) = S3 * j[2][1];
        P_cam.at<double>(2, 2) = S3 * j[2][2];

        // Convert worldPoints into a cv::Mat format for Arun's method
        cv::Mat P_world(3, 3, CV_64F);
        for (int k = 0; k < 3; k++)
        {
            P_world.at<double>(0, k) = worldPoints[k][0];
            P_world.at<double>(1, k) = worldPoints[k][1];
            P_world.at<double>(2, k) = worldPoints[k][2];
        }

        // Apply Horn's method to find the rotation matrix R and translation vector t
        std::pair<cv::Mat, cv::Mat> result = arun(P_world, P_cam);

        // Explicitly extract R and t
        cv::Mat R = result.first;
        cv::Mat t = result.second;

        // Create a new GrunertSolution and populate it
        GrunertSolution solution;
        solution.angles = {alpha, beta, gamma};
        solution.sideLengths = {S1, S2, S3};
        solution.rotationMatrix = R;
        solution.translationVector = t;

        // Add the valid solution to the solutions vector
        solutions.push_back(solution);
    }

    return solutions;
}

double perspectiveSolver::euclideanDistance(const std::vector<double> &point1, const std::vector<double> &point2)
{
    return std::sqrt(std::pow(point2[0] - point1[0], 2) +
                     std::pow(point2[1] - point1[1], 2) +
                     std::pow(point2[2] - point1[2], 2));
}

std::vector<double> perspectiveSolver::cubic(const double b, const double c, const double d)
{
    const double p = c - b * b / 3.0;
    const double q = 2.0 * b * b * b / 27.0 - b * c / 3.0 + d;

    std::vector<double> roots;

    if (p == 0.0)
    {
        roots.push_back(pow(q, 1.0 / 3.0));
        return roots;
    }
    if (q == 0.0)
    {
        roots.push_back(0.0);
        return roots;
    }

    const double t = sqrt(fabs(p) / 3.0);
    const double g = 1.5 * q / (p * t);

    if (p > 0.0)
    {
        roots.push_back(-2.0 * t * sinh(asinh(g) / 3.0) - b / 3.0);
        return roots;
    }

    if (4.0 * p * p * p + 27.0 * q * q < 0.0)
    {
        roots.push_back(2.0 * t * cos(acos(g) / 3.0) - b / 3.0);
        return roots;
    }

    if (q > 0.0)
    {
        roots.push_back(-2.0 * t * cosh(acosh(-g) / 3.0) - b / 3.0);
        return roots;
    }

    roots.push_back(2.0 * t * cosh(acosh(g) / 3.0) - b / 3.0);
    return roots;
}

std::pair<cv::Mat, cv::Mat> perspectiveSolver::arun(const cv::Mat &A, const cv::Mat &B)
{
    // Ensure that A and B have the correct dimensions
    int N = A.cols;
    if (B.cols != N || A.rows != 3 || B.rows != 3)
    {
        throw std::invalid_argument("Input matrices must be 3xN.");
    }

    // Calculate centroids
    cv::Mat A_centroid = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat B_centroid = cv::Mat::zeros(3, 1, CV_64F);

    for (int i = 0; i < N; ++i)
    {
        A_centroid += A.col(i);
        B_centroid += B.col(i);
    }

    A_centroid /= N;
    B_centroid /= N;

    // Calculate the vectors from centroids
    cv::Mat A_prime = cv::Mat::zeros(A.size(), A.type()); // Initialize A_prime with the same size as A
    cv::Mat B_prime = cv::Mat::zeros(B.size(), B.type()); // Initialize B_prime with the same size as B

    for (int i = 0; i < A.cols; ++i)
    {
        A_prime.col(i) = A.col(i) - A_centroid;
        B_prime.col(i) = B.col(i) - B_centroid;
    }

    // Calculate the H matrix
    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < N; i++)
    {
        H += A_prime.col(i) * B_prime.col(i).t();
    }

    // Perform Singular Value Decomposition (SVD) on H
    cv::Mat U, S, Vt;
    cv::SVD::compute(H, S, U, Vt);

    // Calculate the rotation matrix R
    cv::Mat V = Vt.t();
    cv::Mat Ut = U.t();
    double det = cv::determinant(V) * cv::determinant(Ut);
    cv::Mat D = cv::Mat::eye(3, 3, CV_64F);
    D.at<double>(2, 2) = det;

    cv::Mat R = V * D * Ut;

    // Calculate the translation vector t
    cv::Mat t = B_centroid - R * A_centroid;

    return {R, t};
}

std::pair<cv::Mat, cv::Mat> perspectiveSolver::horn(const cv::Mat &A, const cv::Mat &B)
{
    // Ensure that A and B have the correct dimensions
    int N = A.cols;
    if (B.cols != N || A.rows != 3 || B.rows != 3)
    {
        throw std::invalid_argument("Input matrices must be 3xN.");
    }

    // 1. First compute the centroids of both sets of points A and B
    cv::Mat A_centroid = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat B_centroid = cv::Mat::zeros(3, 1, CV_64F);
    for (int i = 0; i < N; ++i)
    {
        A_centroid += A.col(i);
        B_centroid += B.col(i);
    }
    A_centroid /= N;
    B_centroid /= N;

    // 2. Translate the points such that their centroids are at the origin
    cv::Mat A_prime = cv::Mat::zeros(A.size(), A.type());
    cv::Mat B_prime = cv::Mat::zeros(B.size(), B.type());
    for (int i = 0; i < N; i++)
    {
        A_prime.col(i) = A.col(i) - A_centroid;
        B_prime.col(i) = B.col(i) - B_centroid;
    }

    // Step 3: Compute the cross-covariance matrix H
    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < N; i++)
    {
        H += A_prime.col(i) * B_prime.col(i).t();
    }

    // Step 4: Set up the 4x4 matrix K for quaternion computation
    cv::Mat K = cv::Mat::zeros(4, 4, CV_64F);
    K.at<double>(0, 0) = H.at<double>(0, 0) + H.at<double>(1, 1) + H.at<double>(2, 2); // Sum of the diagonal elements
    K.at<double>(0, 1) = H.at<double>(1, 2) - H.at<double>(2, 1);
    K.at<double>(0, 2) = H.at<double>(2, 0) - H.at<double>(0, 2);
    K.at<double>(0, 3) = H.at<double>(0, 1) - H.at<double>(1, 0);

    K.at<double>(1, 0) = H.at<double>(1, 2) - H.at<double>(2, 1);
    K.at<double>(1, 1) = H.at<double>(0, 0) - H.at<double>(1, 1) - H.at<double>(2, 2);
    K.at<double>(1, 2) = H.at<double>(0, 1) + H.at<double>(1, 0);
    K.at<double>(1, 3) = H.at<double>(0, 2) + H.at<double>(2, 0);

    K.at<double>(2, 0) = H.at<double>(2, 0) - H.at<double>(0, 2);
    K.at<double>(2, 1) = H.at<double>(0, 1) + H.at<double>(1, 0);
    K.at<double>(2, 2) = -H.at<double>(0, 0) + H.at<double>(1, 1) - H.at<double>(2, 2);
    K.at<double>(2, 3) = H.at<double>(1, 2) + H.at<double>(2, 1);

    K.at<double>(3, 0) = H.at<double>(0, 1) - H.at<double>(1, 0);
    K.at<double>(3, 1) = H.at<double>(0, 2) + H.at<double>(2, 0);
    K.at<double>(3, 2) = H.at<double>(1, 2) + H.at<double>(2, 1);
    K.at<double>(3, 3) = -H.at<double>(0, 0) - H.at<double>(1, 1) + H.at<double>(2, 2);

    // Step 5: Perform eigenvalue decomposition on matrix K
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(K, eigenvalues, eigenvectors);

    // Step 6: Extract the quaternion corresponding to the largest eigenvalue
    cv::Mat q = eigenvectors.row(0).t(); // The first row corresponds to the largest eigenvalue

    // Step 7: Convert quaternion to rotation matrix
    double qw = q.at<double>(0, 0);
    double qx = q.at<double>(1, 0);
    double qy = q.at<double>(2, 0);
    double qz = q.at<double>(3, 0);

    cv::Mat R = (cv::Mat_<double>(3, 3) << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
                 2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
                 2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy));

    // Step 8: Compute the translation vector
    cv::Mat t = B_centroid - R * A_centroid;

    // Return the rotation matrix R and translation vector t
    return {R, t};
}

bool perspectiveSolver::areSolutionsSimilar(const GrunertSolution &a, const GrunertSolution &b)
{
    // Look for overlapping euler angle solutions
    const std::vector<double> eulerAnglesA = getEulerAngles(a.rotationMatrix);
    const std::vector<double> eulerAnglesB = getEulerAngles(b.rotationMatrix);
    if (fabs(eulerAnglesA[0] - eulerAnglesB[0]) > EQUAL_TOLERANCE)
    {
        return false;
    }

    if (fabs(eulerAnglesA[1] - eulerAnglesB[1]) > EQUAL_TOLERANCE)
    {
        return false;
    }

    if (fabs(eulerAnglesA[2] - eulerAnglesB[2]) > EQUAL_TOLERANCE)
    {
        return false;
    }

    // Compare translation vectors
    if (cv::norm(a.translationVector - b.translationVector) > TRANSLATION_EQUAL_TOL)
    {
        return false;
    }

    // All checks passed, solutions are similar
    return true;
}

std::vector<std::vector<double>> perspectiveSolver::removeDistortion(const std::vector<std::vector<double>> &points)
{
    std::vector<std::vector<double>> uPoints;
    for (const auto &point : points)
    {
        // Step 1: Normalize the distorted pixel coordinates
        const double x_d = (point.at(0) - CX) / FX;
        const double y_d = (point.at(1) - CY) / FY;

        // Step 2: Initialize undistorted normalized coordinates
        double x_u = x_d;
        double y_u = y_d;

        // Iteratively solve for undistorted coordinates
        for (int iter = 0; iter < MAX_ITER_DIST; iter++)
        {
            const double x_u_prev = x_u;
            const double y_u_prev = y_u;

            const double r_u2 = x_u * x_u + y_u * y_u;
            const double D = 1 + K1 * r_u2 + K2 * r_u2 * r_u2;

            // Corrected update using x_d and y_d
            x_u = x_d / D;
            y_u = y_d / D;

            const double dx = x_u - x_u_prev;
            const double dy = y_u - y_u_prev;

            if ((dx * dx + dy * dy) < CONV_TOLERANCE_DIST * CONV_TOLERANCE_DIST)
            {
                break; // Converged
            }
        }

        // Add the undistorted normalized point to the output vector
        uPoints.push_back({x_u, y_u});
    }
    return uPoints;
}

std::vector<double> perspectiveSolver::quartic(const double b, const double c, const double d, const double e)
{
    std::vector<double> roots;

    const double p = c - 0.375 * b * b;
    const double q = 0.125 * b * b * b - 0.5 * b * c + d;
    const double m = cubic(p, 0.25 * p * p + 0.01171875 * b * b * b * b - e + 0.25 * b * d - 0.0625 * b * b * c,
                           -0.125 * q * q)[0];

    if (q == 0.0)
    {
        if (m < 0.0)
        {
            return roots;
        }

        const double sqrt_2m = sqrt(2.0 * m);
        if (-m - p > 0.0)
        {
            const double delta = sqrt(2.0 * (-m - p));
            roots.push_back(-0.25 * b + 0.5 * (sqrt_2m - delta));
            roots.push_back(-0.25 * b - 0.5 * (sqrt_2m - delta));
            roots.push_back(-0.25 * b + 0.5 * (sqrt_2m + delta));
            roots.push_back(-0.25 * b - 0.5 * (sqrt_2m + delta));
        }
        else if (-m - p == 0.0)
        {
            roots.push_back(-0.25 * b - 0.5 * sqrt_2m);
            roots.push_back(-0.25 * b + 0.5 * sqrt_2m);
        }

        return roots;
    }

    if (m < 0.0)
    {
        return roots;
    }

    const double sqrt_2m = sqrt(2.0 * m);

    if (-m - p + q / sqrt_2m >= 0.0)
    {
        const double delta = sqrt(2.0 * (-m - p + q / sqrt_2m));
        roots.push_back(0.5 * (-sqrt_2m + delta) - 0.25 * b);
        roots.push_back(0.5 * (-sqrt_2m - delta) - 0.25 * b);
    }

    if (-m - p - q / sqrt_2m >= 0.0)
    {
        const double delta = sqrt(2.0 * (-m - p - q / sqrt_2m));
        roots.push_back(0.5 * (sqrt_2m + delta) - 0.25 * b);
        roots.push_back(0.5 * (sqrt_2m - delta) - 0.25 * b);
    }

    return roots;
}

// Helper function to compute the centroid of the quadrilateral
std::vector<double> computeCentroid(const std::vector<std::vector<double>> &points)
{
    double cx = 0, cy = 0;
    for (const auto &point : points)
    {
        cx += point[0];
        cy += point[1];
    }
    cx /= points.size();
    cy /= points.size();
    return {cx, cy};
}

// Helper function to compute the angle relative to the centroid
double computeAngle(const std::vector<double> &point, const std::vector<double> &centroid)
{
    return std::atan2(point[1] - centroid[1], point[0] - centroid[0]);
}

std::vector<std::vector<double>> perspectiveSolver::reorderPoints(const std::vector<std::vector<double>> &points)
{
    // Calculate the centroid of the points
    std::vector<double> centroid = computeCentroid(points);

    // Sort points based on the angle relative to the centroid
    std::vector<std::vector<double>> sortedPoints = points;

    std::sort(sortedPoints.begin(), sortedPoints.end(),
              [&centroid](const std::vector<double> &a, const std::vector<double> &b)
              {
                  return computeAngle(a, centroid) < computeAngle(b, centroid);
              });

    return sortedPoints;
}

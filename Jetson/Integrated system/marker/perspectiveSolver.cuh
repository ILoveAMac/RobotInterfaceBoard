//
// Created by Wihan on 2024-09-27.
//

#ifndef PERSPECTIVESOLVER_CUH
#define PERSPECTIVESOLVER_CUH

// using openCV for matrix operations
#include <opencv2/opencv.hpp>
#include <cmath>
#include <math.h>
#include <complex>
#include <vector>
#include <tuple>

// Ferrari method parameters
#define MAX_ITER 1000
#define TOLERANCE 1e-6

// Grunert's Method Parameters
#define REAL_TOLERANCE 1e-3
#define EQUAL_TOLERANCE 0.05
#define TRANSLATION_EQUAL_TOL 0.05

// Un-Distortion parameters
#define MAX_ITER_DIST 1000
#define CONV_TOLERANCE_DIST 1e-6

// Camera parameters

// Camera Intrinsics and Distortion Coefficients
// K matrix
#define FX 330.582565765307
#define FY 587.847666790967
#define CX 223.296633717336
#define CY 235.301280343201

// Distortion coefficients
#define K1 0.033724646482670
#define K2 -0.117593449553171

#define P_PI 3.141592653589793238462643383279502884197

struct GrunertSolution
{
    std::vector<double> sideLengths;
    std::vector<double> angles;
    cv::Mat rotationMatrix;
    cv::Mat translationVector;
};

class perspectiveSolver
{
public:
    perspectiveSolver();

    ~perspectiveSolver();

    // This function solves the p3p problem twice,once with only 3 of the 4 points provided
    // and a second time with one of the original 3 points replaced with the unused 4th point
    // The algorithm then finds the overlapping solution from the two solution sets and returns it
    static GrunertSolution solveP4P(
        const std::vector<std::vector<double>> &worldPoints, // 3D coordinates of the reference points
        const std::vector<std::vector<double>> &imagePoints  // 2D coordinates of the points in the image
    );

    // Function to get euler angles from rotation matrix
    static std::vector<double> getEulerAngles(cv::Mat R);

    static std::tuple<std::vector<double>, std::vector<std::vector<double>>> QR_Algorithm(cv::Mat A, double tol = 1e-8, int maxItr = 1000);

private:
    // Grunert's Method
    // Solves the Perspective-3-Point (P3P) problem to find the possible lengths and angles of a tetrahedron
    // formed by a camera center and three known 3D world points, given their corresponding 2D projections.
    // Assumes the camera is calibrated, meaning the image points are in normalized coordinates.
    static std::vector<GrunertSolution> grunertsMethod(
        const std::vector<std::vector<double>> &worldPoints, // 3D coordinates of the reference points
        const std::vector<std::vector<double>> &imagePoints  // 2D coordinates of the points in the image
    );

    // Helper function for Grunert's method
    static double euclideanDistance(const std::vector<double> &point1, const std::vector<double> &point2);

    // Ferrari's Method for Quartic Equations
    static std::vector<double> quartic(double b, double c, double d, double e);

    // Cardano’s method for solving cubic equations.
    // Required by Ferrari's Method
    static std::vector<double> cubic(double b, double c, double d);

    // Function for ridged body transform
    static std::pair<cv::Mat, cv::Mat> arun(const cv::Mat &A, const cv::Mat &B);
    static std::pair<cv::Mat, cv::Mat> horn(const cv::Mat &A, const cv::Mat &B);

    // Helper function to compare GrunertSolution's and check if they are similar
    static bool areSolutionsSimilar(const GrunertSolution &a, const GrunertSolution &b);

    // Function to compute undistorted pixel coordinates
    static std::vector<std::vector<double>> removeDistortion(const std::vector<std::vector<double>> &points);

    // Function to reorder the image points such that they are always in the following order
    // top left, bottom left, bottom right, top right
    static std::vector<std::vector<double>> reorderPoints(const std::vector<std::vector<double>> &points);
};

#endif // PERSPECTIVESOLVER_CUH

#ifndef MARKERSYSTEM_CUH
#define MARKERSYSTEM_CUH

#include <iostream>
#include <vector>
#include <tuple>

// Includes for the marker system
#include <opencv2/opencv.hpp>

#include "marker/imageResizer.cuh"
#include "marker/imageToGreyConverter.cuh"
#include "marker/canny.cuh"
#include "marker/contourDetector.cuh"
#include "marker/markerIsolator.cuh"
#include "marker/perspectiveSolver.cuh"

// Parameters for the marker system
// === CANNY ===

// Standard deviation of the Gaussian filter
#define SIGMA 1.0f
// Kernel size for the Gaussian filter
#define KERNEL_SIZE 9

// Min threshold for the Canny edge detector
#define MIN_THRESHOLD 0.3f
// Max threshold for the Canny edge detector
#define MAX_THRESHOLD 0.8f
// Non-maximum suppression lower threshold
#define NMS_LOW 5.0f

// === Grey conversion ===

// Red factor
#define RED_FACTOR 0.2989f
// Green factor
#define GREEN_FACTOR 0.5870f
// Blue factor
#define BLUE_FACTOR 0.1140f

// Image dimensions
#define IMG_WIDTH 448
#define IMG_HEIGHT 448

class markerSystem
{
public:
    markerSystem();
    ~markerSystem();

    // Detect markers in the image
    // Returns the marker translation and the marker euler angles
    std::tuple<std::vector<double>, std::vector<double>> detectMarkers(float *image); // Image is assumed to be on the GPU

private:
    // Canny
    canny cannyDetector;

    // Image to grey
    imageToGreyConverter greyConverter;

    // Contour detector
    contourDetector contourDetector;

    // RDP
    RDP rdp;

    // Marker isolator
    markerIsolator markerIsolator;
};

#endif // MARKERSYSTEM_CUH
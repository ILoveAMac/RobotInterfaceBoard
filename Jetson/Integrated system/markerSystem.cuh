#ifndef MARKERSYSTEM_CUH
#define MARKERSYSTEM_CUH

// Includes for the marker system
#include <opencv2/opencv.hpp>

#include "marker/imageResizer.cuh"
#include "marker/imageToGreyConverter.cuh"
#include "marker/canny.cuh"
#include "marker/contourDetector.cuh"
#include "marker/markerIsolator.cuh"
#include "marker/perspectiveSolver.cuh"

class markerSystem
{
public:
    markerSystem();
    ~markerSystem();

private:
};

#endif // MARKERSYSTEM_CUH
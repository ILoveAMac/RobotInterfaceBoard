#ifndef YOLO_H
#define YOLO_H

#include <vector>

#include <cuda_fp16.h>

#include "Conv2D.cuh"
#include "FullyConnected.cuh"
#include "Layer.cuh"
#include "MaxPool2D.cuh"
#include "ModelLoadingHelper.cuh"
#include "aiHelperUtils.h"

class yolo
{
public:
    yolo(const std::string &modelPath);
    ~yolo();

    // Function to return final output bounding boxes
    std::vector<std::vector<__half>> getBoxPredictions(__half *inputImage);

private:
    ModelLoadingHelper MLH;
    std::vector<Layer *> model;

    __half *hostOutput;
};

#endif // YOLO_H
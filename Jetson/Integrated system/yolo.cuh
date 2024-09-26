#ifndef YOLO_H
#define YOLO_H

#include <vector>

#include "Conv2D.cuh"
#include "FullyConnected.cuh"
#include "Layer.cuh"
#include "MaxPool2D.cuh"
#include "ModelLoadingHelper.h"
#include "aiHelperUtils.h"

class yolo {
   public:
    yolo(const std::string& modelPath);
    ~yolo();

    // Function to return final output bounding boxes
    std::vector<std::vector<float>> getBoxPredictions(float* inputImage);

   private:
    ModelLoadingHelper mlh;
    std::vector<Layer*> model;

    float* hostOutput;
};

#endif  // YOLO_H
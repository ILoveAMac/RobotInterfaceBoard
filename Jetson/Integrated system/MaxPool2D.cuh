//
// Created by Wihan on 2024-09-09.
//

#ifndef MAXPOOL2D_CUH
#define MAXPOOL2D_CUH
#include <Layer.cuh>

class MaxPool2D : public Layer
{
public:
    // Constructor with input and output dimensions
    MaxPool2D(int inputHeight, int inputWidth, int inputChannels, int outputHeight, int outputWidth, int outputChannels);

    // Destructor
    ~MaxPool2D() override;

    // Function to perform max pooling
    float *forward(const float *input) override;

    // Pooling layer doesn't require loading data, so it's left empty
    void loadData() override {}

    // Override getter functions for output dimensions
    int getOutputHeight() const override;
    int getOutputWidth() const override;
    int getOutputChannels() const override;

    // scratch space for storing intermediate results
    float *d_intermediate;

private:
    // Input shape
    int inputHeight, inputWidth, inputChannels;

    // Output shape
    int outputHeight, outputWidth, outputChannels;

    // Kernel size and stride are fixed
    static constexpr int kernelSize = 2;
    static constexpr int stride = 2;
};

#endif // MAXPOOL2D_CUH

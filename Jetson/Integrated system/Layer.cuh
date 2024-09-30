//
// Created by Wihan on 2024-09-09.
//

#ifndef LAYER_CUH
#define LAYER_CUH

#include <cuda_fp16.h>

class Layer
{
public:
    // Virtual destructor to ensure proper cleanup for derived classes
    virtual ~Layer() {}

    // Virtual function to load data, can be optionally overridden
    virtual void loadData() = 0; // Make it pure virtual if you want it to be implemented by all derived classes

    // Pure virtual function for forward pass
    virtual __half *forward(const __half *input) = 0; // Mark as pure virtual

    // Pure virtual getter functions for output dimensions
    virtual int getOutputHeight() const = 0;
    virtual int getOutputWidth() const = 0;
    virtual int getOutputChannels() const = 0;
};

#endif // LAYER_CUH

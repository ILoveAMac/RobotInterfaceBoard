//
// Created by Wihan on 2024-09-24.
//

#include "canny.cuh"

canny::canny(const int width, const int height, const float sigma, const int kernelSize, const float minThreshold, const float maxThreshold, const int nmsLowerThreshold)
    : width(width), height(height), minThreshold(minThreshold), maxThreshold(maxThreshold), nmsLowerThreshold(nmsLowerThreshold),
      blur(width, height, kernelSize, sigma),
      applySobel(width, height) {}

canny::~canny() {}

float *canny::applyCanny(const float *input) const
{
    // Step 1: Apply Gaussian blur to reduce noise
    const float *d_blurredImg = blur.blur(input, width, height);

    // Step 2: Apply Sobel operator to get gradient magnitude and direction
    const std::vector<float *> sobelResults = applySobel.apply(d_blurredImg);
    const float *gradientMagnitude = sobelResults[0]; // Contains the gradient magnitude in CPU memory
    const float *gradientDirection = sobelResults[1]; // Contains the gradient direction in CPU memory

    // Step 3: Apply Non-Maximum Suppression to thin edges
    float *nonMaxSuppressed = non_max_suppression(gradientMagnitude, gradientDirection);

    // Step 4: Apply Hysteresis to finalize edges
    float *finalEdges = hysteresis(nonMaxSuppressed);

    // Cleanup intermediate results
    delete[] nonMaxSuppressed;  // Host memory allocated in non_max_suppression
    delete[] gradientMagnitude; // Host memory allocated in applySobel
    delete[] gradientDirection; // Host memory allocated in applySobel

    // Return the final edge-detected image in host memory
    return finalEdges;
}

float *canny::non_max_suppression(const float *mag, const float *theta) const
{
    // Allocate memory for the output image
    const auto suppressed = new float[width * height];

    // Loop through every pixel value and make them zero
    for (int i = 0; i < width * height; i++)
    {
        suppressed[i] = 0;
    }

    // Iterate over every pixel, ignoring the image borders
    for (int y = 1; y < height - 1; ++y)
    {
        for (int x = 1; x < width - 1; ++x)
        {
            const int idx = y * width + x;
            float angle = theta[idx] * (180.0f / PI); // Convert to degrees

            // Adjust the angle to be within [0, 180] range
            if (angle < 0)
            {
                angle += 180;
            }

            float q = 0.0f;
            float r = 0.0f;

            // Determine the interpolation direction based on the angle
            if ((angle >= 0 && angle < 22.5) || (angle >= 157.5 && angle <= 180))
            {
                // 0 degrees (horizontal)
                q = mag[idx + 1]; // Right neighbor
                r = mag[idx - 1]; // Left neighbor
            }
            else if (angle >= 22.5 && angle < 67.5)
            {
                // 45 degrees
                q = mag[idx + width + 1]; // Bottom-right neighbor
                r = mag[idx - width - 1]; // Top-left neighbor
            }
            else if (angle >= 67.5 && angle < 112.5)
            {
                // 90 degrees (vertical)
                q = mag[idx + width]; // Bottom neighbor
                r = mag[idx - width]; // Top neighbor
            }
            else if (angle >= 112.5 && angle < 157.5)
            {
                // 135 degrees
                q = mag[idx + width - 1]; // Bottom-left neighbor
                r = mag[idx - width + 1]; // Top-right neighbor
            }

            // Suppress the current pixel if it's not greater than its neighbors
            if (mag[idx] >= q && mag[idx] >= r)
            {
                suppressed[idx] = mag[idx]; // Keep the pixel value
            }
            else
            {
                suppressed[idx] = 0.0f; // Suppress the pixel
            }
        }
    }
    return suppressed;
}

float *canny::hysteresis(const float *input) const
{
    // Allocate memory for the output image
    float *output = new float[width * height];

    // Step 1: Apply double thresholding
    for (int x = 0; x < height; ++x)
    {
        for (int y = 0; y < width; ++y)
        {
            int idx = x * width + y;
            if (input[idx] > maxThreshold)
            {
                // Definitely an edge
                output[idx] = 255.0f;
            }
            else if (input[idx] < minThreshold)
            {
                // Definitely not an edge
                output[idx] = 0.0f;
            }
            else
            {
                // Maybe an edge
                output[idx] = 128.0f;
            }
        }
    }

    // Step 2: Resolve potential edges
    for (int x = 1; x < height - 1; ++x)
    {
        for (int y = 1; y < width - 1; ++y)
        {
            int idx = x * width + y;
            // For each strong edge
            if (output[idx] == 255.0f)
            {
                // Use a stack to track potential edges
                std::stack<std::pair<int, int>> potentialEdges;

                // Check the 8 neighbors
                for (int i = -1; i <= 1; ++i)
                {
                    for (int j = -1; j <= 1; ++j)
                    {
                        if (output[(x + i) * width + (y + j)] == 128.0f)
                        {
                            output[(x + i) * width + (y + j)] = 255.0f;
                            potentialEdges.push({x + i, y + j});
                        }
                    }
                }

                // Recursively check all potential edges until the stack is empty
                while (!potentialEdges.empty())
                {
                    auto &[px, py] = potentialEdges.top();
                    potentialEdges.pop();

                    for (int i = -1; i <= 1; ++i)
                    {
                        for (int j = -1; j <= 1; ++j)
                        {
                            int neighborX = px + i;
                            int neighborY = py + j;
                            if (neighborX > 0 && neighborX < height - 1 && neighborY > 0 && neighborY < width - 1)
                            {
                                int neighborIdx = neighborX * width + neighborY;
                                if (output[neighborIdx] == 128.0f)
                                {
                                    output[neighborIdx] = 255.0f;
                                    potentialEdges.push({neighborX, neighborY});
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Step 3: Clean up remaining potential edges
    for (int x = 0; x < height; ++x)
    {
        for (int y = 0; y < width; ++y)
        {
            int idx = x * width + y;
            if (output[idx] == 128.0f)
            {
                output[idx] = 0.0f;
            }
        }
    }

    return output;
}
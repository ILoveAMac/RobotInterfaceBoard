#include "yolo.cuh"

// Function to convert 2D vector of __half to 2D vector of float
std::vector<std::vector<float>> convertHalfToFloat(const std::vector<std::vector<__half>> &halfVec)
{
    // Determine the size of the outer vector
    size_t numRows = halfVec.size();

    // Initialize the output 2D vector with the same number of rows
    std::vector<std::vector<float>> floatVec;
    floatVec.reserve(numRows); // Reserve space to avoid multiple reallocations

    // Iterate over each row in the input 2D vector
    for (const auto &row : halfVec)
    {
        // Determine the size of the current row
        size_t numCols = row.size();

        // Initialize the inner vector for floats with the same number of columns
        std::vector<float> floatRow;
        floatRow.reserve(numCols); // Reserve space for efficiency

        // Iterate over each __half element in the current row
        for (const auto &h : row)
        {
            // Convert __half to float and add to the floatRow
            floatRow.emplace_back(__half2float(h));
        }

        // Add the converted row to the floatVec
        floatVec.emplace_back(std::move(floatRow));
    }

    return floatVec;
}

yolo::yolo(const std::string &modelPath) : MLH(ModelLoadingHelper(modelPath))
{
    // Build the model
    this->model.push_back(new Conv2D(5, 32, 2, 2, "0", MLH, 224, 224, 32, 448, 448, 3));
    this->model.push_back(new MaxPool2D(224, 224, 32, 112, 112, 32));
    this->model.push_back(new Conv2D(3, 64, 1, 1, "2", MLH, 112, 112, 64, 112, 112, 32));
    this->model.push_back(new MaxPool2D(112, 112, 64, 56, 56, 64));
    this->model.push_back(new Conv2D(1, 64, 1, 0, "4", MLH, 56, 56, 64, 56, 56, 64));
    this->model.push_back(new Conv2D(3, 128, 1, 1, "5", MLH, 56, 56, 128, 56, 56, 64));
    this->model.push_back(new Conv2D(1, 128, 1, 0, "6", MLH, 56, 56, 128, 56, 56, 128));
    this->model.push_back(new Conv2D(3, 256, 1, 1, "7", MLH, 56, 56, 256, 56, 56, 128));
    this->model.push_back(new MaxPool2D(56, 56, 256, 28, 28, 256));
    this->model.push_back(new Conv2D(1, 128, 1, 0, "9", MLH, 28, 28, 128, 28, 28, 256));
    this->model.push_back(new Conv2D(3, 256, 1, 1, "10", MLH, 28, 28, 256, 28, 28, 128));
    this->model.push_back(new Conv2D(1, 256, 1, 0, "11", MLH, 28, 28, 256, 28, 28, 256));
    this->model.push_back(new Conv2D(3, 256, 1, 1, "12", MLH, 28, 28, 256, 28, 28, 256));
    this->model.push_back(new MaxPool2D(28, 28, 256, 14, 14, 256));
    this->model.push_back(new Conv2D(1, 256, 1, 0, "14", MLH, 14, 14, 256, 14, 14, 256));
    this->model.push_back(new Conv2D(3, 256, 1, 1, "15", MLH, 14, 14, 256, 14, 14, 256));
    this->model.push_back(new Conv2D(3, 256, 2, 1, "16", MLH, 7, 7, 256, 14, 14, 256));
    this->model.push_back(new Conv2D(3, 64, 1, 1, "17", MLH, 7, 7, 64, 7, 7, 256));
    // Fully connected layers
    this->model.push_back(new FullyConnected(64 * 7 * 7, 512, MLH, "1", true));
    this->model.push_back(new FullyConnected(512, 7 * 7 * 2 * 5, MLH, "4", false));

    // Load the weights
    for (const auto layer : model)
    {
        layer->loadData();
    }

    // Allocate memory for the output
    this->hostOutput = static_cast<__half *>(malloc(7 * 7 * 10 * sizeof(__half)));
}

yolo::~yolo()
{
    // Clean up layers
    for (auto layer : this->model)
    {
        delete layer;
    }

    if (this->hostOutput)
    {
        free(this->hostOutput);
    }
}

std::vector<std::vector<float>> yolo::getBoxPredictions(__half *inputImage)
{
    __half *output = nullptr;
    for (const auto &layer : this->model)
    {
        output = layer->forward(inputImage);
        inputImage = output;
        cudaDeviceSynchronize();
    }

    // Copy the data from GPU to CPU
    cudaMemcpy(this->hostOutput, output, 7 * 7 * 10 * sizeof(__half), cudaMemcpyDeviceToHost);

    // Convert half to float
    std::vector<std::vector<float>> floatOutput = convertHalfToFloat(aiHelperUtils::reshapeOutput(this->hostOutput, 7 * 7 * 10));

    // Get the final bounding boxes
    return aiHelperUtils::getFinalBoundingBoxes(floatOutput);
}
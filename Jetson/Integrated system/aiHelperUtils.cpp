#include "aiHelperUtils.h"

aiHelperUtils::aiHelperUtils() {}

aiHelperUtils::~aiHelperUtils() {}

std::vector<std::vector<float>> aiHelperUtils::getFinalBoundingBoxes(const float *detections)
{
    // Vecttor to store the bounding boxes
    std::vector<std::vector<float>> boxes;
    for (int i = 0; i < GRID_SIZE; i++)
    {
        for (int j = 0; j < GRID_SIZE; j++)
        {
            for (int b = 0; b < NUM_BOXES; b++)
            { // Loop over each bounding box in the cell
                // Calculate the starting index for the current bounding box (5 values per bounding box)
                const int index = (i * GRID_SIZE + j) * (5 * NUM_BOXES) + b * 5;

                // Extract all of the bounding boxes and store them in the boxes vector
                float x_offset = detections[index];           // x relative to the grid cell
                float y_offset = detections[index + 1];       // y relative to the grid cell
                float w = detections[index + 2] * IMG_HEIGHT; // Width relative to image size
                float h = detections[index + 3] * IMG_WIDTH;  // Height relative to image size
                float c = detections[index + 4];              // Confidence for the bounding box

                float x_center = (j + x_offset) * (IMG_HEIGHT / GRID_SIZE); // Absolute x-center
                float y_center = (i + y_offset) * (IMG_WIDTH / GRID_SIZE);  // Absolute y-center

                std::vector<float> box = {x_center, y_center, w, h, c};

                // Push the box to the boxes vector
                boxes.push_back(box);
            }
        }
    }

    // Perform non-maximum suppression to remove overlapping bounding boxes
    return aiHelperUtils::nonMaxSuppression(boxes);
}

cv::Mat aiHelperUtils::drawBoundingBoxes(cv::Mat frame, std::vector<std::vector<float>> boxes)
{
    for (std::vector<float> box : boxes)
    {
        // Extract the bounding box data
        float x_center = box[0];
        float y_center = box[1];
        float w = box[2];
        float h = box[3];
        float c = box[4];

        // Calculate the top-left and bottom-right points of the bounding box
        int x1 = static_cast<int>(x_center - w / 2.0);
        int y1 = static_cast<int>(y_center - h / 2.0);
        int x2 = static_cast<int>(x_center + w / 2.0);
        int y2 = static_cast<int>(y_center + h / 2.0);

        // // Draw the bounding box on the image
        // cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

        // // Convert confidence to string and format it
        // std::string label = cv::format("%.2f", c);

        // // Set the position for the confidence label (above the top-left corner of the bounding box)
        // int baseline = 0;
        // cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        // int label_x = std::max(x1, 0);                     // Ensure the label is inside the image boundaries
        // int label_y = std::max(y1 - label_size.height, 0); // Display above the box

        // // Draw the label background rectangle
        // cv::rectangle(frame, cv::Point(label_x, label_y), cv::Point(label_x + label_size.width, label_y + label_size.height + baseline),
        //               cv::Scalar(0, 255, 0), cv::FILLED);

        // // Put the confidence text on the image
        // cv::putText(frame, label, cv::Point(label_x, label_y + label_size.height),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    return frame;
}

std::vector<std::vector<float>> aiHelperUtils::nonMaxSuppression(std::vector<std::vector<float>> boxes)
{
    // Remove any predictions from the list that have a confidence score less than the threshold
    // confidences are stored in the last element of the vector
    std::vector<std::vector<float>> filtered_boxes;
    for (int i = 0; i < boxes.size(); i++)
    {
        if (boxes[i][4] > CONF_THRESH)
        {
            filtered_boxes.push_back(boxes[i]);
        }
    }

    // Sort the boxes based on their confidence scores, highest first
    std::sort(filtered_boxes.begin(), filtered_boxes.end(), [](const std::vector<float> &a, const std::vector<float> &b)
              { return a[4] > b[4]; });

    // Perform non-maximum suppression
    std::vector<std::vector<float>> final_boxes;
    while (filtered_boxes.size() > 0)
    {
        std::vector<float> chosenBox = filtered_boxes[0];
        // Add the chosen box to the final list
        final_boxes.push_back(chosenBox);
        // Remove the chosen box from the list
        filtered_boxes.erase(filtered_boxes.begin());

        // Remove any boxes that have a high IoU with the chosen box
        filtered_boxes.erase(std::remove_if(filtered_boxes.begin(), filtered_boxes.end(),
                                            [chosenBox](const std::vector<float> &box)
                                            {
                                                return aiHelperUtils::iou(chosenBox, box) > IOU_NMS_THRESH;
                                            }),
                             filtered_boxes.end());
    }

    return final_boxes;
}

float aiHelperUtils::iou(std::vector<float> box1, std::vector<float> box2)
{
    // Calculate the intersection area of the two boxes
    float box1_x1 = box1[0] - box1[2] / 2;
    float box1_x2 = box1[0] + box1[2] / 2;
    float box1_y1 = box1[1] - box1[3] / 2;
    float box1_y2 = box1[1] + box1[3] / 2;

    float box2_x1 = box2[0] - box2[2] / 2;
    float box2_x2 = box2[0] + box2[2] / 2;
    float box2_y1 = box2[1] - box2[3] / 2;
    float box2_y2 = box2[1] + box2[3] / 2;

    float x1 = std::max(box1_x1, box2_x1);
    float y1 = std::max(box1_y1, box2_y1);
    float x2 = std::min(box1_x2, box2_x2);
    float y2 = std::min(box1_y2, box2_y2);

    float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);

    float box1_area = fabs(box1[2] * box1[3]);
    float box2_area = fabs(box2[2] * box2[3]);

    return intersection / (box1_area + box2_area - intersection + 1E-6);
}

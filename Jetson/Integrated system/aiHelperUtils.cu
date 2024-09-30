#include "aiHelperUtils.cuh"

aiHelperUtils::aiHelperUtils() {}

aiHelperUtils::~aiHelperUtils() {}

std::vector<std::vector<__half>> aiHelperUtils::getFinalBoundingBoxes(const __half *detections)
{
    // Vecttor to store the bounding boxes
    std::vector<std::vector<__half>> boxes;
    for (int i = 0; i < GRID_SIZE; i++)
    {
        for (int j = 0; j < GRID_SIZE; j++)
        {
            for (int b = 0; b < NUM_BOXES; b++)
            { // Loop over each bounding box in the cell
                // Calculate the starting index for the current bounding box (5 values per bounding box)
                const int index = (i * GRID_SIZE + j) * (5 * NUM_BOXES) + b * 5;

                // Extract all of the bounding boxes and store them in the boxes vector
                __half x_offset = detections[index];                                // x relative to the grid cell
                __half y_offset = detections[index + 1];                            // y relative to the grid cell
                __half w = __hmul(detections[index + 2], __float2half(IMG_HEIGHT)); // Width relative to image size
                __half h = __hmul(detections[index + 3], __float2half(IMG_WIDTH));  // Height relative to image size
                __half c = detections[index + 4];                                   // Confidence for the bounding box

                __half x_center = __hmul(__hadd(__float2half(j), x_offset), __float2half(IMG_HEIGHT / GRID_SIZE)); // Absolute x-center
                __half y_center = __hmul(__hadd(__float2half(i), y_offset), __float2half(IMG_WIDTH / GRID_SIZE));  // Absolute y-center

                std::vector<__half> box = {x_center, y_center, w, h, c};

                // Push the box to the boxes vector
                boxes.push_back(box);
            }
        }
    }

    // Perform non-maximum suppression to remove overlapping bounding boxes
    return aiHelperUtils::nonMaxSuppression(boxes);
}

cv::Mat aiHelperUtils::drawBoundingBoxes(cv::Mat frame, std::vector<std::vector<__half>> boxes)
{
    for (std::vector<__half> box : boxes)
    {
        // Extract the bounding box data
        __half x_center = box[0];
        __half y_center = box[1];
        __half w = box[2];
        __half h = box[3];
        __half c = box[4];

        // Calculate the top-left and bottom-right points of the bounding box
        __half half_two = __float2half(2.0);
        int x1 = static_cast<int>(__hsub(x_center, __hdiv(w, half_two)));
        int y1 = static_cast<int>(__hsub(y_center, __hdiv(h, half_two)));
        int x2 = static_cast<int>(__hadd(x_center, __hdiv(w, half_two)));
        int y2 = static_cast<int>(__hadd(y_center, __hdiv(h, half_two)));

        // Draw the bounding box on the image
        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

        // Convert confidence to string and format it
        std::string label = cv::format("%.2f", c);

        // Set the position for the confidence label (above the top-left corner of the bounding box)
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        int label_x = std::max(x1, 0);                     // Ensure the label is inside the image boundaries
        int label_y = std::max(y1 - label_size.height, 0); // Display above the box

        // Draw the label background rectangle
        cv::rectangle(frame, cv::Point(label_x, label_y), cv::Point(label_x + label_size.width, label_y + label_size.height + baseline),
                      cv::Scalar(0, 255, 0), cv::FILLED);

        // Put the confidence text on the image
        cv::putText(frame, label, cv::Point(label_x, label_y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    return frame;
}

std::vector<std::vector<__half>> aiHelperUtils::nonMaxSuppression(std::vector<std::vector<__half>> boxes)
{
    // Remove any predictions from the list that have a confidence score less than the threshold
    // confidences are stored in the last element of the vector
    std::vector<std::vector<__half>> filtered_boxes;
    for (int i = 0; i < boxes.size(); i++)
    {
        if (__half2float(boxes[i][4]) > CONF_THRESH)
        {
            filtered_boxes.push_back(boxes[i]);
        }
    }

    // Sort the boxes based on their confidence scores, highest first
    std::sort(filtered_boxes.begin(), filtered_boxes.end(), [](const std::vector<__half> &a, const std::vector<__half> &b)
              { return a[4] > b[4]; });

    // Perform non-maximum suppression
    std::vector<std::vector<__half>> final_boxes;
    while (filtered_boxes.size() > 0)
    {
        std::vector<__half> chosenBox = filtered_boxes[0];
        // Add the chosen box to the final list
        final_boxes.push_back(chosenBox);
        // Remove the chosen box from the list
        filtered_boxes.erase(filtered_boxes.begin());

        // Remove any boxes that have a high IoU with the chosen box
        filtered_boxes.erase(std::remove_if(filtered_boxes.begin(), filtered_boxes.end(),
                                            [chosenBox](const std::vector<__half> &box)
                                            {
                                                return aiHelperUtils::iou(chosenBox, box) > IOU_NMS_THRESH;
                                            }),
                             filtered_boxes.end());
    }

    return final_boxes;
}

__half aiHelperUtils::iou(std::vector<__half> box1, std::vector<__half> box2)
{
    // Calculate the intersection area of the two boxes
    __half half_two = __float2half(2.0);
    __half box1_x1 = __hsub(box1[0], __hdiv(box1[2], half_two));
    __half box1_x2 = __hadd(box1[0], __hdiv(box1[2], half_two));
    __half box1_y1 = __hsub(box1[1], __hdiv(box1[3], half_two));
    __half box1_y2 = __hadd(box1[1], __hdiv(box1[3], half_two));

    __half box2_x1 = __hsub(box2[0], __hdiv(box2[2], half_two));
    __half box2_x2 = __hadd(box2[0], __hdiv(box2[2], half_two));
    __half box2_y1 = __hsub(box2[1], __hdiv(box2[3], half_two));
    __half box2_y2 = __hadd(box2[1], __hdiv(box2[3], half_two));

    __half x1 = hmax(box1_x1, box2_x1);
    __half y1 = hmax(box1_y1, box2_y1);
    __half x2 = hmin(box1_x2, box2_x2);
    __half y2 = hmin(box1_y2, box2_y2);

    __half zero = __float2half(0.0f);
    __half intersection = __hmul(__hmax(zero, __hsub(x2, x1)), __hmax(zero, __hsub(y2, y1)));

    __half box1_area = __hmul(fabs(box1[2]), fabs(box1[3]));
    __half box2_area = __hmul(fabs(box2[2]), fabs(box2[3]));

    return __hdiv(intersection, __hsub(__hadd(box1_area, box2_area), __hadd(intersection, __float2half(1E-6))));
}

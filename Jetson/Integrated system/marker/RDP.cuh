//
// Created by Wihan on 2024-09-25.
//

#ifndef RDP_CUH
#define RDP_CUH
#include <vector>

class RDP {
public:
    RDP();
    ~RDP();

    static std::vector<std::tuple<int, int>> ramerDouglasPeucker(const std::vector<std::tuple<int, int>> &contour, float epsilon);
    static float arcLength(const std::vector<std::tuple<int, int>> &contour);
private:
    static float perpendicularDistance(const std::tuple<int, int> &point, const std::tuple<int, int> &start, const std::tuple<int, int> &end);
    static float euclideanDistance(const std::tuple<int, int> &start, const std::tuple<int, int> &end);
};


#endif //RDP_CUH

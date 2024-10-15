//
// Created by Wihan on 2024-09-25.
//

#ifndef MARKERISOLATOR_CUH
#define MARKERISOLATOR_CUH
#include <vector>
#include "RDP.cuh"

class markerIsolator {
public:
  markerIsolator();
  ~markerIsolator();

  static std::vector<std::vector<std::vector<int>>> isolateMarkersFromContours(const std::vector<std::vector<int>> &contours);
private:
  static std::vector<std::vector<std::vector<int>>> removeInvalidMarkers(const std::vector<std::vector<std::vector<int>>> &potentialMarkers);
  static std::tuple<int, int> getQuadCentroid(const std::vector<std::tuple<int, int>> &quad);
  static float getQuadAspectRatio(const std::vector<std::tuple<int, int>> &quad);

  static std::vector<std::tuple<int, int>>  getSimplifiedPolygonFromContour(std::vector<int> contour);
  static bool isQuad(const std::vector<std::tuple<int, int>> &quad);

  static float calculateArea(const std::vector<std::tuple<int, int>> &quad);

  static std::vector<std::tuple<int, int>> ensureClosedContour(const std::vector<std::tuple<int, int>>& contour);
  static float euclideanDistance(const std::tuple<int, int> &start, const std::tuple<int, int> &end);

  static bool isQuadAInsideQuadB(const std::vector<std::tuple<int, int>> &quadA, const std::vector<std::tuple<int, int>> &quadB);
  static bool isPointInsidePolygon(const std::tuple<int, int>& point, const std::vector<std::tuple<int, int>>& polygon);
};



#endif //MARKERISOLATOR_CUH

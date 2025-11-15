#pragma once
#include <opencv2/opencv.hpp>
#include "armor_detector/types.hpp"

class PNPSolver {
public:
    PNPSolver() = default;

    // TODO: solvePnP 算距离与姿态
    bool solve(const Armor &armor, double &yaw, double &pitch, double &distance) {
        (void)armor;
        yaw = pitch = distance = 0.0;
        return false; 
    }
};

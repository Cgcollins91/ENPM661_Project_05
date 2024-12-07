/* 
 * File: quaternion.h
 * Description: A class for working with quaternion data.
 * Author: James Fehrmann
 * Created: 2024-09
 * License: MIT License
 * Version: 1.0.0
 *
 */

#pragma once

#include <cmath>
#include <string>
#include <vector>

class Quaternion {
  public:
    double w, x, y, z;
    Quaternion(double w, double x, double y, double z);
    std::vector<double> asVector() const;
    std::string toString() const;
    std::vector<std::vector<double>> asTransformationMatrix(std::vector<double> position) const;
    [[nodiscard]] double magnitude() const;
    [[nodiscard]] double magnitudeSquared() const;
    [[nodiscard]] Quaternion normal() const;
    [[nodiscard]] Quaternion conjugate() const;
    [[nodiscard]] Quaternion inverse() const;
    [[nodiscard]] Quaternion relative(const Quaternion &q2) const;
    [[nodiscard]] Quaternion multiply(const Quaternion &q2) const;
    [[nodiscard]] double angleToRad(const Quaternion &q2) const;
    [[nodiscard]] double angleToDeg(const Quaternion &q2) const;
    [[nodiscard]] double yawAngleRad() const;
    [[nodiscard]] double yawAngleDeg() const;
    Quaternion operator*(const Quaternion &q) const;
    Quaternion operator*(double scalar) const;
    friend Quaternion operator*(double scalar, const Quaternion &q2);

  private:
};

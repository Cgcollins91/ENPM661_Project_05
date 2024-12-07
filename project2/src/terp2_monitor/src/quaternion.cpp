/*
 * File: quaternion.cpp
 * Description: A class for working with quaternion data.
 * Author: James Fehrmann
 * Created: 2024-09
 * License: MIT License
 * Version: 1.0.0
 *
 */

#include "quaternion.h"
#include <algorithm>
#include <iomanip>
#include <sstream>

Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

std::string Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{" << w << ", " << x << ", " << y << ", " << z << "}";
    return oss.str();
}

std::vector<std::vector<double>> Quaternion::asTransformationMatrix(std::vector<double> position) const {
    double r11 = 1 - (2 * y * y) - (2 * z * z);
    double r12 = (2 * x * y) - (2 * z * w);
    double r13 = (2 * x * z) + (2 * y * w);
    double r21 = (2 * x * y) + (2 * z * w);
    double r22 = 1 - (2 * x * x) - (2 * z * z);
    double r23 = (2 * y * z) - (2 * x * w);
    double r31 = (2 * x * z) - (2 * y * w);
    double r32 = (2 * y * z) + (2 * x * w);
    double r33 = 1 - (2 * x * x) - (2 * y * y);
    if (r11 < 0.00001) r11 = 0.0;
    if (r12 < 0.00001) r12 = 0.0;
    if (r13 < 0.00001) r13 = 0.0;
    if (r21 < 0.00001) r21 = 0.0;
    if (r22 < 0.00001) r22 = 0.0;
    if (r23 < 0.00001) r23 = 0.0;
    if (r31 < 0.00001) r31 = 0.0;
    if (r32 < 0.00001) r32 = 0.0;
    if (r33 < 0.00001) r33 = 0.0;
    std::vector<double> row1 = {r11, r12, r13, position[0]};
    std::vector<double> row2 = {r21, r22, r23, position[1]};
    std::vector<double> row3 = {r31, r32, r33, position[2]};
    std::vector<double> row4 = {0.0, 0.0, 0.0, 1.0};
    std::vector<std::vector<double>> transform = {row1, row2, row3, row4};
    return transform;
}

Quaternion Quaternion::operator*(const Quaternion &q) const {
    return multiply(q);
}
Quaternion Quaternion::operator*(double scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

// friend
Quaternion operator*(double scalar, const Quaternion &q) {
    return Quaternion(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
}

Quaternion Quaternion::multiply(const Quaternion &q2) const {
    double tw = w * q2.w - x * q2.x - y * q2.y - z * q2.z;
    double tx = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
    double ty = w * q2.y - x * q2.z + y * q2.w + z * q2.x;
    double tz = w * q2.z + x * q2.y - y * q2.x + z * q2.w;
    return Quaternion(tw, tx, ty, tz);
}

Quaternion Quaternion::conjugate() const {
    return {w, -x, -y, -z};
}

Quaternion Quaternion::inverse() const {
    return conjugate() * (1 / magnitudeSquared());
}

double Quaternion::magnitudeSquared() const {
    return w * w + x * x + y * y + z * z;
}

double Quaternion::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

Quaternion Quaternion::normal() const {
    double m = magnitude();
    return Quaternion(w / m, x / m, y / m, z / m);
}

Quaternion Quaternion::relative(const Quaternion &q2) const {
    return inverse().multiply(q2.normal());
}

double Quaternion::angleToRad(const Quaternion &q2) const {
    return 2.0 * std::acos(std::clamp(relative(q2).w, -1.0, 1.0));
}

double Quaternion::angleToDeg(const Quaternion &q2) const {
    return angleToRad(q2) * 180 / 3.14159265358979323846;
}

double Quaternion::yawAngleRad() const {
    double _w = normal().w;
    double _x = normal().x;
    double _y = normal().y;
    double _z = normal().z;
    return std::atan2(2 * (_w * _z + _x * _y), 1.0 - 2.0 * (_y * _y + _z * _z));
}

double Quaternion::yawAngleDeg() const {
    return yawAngleRad() * 180 / 3.14159265358979323846;
}

std::vector<double> Quaternion::asVector() const {
    std::vector<double> v;
    v.push_back(w);
    v.push_back(x);
    v.push_back(y);
    v.push_back(z);
    return v;
}

/* 
 * File: quaternion.cpp
 * Description: A class for working with quaternion data.
 * Author: James Fehrmann
 * Created: 2024-09
 * License: MIT License
 * Version: 1.0.0
 *
 */

#include <sstream>
#include <algorithm>
#include <iomanip>
#include "quaternion.h"

Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

std::string Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{" << w << ", " << x << ", " << y << ", " << z << "}";
    return oss.str();
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

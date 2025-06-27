#pragma once
#include <Eigen/Dense>

class KalmanFilter2D {
public:
    KalmanFilter2D();
    void init(double x, double y, double vx = 0, double vy = 0);
    void predict(double dt);
    void update(double x, double y);
    double getX() const;
    double getY() const;
    double getVX() const;
    double getVY() const;
private:
    Eigen::Vector4d state_; // [x, y, vx, vy]
    Eigen::Matrix4d P_;     // Covariance
    Eigen::Matrix4d F_;     // State transition
    Eigen::Matrix<double, 2, 4> H_; // Measurement
    Eigen::Matrix2d R_;     // Measurement noise
    Eigen::Matrix4d Q_;     // Process noise
    bool initialized_;
};

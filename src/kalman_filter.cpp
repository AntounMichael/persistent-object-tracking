#include "kalman_filter.hpp"

KalmanFilter2D::KalmanFilter2D() : initialized_(false) {
    state_.setZero();
    P_.setIdentity();
    F_.setIdentity();
    H_.setZero();
    H_(0,0) = 1; H_(1,1) = 1;
    R_.setIdentity();
    Q_.setIdentity();
}

void KalmanFilter2D::init(double x, double y, double vx, double vy) {
    state_ << x, y, vx, vy;
    P_ = Eigen::Matrix4d::Identity() * 1.0;
    initialized_ = true;
}

void KalmanFilter2D::predict(double dt) {
    if (!initialized_) return;
    F_.setIdentity();
    F_(0,2) = dt;
    F_(1,3) = dt;
    Q_ = Eigen::Matrix4d::Identity() * 0.01;
    state_ = F_ * state_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter2D::update(double x, double y) {
    if (!initialized_) return;
    Eigen::Vector2d z;
    z << x, y;
    Eigen::Vector2d y_res = z - H_ * state_;
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
    state_ = state_ + K * y_res;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
}

double KalmanFilter2D::getX() const { return state_(0); }
double KalmanFilter2D::getY() const { return state_(1); }
double KalmanFilter2D::getVX() const { return state_(2); }
double KalmanFilter2D::getVY() const { return state_(3); }

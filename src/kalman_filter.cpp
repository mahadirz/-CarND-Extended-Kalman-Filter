#include "kalman_filter.h"
#include "tools.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
     * TODO: predict the state
     */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     * TODO: update the state by using Kalman Filter equations
     */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    KalmanFilter::CommonUpdate(y);
}

void KalmanFilter::CommonUpdate(const VectorXd &y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

    //cout << "H_ " << H_ << " H_ " << H_.size() << endl << endl;
    //cout << "P_ " << P_ << " P_ " << P_.size() << endl << endl;
    //cout << "Ht " << Ht << " P_ " << Ht.size() << endl << endl;
    //cout << "R_ " << R_ << " R_ " << R_.size() << endl << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     * TODO: update the state by using Extended Kalman Filter equations
     */
    VectorXd z_pred(3);
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    double ro = sqrt(pow(px, 2) + pow(py, 2));
    double phi = atan2(py, px);
    // avoiding division by zero
    if (ro < 1e-7) {
        ro = 1e-7;
    }
    double ro_dot = (px * vx + py * vy) / ro;
    z_pred << ro, phi, ro_dot;
    VectorXd y = z - z_pred;
    // Normalizing Angle
    // https://stackoverflow.com/a/36222552/11189869
    if (y(1) > M_PI | y(1) < -M_PI) {
        //cout << "Normalizing " << y(1) << endl;
        y(1) = atan2(sin(y(1)), cos(y(1)));
        //cout << "after Normalizing " << y(1) << endl;
    }

    KalmanFilter::CommonUpdate(y);
}

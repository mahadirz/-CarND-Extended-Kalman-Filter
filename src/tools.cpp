#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "estimations.size() " << estimations.size() << endl;
        cout << "ground_truth.size() " << ground_truth.size() << endl;
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    // accumulate squared residuals
    for (unsigned int i=0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        // coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    // calculate the mean
    rmse = rmse/estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);
    Hj << 0,0,0,0,
            0,0,0,0,
            0,0,0,0;
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float a = pow(px,2) + pow(py,2);
    float b = sqrt(a);
    // let a = px^2 + py^2
    // a^(3/2) = = a^(1/2) * a^(2/2) =  sqrt(a) * a
    // float c = b * a;
    float c = pow(a, 3.0/2.0);

    // check division by zero
    if(a == 0.0){
        //cout << "Division by zero!";
        return Hj;
    }

    // compute the Jacobian matrix
    Hj(0,0) =  px / b;
    Hj(0,1) =  py / b;
    Hj(0,2) = 0;
    Hj(0,3) = 0;

    Hj(1,0) =  -py / a;
    Hj(1,1) =  px / a;
    Hj(1,2) = 0;
    Hj(1,3) = 0;

    Hj(2,0) =  py * (vx*py - vy*px) / c;
    Hj(2,1) =  px * (vy*px - vx*py) / c;
    Hj(2,2) = px / b;
    Hj(2,3) = py / b;


    return Hj;
}

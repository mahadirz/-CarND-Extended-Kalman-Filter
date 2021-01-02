#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

/**
 * https://stackoverflow.com/a/868894/11189869
 * @param begin
 * @param end
 * @param option
 * @return
 */
char *getCmdOption(char **begin, char **end, const std::string &option) {
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return *itr;
    }
    return 0;
}

/**
 * https://stackoverflow.com/a/868894/11189869
 * @param begin
 * @param end
 * @param option
 * @return
 */
bool cmdOptionExists(char **begin, char **end, const std::string &option) {
    return std::find(begin, end, option) != end;
}

/**
 * Handle the Laser & Radar measurements
 * @param fusionEKF
 * @param tools
 * @param line
 * @param estimations
 * @param ground_truth
 * @param verbose
 */
void handleMeasurementReading(
        FusionEKF &fusionEKF,
        Tools &tools,
        const string &line,
        vector<VectorXd> &estimations,
        vector<VectorXd> &ground_truth,
        bool verbose
) {
    MeasurementPackage meas_package;
    istringstream iss(line);
    string sensor_type;
    // first tab is a type
    iss >> sensor_type;
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {
        // laser measurement
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        float x;
        float y;
        iss >> x;
        iss >> y;
        meas_package.raw_measurements_ << x, y;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;

    } else if (sensor_type.compare("R") == 0) {
        //Radar measurements
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro, theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);

    // Call ProcessMeasurement(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's
    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1 = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);

    VectorXd estimate(4);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);

    if (verbose) {
        cout << "x_ = " << fusionEKF.ekf_.x_ << endl;
        cout << "P_ = " << fusionEKF.ekf_.P_ << endl;
    }

}

int main(int argc, char *argv[]) {
    uWS::Hub h;

    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    char *filename = getCmdOption(argv, argv + argc, "-f");

    if (filename) {
        //cout << "filename " << filename << endl;
        ifstream in_file(filename, ifstream::in);

        if (!in_file.is_open()) {
            cout << "Cannot open input file: " << filename << endl;
        }

        string line;
        // set i to get only first 3 measurments
        int i = 0;
        while (getline(in_file, line)) {

            handleMeasurementReading(fusionEKF, tools, line, estimations, ground_truth, false);
            VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
            //cout << "px\tpy\tvx\tvy\tgt_px\tgt_py\tgt_vx\tgt_vy\trmse_px_pass\trmse_py_pass\trmse_vx_pass\trmse_vy_pass" << endl;
            cout << fusionEKF.ekf_.x_(0) << "\t";
            cout << fusionEKF.ekf_.x_(1) << "\t";
            cout << fusionEKF.ekf_.x_(2) << "\t";
            cout << fusionEKF.ekf_.x_(3) << "\t";
            cout << ground_truth[ground_truth.size() - 1](0) << "\t";
            cout << ground_truth[ground_truth.size() - 1](1) << "\t";
            cout << ground_truth[ground_truth.size() - 1](2) << "\t";
            cout << ground_truth[ground_truth.size() - 1](3) << "\t";
            cout << RMSE(0) << "\t";
            cout << RMSE(1) << "\t";
            cout << RMSE(2) << "\t";
            cout << RMSE(3) << "\t";
            cout << (RMSE(0) <= 0.11) << "\t";
            cout << (RMSE(1) <= 0.11) << "\t";
            cout << (RMSE(2) <= 0.52) << "\t";
            cout << (RMSE(3) <= 0.52) << "\t" << endl;

            ++i;
        }

        if (in_file.is_open()) {
            in_file.close();
        }

    } else {
        h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth]
                            (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                             uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            if (length && length > 2 && data[0] == '4' && data[1] == '2') {
                auto s = hasData(string(data));

                if (s != "") {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry") {
                        // j[1] is the data JSON object
                        string sensor_measurement = j[1]["sensor_measurement"];

                        handleMeasurementReading(fusionEKF, tools, sensor_measurement, estimations, ground_truth, true);

                        VectorXd estimate(4);

                        double p_x = fusionEKF.ekf_.x_(0);
                        double p_y = fusionEKF.ekf_.x_(1);
                        //double v1 = fusionEKF.ekf_.x_(2);
                        //double v2 = fusionEKF.ekf_.x_(3);

                        VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

                        assert(estimations[estimations.size() - 1](0) == p_x);
                        assert(estimations[estimations.size() - 1](1) == p_y);

                        json msgJson;
                        msgJson["estimate_x"] = p_x;
                        msgJson["estimate_y"] = p_y;
                        msgJson["rmse_x"] = RMSE(0);
                        msgJson["rmse_y"] = RMSE(1);
                        msgJson["rmse_vx"] = RMSE(2);
                        msgJson["rmse_vy"] = RMSE(3);
                        auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                        // std::cout << msg << std::endl;
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }  // end "telemetry" if

                } else {
                    string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }  // end websocket message if

        }); // end h.onMessage

        h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
            std::cout << "Connected!!!" << std::endl;
        });

        h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                               char *message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

        int port = 4567;
        if (h.listen(port)) {
            std::cout << "Listening to port " << port << std::endl;
        } else {
            std::cerr << "Failed to listen to port" << std::endl;
            return -1;
        }

        h.run();
    }
}
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "plutodrone/PlutoPilot.h"
#include <librealsense2/rs.hpp>


// plutodrone::PlutoPilot::Request myFunction(const plutodrone::PlutoPilot::Request &req)
// {
//     // Print the data one time
//     ROS_INFO("Anchor_1=%f Anchor_2=%f Anchor_3=%f", 
//              (req.a1 / 536870912.0), 
//              (req.a2 / 536870912.0), 
//              (req.a3 / 536870912.0));

//     // Modify the request if necessary (e.g., scaling the values)
//     plutodrone::PlutoPilot::Request new_req = req;
//     new_req.a1 = req.a1 / 536870912.0;
//     new_req.a2 = req.a2 / 536870912.0;
//     new_req.a3 = req.a3 / 536870912.0;

//     // Returning the modified request object
//     return new_req;
// }



cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 385.0124976    ,0. ,317.58273274,
                                                    0. ,385.33449174 ,231.94288763,
                                                        0, 0, 1);
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.05587201  ,0.07173754 ,-0.00471733 ,-0.00309501 ,-0.11322034);

// cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 649.75666738 ,  0.    ,     661.4503591 ,
//                                                     0.0   ,      653.15699671 , 349.48366792,
//                                                         0, 0, 1);
// cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.255  , 1.684,   0.00516, 0.00031,  -1.6281);


double marker_size = 0.05f;

double getYawAngle(const cv::Vec3d& rvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    return atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;
}


class PIDController {
public:

    float Kp, Ki, Kd, output_min, output_max;

    float prev_error = 0, derivative = 0, output = 0;
    float dt = 0.1;

    PIDController(float Kp, float Ki, float Kd, float output_min, float output_max)
        : Kp(Kp), Ki(Ki), Kd(Kd), output_min(output_min), output_max(output_max) {}

    float PID_Output(float error, bool operation) {

        derivative = error - prev_error;

        if (operation) {
            output = ((Kp * error) + (Kd * derivative)) + 1500;
        }
        else {
            output = -((Kp * error) + (Kd * derivative)) + 1500;
        }

        prev_error = error;

        // Apply Ceiling
        if (output < output_min) { output = output_min; }
        else if (output > output_max) { output = output_max; }

        return output;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pid_publisher");
    ros::NodeHandle nh;
    ros::Publisher pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/pid_values", 10);
    ros::Rate rate(100);

    PIDController roll_pid(0.35, 0.0, 0.0, 1400, 1600);
    PIDController pitch_pid(0.35, 0.0, 0.0, 1400, 1600);
    PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);
    PIDController throttle_pid(25, 0.0, 5, 1300, 1800);

    float error_x = 0, error_y = 0, error_yaw = 0, alt_error = 0, estimated_distance = 0;

    float roll_pid_output = 0, pitch_pid_output = 0, yaw_pid_output = 0, throttle_pid_output = 0;

    int frame_center_x,frame_center_y,center_x,center_y;
    int alt_hold = 20;

    float depth_value; //because realsense provides data in 16 bits
    
    bool detected = false;

    float last_known_distance = 0, last_yaw_error = 0;
    std::vector<cv::Point2f> prev_points, next_points;
    std::vector<int> last_known_position;

    std::vector<uchar> status;
    std::vector<float> err;

    // cv::VideoCapture cap("http://192.168.4.3:4747/video");
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640,480, RS2_FORMAT_BGR8, 60);
    config.enable_stream(RS2_STREAM_DEPTH,  640,480, RS2_FORMAT_Z16, 60);
    pipeline.start(config);
 

    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::Mat frame, gray, prev_gray;

    while (ros::ok())
    {
        detected = false;
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        
        cv::Mat frame(cv::Size( 640,480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth = cv::Mat(cv::Size( 640,480), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);  // Convert depth frame to Mat

        cv::Mat gray;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        if (!ids.empty())
        {
            // ROS_INFO("ID found");

            for (size_t i = 0; i < ids.size(); i++)
            {
                if (ids[i] == 69)
                {
                    detected = true;
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);

                    center_x = (corners[i][0].x + corners[i][2].x) / 2;
                    center_y = (corners[i][0].y + corners[i][2].y) / 2;

                    frame_center_x = frame.cols / 2;
                    frame_center_y = frame.rows / 2;

                    error_x = center_x - frame_center_x;
                    error_y = center_y - frame_center_y;
                    
                    //realsense_depth
                    depth_value = depth.at<ushort>(center_y, center_x) / 1000.0f;  // Depth value in millimeters
        

                    last_known_position = {center_x, center_y};

                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
                    cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size);
                    if (!rvecs.empty()) { // Ensure rvecs has elements before accessing
                        estimated_distance = cv::norm(tvecs[0]);
                        // estimated_distance = myFunction();

                        last_known_distance = estimated_distance != 0 ? estimated_distance:last_known_distance;
                        error_yaw = getYawAngle(rvecs[0]); // Pass the first element

                        // std::cout << "Estimated Distance: " << estimated_distance << std::endl;
                        // std::cout << "Yaw Error: " << error_yaw << " degrees" << std::endl;
                    }
                }
            }
        }

        if ((!detected) && !(last_known_position.empty()))
        {
            if (!prev_gray.empty() && !prev_points.empty())
            {
                cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_points, next_points, status, err);

                if ((!next_points.empty()) && (status[0] == 1))
                {
                    last_known_position = {next_points[0].x, next_points[0].y};
                    error_x = last_known_position[0] - frame.cols / 2;
                    error_y = last_known_position[1] - frame.rows / 2;
                    estimated_distance = last_known_distance;
                    error_yaw = last_yaw_error;

                    //realsense_depth
                    depth_value = depth.at<ushort>(last_known_position[0],last_known_position[1]) / 1000.0f;
                    // Draw the tracked point using cv::circle
                    
                    cv::circle(frame, cv::Point(last_known_position[0], last_known_position[1]), 10, cv::Scalar(255, 0, 0), -1);
                    cv::putText(frame, "Optical Flow Tracking", cv::Point(last_known_position[0] + 10, last_known_position[1]),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
                }
            }
        }

        if (estimated_distance < 1.8)
        {
            alt_error = alt_hold - (estimated_distance * 100);

            roll_pid_output = roll_pid.PID_Output(error_x, true);
            pitch_pid_output = pitch_pid.PID_Output(error_y, false);
            yaw_pid_output = yaw_pid.PID_Output(error_yaw, true);
            throttle_pid_output = throttle_pid.PID_Output(alt_error, true);
        }

        std_msgs::Int16MultiArray msg;
        msg.data.clear();
        msg.data.push_back(static_cast<int>(roll_pid_output));
        msg.data.push_back(static_cast<int>(pitch_pid_output));
        msg.data.push_back(static_cast<int>(yaw_pid_output));
        msg.data.push_back(static_cast<int>(throttle_pid_output));
        pid_pub.publish(msg);


        ROS_INFO("Published: Dist = %f, Roll=%d, Pitch=%d, Yaw=%d, Throttle=%d",
                static_cast<float>(estimated_distance),static_cast<int>(roll_pid_output), static_cast<int>(pitch_pid_output),
                static_cast<int>(yaw_pid_output), static_cast<int>(throttle_pid_output));

        prev_gray = gray.clone();
        if (!last_known_position.empty()) {
            prev_points = {cv::Point2f(last_known_position[0], last_known_position[1])};
        }

        cv::imshow("Aruco Tracking with PID", frame);

        if (cv::waitKey(1) == 'q') break;

        rate.sleep();
    }

    pipeline.stop();
    cv::destroyAllWindows();
    return 0;
}
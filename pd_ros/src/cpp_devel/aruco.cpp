#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 437.48477209, 0, 479.37404252,
                                                    0, 438.32360844, 259.97078463,
                                                    0, 0, 1);

cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.06249348, 0.14815148, -0.00491044, -0.00591934, -0.12213315);

double marker_size = 0.05;

double getYawAngle(const cv::Mat& rvec)
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

    
    float PID_Output(float error){
    
        derivative = error - prev_error;
    
        output = ((Kp * error) + (Kd * derivative)) + 1500;
        
        prev_error = error;
    
        // Apply Ceiling
        if (output < output_min) {output = output_min;}
        else if (output > output_max) {output = output_max;}
    
        return output;
    }
    
    };
    

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "aruco_pid_publisher");
    ros::NodeHandle nh;
    ros::Publisher pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/pid_values", 10);
    ros::Rate rate(100);

    PIDController roll_pid(0.3, 0.01, 0.3, 1400, 1600);
    PIDController pitch_pid(0.5, 0.01, 0.5, 1400, 1600);
    PIDController yaw_pid(1.5, 0.02, 0.7, 1200, 1800);
    PIDController throttle_pid(1.5, 0.01, 0.5, 1300, 1800);

    float error_x = 0, error_y = 0, error_yaw = 0, error_throttle = 0;
    float roll_error = 0, pitch_error = 0, yaw_error = 0, throttle_error = 0;

    int alt_hold = 50;

    cv::VideoCapture cap("http://192.168.85.39:5050/video");
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video stream." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::Mat frame, gray;


    while (ros::ok()) 
    {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        bool detected = false;
        int alt_hold = 50;
        int error_x = 0, error_y = 0, yaw_error = 0, alt_error = 0;
        double estimated_distance = 0;

        

        if (!ids.empty()) 
        {
            for (size_t i = 0; i < ids.size(); i++) 
            {
                if (ids[i] == 69) 
                {
                    detected = true;
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);

                    int center_x = (corners[i][0].x + corners[i][2].x) / 2;
                    int center_y = (corners[i][0].y + corners[i][2].y) / 2;
                    
                    int frame_center_x = frame.cols / 2;
                    int frame_center_y = frame.rows / 2;
                    error_x = center_x - frame_center_x;
                    error_y = center_y - frame_center_y;

                    // cv::Mat rvec, tvec;
                    // cv::aruco::estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs, rvec, tvec);
                    // cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size);
                    // estimated_distance = cv::norm(tvec);
                    // yaw_error = getYawAngle(rvec);

                    std::vector<cv::Mat> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
                    cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size);
                    estimated_distance = cv::norm(tvecs[0]);
                    yaw_error = getYawAngle(rvecs[0]);
                }
            }
        }

        if(estimated_distance < 1.8)
        {   
            alt_error = alt_hold - (estimated_distance*100);

            roll_error = roll_pid.PID_Output(error_x);
            pitch_error = pitch_pid.PID_Output(error_y);
            yaw_error = yaw_pid.PID_Output(error_yaw);
            throttle_error = throttle_pid.PID_Output(alt_error);
        }
        // else //else condition if needed when aruco not detected fro stabilizing
        // {
        //     roll_error = pitch_error = yaw_error = throttle_error = 1500;
        // }

        std_msgs::Int16MultiArray msg;
        msg.data.clear();
        msg.data.push_back(static_cast<int>(roll_error));
        msg.data.push_back(static_cast<int>(pitch_error));
        msg.data.push_back(static_cast<int>(yaw_error));
        msg.data.push_back(static_cast<int>(throttle_error));
        pid_pub.publish(msg);

        ROS_INFO("Published: Roll=%d, Pitch=%d, Yaw=%d, Throttle=%d", 
                 static_cast<int>(roll_error), static_cast<int>(pitch_error), 
                 static_cast<int>(yaw_error), static_cast<int>(throttle_error));

        cv::imshow("Aruco Tracking with PID", frame);
        
        if (cv::waitKey(1) == 'q') break;

        rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
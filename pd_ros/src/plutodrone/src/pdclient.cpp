// INCLUDE HEADERS

//ROS HEADERS
#include "ros/ros.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include "plutodrone/Drone_stats.h"
#include "plutodrone/SetPos.h"

//STD CPP HEADERS
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <numeric>
#include <Eigen/Dense>

//IMAGE PROCESSING HEADERS
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


// NAMESPACES
using namespace Eigen;
using namespace std;
using namespace cv;


//GLOBAL INITIALIZATIONS AND DECLARATIONS
// Camera parameters for RealSense D455F
const double fx = 390.23;
const double fy = 390.23;
const double cx = 317.58273274;
const double cy = 231.94288763;
const double tag_size = 0.05;  // 5 cm

//CHANGE ACCORDING TO CAM FRAME
int frame_centre_x = 320,frame_centre_y = 240;

double xA1 = 0, yA1 = 1, zA1 = 0;
double xA2 = 0, yA2 = 0, zA2 = 0;
double xA3 = 1, yA3 = 0, zA3 = 0;

double s1_true = 0.87;
double s2_true = 0.64;
double s3_true = 0.65;

double x_true = 0.56,y_true = 0.30;
double x_raw,y_raw,x_sum,y_sum;

double s1_filtered,s2_filtered, s3_filtered;

const int CALIBRATION_SAMPLES = 100;

plutodrone::Drone_stats current_drone_data;
std::mutex data_mutex; 

int pos_step = 50;
int alt_hold = 500;


//DATACALLBACK TO FETCH NEW DRONE STATS
void dataCallback(const plutodrone::Drone_stats::ConstPtr& msg) 
{
    
    std::lock_guard<std::mutex> lock(data_mutex);
    // Store the received data in the global variable
    current_drone_data = *msg;
    // Print the received data
    // ROS_INFO("Received data: ");
    // ROS_INFO("AccX: %f, AccY: %f, AccZ: %f", msg->accX, msg->accY, msg->accZ);
    // ROS_INFO("GyroX: %f, GyroY: %f, GyroZ: %f", msg->gyroX, msg->gyroY, msg->gyroZ);
    // ROS_INFO("MagX: %f, MagY: %f, MagZ: %f", msg->magX, msg->magY, msg->magZ);
    // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", msg->roll, msg->pitch, msg->yaw);
    // ROS_INFO("Altitude: %f, Battery: %f, RSSI: %f", msg->alt, msg->battery, msg->rssi);
    // ROS_INFO("Anchor 1: %f, Anchor 2: %f, Anchor 3: %f", msg->a1, msg->a2, msg->a3);
}


//CLASS KALMAN FOR FILTERING (STATIC)
class kalman_filter
{
private:
float Q, R, P, K = 0, x = 0;
public:
    kalman_filter(float Q_set,float R_set,float P_set)
    {   
        Q = Q_set;
        R = R_set;
        P = P_set;
    
    };
    
    float kalman_filter_calc(float measurement)
    {
        P = P + Q;
        K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;
        return x;
    };
};


//CLASS KALMAN FOR FILTERING (DYNAMIC)
class Kalman1D {
    public:
        // Variables for Kalman filter
        Vector3d x;
        Matrix3d A, P, Q;
        Vector3d B;
        RowVector3d H;
        double R;
        double dt = 0.30;
        double s_true;
        double s_sum = 0;
        double s_raw;
        double s_offset = 0;
    
        // Constructor
        Kalman1D(double initial_value){
            A << 1, dt, 0.5 * dt * dt,
                    0, 1, dt,
                    0, 0, 1;
        
            B << 0.5 * dt * dt, dt, 1;
            H << 1, 0, 0;
        
            P = Matrix3d::Identity();
            Q = Matrix3d::Zero();
            Q(1, 1) = 8.4154e-05;
            Q(2, 2) = 7.0818e-09;
        
            R = 0.0015;
        
            x << initial_value, 0, 0;
            s_true = initial_value;
        };
        

        // Update function for Kalman filter
        void update(double measurement){
            x = A * x ;
            P = A * P * A.transpose() + Q;
        
            Vector3d K = P * H.transpose() / (H * P * H.transpose() + R);
            x = x + K * (measurement - H * x);
            P = (Matrix3d::Identity() - K * H) * P;
        };
    
        // Get the filtered value
        double getFilteredValue() const{
            return x(0);
        };
    
        // Calibration function
        double calib(int index){
            if(index < CALIBRATION_SAMPLES)
            {
                s_sum += s_raw;
                cout << "raw_data-> " << s_raw << "\n\n";
                cout << "Offsets -> " << s_sum<< "\n\n";
                ROS_INFO("Calibrating anchors...: %d",index);
        
            
            }else
            {  
                s_sum += s_raw;
                s_offset = (s_sum / (CALIBRATION_SAMPLES)) - s_true;
                ROS_INFO("Calibration complete for anchor setpoint : %f",s_true);
                ROS_INFO("Offsets -> %f",s_offset );
            }
            
            return s_offset;
        };

    };


//XY CALIBRATION FUNCTION
VectorXd xy_calib(int index, double x_raw, double y_raw)
{
    VectorXd xy_calib_result(2);
    if(index < CALIBRATION_SAMPLES )
    {
        x_sum += x_raw;
        y_sum += y_raw;
        cout << "raw_data-> " << x_raw << "\n\n";
        cout << "raw_data-> " << y_raw << "\n\n";

        ROS_INFO("Calibrating xy .... : %d",index);

    
    }else
    {   // Offset = measured avg - true value
        x_sum += x_raw;
        y_sum += y_raw;

        double x_offset = (x_sum / CALIBRATION_SAMPLES) - x_true;
        double y_offset = (y_sum / CALIBRATION_SAMPLES) - y_true;

        xy_calib_result(0) = x_offset;
        xy_calib_result(1) = y_offset ;
            
        ROS_INFO("Calibration Complete for xy");
        ROS_INFO("Offsets for x -> %f",xy_calib_result(0) );
        ROS_INFO("Offsets for y -> %f",xy_calib_result(1) );

    }
    
    return xy_calib_result;
};


// TRILATERATION FUNCTION
VectorXd trilateration(
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double s1, double s2, double s3
) {
    MatrixXd A(3, 3);
    VectorXd B(3);

    A(0, 0) = 2 * (x2 - x1);
    A(0, 1) = 2 * (y2 - y1);
    A(0, 2) = 2 * (z2 - z1);
    B(0) = s1 * s1 - s2 * s2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2 - z1 * z1 + z2 * z2;

    A(1, 0) = 2 * (x3 - x2);
    A(1, 1) = 2 * (y3 - y2);
    A(1, 2) = 2 * (z3 - z2);
    B(1) = s2 * s2 - s3 * s3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3 - z2 * z2 + z3 * z3;

    A(2, 0) = 2 * (x1 - x3);
    A(2, 1) = 2 * (y1 - y3);
    A(2, 2) = 2 * (z1 - z3);
    B(2) = s3 * s3 - s1 * s1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;

    VectorXd result = A.colPivHouseholderQr().solve(B);

    return result;
    
}


//PID CONTROLLER CLASS
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

//FUNCTION TO MANUALLY CHANGE SETPOINTS FOR DEBUGGING PURPOSES
void change_setpoint(const std_msgs::Int16::ConstPtr& msg) {
    int key = msg->data;
    int new_hold;
    switch (key) {
        case 63: new_hold = frame_centre_x + pos_step; frame_centre_x = new_hold; break;
        case 64: new_hold = frame_centre_x - pos_step; frame_centre_x = new_hold; break;
    }
    
}
    


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pdclient");
    ros::NodeHandle nh;

    // Create a request & response object and set the position values
    plutodrone::SetPos::Request req;
    plutodrone::SetPos::Response res;

    // Wait for the service to be available
    ros::service::waitForService("set_position");
    ros::ServiceClient client = nh.serviceClient<plutodrone::SetPos>("set_position");

    // Subscriber for getting updated drone data
    ros::Subscriber dataSubscriber = nh.subscribe("drone_data", 1000, dataCallback);
    
    // Publisher to publish roll,pitch,yaw for drone control
    ros::Publisher pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/pid_values", 10);
    std_msgs::Int16MultiArray pid_msg;

    // Subscriber for autopilot and debugging drone control, can be removed later
    ros::Subscriber set_point = nh.subscribe("/input_key", 1, change_setpoint);

    // Ros Rate
    ros::Rate rate(100);

    // Camera view publisher for gui
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 10);

    // Initializing PID and Kalman classes
    PIDController roll_pid(0.25, 0.0,6.35, 1300, 1700);
    PIDController pitch_pid(0.25, 0.0, 6.35, 1300, 1700);
    PIDController throttle_pid(1.425, 0.0, 0.5, 1300, 1800);
    PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);

    kalman_filter z_kal(0.03,0.1,1);
    kalman_filter x_kal(0.09,0.5,1);
    kalman_filter y_kal(0.09,0.5,1);

    Kalman1D kf1(s1_true);
    Kalman1D kf2(s2_true);
    Kalman1D kf3(s3_true);

    // Image Processing Initializations
    std::vector<cv::Point2f> prev_points, next_points;
    std::vector<int> last_known_position;
    std::vector<uchar> status;
    std::vector<float> err;

    // April Tag Initializations
    apriltag_family_t* tf = tag16h5_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.05;
    td->decode_sharpening = 0.25;
    td->refine_edges = 0;

    // Cam Initialization
    cv::VideoCapture cap(2);

    if (!cap.isOpened()) {
        ROS_ERROR("Error: Could not open the camera.");
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame, gray, prev_gray;

    cv_bridge::CvImagePtr cv_ptr; //warn in main code it was declared in the loop

    float img_errorx = 0,img_errory = 0,img_erroryaw = 0;
    float last_known_distance = 0, last_yaw_error = 0;
    bool detected = false;

    // Calibration Initializations
    VectorXd raw_xy, xy_offsets;


    //setpoints
    float setpointx = 0, setpointy = 0, setpointz = 0;

    // Error Calculation variable Initialization
    float error_x = 0, error_y = 0, error_yaw = 0, alt_error = 0, estimated_distance = 0;
    float roll_pid_output = 0, pitch_pid_output = 0, yaw_pid_output = 0, throttle_pid_output = 0;
    float tof_data; //because realsense provides data in 16 bits

    //station cam range, set accordingly
    const int dockx=0, docky=0, dockz=0;
    const int dockrange = 10; 


    // Calibration on Anchor Points only
    // this_thread::sleep_for(chrono::milliseconds(500));
    ros::Duration(0.5).sleep();
    for (size_t i = 0; i <= CALIBRATION_SAMPLES; i++)
    {   
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        kf1.s_raw = (current_drone_data.a1/1000);
        kf2.s_raw = (current_drone_data.a2/1000);
        kf3.s_raw = (current_drone_data.a3/1000);

        kf1.s_offset = kf1.calib(i);
        kf2.s_offset = kf2.calib(i);
        kf3.s_offset = kf3.calib(i);
        
    }


    //Calibration on x,y Points only
    // this_thread::sleep_for(chrono::milliseconds(500));
    ros::Duration(0.5).sleep();
    for (size_t i = 0; i <= CALIBRATION_SAMPLES; i++)
    {   
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        kf1.s_raw = (current_drone_data.a1/1000);
        kf2.s_raw = (current_drone_data.a2/1000);
        kf3.s_raw = (current_drone_data.a3/1000);

        kf1.update(kf1.s_raw - kf1.s_offset);
        kf2.update(kf2.s_raw - kf2.s_offset);
        kf3.update(kf3.s_raw - kf3.s_offset);

        s1_filtered = kf1.getFilteredValue();
        s2_filtered = kf2.getFilteredValue();
        s3_filtered = kf3.getFilteredValue();

        raw_xy = trilateration(xA1, yA1, zA1, xA2, yA2, zA2, xA3, yA3, zA3, s1_filtered, s2_filtered, s3_filtered);

        x_raw = raw_xy(0);
        y_raw = raw_xy(1);

        xy_offsets = xy_calib(i, x_raw, y_raw);


    }



    while (ros::ok())
    {   
        // updating anchor values after subtracting offsets
        kf1.update((current_drone_data.a1/1000) - kf1.s_offset);
        kf2.update((current_drone_data.a2/1000) - kf2.s_offset);
        kf3.update((current_drone_data.a3/1000) - kf3.s_offset);
        
        // anchor values after kalman filter
        s1_filtered = kf1.getFilteredValue();
        s2_filtered = kf2.getFilteredValue();
        s3_filtered = kf3.getFilteredValue();
        
        // applying trilateration to the anchor values
        raw_xy = trilateration(xA1, yA1, zA1, xA2, yA2, zA2, xA3, yA3, zA3, s1_filtered, s2_filtered, s3_filtered);
        
        // subtracting xy calib offsets
        x_raw = raw_xy(0) - xy_offsets(0);
        y_raw = raw_xy(1) - xy_offsets(1);

        std::lock_guard<std::mutex> lock(data_mutex); //warn check usage
        tof_data = z_kal.kalman_filter_calc(current_drone_data.alt);

        ROS_INFO("s1 filtered : %f | s2 filtered : %f | s3 filtered : %f",s1_filtered,s2_filtered,s3_filtered);
        ROS_INFO("x_FINAL : %f | y_FINAL: %f",x_raw,y_raw);


        // assigning the x,y,z to service request params
        req.pos_x = x_raw;
        req.pos_y = y_raw;
        req.pos_z = tof_data;

        // Call the service and check the result
        if (client.call(req, res))
        {
            setpointx = res.set_x;
            setpointy = res.set_y;
            setpointz = res.set_z;
            ROS_INFO("Next Setpoint: set_x=%.2f, set_y=%.2f, set_z=%.2f", res.set_x, res.set_y, res.set_z);
            
        }
        else
        {
            ROS_ERROR("Failed to call service set_position");
            return 1;
        }



        //April Tag setection
        detected = false;

        cap.read(frame);
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t img_header = {gray.cols, gray.rows, gray.cols, gray.data};
        zarray_t* detections = apriltag_detector_detect(td, &img_header);

        for (int i = 0; i < zarray_size(detections); i++) 
        {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            if (det->id == 0 && det->decision_margin > 50) 
            {
                detected = true;

                apriltag_detection_info_t info = {det, tag_size, fx, fy, cx, cy};
                apriltag_pose_t pose;
                estimate_tag_pose(&info, &pose);

                // estimated_distance = (pose.t->data[2]*1000) + 30;
                estimated_distance = tof_data;
                
                int tag_center_x = det->c[0];
                int tag_center_y = det->c[1];

                img_errorx = tag_center_x - frame_centre_x;
                img_errory = tag_center_y - frame_centre_y;

                img_erroryaw = atan2(pose.R->data[3], pose.R->data[0]) * 180.0 / CV_PI;

                last_known_position = {tag_center_x, tag_center_y};
                last_known_distance = estimated_distance;
                last_yaw_error = img_erroryaw;

                for (int j = 0; j < 4; j++) 
                {
                    line(frame, Point(det->p[j][0], det->p[j][1]), Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]), Scalar(0, 255, 0), 2);
                }
                circle(frame, Point(tag_center_x, tag_center_y), 5, Scalar(0, 0, 255), -1);
                break;
            }
            else
            {
                estimated_distance = tof_data;
                // estimated_distance = last_known_distance;
            }
        }

        if (!detected && !last_known_position.empty()) {
            if (!prev_gray.empty() && !prev_points.empty()) {
                calcOpticalFlowPyrLK(prev_gray, gray, prev_points, next_points, status, err);
                if (!next_points.empty() && status[0] == 1) {
                    last_known_position = {static_cast<int>(next_points[0].x), static_cast<int>(next_points[0].y)};
                    img_errorx = last_known_position[0] - (frame.cols / 2);
                    img_errory = last_known_position[1] - (frame.rows / 2);
                    img_erroryaw = last_yaw_error;

                    circle(frame, Point(last_known_position[0], last_known_position[1]), 10, Scalar(255, 0, 0), -1);
                    putText(frame, "Optical Flow Tracking", Point(last_known_position[0] + 10, last_known_position[1]), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
                }
            }
        }

        //warn little confused about this if statement 
        if((x_raw < dockx+dockrange) && (x_raw > dockx-dockrange) && (y_raw < docky+dockrange) && (y_raw > docky-dockrange) && (tof_data < dockz+dockrange) && (tof_data > dockz-dockrange) && ((setpointx == dockx) && (setpointy == docky) && (setpointz == dockz)))
        {
            alt_error = setpointz - estimated_distance;
            error_x = x_kal.kalman_filter_calc((img_errorx/fx)*estimated_distance);
            error_y = y_kal.kalman_filter_calc((img_errory/fy)*estimated_distance);
            error_yaw = img_erroryaw;
        }
        else
        {
            alt_error = setpointz - tof_data;
            error_x = setpointx - x_raw;
            error_y = setpointy - y_raw;
            error_yaw = 0;
        }

        roll_pid_output = roll_pid.PID_Output(error_x, false);
        pitch_pid_output = pitch_pid.PID_Output(error_y, false);
        yaw_pid_output = yaw_pid.PID_Output(error_yaw, true);
        throttle_pid_output = throttle_pid.PID_Output(alt_error, true);

        pid_msg.data.clear();
        pid_msg.data.push_back(static_cast<int>(roll_pid_output));
        pid_msg.data.push_back(static_cast<int>(pitch_pid_output));
        pid_msg.data.push_back(static_cast<int>(yaw_pid_output));
        pid_msg.data.push_back(static_cast<int>(throttle_pid_output));
        pid_pub.publish(pid_msg);


        prev_gray = gray.clone();
        
        if (!last_known_position.empty()) {
            prev_points = {cv::Point2f(last_known_position[0], last_known_position[1])};
        }


        int text_x = frame.cols - 250;
        putText(frame, "Battery: " + to_string(current_drone_data.battery), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "X Error: " + to_string(int(error_x)), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Y Error: " + to_string(int(error_y)), Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Yaw: " + to_string(int(error_yaw)), Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Distance: " + to_string(estimated_distance), Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Roll PID: " + to_string(int(roll_pid_output)), Point(text_x, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Pitch PID: " + to_string(int(pitch_pid_output)), Point(text_x, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Yaw PID: " + to_string(int(yaw_pid_output)), Point(text_x, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Throttle PID: " + to_string(int(throttle_pid_output)), Point(text_x, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);


        cv::imshow("April Tracking with PID", frame); // warn imshow window



        // Publish the image for gui
        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ros::Time::now(); // Time stamp
        cv_ptr->header.frame_id = "camera";      // Frame ID
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Encoding format
        cv_ptr->image = frame; // The captured frame
        image_pub.publish(cv_ptr->toImageMsg());


        if (cv::waitKey(1) == 'q') break;


        apriltag_detections_destroy(detections);
        rate.sleep();
        ros::spinOnce();

    }
    cap.release();
    cv::destroyAllWindows();


    return 0;
}

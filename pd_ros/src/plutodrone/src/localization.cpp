#include <iostream>
#include <cmath>
#include <thread>
#include <ros/ros.h>
#include <chrono>
#include "plutodrone/Drone_stats.h"
#include <Eigen/Dense>
#include <vector>
#include <numeric>

using namespace Eigen;
using namespace std;

plutodrone::Drone_stats current_drone_data;
std::mutex data_mutex; 

double xA1 = 0, yA1 = 1, zA1 = 0;
double xA2 = 0, yA2 = 0, zA2 = 0;
double xA3 = 1, yA3 = 0, zA3 = 0;


double s1_true = 0.96;
double s2_true = 0.72;
double s3_true = 0.60;

double x_true = 0.56,y_true = 0.30;
double x_raw,y_raw,x_sum,y_sum;


double s1_filtered,s2_filtered, s3_filtered;

// VectorXd xy_calib_result;
const int CALIBRATION_SAMPLES = 100;

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
        }
        ;
    
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
             
            // cout << "Offsets X -> " << x_offset<< "\n\n";
            // cout << "Offsets Y -> " << y_offset<< "\n\n";
            // cout << "Calibration complete for xy.\n";
            ROS_INFO("Calibration Complete for xy");
            ROS_INFO("Offsets for x -> %f",xy_calib_result(0) );
            ROS_INFO("Offsets for y -> %f",xy_calib_result(1) );

        }
        
        return xy_calib_result;
    };
    
// Trilateration function (2D, Z = 0)
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

    // cout << "Estimated Position:\n";
    // cout << "x: " << result(0) << "  y: " << result(1) <<  "\n\n";
    // ROS_INFO("Co-ordinates: ");

    return result;
    
}

void dataCallback(const plutodrone::Drone_stats::ConstPtr& msg) {
    
    std::lock_guard<std::mutex> lock(data_mutex);
    // Store the received data in the global variable
    current_drone_data = *msg;
    // ROS_INFO("Anchor 1: %f, Anchor 2: %f, Anchor 3: %f", msg->a1, msg->a2, msg->a3);
}


int main(int argc, char** argv)
  {
    ros::init(argc, argv, "Localization");
    ros::NodeHandle nh;
    ros::Subscriber dataSubscriber = nh.subscribe("drone_data", 1000, dataCallback);
    ros::Rate rate(100);

    Kalman1D kf1(s1_true);
    Kalman1D kf2(s2_true);
    Kalman1D kf3(s3_true);

    VectorXd raw_xy, xy_offsets;
    
    // for (size_t i = 0; i <= CALIBRATION_SAMPLES; i++)
    // {   
    //     this_thread::sleep_for(chrono::milliseconds(500));
    //     ros::spinOnce();
    //     kf1.s_raw = (current_drone_data.a1/1000);
    //     kf2.s_raw = (current_drone_data.a2/1000);
    //     kf3.s_raw = (current_drone_data.a3/1000);

    //     kf1.update(kf1.s_raw);
    //     kf2.update(kf2.s_raw);
    //     kf3.update(kf3.s_raw);
        
    //     // s1_filtered = kf1.getFilteredValue();
    //     // s2_filtered = kf2.getFilteredValue();
    //     // s3_filtered = kf3.getFilteredValue();

    //     // raw_xy = trilateration(xA1, yA1, zA1, xA2, yA2, zA2, xA3, yA3, zA3, s1_filtered, s2_filtered, s3_filtered);

    //     // x_raw = raw_xy(0);
    //     // y_raw = raw_xy(1);
    //     // xy_offsets = xy_calib(i, x_raw, y_raw,CALIBRATION_SAMPLES);

    //     // kf1.s_offset = kf1.calib(i);
    //     // kf2.s_offset = kf2.calib(i);
    //     // kf3.s_offset = kf3.calib(i);
        
    // }

    this_thread::sleep_for(chrono::milliseconds(500));
    // ////////////////////Calibration on anchor points only
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
    // //////////////////////////Calibration on x,y points only

    this_thread::sleep_for(chrono::milliseconds(500));


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
        
        // tof_data = z_kal.kalman_filter_calc(current_drone_data.alt);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        kf1.update((current_drone_data.a1/1000) - kf1.s_offset);
        kf2.update((current_drone_data.a2/1000) - kf2.s_offset);
        kf3.update((current_drone_data.a3/1000) - kf3.s_offset);
        
        // ROS_INFO("s1 %f",(current_drone_data.a1/1000) - kf1.s_offset);
        // ROS_INFO("s1 %f",(current_drone_data.a2/1000) - kf2.s_offset);
        // ROS_INFO("s1 %f",(current_drone_data.a3/1000) - kf3.s_offset);
        
        s1_filtered = kf1.getFilteredValue();
        s2_filtered = kf2.getFilteredValue();
        s3_filtered = kf3.getFilteredValue();

        // ROS_INFO("s1 raw %f",(current_drone_data.a1/1000));
        // ROS_INFO("s1 offset %f",(current_drone_data.a1/1000) - kf1.s_offset);
        // ROS_INFO("s1 raw : %f | s2 raw : %f | s3 raw : %f",(current_drone_data.a1/1000- kf1.s_offset),(current_drone_data.a2/1000- kf2.s_offset),(current_drone_data.a3/1000- kf3.s_offset));
        ROS_INFO("s1 filtered : %f | s2 filtered : %f | s3 filtered : %f",s1_filtered,s2_filtered,s3_filtered);

        // ROS_INFO("s2 raw %f",(current_drone_data.a2/1000));
        // ROS_INFO("s2 offset %f",(current_drone_data.a2/1000) - kf2.s_offset);
        // ROS_INFO("s2 filtered %f",s2_filtered);

        // ROS_INFO("s3 raw %f",(current_drone_data.a3/1000));
        // ROS_INFO("s3 offset %f",(current_drone_data.a3/1000) - kf3.s_offset);
        // ROS_INFO("s3 filtered %f",s3_filtered);  

        raw_xy = trilateration(xA1, yA1, zA1, xA2, yA2, zA2, xA3, yA3, zA3, s1_filtered, s2_filtered, s3_filtered);
        // x_raw = raw_xy(0);
        // y_raw = raw_xy(1);
        // ROS_INFO("x_FINAL : %f | y_FINAL: %f",x_raw,y_raw);
        x_raw = raw_xy(0) - xy_offsets(0);
        y_raw = raw_xy(1) - xy_offsets(1);
        
        // ROS_INFO("x_raw : %f | x_offset : %f | x_final : %f",raw_xy(0),xy_offsets(0),x_raw);
        // ROS_INFO("y_raw : %f | y_offset : %f | y_final : %f",raw_xy(1),xy_offsets(1),y_raw);
        ROS_INFO("x_FINAL : %f | y_FINAL: %f",x_raw,y_raw);
        
        // rate.sleep();            
    }
  } 
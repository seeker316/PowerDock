#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "camera_feed_publisher");
    ros::NodeHandle nh;

    // Create an image transport publisher to publish to the /camera/image_raw topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 10);

    // Open the default camera (usually the webcam)
    cv::VideoCapture cap(6);

    // Check if the camera opened successfully
    if (!cap.isOpened())
    {
        ROS_ERROR("Could not open the camera!");
        return -1;
    }

    // Set the publishing rate (10 Hz)
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // Capture a frame from the camera
        cv::Mat frame;
        cap >> frame;

        if (!frame.empty())
        {
            // Convert the captured frame (cv::Mat) to a ROS Image message
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
                cv_ptr->header.stamp = ros::Time::now(); // Time stamp
                cv_ptr->header.frame_id = "camera";      // Frame ID
                cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Encoding format
                cv_ptr->image = frame; // The captured frame

                // Publish the image
                image_pub.publish(cv_ptr->toImageMsg());
                ROS_INFO("publishing");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

        // Spin once to handle callbacks and sleep to maintain the loop rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Release the camera when the node is shutdown
    cap.release();

    return 0;
}

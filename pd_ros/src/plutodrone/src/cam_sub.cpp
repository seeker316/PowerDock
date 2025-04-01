#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convert the ROS image message to a CV image (OpenCV format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Display the image
        cv::imshow("Camera Feed", cv_ptr->image);
        cv::waitKey(30); // Wait for a short period to display the image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "camera_feed_subscriber");
    ros::NodeHandle nh;

    // Create an ImageTransport object to subscribe to image topics
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    // OpenCV: Create a window to display the image
    cv::namedWindow("Camera Feed");

    // Spin the ROS node and handle callbacks
    ros::spin();

    // Destroy the OpenCV window once the node shuts down
    cv::destroyWindow("Camera Feed");

    return 0;
}

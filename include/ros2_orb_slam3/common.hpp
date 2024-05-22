// Include file
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

#include <fstream> // Input/output stream class to operate on files.

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Node specific definitions
class MonocularMode : public rclcpp::Node {
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
public:
    std::string experimentConfig; // String to receive settings sent by the python driver
    double timeStep{}; // Timestep data received from the python node
    std::string receivedConfig;

    //* Class constructor
    MonocularMode(); // Constructor

    ~MonocularMode() override; // Destructor

private:
    // Class internal variables
    std::string homeDir;
    std::string packagePath = "ros2_test/src/ros2_orb_slam3/"; //! Change to match path to your workspace
    std::string OPENCV_WINDOW; // Set during initialization
    std::string nodeName; // Name of this node
    std::string vocFilePath; // Path to ORB vocabulary provided by DBoW2 package
    std::string settingsFilePath; // Path to settings file provided by ORB_SLAM3 package
    std::string settingsFileName;

    std::string pubConfigAckName; // Publisher topic name
    std::string imageTopic; // Topic to subscribe to receive RGB images from a python node
    std::string subTimestepMsgName; // Topic to subscribe to receive the timestep related to the

    //* Definitions of publisher and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;

    //* ORB_SLAM3 related variables
    ORB_SLAM3::System *pAgent{}; // pointer to a ORB SLAM3 object
    ORB_SLAM3::System::eSensor sensorType;
    bool enablePangolinWindow = false; // Shows Pangolin window output
    bool enableOpenCVWindow = false; // Shows OpenCV window output

    // Callback to process settings sent over by Python node
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    //* Helper functions
    // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
    void initializeVSLAM(std::string &configString); //* Method to bind an initialized VSLAM framework to this node
};

#endif

/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp") {
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/

    //* Find path to home directory
    homeDir = getenv("HOME");
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("voc_file", homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin"); // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file_path", homeDir + "/" + packagePath + "orb_slam3/config/Monocular/"); // path to settings file
    this->declare_parameter("settings_file_name", "EuRoC.yaml");
    this->declare_parameter("image_topic", "/camera/color/image_raw");

    //* Populate parameter values
    this->get_parameter("voc_file", vocFilePath);
    this->get_parameter("settings_file_path", settingsFilePath);
    this->get_parameter("settings_file_name", settingsFileName);
    this->get_parameter("image_topic", imageTopic);

    //DEBUG print
    RCLCPP_INFO(this->get_logger(), "voc_file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_name: %s", settingsFileName.c_str());

    //* subscrbite to the image messages coming from the Python driver node
    imageSub = this->create_subscription<sensor_msgs::msg::Image>(
        imageTopic, 1, std::bind(&MonocularMode::ImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    initializeVSLAM(settingsFileName);
}

//* Destructor
MonocularMode::~MonocularMode() {
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string &configString) {
    settingsFilePath = settingsFilePath + "/" + configString;

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "MonocularMode node initialized");
}

double StampToSec(builtin_interfaces::msg::Time stamp)
{
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
}

//* Callback to process image message and run SLAM node
void MonocularMode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient

    //* Convert ROS image to openCV image
    try {
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope

        // DEBUGGING, Show image
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, StampToSec(msg->header.stamp));//todo set step

    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
}

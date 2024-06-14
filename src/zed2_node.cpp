#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include "defines.hpp"
// #include <zed-open-capture/videocapture.hpp>
// #include <zed-open-capture/sensorcapture.hpp>
// #include <zed-open-capture/defines.hpp>
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define FPS sl_oc::video::FPS::FPS_60
#define RES sl_oc::video::RESOLUTION::HD720
#define VERBOSITY sl_oc::VERBOSITY::INFO

#define DEG2RAD(x) ((x) * M_PI / 180.0)

class ZED2Node {
public:
    ZED2Node() : nh("~"){
        initCapture();

        img_pub1 = nh.advertise<sensor_msgs::Image>("/camera1/img", 10);
        img_pub2 = nh.advertise<sensor_msgs::Image>("/camera2/img", 10);
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);

        img_rate = nh.param<double>("img_publish_rate", 60.0);
        imu_rate = nh.param<double>("imu_publish_rate", 400.0);

        img_timer = nh.createTimer(ros::Duration(1.0 / img_rate), &ZED2Node::imagePub, this);
        imu_timer = nh.createTimer(ros::Duration(1.0 / imu_rate), &ZED2Node::imuPub, this);

    }

    ~ZED2Node(){
        delete sensors;
        delete cameras;
    }

    void imagePub(const ros::TimerEvent&) {
        sl_oc::video::Frame frame = cameras->getLastFrame();

        if (frame.data == nullptr) {
            ROS_WARN("Dropped a frame!");
            frame = last_frame;
        } else {
            last_frame = frame;
        }

        cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR;
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

        int width = frame.width;
        int height = frame.height;
        int midpoint = width / 2;

        cv::Rect leftROI(0, 0, midpoint, height);
        cv::Rect rightROI(midpoint, 0, width - midpoint, height);

        cv::Mat left = frameBGR(leftROI);
        cv::Mat right = frameBGR(rightROI);

        sensor_msgs::ImagePtr img_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
        sensor_msgs::ImagePtr img_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right).toImageMsg();

        img_pub1.publish(img_msg1);
        img_pub2.publish(img_msg2);
    }

    void imuPub(const ros::TimerEvent&) {
        
        const sl_oc::sensors::data::Imu imu = sensors->getLastIMUData(1500);
        if(imu.valid == sl_oc::sensors::data::Imu::NEW_VAL){
            sensor_msgs::Imu imu_msg;

            ros::Time ts;
            ts.fromNSec(imu.timestamp);

            imu_msg.header.stamp = ts;

            imu_msg.linear_acceleration.x = imu.aX;
            imu_msg.linear_acceleration.y = imu.aY;
            imu_msg.linear_acceleration.z = imu.aZ;

            imu_msg.angular_velocity.x = DEG2RAD(imu.gX);
            imu_msg.angular_velocity.y = DEG2RAD(imu.gY);
            imu_msg.angular_velocity.z = DEG2RAD(imu.gZ);

            imu_pub.publish(imu_msg);
        }
        
    }

    void initCapture() {
        sensors = new sl_oc::sensors::SensorCapture(VERBOSITY);

        params.res = RES;
        params.fps = FPS;
        cameras = new sl_oc::video::VideoCapture(params);

        std::vector<int> devices = sensors->getDeviceList();
        if(devices.size() == 0){
            ROS_FATAL("No available Zed Camera!");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        if(!sensors->initializeSensors(devices[0])){
            ROS_FATAL("Can't connect to the device!");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        if (!cameras->initializeVideo()) {
            ROS_FATAL("Cannot open camera video capture!");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        std::cout << "Connected to camera sn: " << cameras->getSerialNumber() << "[" << cameras->getDeviceName() << "]" << std::endl;
    }

private:
    sl_oc::video::VideoParams params;
    sl_oc::video::VideoCapture* cameras;
    sl_oc::sensors::SensorCapture* sensors;
    sl_oc::video::Frame last_frame;
    ros::NodeHandle nh;
    ros::Publisher img_pub1;
    ros::Publisher img_pub2;
    ros::Publisher imu_pub;
    ros::Timer img_timer;
    ros::Timer imu_timer;
    double img_rate;
    double imu_rate;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed2_node");
    ZED2Node node;
    ros::spin();
    return 0;
}

#include <string>
#include <set>

// chilitags
#include <chilitags/chilitags.hpp>

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#define USE_CHILITAGS_DEFAULT_PARAM -1

class ChilitagsDetector
{
public:
    ChilitagsDetector(ros::NodeHandle& rosNode,
                      const std::string& configuration);


    tf::Transform transform;
    std::string camera_frame;
    bool object_found;

    void startLooking();
    void stopLooking();

private:

    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;

    std::string _configuration;

    image_geometry::PinholeCameraModel cameramodel;
    cv::Mat cameraMatrix, distCoeffs;
    bool firstUncalibratedImage;

    cv::Mat inputImage;

    void setROSTransform(cv::Matx44d trans, tf::Transform& transform);


    void findMarkers(const sensor_msgs::ImageConstPtr& msg,
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
};


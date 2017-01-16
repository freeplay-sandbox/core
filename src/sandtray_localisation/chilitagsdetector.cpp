#include "chilitagsdetector.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the markers transformation should be published?
// this allow to compensate for the 'slowness' of tag detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

ChilitagsDetector::ChilitagsDetector(ros::NodeHandle& rosNode,
                                    const string& configuration):
            rosNode(rosNode),
            object_found(false),
            it(rosNode),
            firstUncalibratedImage(true),
            chilitags3d(cv::Size(0,0)) // will call setDefaultTagSize with default chilitags parameter values

{



    chilitags3d.readTagConfiguration(configuration, true, true);

}

void ChilitagsDetector::startLooking() {

    ROS_INFO("Starting the chilitags detector");
    object_found = false;
    sub = it.subscribeCamera("image", 1, &ChilitagsDetector::findMarkers, this);

}

void ChilitagsDetector::stopLooking() {
    ROS_INFO("Closing the chilitags detector");
    sub.shutdown();
}

void ChilitagsDetector::setROSTransform(Matx44d trans, tf::Transform& transform)
{
    transform.setOrigin( tf::Vector3( trans(0,3) / 1000,
                                    trans(1,3) / 1000,
                                    trans(2,3) / 1000) );

    tf::Quaternion qrot;
    tf::Matrix3x3 mrot(
        trans(0,0), trans(0,1), trans(0,2),
        trans(1,0), trans(1,1), trans(1,2),
        trans(2,0), trans(2,1), trans(2,2));
    mrot.getRotation(qrot);
    transform.setRotation(qrot);
}

void ChilitagsDetector::findMarkers(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    // updating the camera model is cheap if not modified
    cameramodel.fromCameraInfo(camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(firstUncalibratedImage) {
            ROS_ERROR("Camera publishes uncalibrated images. Can not detect markers.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        firstUncalibratedImage = false;
        return;
    }
    camera_frame = cameramodel.tfFrame();
    firstUncalibratedImage = true;
    // TODO: can we avoid to re-set the calibration matrices for every frame? ie,
    // how to know that the camera info has changed?
    chilitags3d.setCalibration(cameramodel.intrinsicMatrix(), 
                                cameramodel.distortionCoeffs());

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

        /********************************************************************
        *                      Markers detection                           *
        ********************************************************************/

    auto foundObjects = chilitags3d.estimate(inputImage);

    if(foundObjects.empty()) {
        ROS_WARN("Sandtray not yet seen...");
        return;
    }

    // we have found our tag. Good. We can unsubscribe
    ROS_INFO("Marker found! Nice! Closing the chilitags detector.");
    sub.shutdown();

    for (auto& kv : foundObjects) {


        setROSTransform(kv.second, 
                        transform);
        object_found = true;
    }

}


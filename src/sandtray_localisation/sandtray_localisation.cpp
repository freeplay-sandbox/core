#include <functional>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include "chilitagsdetector.hpp"

using namespace std;


bool foundSandtray = false;
tf::Transform target_transform;

string targetFrame;
string referenceFrame;

void onSignal(ros::NodeHandle& rosNode, shared_ptr<tf::TransformListener> tl, shared_ptr<ChilitagsDetector> detector, const std_msgs::EmptyConstPtr& sig) {

    ROS_INFO("Received signal for localisation! Searching the sandtray for 5sec max");

    detector->startLooking();

    float elapsedTime = 0;
    while(!detector->object_found and elapsedTime < 5) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        elapsedTime += 0.1;
    }

    detector->stopLooking();

    if(!detector->object_found) {// timeout!
        ROS_ERROR_STREAM("Could not see the sandtray after " << elapsedTime << "s. You might want to make sure the sandtray is in the field of view of the robot's camera!");
        return;
    }

    foundSandtray = true;

    // Re-compute the *current* transformation between the camera and the reference frame
    tf::StampedTransform camera_to_reference;

    tl->waitForTransform(referenceFrame, detector->camera_frame, ros::Time(), ros::Duration(1));
    tl->lookupTransform(referenceFrame, detector->camera_frame, ros::Time(), camera_to_reference);

    // store the final transform sandtray -> reference frame
    target_transform.mult(camera_to_reference, detector->transform);
    //target_transform = detector->transform;



}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "sandtray_localisation");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    auto tl = make_shared<tf::TransformListener>();
    tf::TransformBroadcaster br;

    // load parameters
    string signalingTopic;
    _private_node.param<string>("signaling_topic", signalingTopic, "sandtray_localising");

    _private_node.param<string>("target_frame", targetFrame, "sandtray");
    _private_node.param<string>("reference_frame", referenceFrame, "odom");
    string markerId;
    _private_node.param<string>("marker_id", markerId, "709");
    string markerSize;
    _private_node.param<string>("marker_size", markerSize, "400");

    string configuration = "%YAML:1.0\n" + targetFrame + ":\n    - tag: " + markerId + "\n      size: " + markerSize + "\n";

    while(!tl->frameExists(referenceFrame)) {
        ROS_WARN_STREAM("Waiting for reference frame " << referenceFrame << " to become available...");
        ros::Duration(1).sleep();
    }
    
    // initialize the detector by subscribing to the camera video stream
    auto detector = make_shared<ChilitagsDetector>(rosNode, configuration);

    ros::Subscriber signal_sub = rosNode.subscribe<std_msgs::Empty>(signalingTopic, 
                                                                    1, 
                                                                    boost::bind(onSignal, 
                                                                                rosNode, 
                                                                                tl,
                                                                                detector,
                                                                                _1));
    

    ROS_INFO_STREAM("sandtray_localisation is ready. " << referenceFrame << " -> sandtray transformation will be published/updated on TF when localisation is triggered on " << signalingTopic);
    

    ros::Rate r(10);
    while(ros::ok()) {


        if(foundSandtray) {
            br.sendTransform(
                tf::StampedTransform(target_transform, 
                                     ros::Time::now(), 
                                     referenceFrame,
//                                   detector->camera_frame,
                                     targetFrame));

        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


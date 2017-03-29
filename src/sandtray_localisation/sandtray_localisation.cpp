#include <functional>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include "chilitagsdetector.hpp"

using namespace std;


bool foundSandtray = false;

tf::Transform upsidedown; // represent a rotation of 180deg around X -- needed to convert markers as seen by chilitags with TF frames broadcasted by Qt
tf::Transform robot_reference2target;

string targetFrame;
string markerFrame;
string robotReferenceFrame;

ros::Publisher speechSignaling;

void onSignal(ros::NodeHandle& rosNode, shared_ptr<tf::TransformListener> tl, shared_ptr<ChilitagsDetector> detector, const std_msgs::EmptyConstPtr& sig) {

    ROS_INFO("Received signal for localisation! Looking for the fiducial marker on the sandtray for 5sec max");

    detector->startLooking();

    float elapsedTime = 0;
    while(!detector->object_found and elapsedTime < 5) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        elapsedTime += 0.1;
    }

    detector->stopLooking();

    if(!detector->object_found) {// timeout!
        ROS_ERROR_STREAM("Could not see the fiducial marker after " << elapsedTime << "s. You might want to make sure the sandtray is in the field of view of the robot's camera!");
        return;
    }

    foundSandtray = true;

    // Re-compute the *current* transformation between the camera and the robot reference frame
    tf::StampedTransform robot_reference2camera;

    tl->waitForTransform(robotReferenceFrame, detector->camera_frame, ros::Time(), ros::Duration(1));
    tl->lookupTransform(robotReferenceFrame, detector->camera_frame, ros::Time(), robot_reference2camera);

    tf::StampedTransform marker2target;
    tl->lookupTransform(markerFrame, targetFrame, ros::Time(), marker2target);

    // finally, compute the final transform:
    // odom -> camera -> marker -> sandtray
    robot_reference2target = robot_reference2camera * detector->transform * upsidedown * marker2target;

    ROS_INFO("Found the fiducial marker! Starting to broadcast the robot's odom->sandtray transform");
    std_msgs::String msg;
    msg.data = "I know where I am now!";
    speechSignaling.publish(msg);

}

int main(int argc, char* argv[])
{

    upsidedown = tf::Transform(tf::Quaternion(1.0,0.0,0.0,0.0)); // pure rotation of 180deg around X

    //ROS initialization
    ros::init(argc, argv, "sandtray_localisation");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    speechSignaling = rosNode.advertise<std_msgs::String>("speech",1);

    auto tl = make_shared<tf::TransformListener>();
    tf::TransformBroadcaster br;

    // load parameters
    string signalingTopic;
    _private_node.param<string>("signaling_topic", signalingTopic, "sandtray_localising");

    _private_node.param<string>("target_frame", targetFrame, "sandtray");
    _private_node.param<string>("marker_frame", markerFrame, "fiducial_marker");
    _private_node.param<string>("robot_reference_frame", robotReferenceFrame, "odom");
    string markerId;
    _private_node.param<string>("marker_id", markerId, "709");
    string markerSize;
    _private_node.param<string>("marker_size", markerSize, "100");

    string configuration = "%YAML:1.0\n" + targetFrame + ":\n    - tag: " + markerId + "\n      size: " + markerSize + "\n";

    while(!tl->frameExists(robotReferenceFrame)) {
        ROS_WARN_STREAM("Waiting for reference frame " << robotReferenceFrame << " to become available...");
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
    

    ROS_INFO_STREAM("sandtray_localisation is ready. " << robotReferenceFrame << " -> sandtray transformation will be published/updated on TF when localisation is triggered on " << signalingTopic);
    

    ros::Rate r(10);
    while(ros::ok()) {


        if(foundSandtray) {
            br.sendTransform(
                tf::StampedTransform(robot_reference2target, 
                                     ros::Time::now(), 
                                     robotReferenceFrame,
                                     targetFrame));

        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


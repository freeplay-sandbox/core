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

tf::Transform robot_reference2target;

string targetFrame;
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
        ROS_ERROR_STREAM("Could not see any fiducial markers after " << elapsedTime << "s. You might want to make sure the sandtray is in the field of view of the robot's camera!");
        std_msgs::String msg;
        msg.data = "I am lost!";
        speechSignaling.publish(msg);
        return;
    }

    foundSandtray = true;

    // Re-compute the *current* transformation between the camera and the robot reference frame
    tf::StampedTransform robot_reference2camera;

    tl->waitForTransform(robotReferenceFrame, detector->camera_frame, ros::Time(), ros::Duration(1));
    tl->lookupTransform(robotReferenceFrame, detector->camera_frame, ros::Time(), robot_reference2camera);

    // finally, compute the final transform:
    // odom -> camera -> marker (= sandtray origin)
    robot_reference2target = robot_reference2camera * detector->transform;

    ROS_INFO("Found the fiducial markers! Starting to broadcast the robot's odom->sandtray transform");
    std_msgs::String msg;
    msg.data = "I know where I am now!";
    speechSignaling.publish(msg);

}

int main(int argc, char* argv[])
{

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
    _private_node.param<string>("robot_reference_frame", robotReferenceFrame, "odom");

    const int MARKER_SIZE = 80; //mm
    const int MARKER_PADDING = 20; //mm
    const int FIRST_MARKER_X = 60; //mm
    const int FIRST_MARKER_Y = 25; //mm
    const int MARKERS_PER_LINE = 5;


    string configuration = "%YAML:1.0\n"
                           "markers:\n";

    for (int tag : {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}) {
        int x = FIRST_MARKER_X + (tag % MARKERS_PER_LINE) * (MARKER_SIZE + MARKER_PADDING);
        int y = FIRST_MARKER_Y + (tag / MARKERS_PER_LINE) * (MARKER_SIZE + MARKER_PADDING);

        configuration +=   "    - tag: " + to_string(tag + 700) + "\n"
                           "      size: " + to_string(MARKER_SIZE) + "\n"
                           "      translation: [" + to_string(x) + ", " + to_string(-y) + ", 0]\n" // note the '-y' to match sandtray orientation
                           "      rotation: [180, 0, 0]\n" // match sandtray orientation
                           "      keep: 0\n";
    }

    while(!tl->frameExists(robotReferenceFrame) && ros::ok()) {
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


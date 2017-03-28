#include <cmath>

#include <functional>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>

using namespace std;

tf::StampedTransform sandtray_centre;

string targetFrame;
string gazeFrame;
string referenceFrame;

bool calibrationrunning = false;

size_t nb_measures;
double total_distance;
double total_baseline_distance;

void onSignal(ros::NodeHandle& rosNode, shared_ptr<tf::TransformListener> tl, const std_msgs::EmptyConstPtr& sig) {

    if (calibrationrunning) {
        ROS_INFO("Stopping visual focus calibration.");
        calibrationrunning = false;

        ROS_INFO("Calibration summary");
        ROS_INFO("===================");
        ROS_INFO_STREAM("Total distance gaze to target: " << total_distance << "m");
        ROS_INFO_STREAM("Average distance gaze to target: " << total_distance/nb_measures << "m");
        ROS_INFO_STREAM("Total baseline distance (from sandtray centre) to target: " << total_baseline_distance << "m");
        ROS_INFO_STREAM("Average baseline distance (from sandtray centre) to target: " << total_baseline_distance/nb_measures << "m");
        ROS_INFO_STREAM("Improvement wrt baseline: " << ceil((total_baseline_distance - total_distance)/total_baseline_distance * 100) << "%");



    }
    else {
        ROS_INFO_STREAM("Starting visual focus calibration. First, waiting for the target to be published...");

        tl->waitForTransform(gazeFrame, targetFrame, ros::Time(), ros::Duration(1));

        // initially, the visual target lays at the centre of the sandtray.
        // Use this position as a gaze baseline (as if the user is always staring at the centre of the screen)
        tl->lookupTransform(targetFrame, referenceFrame, ros::Time(), sandtray_centre);

        ROS_INFO("Alright, starting!");

        nb_measures = 0;
        total_distance = 0;
        total_baseline_distance = 0;
        calibrationrunning = true;

    }

}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "visualfocus_calibration");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    auto tl = make_shared<tf::TransformListener>();

    // load parameters
    string signalingTopic;
    _private_node.param<string>("signaling_topic", signalingTopic, "visualfocus_calibration");

    _private_node.param<string>("target_frame", targetFrame, "visual_target");
    _private_node.param<string>("gaze_frame", gazeFrame, "gazepose_0");
    _private_node.param<string>("reference_frame", referenceFrame, "sandtray");

    while(!tl->frameExists(referenceFrame)) {
        ROS_WARN_STREAM("Waiting for reference frame " << referenceFrame << " to become available...");
        ros::Duration(1).sleep();
    }
    while(!tl->frameExists(gazeFrame)) {
        ROS_WARN_STREAM("Waiting for gaze frame " << gazeFrame << " to become available...");
        ros::Duration(1).sleep();
    }

    

    ros::Subscriber signal_sub = rosNode.subscribe<std_msgs::Empty>(signalingTopic, 
                                                                    1, 
                                                                    boost::bind(onSignal, 
                                                                                rosNode, 
                                                                                tl,
                                                                                _1));

    ROS_INFO_STREAM("visualfocus_calibration is ready. Distance between " << targetFrame << " and " << gazeFrame << " will be computed when calibration is triggered on " << signalingTopic);

    tf::StampedTransform gaze_to_target;
    tf::StampedTransform target_to_reference;


    ROS_INFO_STREAM("Distance (cm), distance x, distance y");
    ros::Rate r(10);
    while(ros::ok()) {

        if(calibrationrunning) {
            tl->lookupTransform(gazeFrame, targetFrame, ros::Time(), gaze_to_target);
            tl->lookupTransform(targetFrame, referenceFrame, ros::Time(), target_to_reference);
            nb_measures += 1;
            double dist_x = gaze_to_target.getOrigin().x();
            double dist_y = gaze_to_target.getOrigin().y();
            double distance = sqrt(gaze_to_target.getOrigin().x() * gaze_to_target.getOrigin().x() + \
                                   gaze_to_target.getOrigin().y() * gaze_to_target.getOrigin().y());
            total_distance += distance;

            double baseline_distance = sqrt(
                            (target_to_reference.getOrigin().x() - sandtray_centre.getOrigin().x()) *
                            (target_to_reference.getOrigin().x() - sandtray_centre.getOrigin().x()) +
                            (target_to_reference.getOrigin().y() - sandtray_centre.getOrigin().y()) *
                            (target_to_reference.getOrigin().y() - sandtray_centre.getOrigin().y()));
            total_baseline_distance += baseline_distance;

            ROS_INFO_STREAM(distance * 100 << "," << dist_x * 100 << "," << dist_y * 100);
        }

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}


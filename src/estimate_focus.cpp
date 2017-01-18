#include <string>
#include <vector>
#include <sstream>
#include <chrono>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <playground_builder/estimate_focusConfig.h> // generated header from cfg/estimate_focus.cfg

#include <playground_builder/AttentionTargetsStamped.h>
#include <playground_builder/AttentionTarget.h>

using namespace std;

static const string HUMAN_FRAME_PREFIX = "face_";

static const double FOV = 20. / 180 * M_PI; // radians
static const float RANGE = 3; //m

std::chrono::milliseconds MIN_ATTENTIONAL_SPAN(1000); // minimum time, in ms, that an object must remain in focus to be considered attended

map<std::string, std::chrono::system_clock::time_point> last_seen_frames;

static std_msgs::ColorRGBA GREEN;

void callback(playground_builder::estimate_focusConfig &config, uint32_t level) {
    cout << "Hit callback" << endl;
    ROS_INFO("Reconfiguring: attentional span is now %dms", config.attentional_span);
    MIN_ATTENTIONAL_SPAN = chrono::milliseconds(config.attentional_span);
}

visualization_msgs::Marker makeMarker(int id, const string& frame, std_msgs::ColorRGBA color) {

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "focus_of_attention";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color = color;

    marker.lifetime = ros::Duration(0.5);

    return marker;
}

bool isInFieldOfView(const tf::TransformListener& listener, const string& target_frame, const string& observer_frame) {

    if (target_frame == observer_frame) return false;

    tf::StampedTransform transform;

    try{
        listener.lookupTransform(observer_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // object behind the observer?
    if (transform.getOrigin().x() < 0) return false;

    // the field of view's main axis is the observer's X axis. So, distance to main axis is
    // simply sqrt(y^2 + z^2)
    double distance_to_main_axis = sqrt(transform.getOrigin().y() * transform.getOrigin().y() + transform.getOrigin().z() * transform.getOrigin().z());

    double fov_radius_at_x = tan(FOV/2) * transform.getOrigin().x();

    //ROS_INFO_STREAM(target_frame << ": distance_to_main_axis: " << distance_to_main_axis << ", fov_radius_at_x: " << fov_radius_at_x);
    if (distance_to_main_axis < fov_radius_at_x) return true;

    return false;

}

int main( int argc, char** argv )
{
    GREEN.r = 0.; GREEN.g = 1.; GREEN.b = 0.; GREEN.a = 0.5;

    ros::init(argc, argv, "estimate_focus");

    dynamic_reconfigure::Server<playground_builder::estimate_focusConfig> server;
    dynamic_reconfigure::Server<playground_builder::estimate_focusConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    ros::NodeHandle n;
    ros::Rate r(30);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("estimate_focus", 1);
    ros::Publisher fov_pub = n.advertise<sensor_msgs::Range>("face_0_field_of_view", 1);
    ros::Publisher attentional_targets_pub = n.advertise<playground_builder::AttentionTargetsStamped>("attention_targets", 1);

    tf::TransformListener listener;
    vector<string> frames;


    // Prepare a range sensor msg to represent the fields of view
    sensor_msgs::Range fov;

    fov.radiation_type = sensor_msgs::Range::INFRARED;
    fov.field_of_view = FOV;
    fov.min_range = 0;
    fov.max_range = 10;
    fov.range = RANGE;


    ROS_INFO("Waiting until a face becomes visible...");
    while (!listener.waitForTransform("sandtray", "face_0", ros::Time::now(), ros::Duration(.5))) {
        ROS_DEBUG("Still no face visible...");
        r.sleep();
    }

    ROS_INFO("Face detected! We can start estimating the focus of attention...");

    while (ros::ok())
    {

        while (marker_pub.getNumSubscribers() < 1 && fov_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker or field of view");
            ros::spinOnce();
            r.sleep();
        }

        frames.clear();
        listener.getFrameStrings(frames);

        int nb_faces = 0;

        for(auto frame : frames) {
            if(frame.find(HUMAN_FRAME_PREFIX) == 0) {

                nb_faces++;

                int face_idx = stoi(frame.substr(HUMAN_FRAME_PREFIX.length(), 1));

                if (face_idx == 0) {

                    playground_builder::AttentionTargetsStamped targets;
                    targets.header.frame_id = frame;
                    targets.header.stamp = ros::Time::now();

                    map<std::string, std::chrono::system_clock::time_point> seen_frames;

                    for(size_t i = 0 ; i < frames.size(); ++i) {
                        auto now = std::chrono::system_clock::now();
                        if(isInFieldOfView(listener, frames[i], frame)) {

                            if (last_seen_frames.count(frames[i])) {
                                auto lasttime_seen = last_seen_frames[frames[i]];
                                seen_frames[frames[i]] = lasttime_seen;
                                if(now - lasttime_seen > MIN_ATTENTIONAL_SPAN) {
                                    ROS_DEBUG_STREAM(frames[i] << " is being attended to by " << frame);
                                    marker_pub.publish(makeMarker(i, frames[i], GREEN));

                                    playground_builder::AttentionTarget target;
                                    target.modality = playground_builder::AttentionTarget::VISUAL;
                                    target.frame_id = frames[i];
                                    targets.targets.push_back(target);
                                }
                            }
                            else {
                                seen_frames[frames[i]] = now;
                            }
                        }
                    }

                    last_seen_frames = seen_frames;

                    attentional_targets_pub.publish(targets);

                    fov.range = RANGE;
                    fov.header.stamp = ros::Time::now();
                    fov.header.frame_id = frame;
                    fov_pub.publish(fov); 
                }
                else {
                    ROS_WARN_THROTTLE(10, "Found a second human face. estimate_focus only processing face_0 for now");
                }
            }
        }
        if (nb_faces == 0) {

            // hide the field of view
            fov.range = 0;

            fov.header.stamp = ros::Time::now();
            fov.header.frame_id = "face_0";
            fov_pub.publish(fov); 

        }
        ros::spinOnce();
        r.sleep();
    }
}

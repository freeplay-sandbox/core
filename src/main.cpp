#include <string>
#include <map>

// opencv3
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

const double physicalMapWidth = 0.62; //m
const double physicalMapHeight = 0.42; //m

const double RESOLUTION=0.01; // m per cell

map<string, vector<geometry_msgs::PointStamped>> footprints;

void process(const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::ImageConstPtr& depth_msg) {

    auto rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image; 
    auto depth = cv_bridge::toCvShare(depth_msg)->image; 


    //uint16_t *image_data = (uint16_t *) depth.data;

    //float g_depth_avg;
    //double depth_total = 0;
    //int depth_count = 0;
    //for (unsigned int i = 0; i < depth_msg->height * depth_msg->width; ++i)
    //{
    //    //if ((0 < *image_data) && (*image_data <= g_max_z))
    //    if ((0 < *image_data))
    //    {
    //        depth_total += *image_data;
    //        depth_count++;
    //    }
    //    image_data++;
    //}
    //if (depth_count != 0)
    //{
    //    g_depth_avg = static_cast<float>(depth_total / depth_count);
    //}

    //ROS_INFO_STREAM("Avg depth: " << g_depth_avg);

    depth.convertTo(depth, CV_32F); // thresholding works on CV_8U or CV_32F but not CV_16U
    imshow("Input depth", depth);
    threshold(depth, depth, 0.5, 1.0, THRESH_BINARY_INV);
    depth.convertTo(depth, CV_8U); // masking requires CV_8U. All non-zero values are kept, so '1.0' is fine

    Mat maskedImage;
    rgb.copyTo(maskedImage, depth);

    //imshow("Input RGB", rgb);
    imshow("Masked input", maskedImage);
    waitKey(10);
}

void getFootprints(const visualization_msgs::MarkerArray& markers) {

    for (const auto& marker : markers.markers) {
        vector<geometry_msgs::PointStamped> bb;

        for (const auto& point : marker.points) {
            geometry_msgs::PointStamped p;
            p.header.frame_id=marker.header.frame_id;
            p.point = point;

            bb.push_back(p);
        }
        footprints[marker.header.frame_id] = bb;

        cout << "Got footprint for " << marker.header.frame_id << ": ";
        for(const auto& p : bb) {
         cout << "(" << p.point.x << "," << p.point.y << "), ";
        }
        cout << endl;

    }

}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "zoo_map_maker");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    tf::TransformListener listener;


    ros::Subscriber footprints_sub = rosNode.subscribe("footprints", 10, getFootprints);
    ros::Publisher occupancygrid_pub = rosNode.advertise<nav_msgs::OccupancyGrid>("map", 1);

    //image_transport::ImageTransport it(rosNode);
    //image_transport::Publisher occupancygrid_img_pub = it.advertise("map/image", 1);

    ROS_INFO("zoo_map_maker is ready. Waiting for footprints on /footprints + TF updates");

    ros::Rate loop_rate(5);
    while(rosNode.ok()){

        Mat occupancygrid((int)(physicalMapHeight / RESOLUTION),(int)(physicalMapWidth / RESOLUTION),  CV_8UC1, 255);

        nav_msgs::OccupancyGrid map;
        map.header.frame_id = "/sandtray";
        map.info.map_load_time = ros::Time::now();
        map.info.resolution = RESOLUTION;
        map.info.width = occupancygrid.size().width;
        map.info.height = occupancygrid.size().height;
        map.info.origin.position.y=-physicalMapHeight;


        for (auto const& kv : footprints) {

            vector<cv::Point> poly;

            for (const auto& p : kv.second) {
                geometry_msgs::PointStamped base_point;
                listener.transformPoint("/sandtray", p, base_point);
                poly.push_back(cv::Point(base_point.point.x / RESOLUTION, -base_point.point.y / RESOLUTION));
            }

            fillConvexPoly(occupancygrid,poly,0);
    }
        flip(occupancygrid, occupancygrid,0);
        blur(occupancygrid, occupancygrid,Size(3,3));

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", occupancygrid).toImageMsg();

        vector<signed char> grid;
        for(size_t i = 0; i < occupancygrid.total(); ++i) {
            grid.push_back(100-occupancygrid.data[i]*100/255); // replace with a memcpy?
        }
        map.data = grid;

        //occupancygrid_img_pub.publish(msg);
        occupancygrid_pub.publish(map);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


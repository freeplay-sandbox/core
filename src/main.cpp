#include <string>
#include <map>
#include <utility> // for std::pair
#include <iterator>
#include <algorithm>
#include <queue>

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

const double physicalMapWidth = 0.62; //m
const double physicalMapHeight = 0.42; //m

const double RESOLUTION=0.01; // m per cell

const double delayBeforeExecuting = 0.5; // sec
const double t0 = 1; //sec - extra time for the robot to get to the first point in traj
const double dt = 0.1; //sec - seconds between points in traj

map<string, vector<geometry_msgs::PointStamped>> footprints;

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

typedef std::pair<int, int> Location;

inline double heuristic(Location a, Location b) {
  int x1, y1, x2, y2;
  tie (x1, y1) = a;
  tie (x2, y2) = b;
  return abs(x1 - x2) + abs(y1 - y2);
}

inline uint8_t cost(const nav_msgs::OccupancyGrid map, Location a) {
    int x, y;
    tie (x, y) = a;

    assert(x>=0 && y>=0 && x<map.info.width && y<map.info.height);

    auto raw = map.data[(map.info.height-y) * map.info.width + x];
    if (raw < 0) raw = 50; // raw = -1 -> occupancy unknown; let assign an average cost

    return raw * 10;
}

inline Location to_map(double x, double y) {

    return {(int)(x / RESOLUTION), (int)(-y / RESOLUTION)};
}

inline std::pair<double, double> from_map(Location a) {

    int x, y;
    tie (x, y) = a;

    return {x * RESOLUTION, -y * RESOLUTION};
}

// from http://www.redblobgames.com/pathfinding/a-star/implementation.html
template<typename T, typename priority_t>
struct PriorityQueue {
  typedef pair<priority_t, T> PQElement;
  priority_queue<PQElement, vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  inline T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

std::vector<Location> neighbours(const nav_msgs::OccupancyGrid& map, Location p){

    int width=(int)map.info.width, height=(int)map.info.height;

    vector<Location> nghbs;

    if (p.first > 0) nghbs.push_back({p.first-1, p.second});
    if (p.first > 0 && p.second > 0) nghbs.push_back({p.first-1, p.second-1});
    if (p.second > 0) nghbs.push_back({p.first, p.second-1});
    if (p.first < width-1 && p.second > 0) nghbs.push_back({p.first+1, p.second-1});
    if (p.first < width-1) nghbs.push_back({p.first+1, p.second});
    if (p.first < width-1 && p.second < height-1) nghbs.push_back({p.first+1, p.second+1});
    if (p.second < height-1) nghbs.push_back({p.first, p.second+1});
    if (p.first > 0 && p.second < height-1) nghbs.push_back({p.first-1, p.second+1});

    return nghbs;

}

// based on http://www.redblobgames.com/pathfinding/a-star/implementation.html
std::vector<Location> astar(const nav_msgs::OccupancyGrid& map, Location start, Location goal) {

    vector<Location> path;

    std::map<Location,Location> came_from;
    std::map<Location,int> cost_so_far;

    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;


    cout << "Cost map:" << endl;
    for(uint8_t y=0;y<map.info.height;y++) {
        for(uint8_t x=0;x<map.info.width;x++) {
            cout << (int)(cost(map, {x,y}) / 100.);
        }
        cout << endl;
    }
    cout << "Starting A*...";
    while (!frontier.empty()) {
        auto current = frontier.get();

        if (current == goal) {
            break;
        }

        for (auto next : neighbours(map, current)) {
            int new_cost = cost_so_far[current] + cost(map, next);
            if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }

    Location next = goal;
    while(true) {
        path.push_back(next);
        if (next == start) break;
        next = came_from[next];
    }
    std::reverse(std::begin(path), std::end(path));

    cout << "done." << endl;
    cout << "Publishing path (" << path.size() << " points)" << endl;
    return path;

}

nav_msgs::Path plan(const nav_msgs::OccupancyGrid& map, const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point) {

    auto start = to_map(start_point.x, start_point.y);
    auto goal = to_map(goal_point.x, goal_point.y);


    nav_msgs::Path path;
    path.header.frame_id = "/sandtray";
    path.header.stamp = ros::Time::now() + ros::Duration(delayBeforeExecuting);

    auto points = astar(map, start, goal);

    size_t i = 0;
    for (auto p : points) {
        double x, y;
        tie (x, y) = from_map(p);
        auto point = geometry_msgs::PoseStamped();
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.header.frame_id = "/sandtray";
        point.header.stamp = ros::Time(t0 + i * dt);

        path.poses.push_back(point);
        
        i++;

    }

    return path;
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
    ros::Publisher path_pub = rosNode.advertise<nav_msgs::Path>("zoo_manipulation_path", 1);

    //image_transport::ImageTransport it(rosNode);
    //image_transport::Publisher occupancygrid_img_pub = it.advertise("map/image", 1);

    ROS_INFO("zoo_map_maker is ready. Waiting for footprints on /footprints + TF updates");

    ros::Rate loop_rate(5);
    size_t counter = 1;

    geometry_msgs::PointStamped zebraOrigin;
    zebraOrigin.header.frame_id="/zebra";

    geometry_msgs::PointStamped hippoOrigin;
    hippoOrigin.header.frame_id="/hippo";

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

        if (counter % 25  == 0){
            geometry_msgs::PointStamped start_point;
            geometry_msgs::PointStamped goal_point;
            listener.transformPoint("/sandtray", zebraOrigin, start_point);
            listener.transformPoint("/sandtray", hippoOrigin, goal_point);
            path_pub.publish(plan(map, start_point.point, goal_point.point));
        }
            
        ros::spinOnce();
        loop_rate.sleep();

        counter++;
    }

    return 0;
}


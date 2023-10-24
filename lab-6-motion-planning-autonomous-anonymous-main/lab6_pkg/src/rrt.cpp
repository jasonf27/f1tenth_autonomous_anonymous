// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

#define MAX_ITER 10000
#define lookAheadDistance 1.5

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    string occ_topic = "/occ_map";
    this->idx = 0;
    string drive_topic = "/drive";
    
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    occ_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occ_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_goal", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);
    waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_goal", 10);
    
    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "/ego_racecar/odom";
    string scan_topic = "/scan";

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // TODO: create a occupancy grid
    mapSize = vector<double>{6, 5};
    idxSize = vector<int>{int(mapSize[0] / resolution), int(mapSize[1] / resolution)};
    occ = vector<signed char>(idxSize[0] * idxSize[1], 0);
    publishMsg = true;

    // Initialize waypoints
    string file = "src/lab-6-motion-planning-autonomous-anonymous/teleop_waypoints_sim_1.csv";
    initWaypoints(file);

    // Initialize PID parameters
    initPID();

    // Initialize Random Generator
    this->x_dist = std::uniform_real_distribution<>(0, mapSize[0]);
    this->y_dist = std::uniform_real_distribution<> (-mapSize[1] / 2, mapSize[1] / 2);

    this->radius = 0.1;
    this->goalThresh = 0.05;

    this->lastUpdateTime = now().seconds();
    this->rrtIdx = 100;

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::initPID(){
    this->kp = 1.0;
    this->kd = 0;
    this->ki = 0.00000015;
    this->prevAngle = 0.0;
    this->integral = 0.0;
    this->lastTime = now().seconds();

    this->maxSteeringAngle = M_PI_4;
}

void RRT::initWaypoints(const string fileName){
    ifstream file(fileName);
    string value;
    while(file.good()){
        string line;
        getline(file, line);

        stringstream s(line);

        vector<double> row;
        while(getline(s, value, ',')){
            row.push_back(stod(value));
        }

        this->waypoints.push_back(row);
    }

    // this->lookAheadDistance = 1.5;
    this->maxSteeringAngle = M_PI_4;
}

int RRT::coordToIndex(double x, double y){
    if(x > this->mapSize[0]/ 2 || x < -this->mapSize[0] / 2 || y > this->mapSize[1]/ 2 || y < -this->mapSize[1] / 2) 
        return -1;

    int i = std::floor((this->mapSize[0] / 2 + x) / this->resolution);
    int j = std::floor((this->mapSize[1] / 2 - y) / this->resolution);

    return i * this->idxSize[1] + j;
}

vector<double> RRT::indexToCoord(int index){
    int i = index / this->idxSize[1];
    int j = index % this->idxSize[1];

    double x = this->mapSize[0] / 2 + (i + 0.5) * this->resolution;
    double y = -this->mapSize[1] / 2 - (j + 0.5) * this->resolution;

    return vector<double>{x, y};
}

bool RRT::isFree(double x, double y){
    int idx = this->coordToIndex(x, y);

    return (idx != -1) && (this->occ[idx] == 0);
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
    int rangeIdx = 0;
    double angle = scan_msg->angle_min;
    this->occ = vector<signed char>(this->occ.size(), 0);
    // std::cout << angle << " " << scan_msg->angle_max << " " << scan_msg->angle_increment << std::endl;
    while(angle <= scan_msg->angle_max){
        double range = scan_msg->ranges[rangeIdx];

        double distance = 0;
        while(distance <= range){
            double x = distance * std::cos(angle);
            double y = distance * std::sin(angle);

            int idx = this->coordToIndex(x, y);
            if(idx < 0)
                break;

            this->occ[idx] = 0;

            distance += this->resolution;
        }

        while(1){
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            int idx = this->coordToIndex(x, y);
            if(idx >= 0)
                this->occ[idx] = 100;
            else{
                break;
            }

            range += this->resolution;
        }

        rangeIdx++;
        angle += scan_msg->angle_increment;   
    }

    if(publishMsg){
        nav_msgs::msg::OccupancyGrid msg = nav_msgs::msg::OccupancyGrid();
        msg.header.frame_id = "ego_racecar/base_link";
        msg.header.stamp = now();

        msg.info.resolution = this->resolution;
        msg.info.width = this->idxSize[1];
        msg.info.height = this->idxSize[0];

        msg.info.origin.position.x = -this->mapSize[0] / 2;
        msg.info.origin.position.y = this->mapSize[1] / 2;  
        msg.info.origin.orientation.x = 0;  
        msg.info.origin.orientation.y = 0;  
        msg.info.origin.orientation.z = -0.7071;  
        msg.info.origin.orientation.w = 0.7071;

        msg.data = this->occ;   

        occ_pub_->publish(msg);
    }
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    auto pose = pose_msg->pose.pose;
    auto p = pose.position;
    auto o = pose.orientation;

    double carX = p.x, carY = p.y;
    double carYaw = std::atan2( 2.0 * (o.w * o.z + o.x * o.y),
                                1.0 - 2.0 * (o.y * o.y + o.z * o.z));

    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    vector<double> rrt_goal;
    if(now().seconds() - this->lastUpdateTime < 0.25
        && this->rrtIdx < this->globalPath.size()){
        
        rrt_goal = this->globalPath[this->rrtIdx];

        this->rrtIdx++;
    }
    else
    {    // Find Goal Locally
        double goalX_global = -1, goalY_global = -1;
        for(auto point: this->waypoints){
            double dx = point[0] - carX, dy = point[1] - carY;
            double dist = sqrt(dx * dx + dy * dy);

            if(abs(dist - lookAheadDistance) > 0.2)
                continue;

            double goalX = cos(yaw) * dx + sin(yaw) * dy;
            double goalY = -sin(yaw) * dx + cos(yaw) * dy;

            if(abs(std::atan(goalX / goalY)) < M_PI_2 && goalX > 0){
                goalX_global = point[0];
                goalY_global = point[1];
                break;
            }
        }

        this->pubMarker(goalX_global, goalY_global);

        double dx = goalX_global - carX, dy = goalY_global - carY;
        this->goalX = dx * cos(-carYaw) - dy * sin(-carYaw);
        this->goalY = dx * sin(-carYaw) + dy * cos(-carYaw);

        // tree as std::vector
        std::vector<RRT_Node> tree;

        RRT_Node root;
        root.x = 0;
        root.y = 0;
        root.is_root = true;
        root.parent = -1;

        tree.push_back(root);

        // cout << "Finding Path" << endl;
        vector<RRT_Node> path;
        // TODO: fill in the RRT main loop
        for(int i = 0; i < MAX_ITER; i++){
            vector<double> sampledPoint = this->sample();
            if(sampledPoint.empty()){
                continue;
            }

            int nearestIdx = this->nearest(tree, sampledPoint);

            auto newNode = this->steer(tree[nearestIdx], sampledPoint);
            newNode.parent = nearestIdx;

            if(!this->check_collision(tree[nearestIdx], newNode)){

                /* RRT* */
                // cout << "Finding neighbors" << endl;
                auto neighborhood = this->near(tree, newNode);
                
                // cout << "Min Finding" << endl;
                int minNodeIdx = nearestIdx;
                newNode.cost = tree[nearestIdx].cost + this->line_cost(newNode, tree[nearestIdx]);
                for(auto nodeIdx: neighborhood){
                    double cost = tree[nodeIdx].cost + this->line_cost(newNode, tree[nodeIdx]);
                    
                    if(!this->check_collision(tree[nodeIdx], newNode) && cost < newNode.cost){
                        minNodeIdx = nodeIdx;
                        newNode.cost = cost;
                    }
                }

                newNode.parent = minNodeIdx;

                for(int nodeIdx: neighborhood){
                    double cost = newNode.cost + this->line_cost(newNode, tree[nodeIdx]);
                    if(!this->check_collision(tree[nodeIdx], newNode) && cost < tree[nodeIdx].cost){
                        tree[nodeIdx].cost = cost;
                        tree[nodeIdx].parent = tree.size();
                    }
                }
                /* RRT* End*/

                tree.push_back(newNode);
                path = this->find_path(tree, newNode);
                if(!path.empty()){
                    break;
                }
            }
        }

        if(path.empty()){
            RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT Found No New Path");
            return;
        }

        this->rrtIdx = 0;
        this->globalPath.clear();
        for(auto node: path){
            auto globalPose = this->localToGlobal(node.x, node.y, carX, carY, yaw);
            this->globalPath.push_back(globalPose);
        }

        rrt_goal = this->globalPath[this->rrtIdx];
        this->lastUpdateTime = now().seconds();
    }

    if(this->globalPath.empty())
        return;

    rrt_goal = this->globalToLocal(rrt_goal[0], rrt_goal[1], carX, carY, yaw);
    this->publishDriveMsg(rrt_goal);
    // path found as Path message

    auto msg = nav_msgs::msg::Path();
    msg.header.frame_id = "/map";
    // msg.header.frame_id = "ego_racecar/base_link";
    msg.header.stamp = now();
    for(auto node: this->globalPath){
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = node[0];//node.x;
        pose.pose.position.y = node[1];//node.y;

        msg.poses.push_back(pose);
    }

    this->path_pub_->publish(msg);
}

void RRT::pubMarker(double x, double y, string frame_id){
        // cout << "init Marker" << endl;
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.id = this->idx++;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    this->marker_pub_->publish(marker);
}

void RRT::publishDriveMsg(vector<double> goal){

    // cout << "Pure Pursuit" << endl;
    double KP_Pursuit = 0.7;
    double curvature = 2.0 * goal[1] / pow(lookAheadDistance, 2);

    double steeringAngle = KP_Pursuit * curvature;

    double t = now().seconds();
    double dt = 1.0 * (t - this->lastTime);

    this->integral += this->prevAngle * dt;
    double angleControl = (this->kp * steeringAngle + 
                            this->kd * (steeringAngle - this->prevAngle) / dt +
                            this->ki * this->integral);
    this->lastTime = t;

    angleControl = min(this->maxSteeringAngle, max(-this->maxSteeringAngle, angleControl));
    this->prevAngle = angleControl;

    double speed = 1.5;
    float video_mult = 11.0 / 39;
    speed *= video_mult;

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = angleControl;
    if(abs(angleControl) > 20.0 / 180.0 * (22/7))
        drive_msg.drive.speed = float(1.5 * speed);
    else if(abs(angleControl) > 10.0 / 180.0 * (22/7))
        drive_msg.drive.speed = float(1.75 * speed);
    else
        drive_msg.drive.speed = float(2.0 * speed);

    this->drive_pub_->publish(drive_msg);
}

vector<double> RRT::globalToLocal(double globalX, double globalY, double carX, double carY, double yaw){
    double dx = globalX - carX;
    double dy = globalY - carY;

    double x = dx * cos(-yaw) - dy * sin(-yaw);
    double y = dx * sin(-yaw) + dy * cos(-yaw);

    return vector<double>{x, y};
}

vector<double> RRT::localToGlobal(double localX, double localY, double carX, double carY, double yaw){
    double x = localX * cos(yaw) - localY * sin(yaw);
    double y = localX * sin(yaw) + localY * cos(yaw);

    return vector<double>{carX + x, carY + y};
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    double length = this->mapSize[0], width = this->mapSize[1];
    for(int iter = 0; iter < 100; iter++){
        double x = x_dist(gen);
        double y = y_dist(gen);

        if(this->isFree(x, y)){
            return std::vector<double>{x, y};
        }
    }

    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    double minDist = 10000;

    for(int i = 0; i < tree.size(); i++){
        auto node = tree[i];

        double dx = (sampled_point[0] - node.x);
        double dy = (sampled_point[1] - node.y);
        double dist = sqrt(dx * dx + dy * dy);

        if(dist < minDist){
            minDist = dist;
            nearest_node = i;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method
    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;

    double theta = std::atan2(dy, dx);
    double dist = sqrt(dx * dx + dy * dy);

    if(this->radius > dist){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else{
        new_node.x = nearest_node.x + this->radius * cos(theta);
        new_node.y = nearest_node.y + this->radius * sin(theta);
    }

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    double dx = new_node.x - nearest_node.x;
    double dy = new_node.y - nearest_node.y;
    double dist = sqrt(dx * dx + dy * dy);

    int numSamples = std::ceil(dist / this->resolution);
    for(int i = 0; i < numSamples; i++){
        double x = nearest_node.x + dx * (i / numSamples);
        double y = nearest_node.y + dy * (i / numSamples);

        int idx = this->coordToIndex(x, y);
        if(this->occ[idx] == 1)
            return true; 
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double dx = latest_added_node.x - goal_x;
    double dy = latest_added_node.y - goal_y;
    double dist = sqrt(dx * dx + dy * dy);

    return dist < this->goalThresh;


    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method
    if(this->is_goal(latest_added_node, this->goalX, this->goalY)){
        found_path.push_back(latest_added_node);
        while(!found_path.back().is_root){
            found_path.push_back(tree[found_path.back().parent]);
        }
    }

    reverse(found_path.begin(), found_path.end());

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    if(node.is_root){
        return 0;
    }

    if(tree[node.parent].is_root){
        return this->line_cost(node, tree[node.parent]);
    }

    return tree[node.parent].cost + this->line_cost(node, tree[node.parent]);
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    double dx = n1.x - n2.x, dy = n1.y - n2.y;
    cost = sqrt(dx * dx + dy * dy);

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    double r = 0.2;
    std::vector<int> neighborhood;
    // TODO:: fill in this method
    for(int i = 0; i < tree.size(); i++){
        double dx = tree[i].x - node.x, dy = tree[i].y - node.y;
        if(sqrt(dx * dx + dy * dy) <= r){
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}
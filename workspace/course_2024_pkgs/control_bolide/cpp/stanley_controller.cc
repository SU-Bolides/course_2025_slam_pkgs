

// This is supposed to be a translation of stanley_controller.py

// The point is to increase the rate from 25 Hz. 


#include <string>
#include <iostream>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
// #include <tf2_eigen/tf2_eigen.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>


#include "ros/ros.h"
#include "control_bolide/SpeedDirection.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"


class WaypointUtils {
    private:
        std::vector<Eigen::Vector2d> waypoints_world;
        std::vector<double> velocities;
        size_t velocity_index;
    public:
        WaypointUtils() {};
        ~WaypointUtils() {};


        std::vector<Eigen::Vector2d> transform_waypoints(const std::vector<Eigen::Vector2d>& waypoints, const Eigen::Vector2d& car_position, const geometry_msgs::Quaternion& pose) {
            std::vector<Eigen::Vector3d> translated_waypoints;
            for (const auto& waypoint : waypoints) {
                translated_waypoints.push_back(waypoint - car_position);
            } 

            Eigen::Quaterniond quaternion(pose.w, pose.x, pose.y, pose.z);
            std::vector<Eigen::Vector2d> transformed_waypoints;
            for (const auto& waypoint : translated_waypoints) {
                transformed_waypoints.push_back(quaternion * waypoint);
            }

            return transformed_waypoints;
        }


        std::pair<Eigen::Vector2d, double> get_closest_waypoint_with_velocity(const geometry_msgs::Pose& pose) {
            
            if(waypoints_world.empty() || velocities.empty()) {
                throw std::runtime_error("Waypoints or velocities are empty.");
            }

            Eigen::Vector3d position(pose.position.x, pose.position.y, 0);

            //TRANSFORM WAYPOINTS

            std::vector<double> distances;
            for (const auto& waypoint : waypoints_world) {
                //get distance from car to all waypoints
                distances.push_back((waypoint - position).norm());
            }

            auto minIt = std::min_element(distances.begin(), distances.end());
            velocity_index = std::distance(distances.begin(), minIt);

            return {waypoints_world[velocity_index], velocities[velocity_index]};
        }
    
};


class StanleyController {
    private:
        float VELOCITY_PERCENTAGE;
        float STEERING_LIMIT_DEG;
        float BRAKE_THRESHOLD_MS;

        float K_E, K_H, K_V, K_pobs, K_dh, K_ds;


        std::string waypoints_path; 
        std::string odom_topic; 
        std::string cmd_topic; 
    

        ros::NodeHandle node_;

        ros::Subscriber odom_sub;
        ros::Subscriber occup_grid_sub;

        ros::Publisher drive_pub;
        ros::Publisher curr_waypoint_pub;
        ros::Publisher waypoint_pub;
        ros::Publisher diag_pub;
        ros::Publisher stanley_avoidance_path_pub;
        ros::Publisher stanley_avoidance_path_array_pub;

        geometry_msgs::Pose current_pose; 
        double curr_angle_vel = 0;

        geometry_msgs::Pose current_pose_wheelbase_front;


        double old_target_vel = 0;
        double target_velocity = 0;



    public:
        StanleyController() {

            ros::NodeHandle private_nh_("~");


            //All the gains for the controller
            private_nh_.param<float>("K_E", K_E, 2.0);
            private_nh_.param<float>("K_H", K_H, 1.5);
            private_nh_.param<float>("K_V", K_V, 0.5);
            private_nh_.param<float>("K_p_obstacle", K_pobs, 0.5);
            private_nh_.param<float>("K_dh", K_dh, 0.0);
            private_nh_.param<float>("K_ds", K_ds, 0.0);
            
            private_nh_.param<float>("velocity_percentage", VELOCITY_PERCENTAGE, 0.3);
            private_nh_.param<float>("steering_limit_deg", STEERING_LIMIT_DEG, 15.2);
            private_nh_.param<float>("brake_threshold_ms", BRAKE_THRESHOLD_MS, 0.5);



            private_nh_.param<std::string>("waypoints_path", waypoints_path, "~/bolide_ws/course_2024_pkgs/control_bolide/racelines/esclangon_loop_2_0.3.csv");
            private_nh_.param<std::string>("odom_topic", odom_topic, "/pf/pos/odom");
            private_nh_.param<std::string>("cmd_topic", cmd_topic, "cmd_vel");

            drive_pub = node_.advertise<control_bolide::SpeedDirection>(cmd_topic, 10);
            curr_waypoint_pub = node_.advertise<visualization_msgs::Marker>("current_waypoint", 10);
            waypoint_pub = node_.advertise<visualization_msgs::Marker>("next_waypoint", 10);
            diag_pub = node_.advertise<std_msgs::Float32MultiArray>("/stanley_diagnostics", 1);

            stanley_avoidance_path_pub = node_.advertise<visualization_msgs::Marker>("stanley_avoidance", 10);
            stanley_avoidance_path_array_pub = node_.advertise<visualization_msgs::MarkerArray>("stanley_avoidance_path", 10);

            odom_sub = node_.subscribe<nav_msgs::Odometry>(odom_topic, 1, std::bind(&StanleyController::odomCB, this, std::placeholders::_1));
        
        }


        ~StanleyController() {};

        void odomCB(const nav_msgs::Odometry::ConstPtr& odom) {
            
            current_pose = odom->pose.pose;
            curr_angle_vel = odom->twist.twist.angular.z;

            {
                tf2::Quaternion current_pose_quaternion(
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w
                );

                tf2::Matrix3x3 mat(current_pose_quaternion);

                tf2::Vector3 point_front(0.195, 0, 0); // Point in front of the car in the car's frame
                tf2::Vector3 point_transformed = mat * point_front;

                point_transformed += tf2::Vector3(current_pose.position.x, current_pose.position.y, 0);

                current_pose_wheelbase_front.position.x = point_transformed.x();
                current_pose_wheelbase_front.position.y = point_transformed.y();
                current_pose_wheelbase_front.position.z = point_transformed.z();
                current_pose_wheelbase_front.orientation = current_pose.orientation;
            
            }    

            old_target_vel = target_velocity;

            std::cout << curr_angle_vel << "\n";
            return; 
        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "stanley_controller_node");
    StanleyController ctrl;

    ros::spin();



}
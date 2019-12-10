#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Optitrack Pose Callback
nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("/H02/odometry", 10, odom_cb);

    // Publishers
    ros::Publisher transform_pub = nh.advertise<geometry_msgs::TransformStamped>
            ("transform", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);

    // Start timer and choose initial setpoint typ
    ros::Time last_request = ros::Time::now();

    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.frame_id = "odom";
    transform_msg.child_frame_id = "ouster_link";
    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create full message with all topics
	// Note: the negative signs are to put the positions and
	// velocities into the NED frame required by roscopter.
	// Orientation changes are handled within roscopter by
	// negating the pitch and yaw values.
        transform_msg.header.stamp = odom.header.stamp;
        transform_msg.transform.translation.x = odom.pose.pose.position.x;
        transform_msg.transform.translation.y = odom.pose.pose.position.y;
        transform_msg.transform.translation.z = odom.pose.pose.position.z;

	// This is still in the NWU frame!!
	transform_msg.transform.rotation.x = odom.pose.pose.orientation.x;
	transform_msg.transform.rotation.y = odom.pose.pose.orientation.y;
	transform_msg.transform.rotation.z = odom.pose.pose.orientation.z;
	transform_msg.transform.rotation.w = odom.pose.pose.orientation.w;
        
	transform_pub.publish(transform_msg);

        ros::spinOnce();
        rate.sleep();
   }

    return 0;
}


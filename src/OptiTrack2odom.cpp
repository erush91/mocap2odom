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
geometry_msgs::PoseStamped pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose = *msg;
}

// Optitrack Velocity Callback
geometry_msgs::TwistStamped vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vel = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/SubT/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_velocity/optitrack_frame/filtered", 10, vel_cb);

    // Publishers
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>
            ("odom/optitrack_NED", 10);
    ros::Publisher att_pub = nh.advertise<geometry_msgs::PointStamped>
            ("odom/attitude_euler", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);

    // Start timer and choose initial setpoint typ
    ros::Time last_request = ros::Time::now();

    nav_msgs::Odometry odom_msgs;
    odom_msgs.header.frame_id = "world";

    geometry_msgs::PointStamped attitude_msg;

    double roll;
    double pitch;
    double yaw;

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create full message with all topics
	// Note: the negative signs are to put the positions and
	// velocities into the NED frame required by roscopter.
	// Orientation changes are handled within roscopter by
	// negating the pitch and yaw values.
        odom_msgs.header.stamp = pose.header.stamp;
        odom_msgs.pose.pose.position.x = pose.pose.position.x;
        odom_msgs.pose.pose.position.y = -pose.pose.position.y;
        odom_msgs.pose.pose.position.z = -pose.pose.position.z;

	// This is still in the NWU frame!!
	odom_msgs.pose.pose.orientation.x = pose.pose.orientation.x;
	odom_msgs.pose.pose.orientation.y = pose.pose.orientation.y;
	odom_msgs.pose.pose.orientation.z = pose.pose.orientation.z;
	odom_msgs.pose.pose.orientation.w = pose.pose.orientation.w;
        
        // This is used for tranformations, not needed right now
        tf::Quaternion tf_quat;
	tf::Quaternion tf_out;
	tf::Quaternion q_rot;
	tf::Transform tf_rot;
	
	q_rot.setRPY(M_PI, 0,0);
        tf_rot.setRotation(q_rot);
	tf::quaternionMsgToTF(pose.pose.orientation, tf_quat);
	
	tf_out = tf_rot*tf_quat;
	tf::Matrix3x3(tf_out).getRPY(roll, pitch, yaw);	

	// This is attitude in NED frame
	attitude_msg.header.stamp = pose.header.stamp;
	attitude_msg.point.x = roll- M_PI/2;
	attitude_msg.point.y = pitch;
	attitude_msg.point.z = yaw;
        att_pub.publish(attitude_msg);

	// Transform the velocity to the vehicle-1 frame
	//odom_msgs.twist.twist.linear.x = cos(yaw)*vel.twist.linear.x-sin(yaw)*vel.twist.linear.y;
	//odom_msgs.twist.twist.linear.y = -1*(sin(yaw)*vel.twist.linear.x+cos(yaw)*vel.twist.linear.y);
	//odom_msgs.twist.twist.linear.z = -vel.twist.linear.z;
	
	// This is just the velocity in the 	
	odom_msgs.twist.twist.linear.x = vel.twist.linear.x;
	odom_msgs.twist.twist.linear.y = -vel.twist.linear.y;
	odom_msgs.twist.twist.linear.z = -vel.twist.linear.z;
	
	// TO DO: Compute angular velocity from optitrack orienatation
	odom_msgs.twist.twist.angular.x = vel.twist.angular.x;
	odom_msgs.twist.twist.angular.x = 0;
	odom_msgs.twist.twist.angular.y = -vel.twist.angular.y;
	odom_msgs.twist.twist.angular.y = 0;
	odom_msgs.twist.twist.angular.z = -vel.twist.angular.z;
	odom_msgs.twist.twist.angular.z = 0;
        
	odom_pub.publish(odom_msgs);

        ros::spinOnce();
        rate.sleep();
   }

    return 0;
}


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint8_t reach_state = 0;
    // Set our initial shape type to be a CYLINDER
    uint32_t shape = visualization_msgs::Marker::CYLINDER;

    while (ros::ok())
    {
        ros::spinOnce();

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create rviz and pick_object subscriber to the marker");
            sleep(1);
        }

        switch (reach_state)
        {
        case 0:
        {
            marker.action = visualization_msgs::Marker::ADD;
            n.getParam("/pick_up/tx", marker.pose.position.x);
            n.getParam("/pick_up/ty", marker.pose.position.y);
            n.getParam("/pick_up/tz", marker.pose.position.z);
            n.getParam("/pick_up/qx", marker.pose.orientation.x);
            n.getParam("/pick_up/qy", marker.pose.orientation.y);
            n.getParam("/pick_up/qz", marker.pose.orientation.z);
            n.getParam("/pick_up/qw", marker.pose.orientation.w);
            ROS_INFO_ONCE("Pick_up marker published");
            marker_pub.publish(marker);
            // Sleep for 5 s
            ros::Duration(5.0).sleep();
            reach_state = 2;
            break;
        }
        case 2:
        {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO_ONCE("Pick_up marker hidden");
            // Sleep for 5 s
            ros::Duration(5.0).sleep();
            reach_state = 3;
            break;
        }
        case 3:
        {
            marker.action = visualization_msgs::Marker::ADD;
            n.getParam("/drop_off/tx", marker.pose.position.x);
            n.getParam("/drop_off/ty", marker.pose.position.y);
            n.getParam("/drop_off/tz", marker.pose.position.z);
            n.getParam("/drop_off/qx", marker.pose.orientation.x);
            n.getParam("/drop_off/qy", marker.pose.orientation.y);
            n.getParam("/drop_off/qz", marker.pose.orientation.z);
            n.getParam("/drop_off/qw", marker.pose.orientation.w);
            ROS_INFO_ONCE("Drop-off marker published");
            marker_pub.publish(marker);

            reach_state = 4;
            break;
        }

        default:
            break;
        }
    }
}
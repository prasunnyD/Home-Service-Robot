#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

double pose_x;
double pose_y;

void getRobotPoseValues(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ){
  
  pose_x = msg->pose.pose.position.x;
  pose_y = msg->pose.pose.position.y;
  ROS_INFO("x: %f, y: %f", pose_x, pose_y);
  
}
  
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("/amcl_pose", 100, getRobotPoseValues);
    
  double dropOffX = -2.0; 
  double dropOffY = -4.0;
    
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  while (ros::ok())
  {
      
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    sleep(5);
    ROS_INFO("ADDING");
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -2.0;
    marker.pose.position.y = 1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    
    if((marker.pose.position.x == pose_x) & (marker.pose.position.y == pose_y )){
      
      sleep(5);
      ROS_INFO("Picking Up");
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }else if((pose_x == dropOffX) & (pose_y == dropOffY )){
      
      ROS_INFO("Dropping Off");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOffX;
      marker.pose.position.y = dropOffY;
      marker.pose.position.z = 0;
      marker_pub.publish(marker);
    }else{
      marker_pub.publish(marker);
    }
    
    ros::spin();
    r.sleep();
  }

 
}
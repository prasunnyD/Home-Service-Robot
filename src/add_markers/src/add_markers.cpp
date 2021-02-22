#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

double pose_x;
double pose_y;

void getRobotPoseValue(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  
  pose_x = msg->pose.pose.position.x;
  pose_y = msg->pose.pose.position.y;
  
}

double calculatePickUpDistance(double markerLocationX, double markerLocationY, double robotX, double robotY){
  
  double pickUp_distanceX = markerLocationX - robotX;
  double pickUp_distanceY = markerLocationY - robotY;
  double pickUp_distance = sqrt(pow(pickUp_distanceY, 2) + pow(pickUp_distanceX, 2)); // distance formula
  return pickUp_distance;
  
}

double calculateDropOffDistance(double dropOffX, double dropOffY, double robotX, double robotY){
  
  double dropOff_distanceX = dropOffX - robotX;
  double dropOff_distanceY = dropOffY -robotY;
  double dropOff_distance = sqrt(pow(dropOff_distanceY, 2) + pow(dropOff_distanceX, 2));
  return dropOff_distance;
}
  
  
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("/amcl_pose", 100, getRobotPoseValue);
  
  // Setting parameters of pick up and drop off goals
  double dropOffX;
  n.getParam("drop_off_goal_x" , dropOffX);
  double dropOffY;
  n.getParam("drop_off_goal_y" , dropOffY);
  double pickUpX;
  n.getParam("pick_up_goal_x" , pickUpX);
  double pickUpY;
  n.getParam("pick_up_goal_y" , pickUpY);
  bool pickedUp = false;
  // Buffer distance accounting for error 
  double acceptance_distance = 0.2;
    
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
    marker.pose.position.x = pickUpX;
    marker.pose.position.y = pickUpY;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

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
    
    
    ROS_INFO("pose_X: %f, pose_Y: %f", pose_x, pose_y);
    
    double pickUp_distance = calculatePickUpDistance(pickUpX, pickUpY, pose_x, pose_y);
    ROS_INFO("pickUp_distance: %f", pickUp_distance);
    
    double dropOff_distance = calculateDropOffDistance(dropOffX, dropOffY, pose_x, pose_y);
    ROS_INFO("dropOff_distance: %f", dropOff_distance);
    
                                    
    if(pickUp_distance <= acceptance_distance){
      
      ROS_INFO("PICKING UP");
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      sleep(5);
      pickedUp = true;
      
    }else if(dropOff_distance <= acceptance_distance & pickedUp == true){
      
      ROS_INFO("DROPPING OFF");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOffX;
      marker.pose.position.y = dropOffY;
      marker.pose.position.z = 0;
      marker_pub.publish(marker);
      
    }
    
    if(pickedUp != true){
      marker_pub.publish(marker);
    }
    
    ros::spinOnce();
    r.sleep();
  }

 
}
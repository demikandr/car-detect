#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <car_detect/TrackedObject.h>


ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::CUBE;

void bboxCallback(const car_detect::TrackedObjectConstPtr tracked_object)
   {

      car_detect::TrackedObject object = *tracked_object;
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/my_frame";
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = 0;
      marker.type = shape;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = object.pose.x;
      marker.pose.position.y = object.pose.y;
      marker.pose.position.z = object.pose.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = object.dims.x;
      marker.scale.y = object.dims.y;
      marker.scale.z = object.dims.z;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.5f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      // Publish the data.
       marker_pub.publish (marker);
   }


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
 ros::Subscriber sub = n.subscribe("input", 1, bboxCallback);
 marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::spin ();

}
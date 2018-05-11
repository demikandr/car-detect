#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <car_detect/TrackedObject.h>
#include <car_detect/TrackedObjects.h>


ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::LINE_LIST;

void bboxCallback(const car_detect::TrackedObjectsConstPtr tracked_objects)
   {

      car_detect::TrackedObjects objects = *tracked_objects;


       visualization_msgs::Marker marker;
       marker.header.frame_id = "/car_frame";
       marker.header.stamp = ros::Time::now();
       marker.ns = "basic_shapes";
       marker.type = shape;
       marker.action = visualization_msgs::Marker::ADD;
       marker.id = 0;
      for (int i = 0; i < objects.trackedObjects.size(); ++i) {
        car_detect::TrackedObject object = objects.trackedObjects[i];

        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        geometry_msgs::Point p3;
        geometry_msgs::Point p4;
        geometry_msgs::Point p5;
        geometry_msgs::Point p6;
        geometry_msgs::Point p7;
        geometry_msgs::Point p8;

        // 1
        p1.x = object.pose.pose.position.x;
        p1.y = object.pose.pose.position.y;
        p1.z = object.pose.pose.position.z;
        // 2
        p2.x = object.pose.pose.position.x+object.dims.dimensions.x;
        p2.y = object.pose.pose.position.y;
        p2.z = object.pose.pose.position.z;
        // 3
        p3.x = object.pose.pose.position.x;
        p3.y = object.pose.pose.position.y+object.dims.dimensions.y;
        p3.z = object.pose.pose.position.z;
        // 4
        p4.x = object.pose.pose.position.x;
        p4.y = object.pose.pose.position.y;
        p4.z = object.pose.pose.position.z+object.dims.dimensions.z;
        // 5
        p5.x = object.pose.pose.position.x+object.dims.dimensions.x;
        p5.y = object.pose.pose.position.y+object.dims.dimensions.y;
        p5.z = object.pose.pose.position.z;
        // 6
        p6.x = object.pose.pose.position.x+object.dims.dimensions.x;
        p6.y = object.pose.pose.position.y;
        p6.z = object.pose.pose.position.z+object.dims.dimensions.z;
        // 7
        p7.x = object.pose.pose.position.x;
        p7.y = object.pose.pose.position.y+object.dims.dimensions.y;
        p7.z = object.pose.pose.position.z+object.dims.dimensions.z;
        // 8
        p8.x = object.pose.pose.position.x+object.dims.dimensions.x;
        p8.y = object.pose.pose.position.y+object.dims.dimensions.y;
        p8.z = object.pose.pose.position.z+object.dims.dimensions.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.points.push_back(p1);
        marker.points.push_back(p3);

        marker.points.push_back(p1);
        marker.points.push_back(p4);

        marker.points.push_back(p2);
        marker.points.push_back(p5);

        marker.points.push_back(p2);
        marker.points.push_back(p6);

        marker.points.push_back(p3);
        marker.points.push_back(p5);

        marker.points.push_back(p3);
        marker.points.push_back(p7);

        marker.points.push_back(p4);
        marker.points.push_back(p6);

        marker.points.push_back(p4);
        marker.points.push_back(p7);

        marker.points.push_back(p8);
        marker.points.push_back(p5);

        marker.points.push_back(p8);
        marker.points.push_back(p6);

        marker.points.push_back(p8);
        marker.points.push_back(p7);


        
        // marker.pose.position.x = object.pose.pose.position.x;
        // marker.pose.position.y = object.pose.pose.position.y;
        // marker.pose.position.z = object.pose.pose.position.z;
        // // marker.pose.position.x = 0;
        // // marker.pose.position.y = 0;
        // // marker.pose.position.z = 0;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 0.0;

        // // Set the scale of the marker -- 1x1x1 here means 1m on a side
         marker.scale.x = 0.1;
        // marker.scale.y = object.dims.dimensions.y;
        // marker.scale.z = object.dims.dimensions.z;
        // marker.scale.x = 1.0;
        // marker.scale.y = 1.0;
        // marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0;


        // Publish the data.
      }
       marker_pub.publish (marker);
   }


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
 marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("/tracker_fast/object", 1, bboxCallback);

  ros::spin ();

 // while (ros::ok())
 //  {
 //    visualization_msgs::Marker marker;
 //    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
 //    marker.header.frame_id = "/my_frame";
 //    marker.header.stamp = ros::Time::now();

 //    // Set the namespace and id for this marker.  This serves to create a unique ID
 //    // Any marker sent with the same namespace and id will overwrite the old one
 //    marker.ns = "basic_shapes";
 //    marker.id = 0;

 //    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
 //    marker.type = shape;

 //    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
 //    marker.action = visualization_msgs::Marker::ADD;

 //    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 //    marker.pose.position.x = 0;
 //    marker.pose.position.y = 0;
 //    marker.pose.position.z = 0;
 //    marker.pose.orientation.x = 0.0;
 //    marker.pose.orientation.y = 0.0;
 //    marker.pose.orientation.z = 0.0;
 //    marker.pose.orientation.w = 1.0;

 //    // Set the scale of the marker -- 1x1x1 here means 1m on a side
 //    marker.scale.x = 1.0;
 //    marker.scale.y = 1.0;
 //    marker.scale.z = 1.0;

 //    // Set the color -- be sure to set alpha to something non-zero!
 //    marker.color.r = 0.0f;
 //    marker.color.g = 1.0f;
 //    marker.color.b = 0.0f;
 //    marker.color.a = 1.0;

 //    marker.lifetime = ros::Duration();

 //    // Publish the marker
 //    while (marker_pub.getNumSubscribers() < 1)
 //    {
 //      if (!ros::ok())
 //      {
 //        return 0;
 //      }
 //      ROS_WARN_ONCE("Please create a subscriber to the marker");
 //      sleep(1);
 //    }
 //    marker_pub.publish(marker);

 //    // Cycle between different shapes
 //    switch (shape)
 //    {
 //    case visualization_msgs::Marker::CUBE:
 //      shape = visualization_msgs::Marker::SPHERE;
 //      break;
 //    case visualization_msgs::Marker::SPHERE:
 //      shape = visualization_msgs::Marker::ARROW;
 //      break;
 //    case visualization_msgs::Marker::ARROW:
 //      shape = visualization_msgs::Marker::CYLINDER;
 //      break;
 //    case visualization_msgs::Marker::CYLINDER:
 //      shape = visualization_msgs::Marker::CUBE;
 //      break;
 //    }

 //    r.sleep();
 //  }
}
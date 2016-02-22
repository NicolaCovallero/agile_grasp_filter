#include <ros/ros.h>
#include "agile_grasp/Grasps.h" 
#include "agile_grasp/Grasp.h" 
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

// Grasp.msg description:
// center : Point where finishes the grasping action. Point inside the object 
// axis : it is the closing direction (turned by 90 degrees)
// approach: direction of the grasping action
// surface_center : point on the surface
// width: width of the grasp hypotesis - it is generally very small


//---------------------Default parameters --------------
const double THRESHOLD = 0.9;/**< equal to minimum value of cos(alpha)  
                                  where alpha is the angle between the orthogonal vector
                                  and the grasp's approaching direction vector*/
const std::string INPUT_GRASPS_TOPIC = "/find_grasps/grasps";
const double WIDTH_SCALE = 1;
//------------------------------------------------------

uint counter = 0; // to count how many message from agile_grasp have been received

// filtering parameter
double threshold;/**< equal to minimum value of cos(alpha)  
                      where alpha is the angle between the orthogonal vector
                      and the grasp's approaching direction vector*/

//visualization parameters
double marker_lifetime_ = 5.;
double width_scale;/*< scale of the width for visualization*/

// Publishers of the node
ros::Publisher  filtered_grasp_visual_pub,
                filtered_grasp_pub,
                filtered_grasp_closing_direction_pub,
                filtered_grasp_poses_pub;

/*! \brief function to initialize the basic members of the marker object
*/
visualization_msgs::Marker createMarker(const std::string& frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration(marker_lifetime_);
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
} 
 
/*! \brief Computes the quaternion needed rotate the vector v1 up to v2
*
*  This is used only for the Rviz, to get the quaternion for makers set
*  v1 = (1,0,0) for poses v1 = (0,0,1). It has a singularity condition 
*  that is when v2 is aligned to v1 made only for the case v1=(1,0,0)
*  
*  \TODO make the singular condition for the case v1=(0,0,1)
*
*  \param v2 goal vector
*  \param v1 initial vector
*/
tf::Quaternion quaternionFromVector(tf::Vector3 v2,tf::Vector3 v1) //the PoseArray in Rviz is respect with (1,0,0)
{
  // all the normalizations are important! except the last one (with this method!)
  v2.normalize();

  tf::Vector3 cross_vector = v1.cross(v2);
  cross_vector.normalize();
  
  double angle = acos(v1.dot(v2))/2;
  //check for nans values
  // this occures only when v2 has the same direction of v1 (v2.normalize()=v1.normalize())
  if(cross_vector.x() != cross_vector.x()) //this will be true only for nans value
  {
    cross_vector.setX(0);
    cross_vector.setY(0);
    cross_vector.setZ(0);
    // check if the sense of v2 is positive or negative
    if(v2.x()<0)
    { // this means that the sense has to be negative,
      // and it is equal to a rotation in the y axis of M_PI
      angle = angle + M_PI;
      cross_vector.setY(1);
    }
    
  }

  // Build quaternion
  tf::Quaternion quatern; 
  quatern.setX(cross_vector.x() * sin(angle));
  quatern.setY(cross_vector.y() * sin(angle));
  quatern.setZ(cross_vector.z() * sin(angle));
  quatern.setW(cos(angle));  
  quatern.normalize(); 

  return quatern;
}

/*! \brief Function to create the closing direction marker
*
* \param frame reference frame of the marker 
* \param center center point of the marker(it would be the surface_center of the grasp)
* \param axis axis orthogonal to the closing plane
* \param approach approaching vector
* \param seq identifier of the marker
* \param width length of the marker
*/
visualization_msgs::Marker createClosingDirectionMarker(const std::string& frame,const geometry_msgs::Point& center,
                                                        geometry_msgs::Vector3& axis,
                                                        geometry_msgs::Vector3& approach,
                                                        int seq, float width)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.header.seq = seq;
  marker.lifetime = ros::Duration(marker_lifetime_);
  char str[20];
  sprintf(str,"closing_direction_%d",seq); 
  marker.ns = str;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0; 
  marker.type = visualization_msgs::Marker::CUBE;
  marker.color.r = 1.0;
  marker.color.a = 1.0; 

  marker.scale.x = width * width_scale;
  marker.scale.y = 0.002;
  marker.scale.z = 0.002;

  //center
  marker.pose.position.x = center.x;
  marker.pose.position.y = center.y;
  marker.pose.position.z = center.z;  

  // we have now the axis vector that should express the vector of closing direction, 
  // we convert it into a quaternion 
  // is the quaternion  expressed with respect the world? No because we have the axis with respect the
  // the camera frame
  
  // we have to convert the approaching vector to a quaternion for the marker
  tf::Vector3 v2(axis.x,axis.y,axis.z);
  tf::Vector3 v1(1,0,0);
  tf::Quaternion quat_tf = quaternionFromVector(v2,v1);
  // we have to turn this by 90 degrees around the approach direction
  tf::Vector3 approach_vector(approach.x,approach.y,approach.z);
  tf::Quaternion rotation_pi(approach_vector,M_PI/2);//axis angle construction
  quat_tf = rotation_pi * quat_tf;
  marker.pose.orientation.x = quat_tf.x();
  marker.pose.orientation.y = quat_tf.y();
  marker.pose.orientation.z = quat_tf.z();
  marker.pose.orientation.w = quat_tf.w();
  
  return marker;
}

/*! \brief create the marker (ARROW) for the approaching direction of the grasp
* \param frame reference frame of the marker
* \param center center point of the marker (it should be the surface_center point of the grasp)
* \param approach approaching direction vector
* \parma id marker's identifier
* \param r,g,b, color of the marker
* \param alpha transparency factor
* \param diam diameter of the arrow
*/
visualization_msgs::Marker createApproachMarker(const std::string& frame, const geometry_msgs::Point& center, 
  geometry_msgs::Vector3& approach, int id, double r, double g, double b , double alpha, double diam)
{
  visualization_msgs::Marker marker = createMarker(frame);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.id = id;
  marker.scale.x = diam; // shaft diameter
  marker.scale.y = diam; // head diameter
  marker.scale.z = 0.01; // head length
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = alpha;

  geometry_msgs::Point p, q;
  //point 1
  p.x = center.x;
  p.y = center.y;
  p.z = center.z;
  // point 2
  q.x = p.x - 0.03 * approach.x;
  q.y = p.y - 0.03 * approach.y;
  q.z = p.z - 0.03 * approach.z;

  marker.points.push_back(p);
  marker.points.push_back(q);
  return marker;
}

/*! \brief returns the pose
*\TODO the quaternion is not well computed
*/
geometry_msgs::Pose getPose(geometry_msgs::Point& center,
                            geometry_msgs::Vector3& approach)
{
  geometry_msgs::Pose tmp;
 
  tmp.position = center;

  tf::Vector3 v_approach(approach.x,approach.y,approach.z);
  tf::Vector3 v1(1,0,0);
  
  tf::Quaternion quat_tf = quaternionFromVector(v_approach,v1);

  tmp.orientation.x = quat_tf.x();
  tmp.orientation.y = quat_tf.y();
  tmp.orientation.z = quat_tf.z();
  tmp.orientation.w = quat_tf.w();

  return tmp; 
}

/*! \brief Callback for the agile_grasp::Grasps message

  It filters out the grasping poses and publish the markers

*/
void callback(const agile_grasp::Grasps::ConstPtr& msg)
{

  agile_grasp::Grasps filtered_grasps_msg;
  filtered_grasps_msg.header = msg->header;

  //marker for closing direction
  visualization_msgs::MarkerArray closing_dir_markers;

  //only the first message published by agile_grasp is about the handles, the 2nd one the hands 
  // here the handles refer to a cluster of possible grasping poses, 
  // while the hands simply as antipodal grasping poses. 
  counter++;
  if(counter == 1) 
  {

    std::vector<agile_grasp::Grasp> grasps = msg->grasps;
    std::vector<agile_grasp::Grasp> filtered_grasps; //new vector of filtered grasp poses
    
    //check if the orientation is orthogonal to the camera, up to a certain threshold 
    for (std::vector<agile_grasp::Grasp>::iterator i = grasps.begin(); i != grasps.end(); ++i)
    {
      // creating vertical and approching vectors
      geometry_msgs::Vector3 app = i->approach; 
      Eigen::Vector4f orientation, des_orientation;
      orientation << app.x , app.y , app.z , 0;
      orientation.normalize();

      des_orientation << 0 , 0 , 1 , 0;

      //here the kinect is supposed to be perpendicular to the table plane, 
      
      if(des_orientation.dot(orientation) >= threshold)
      {
        filtered_grasps.push_back(*i);
        filtered_grasps_msg.grasps.push_back(*i);
      }

    }

    if(grasps.size() > 0)
      ROS_INFO("size of grasps: %f \nSize of filtered grasps: %f",(double)grasps.size(),(double)filtered_grasps.size());
    else
      ROS_INFO("There are no filtered grasps.");

  
    //------- creating the Markers and the poses----------------------
    visualization_msgs::MarkerArray filtered_grasps_visual;
    filtered_grasps_visual.markers.resize((int)filtered_grasps.size());  

    // THE QUATERNION of the poses is not well done
    geometry_msgs::PoseArray poses;
    poses.header.seq = 0;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = "/camera_rgb_optical_frame";

    
    for (int i=0; i < filtered_grasps.size(); i++)
    {
      geometry_msgs::Point position;
      position.x = filtered_grasps[i].surface_center.x;
      position.y = filtered_grasps[i].surface_center.y;
      position.z = filtered_grasps[i].surface_center.z;
      visualization_msgs::Marker marker = createApproachMarker("camera_rgb_optical_frame", position, filtered_grasps[i].approach,
                                  i, 0., 0., 1., 0.4, 0.004);
      marker.ns = "filtered_grasps_visual ";
      marker.id = i;
      filtered_grasps_visual.markers[i] = marker;
      closing_dir_markers.markers.push_back(createClosingDirectionMarker("camera_rgb_optical_frame",
                                                      position,
                                                      filtered_grasps[i].axis,
                                                      filtered_grasps[i].approach,
                                                      i,filtered_grasps[i].width.data));
      // creating the pose for the i-th grasp pose
      poses.poses.push_back(getPose(position,filtered_grasps[i].approach));

    }
    ROS_INFO("Publish %d markers",(int)filtered_grasps.size()); 

    // VISUALIZATION -----------------------------
    // publishes the filtered grasping approach direction
    filtered_grasp_visual_pub.publish(filtered_grasps_visual);

    // publishes the closing direction of the grasping poses
    filtered_grasp_closing_direction_pub.publish(closing_dir_markers);
    // --------------------------------------------

    // Publish the messages of filtered grasping poses
    filtered_grasp_pub.publish(filtered_grasps_msg);
    filtered_grasp_poses_pub.publish(poses);

  }
  
  if(counter == 2)
    counter = 0; //reset counter

  return;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agile_grasp_filter");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);

  n.param("threshold",threshold,THRESHOLD);
  n.param("width_scale",width_scale,WIDTH_SCALE);

  // input topic
  std::string input_grasps_topic;
  n.param("input_grasps_topic",input_grasps_topic,INPUT_GRASPS_TOPIC);

  // Subscriber
  ros::Subscriber sub = n.subscribe(input_grasps_topic, 1, callback);
  
  // Publisher
  filtered_grasp_visual_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_grasp_visual",1);
  filtered_grasp_closing_direction_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_grasp_closing_direction",1);
  filtered_grasp_pub = n.advertise<agile_grasp::Grasps>("filtered_grasp",1);
  filtered_grasp_poses_pub = n.advertise<geometry_msgs::PoseArray>("filtered_grasp_poses",1);


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
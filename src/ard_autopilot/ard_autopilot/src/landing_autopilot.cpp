#include "ros/ros.h"
#include "ros/package.h"
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <ar_pose/ARMarker.h>
#include <ar_pose/ARMarkers.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/lexical_cast.hpp>
#include "std_msgs/Float32.h"
#include "../include/offset.h"
#include <ros/service_client.h>
#include "sensor_msgs/ChannelFloat32.h"
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>

using namespace std;

bool freashMarker = true;
bool markerSaved = false;
bool markerFound = true;
bool freshStart = true;
bool landingSucceeded = false;

int lastMarkerSeenTime = 0;
int firstMarkerSeenTime = 0;
int exception_count = 0;

// Apsibrėžiame metodą, kuris bus kviečiamas, kai mes aptinkame markerį
void MarkerCallback(const ar_pose::ARMarker::ConstPtr message)
{
  ROS_INFO("Marker detected!");

  lastMarkerSeenTime = message->header.stamp.sec;

  if (freashMarker == true) {
	  firstMarkerSeenTime = lastMarkerSeenTime;
    freashMarker = false;
	  ROS_INFO("First detection at  %d", lastMarkerSeenTime);
  }
}

int main(int argc, char **argv)
{
  // Inicializuojame ROS mazgą
  ros::init(argc, argv, "ard_autopilot");
  ROS_INFO("Landing autopilot started!");

  ros::NodeHandle nodeHandle;

  // Inicijuojame TF transformacijų prenumeratorių
  tf2_ros::Buffer transformBuffer;
  tf2_ros::TransformListener transformListener(transformBuffer);

  ros::Rate rate(1.0);
 
  // Sukuriame markerio radimo su ar_pose temos prenumeratorių
  ros::Subscriber markerSubscriber = nodeHandle.subscribe("/ar_pose_marker", 1, MarkerCallback);
  
  // Inicijuojame drono valdymo komandų publikavimo objektą
  ros::Publisher commandPublisher = nodeHandle.advertise<std_msgs::String>("/uga_tum_ardrone/com", 100);
 
  // Sukuriame temos, skirtos nusileidimo trukmei skaičiuoti, publikatorių
  ros::Publisher timelinePublisher = nodeHandle.advertise<std_msgs::Int32>("/landing_timeline", 100);

  ros::Publisher linearOffsetPublisher = nodeHandle.advertise<geometry_msgs::Vector3>("/linear_offset", 100);
  ros::Publisher rotationalOffsetPub = nodeHandle.advertise<std_msgs::Float32>("/rotation_offset", 100);

  ros::ServiceClient cameraToggleService = nodeHandle.serviceClient<std_srvs::Empty> ( "/togglecam" );

  geometry_msgs::Vector3 offsetMessage;
  std_msgs::Float32 yawMessage;

  // Apsibrėžiame drono valdymo komandas, kurias naudosime
  std_msgs::String clearCommand, autoInitCommand, takeOffCommand, moveRelativelyCommand, landCommand, referenceCommand, maxControlCommand, initialReachDistCommand, stayWithinDistCommand, stayTimeCommand;

  clearCommand.data = "c clearCommands";
  autoInitCommand.data = "c autoInit 500 800";
  referenceCommand.data = "setReference $POSE$";
  maxControlCommand.data = "setMaxControl 1";
  initialReachDistCommand.data = "setInitialReachDist 0.1";
  stayWithinDistCommand.data = "setStayWithinDist 0.1";
  stayTimeCommand.data = "setStayTime 0.5";
  takeOffCommand.data = "c takeoff";
  landCommand.data = "c land";

  Offset offset;
  offset.SetRoll(0);
  offset.SetPitch(0);
  offset.SetGaz(0);
  offset.SetYaw(0);

  int count = 0;

  tfScalar roll, pitch, yaw;

  float targetX, targetY, targetZ, targetYaw;

  std_srvs::EmptyRequest request;
  std_srvs::EmptyResponse response;

  // Nuskaitome x,y,z ašių ir pokrypio paklaidas, leidžiantis ant markerio
  nodeHandle.getParam("/ard_autopilot/target_X", targetX);      // x paklaida
  nodeHandle.getParam("/ard_autopilot/target_Y", targetY);      // y paklaida
  nodeHandle.getParam("/ard_autopilot/target_Z", targetZ);      // z paklaida
  nodeHandle.getParam("/ard_autopilot/target_yaw", targetYaw);  // yaw paklaida

  // Sukuriame pagrindinį ROS mazgo ciklą
  while (nodeHandle.ok())
  {
    geometry_msgs::TransformStamped transformStamped;

      if (freshStart == true)
      {
	      ROS_INFO("target_x: %f", targetX);
	      ROS_INFO("target_y: %f", targetY);
	      ROS_INFO("target_z: %f", targetZ);
	      ROS_INFO("target_yaw: %f", targetYaw);
        freshStart = false;

        // Kameros pakeitis paleidus mazga
        /*
        ros::service::call<> ( "/togglecam", request, response );
	      ROS_INFO("Initial camera change");
	      ros::Duration(1.0).sleep();
	      continue;
        */
      }

    try
    {
      // Bandome gauti tranformaciją tarp drono apatinės kameros ir ar markerio
      transformStamped = transformBuffer.lookupTransform("ar_marker", "ardrone_base_bottomcam", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      // Nepavykus gauti tranformacijos palaukiame vieną sekundę
      ros::Duration(1.0).sleep();
      exception_count++;
      ROS_INFO("Exception %d", exception_count);
      // Tęsiame programos darbą sekančiame while cikle
      continue;
    }

    if (markerSaved == false && firstMarkerSeenTime != 0) {
      std_msgs::Int32 timelineMessage;
      timelineMessage.data = firstMarkerSeenTime;
      // Į „/landing_timeline“ temą publikuojame pirmojo markerio aptikimo laiką
      timelinePublisher.publish(timelineMessage);
      ROS_INFO("Saved first marker detection at  %d", firstMarkerSeenTime);
      markerSaved = true;
    }

    // Iš gautos TF trandformacijos tarp drono ir markerio sukuriame kvaternioną
    tf::Quaternion qaternion(
      transformStamped.transform.rotation.x, 
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z, 
      transformStamped.transform.rotation.w
    );

    // Iš kvaterniono sukuriame tranformacijos matricą
    tf::Matrix3x3 transformationMatrix(qaternion);
    
    // Iš tranformacijos matricos gauname posvyrį, polinkį ir pokrypį
    transformationMatrix.getRPY(roll, pitch, yaw);

    // Nustatome drono judesio objeko reikšmes pagal x, y, z ašis ir pokrypį
    offset.SetRoll(-transformStamped.transform.translation.x);
    offset.SetPitch(-transformStamped.transform.translation.y);
    offset.SetGaz(-abs(transformStamped.transform.translation.z));
    offset.SetYaw(((float)yaw * 180 / PI));

    offsetMessage.x = (offset.GetRoll());
    offsetMessage.y = (offset.GetPitch());
    offsetMessage.z = (offset.GetGaz());
    
    linearOffsetPublisher.publish(offsetMessage);
    yawMessage.data = offset.GetYaw();
    rotationalOffsetPub.publish<>(yawMessage);

    // Atimame nusileidmo taško paklaidos dydžius
    offset.ReduceOffsetToZero(offset, targetX, targetY, targetZ, targetYaw);

    moveRelativelyCommand.data = "c moveByRel " + 
                       boost::lexical_cast<std::string>(offset.GetRoll()) + " " +
                       boost::lexical_cast<std::string>(offset.GetPitch()) + " " +
                       boost::lexical_cast<std::string>(offset.GetGaz()) + " " +
                       boost::lexical_cast<std::string>(offset.GetYaw());

    if (moveRelativelyCommand.data.compare("c moveByRel 0 0 0 0") == 0)
    {
      ROS_INFO("Destination reached");
      commandPublisher.publish<>(moveRelativelyCommand);
      // Nuleidžiam droną ant markerio
      commandPublisher.publish<>(landCommand);
	
      if (landingSucceeded == false) {
        int currentTimeInSecForLanding = (int)ros::Time::now().sec;
        std_msgs::Int32 timelineMessage;
        timelineMessage.data = currentTimeInSecForLanding;
        // Publikuojame į „/landing_timeline“ temą drono nuleidimo ant markerio laiką
        timelinePublisher.publish(timelineMessage);
        ROS_INFO("Landing succeeded at  %d", currentTimeInSecForLanding);
        landingSucceeded = true;
      }

	    ros::Duration ( 5.0 ).sleep();

      // Išjungiame drono autopiloto mazgą
      ros::shutdown();
    }
    else
    {
      if (lastMarkerSeenTime == 0)
      {
      	ROS_INFO("Fresh start");
      }
      else
      {
        int currentTimeInSec = (int)ros::Time::now().sec;
        if (currentTimeInSec == lastMarkerSeenTime)
        {
          commandPublisher.publish<>(clearCommand);
          if (count == 1)
          {
            ROS_INFO("Initialization");
            commandPublisher.publish<>(autoInitCommand);
            commandPublisher.publish<>(referenceCommand);
            commandPublisher.publish<>(maxControlCommand);
            commandPublisher.publish<>(initialReachDistCommand);
            commandPublisher.publish<>(stayWithinDistCommand);
            commandPublisher.publish<>(stayTimeCommand);
          }
          else
          {
	          ROS_INFO("Reaching landing spot");
            // Judame link markerio
            commandPublisher.publish<>(moveRelativelyCommand);
            cout << moveRelativelyCommand.data << endl;
          }
          cout << endl;
          ros::Duration ( 2.0 ).sleep();
        }
        else
        {
          // Pranešame konsolėje apie pamestą markerį
          ROS_WARN("Marker is lost!");

          // Išvalome drono komandų eilę
          commandPublisher.publish<>(clearCommand);

          // Sukuriame komandą kilti 80 cm aukštyn
          moveRelativelyCommand.data = "c moveByRel 0 0 0.8 0";

          // Siunčiame dronui komandą kilti aukštyn
          commandPublisher.publish<>(moveRelativelyCommand);
          cout << moveRelativelyCommand.data << endl;
          cout << endl;
          ros::Duration ( 2.0 ).sleep();
        }
      }
    }
    count++;
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}


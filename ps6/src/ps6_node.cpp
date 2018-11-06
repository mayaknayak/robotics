#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>

using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam1_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if (g_take_new_snapshot) {
        ROS_INFO_STREAM("image from cam2: " << message_holder << endl);
		ROS_INFO_STREAM("image from cam2: " << message_holder.models.size() << endl);
        g_cam1_data = message_holder;
		ROS_INFO_STREAM("g_cam2_data: " << g_cam1_data.models.size() << endl);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);
	
    //attempt to start up service
    startup_srv.response.success = false;
    while (!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from startup service");

    //attempt to start conveyor belt
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while (!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box");
    
    ROS_INFO("got success response from conveyor service");

    //Monitor the position of the box
    bool see_box = false;
    bool is_under = false;
    while (!see_box) {

	//take a snapshot to figure out whereabouts of box
	g_take_new_snapshot = true;

	//keep going until you see a box
	while (g_cam1_data.models.size() < 1) {
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Box found");
	see_box = true;

	//process information about the box until it is not under the camera
	while (!is_under) {
		//the box is not under the camera so keep taking snapshots
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		if (g_cam1_data.models[0].pose.position.z > -0.1 && g_cam1_data.models[0].pose.position.z < 0.1) {

			//The box is now directly under the camera with a margin of error of (-0.1, 0.1)
			ROS_INFO("The box is under the camera");
			is_under = true;

			//stop the conveyor belt
			conveyor_srv.request.power = 0.0; 
			
			conveyor_srv.response.success = false;
			while (!conveyor_srv.response.success) {
				ROS_WARN("not successful stopping conveyor yet...");
				conveyor_client.call(conveyor_srv);
				ros::Duration(0.5).sleep();
			}

			//stay under the camera for 5 seconds per instructions
			ros::Duration(5.0).sleep(); 

			//restart the conveyor
			conveyor_srv.request.power = 100.0;
			conveyor_srv.response.success = false;
			while (!conveyor_srv.response.success) {
				ROS_WARN("not successful starting conveyor yet...");
				conveyor_client.call(conveyor_srv);
				ros::Duration(0.5).sleep();
			}
		}
	}
	
	//Done monitoring the box
	g_take_new_snapshot = false;
    }

    // Have drone pick up box 
    drone_srv.request.shipment_type = "order_0_shipment_0";
    drone_srv.response.success = false;
    while (!drone_srv.response.success) {
        ROS_WARN("not successful starting drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from drone service");
}


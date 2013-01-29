/*
 * main.cpp
 *
 *  This is a demo application to show how an interface to the water_simulation node can look like
 *
 *
 *  Created on: Jun 18, 2012
 *      Author: lengelhan
 */

using namespace std;

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "water_simulation/simulator_init.h"
#include "water_simulation/simulator_step.h"
#include "water_simulation/msg_source_sink.h"

namespace enc = sensor_msgs::image_encodings;


int main(int argc, char ** argv){

 ros::init(argc, argv,"simulation_caller");

 ros::NodeHandle nh;

 // call with filename as parameter
 // possible examples can be found in the imgs-folder of the water_simulation stack
 if (argc <= 1){
  ROS_INFO("call with path to a grayscale image as parameter");
  return -1;
 }

 cv::Mat land;
 float scale = 0.4; // don't run simulation on full resolution to speed up

 land = cv::imread(argv[1],0);
 cv::resize(land, land, cv::Size(),scale,scale, CV_INTER_CUBIC);
 land.convertTo(land, CV_32FC1,0.2/255); // full white corresponds to a height of 0.2m

 cv_bridge::CvImage out_msg;
 out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
 out_msg.image    = land;


 water_simulation::simulator_init srv_msg;
 srv_msg.request.id = 42; // set id to some arbitrary value
 srv_msg.request.land_img = *out_msg.toImageMsg();
 srv_msg.request.viscosity = 1; // viscosity of simulation in the interval (0,1]
 srv_msg.request.add_sink_border = true; // water at the border of the grid vanishes


 ros::service::waitForService("srv_simulator_init",ros::DURATION_MAX);
 ros::service::waitForService("srv_simulator_step",ros::DURATION_MAX);


 if (ros::service::call("srv_simulator_init", srv_msg)){
  ROS_INFO("simulation was initialized");
 }else{
  ROS_WARN("init failed");
 }

 water_simulation::simulator_step msg_step;
 msg_step.request.id = srv_msg.request.id;
 msg_step.request.iteration_cnt = 10;



 water_simulation::msg_source_sink source;
 source.height = 0.2; // new height in m
 source.x = 246; // position in grid
 source.y = 96;
 source.radius = 10; // radius of new source
 source.additive = false;
 msg_step.request.sources_sinks.push_back(source);

// // remove all water in this area
// source.height = 0.0; // all water in this area vanishes
// source.x = 200;
// source.radius = 20; // large sink
// msg_step.request.sources_sinks.push_back(source);

 cv::namedWindow("water");

 ros::Rate r(20);
 int iter = 0;
 while (ros::ok()){

   ROS_INFO("Sending step %i", iter++);

  if (ros::service::call("srv_simulator_step", msg_step)){

   if (!msg_step.response.valid_id){
    ROS_WARN("SENT request with wrong id!");

   }else{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_step.response.water_img, enc::TYPE_32FC1);

    cv::imshow("water", cv_ptr->image*10);
    cv::waitKey(10);
   }

  }

  r.sleep();


 }




}

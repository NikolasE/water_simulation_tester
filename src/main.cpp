/*
 * main.cpp
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
namespace enc = sensor_msgs::image_encodings;

#include "water_simulation/simulator_init.h"
#include "water_simulation/simulator_step.h"
#include "water_simulation/msg_source_sink.h"




int main(int argc, char ** argv){

 ros::init(argc, argv,"simulation_caller");

 ros::NodeHandle nh;

 assert(argc > 1);

 cv::Mat land;
 float scale = 0.4;

 land = cv::imread(argv[1],0);
 cv::resize(land, land, cv::Size(),scale,scale, CV_INTER_CUBIC);
 land.convertTo(land, CV_64FC1,0.2/255);

 cv_bridge::CvImage out_msg;
 out_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1; // Or whatever
 out_msg.image    = land;


 water_simulation::simulator_init srv_msg;
 srv_msg.request.id = 42;
 srv_msg.request.land_img = *out_msg.toImageMsg();
 srv_msg.request.viscosity = 0.9;


 ros::service::waitForService("srv_simulator_init",ros::DURATION_MAX);
 ros::service::waitForService("srv_simulator_step",ros::DURATION_MAX);


 if (ros::service::call("srv_simulator_init", srv_msg)){
  ROS_INFO("simulation was initialized");
 }else{
  ROS_WARN("init failed");
 }

 water_simulation::simulator_step msg_step;
 msg_step.request.id = srv_msg.request.id;
 msg_step.request.iteration_cnt = 50;

 water_simulation::msg_source_sink source;
 source.height = 0.2;
 source.x = 246;
 source.y = 96;
 source.radius = 10;
 msg_step.request.sources_sinks.push_back(source);

 source.height = 0.0;
 source.x = 200;
 source.radius = 20;
 msg_step.request.sources_sinks.push_back(source);

 cv::namedWindow("water");

 cv::Mat water_img;
 ros::Rate r(10);
 while (ros::ok()){



  if (ros::service::call("srv_simulator_step", msg_step)){

   if (!msg_step.response.valid_id){
    ROS_WARN("SENT request with wrong id!");

   }else{
    // ROS_INFO("new steps");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_step.response.water_img, enc::TYPE_64FC1);

    cv::imshow("water", cv_ptr->image*10);
    cv::waitKey(1);
   }

  }

  r.sleep();


 }




}

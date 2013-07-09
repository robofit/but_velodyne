/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/06/2013
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <nodelet/nodelet.h>
#include "rt_road_detection/detectors/sample_hue_detector.h"

namespace rt_road_detection {

	class SampleHueDetectorNodelet : public nodelet::Nodelet
	   {

		   boost::shared_ptr<image_transport::ImageTransport> it_;
		   boost::shared_ptr<SampleHueDetector> det_;

		   image_transport::Publisher pub_;
		   image_transport::Subscriber sub_;

		   void imageCallback(const sensor_msgs::ImageConstPtr& msg);

		   virtual void onInit();


	   };


	void SampleHueDetectorNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

		cv_bridge::CvImagePtr rgb;

		try {

			rgb = cv_bridge::toCvCopy(msg, msg->encoding);

		}
		catch (cv_bridge::Exception& e)
		{
		  NODELET_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		  return;
		}

		NODELET_INFO_ONCE("Received first RGB image.");

		if (pub_.getNumSubscribers() == 0) return;

		NODELET_INFO_ONCE("Publishing first detection.");

		cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);

		det_->detect(rgb,out_msg);

		pub_.publish(out_msg->toImageMsg());

	}

	void SampleHueDetectorNodelet::onInit() {

		ros::NodeHandle &nh = getNodeHandle();
		ros::NodeHandle &private_nh = getPrivateNodeHandle();
		it_.reset(new image_transport::ImageTransport(private_nh));

		int hue_min,hue_max,median_ks;

		private_nh.param("hue_min",hue_min,30);
		private_nh.param("hue_max",hue_max,70);
		private_nh.param("median_ks",median_ks,7);

		det_.reset(new SampleHueDetector(hue_min,hue_max,median_ks));

		std::string top_rgb_in = "rgb_in";
		std::string top_det_out = "det_out";

		if (top_rgb_in == ros::names::remap(top_rgb_in)) ROS_WARN("Topic %s was not remapped!",top_rgb_in.c_str());
		else ROS_INFO("Topic %s remapped to %s.",top_rgb_in.c_str(),ros::names::remap(top_rgb_in).c_str());

		if (top_det_out == ros::names::remap(top_det_out)) ROS_WARN("Topic %s was not remapped!",top_det_out.c_str());
		else ROS_INFO("Topic %s remapped to %s.",top_det_out.c_str(),ros::names::remap(top_det_out).c_str());

		//image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		sub_ = it_->subscribe(top_rgb_in, 1, &SampleHueDetectorNodelet::imageCallback,this);
		pub_ = it_->advertise(top_det_out,1);

		NODELET_INFO("SampleHueDetectorNodelet loaded.");

	}

} // namespace

#include <pluginlib/class_list_macros.h>


// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_EXPORT_CLASS(rt_road_detection::SampleHueDetectorNodelet,nodelet::Nodelet)

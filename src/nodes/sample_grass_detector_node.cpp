#include "rt_road_detection/sample_grass_detector.h"

using namespace rt_road_detection;
using namespace std;

image_transport::Publisher pub;
SampleRoadDetector *det;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr rgb;

	try {

		rgb = cv_bridge::toCvCopy(msg, msg->encoding);

	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  return;
	}

	ROS_INFO_ONCE("Received first RGB image.");

	if (pub.getNumSubscribers() == 0) return;

	ROS_INFO_ONCE("Publishing first detection.");

	cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);

	det->detect(rgb,out_msg);

	pub.publish(out_msg->toImageMsg());

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "grass_detector_node");
  ros::NodeHandle nh("~");

  int hue_min,hue_max,median_ks;

  ros::param::param("~hue_min",hue_min,90);
  ros::param::param("~hue_max",hue_max,130);
  ros::param::param("~median_ks",median_ks,11);

  string top_rgb_in = "rgb_in";
  string top_det_out = "det_out";

  if (top_rgb_in == ros::names::remap(top_rgb_in)) ROS_WARN("Topic %s was not remapped!",top_rgb_in.c_str());
  else ROS_INFO("Topic %s remapped to %s.",top_rgb_in.c_str(),ros::names::remap(top_rgb_in).c_str());

  if (top_det_out == ros::names::remap(top_det_out)) ROS_WARN("Topic %s was not remapped!",top_det_out.c_str());
  else ROS_INFO("Topic %s remapped to %s.",top_det_out.c_str(),ros::names::remap(top_det_out).c_str());

  det = new SampleRoadDetector(hue_min,hue_max,median_ks);

  image_transport::ImageTransport it(nh);

  // image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  image_transport::Subscriber sub = it.subscribe(ros::names::remap(top_rgb_in), 1, imageCallback);
  pub = it.advertise(ros::names::remap(top_det_out),1);

  ROS_INFO("Node started.");
  ros::spin();

  delete det;

}

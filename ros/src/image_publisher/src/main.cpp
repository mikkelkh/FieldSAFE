#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char **argv) {

  std::string topicArg;
  float rateArg;
  std::string mapImagePathArg;
  std::string encoding;

  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle n("~");

  ROS_INFO("Started map_publisher");

  n.param<std::string>("image_path", mapImagePathArg, "/foo/bar.png");
  n.param<std::string>("topic", topicArg, "/map");
  n.param<float>("rate", rateArg, 1);
  ros::Rate rate(rateArg); // The rate with which we send

  // LOAD
  cv::Mat image(cv::imread( mapImagePathArg, 1 ));
  if( !image.data ) {
    throw std::runtime_error("Loading image failed");
  }

  // Sanity checks
  if (image.depth() != CV_8U) {
      ROS_ERROR_STREAM("Can only handle images 8 bit depth = ");
      return 0;
  }
  if (image.channels() == 1 /*It's an gray image*/) {
      encoding = "mono8";
  } else if (image.channels() == 3) {
      encoding = "bgr8";
  } else {
      ROS_ERROR_STREAM("Can only handle images with 1 or 3 channels: num channels = " << image.channels());
      return 0;
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();

  ros::Publisher publisher = n.advertise<sensor_msgs::Image>(topicArg, 1);

  while (n.ok()) {
    publisher.publish(msg);
    rate.sleep();
  }
  return 0;
}

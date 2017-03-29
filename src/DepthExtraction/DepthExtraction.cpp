// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

cv::Mat image;

bool bol=false;
class PointCloudToImage
{
public:
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub_.publish (image_); //publish our cloud image
  }
  PointCloudToImage () : cloud_topic_("/depth/points"),image_topic_("output")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    char str[10];
    image =  cv_bridge::toCvShare(msg, "bgr8")->image;
    //waitKey(0);
    cv::vector<cv::Mat> rgbChannels(3);
    cv::split(image, rgbChannels);
 
    // Show individual channels
    cv::Mat  fin_img;
    cv::vector<cv::Mat> channels;
    channels.push_back(
                      (rgbChannels[0]<197  )&(rgbChannels[0]>199)&
                        (rgbChannels[1]<156)&(rgbChannels[1]>158)&
                        (rgbChannels[2]<95)&(rgbChannels[2]>97)
                      );
    cv::merge(channels, fin_img);
    cv::bitwise_and(image,image,fin_img,fin_img);
    //namedWindow("road",1);imshow("road", fin_img);
    cv::imshow("view",image);

    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    bol =false;
  }
}


int main (int argc, char **argv)
{

  cv::namedWindow("view");
  
  ros::init (argc, argv, "convert_pointcloud_to_image");
  ros::NodeHandle nh;
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("output", 1, imageCallback);
 

  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  cv::destroyWindow("view");
  return 0;
}
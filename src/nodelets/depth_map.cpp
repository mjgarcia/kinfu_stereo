#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>

namespace kinfu_stereo {

using namespace sensor_msgs;
using namespace stereo_msgs;

class DepthMapNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  ros::Subscriber sub_disparity_;

  // Publications
  boost::mutex connect_mutex_;
  boost::shared_ptr<image_transport::Publisher> pub_depth_map_;
  
  virtual void onInit();

  void connectCb();

  void disparityCb(const DisparityImageConstPtr& disp_msg);
};

void DepthMapNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &local_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(local_nh));

  std::string topic = nh.resolveName("disparity");
  
  // Subscribe to the disparity
  sub_disparity_ = nh.subscribe<stereo_msgs::DisparityImage>(topic, 1,
			  &DepthMapNodelet::disparityCb, this);
 
  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&DepthMapNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_points2_
  
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  pub_depth_map_.reset(new image_transport::Publisher(it_->advertise("depth_map",  1, connect_cb, connect_cb)));

}

// Handles (un)subscribing when clients (un)subscribe
void DepthMapNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_map_->getNumSubscribers() == 0)
  {
    sub_disparity_.shutdown();
  }
  else if (!sub_disparity_)
  {
    ros::NodeHandle &nh = getNodeHandle();
    std::string topic = nh.resolveName("disparity");
    sub_disparity_ =  nh.subscribe<stereo_msgs::DisparityImage>
      (topic, 1,&DepthMapNodelet::disparityCb, this);
  }
}

void DepthMapNodelet::disparityCb(const DisparityImageConstPtr& disp_msg)
{
  const Image& dimage = disp_msg->image;
  // Focal length
  const float f = disp_msg->f;
  // Baseline
  const float T = disp_msg->T;

  const float* disp_data = reinterpret_cast<const float*>(&dimage.data[0]);

  // Allocate new Image message
  sensor_msgs::ImagePtr depth_map_msg( new sensor_msgs::Image );
  depth_map_msg->header   = dimage.header;
  depth_map_msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_map_msg->height   = dimage.height;
  depth_map_msg->width    = dimage.width;
  depth_map_msg->step     = dimage.width * sizeof (uint16_t);
  depth_map_msg->data.resize( depth_map_msg->height * depth_map_msg->step);
  
  uint16_t* depth_map_data = reinterpret_cast<uint16_t*>(&depth_map_msg->data[0]);

  float fact = T*f*1000.0f;
  uint16_t bad_point = std::numeric_limits<uint16_t>::max();
 
  for (unsigned index = 0; index < depth_map_msg->height * depth_map_msg->width; ++index)
  {
    float disp = disp_data[index];
    // In milimeters
    depth_map_data[index] = (disp == -1.0f || disp == 0.0f) ? bad_point : (uint16_t)(fact/disp);
  }

  if(pub_depth_map_)
    pub_depth_map_->publish(depth_map_msg);
}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(kinfu_stereo, depth_map,
                        kinfu_stereo::DepthMapNodelet, nodelet::Nodelet)

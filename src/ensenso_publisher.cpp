#include <iostream>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/** @brief Convenience typedef for XYZ point clouds */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

/** @brief PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso_ptr;

std::string camera_frame_id;
bool FlexView;
int FlexViewImages;


cv::Mat toImage(const PointCloudXYZ& cloud)
{
  std::cout << "CREATING IMAGE W/ DIMS " << cloud.height << " " << cloud.width << "\n";
  cv::Mat mat;
  mat.create(cloud.height, cloud.width, CV_16U);

  for (std::size_t i = 0; i < cloud.height; i++)
  {
    for (std::size_t j = 0; j < cloud.width; ++j)
    {
      mat.at<uint16_t>(i, j) = static_cast<uint16_t>(cloud(j, i).z * 1000.0f);
    }
  }

  return mat;
}

std::vector<unsigned char> getRectifiedImages()
{
  int width, height, channels, bytes_per_channel;
  bool is_float;
  double time_stamp;

  ensenso_ptr->camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bytes_per_channel,
                                                                           &is_float, &time_stamp);

  ROS_INFO("Rectified image data: %d %d", width, height);
  ROS_INFO("%d %d", channels, bytes_per_channel);
  ROS_INFO("is float = %d and timestamp = %f", static_cast<int>(is_float), time_stamp);

  std::vector<unsigned char> image_data;
  image_data.resize(width * height);

  ensenso_ptr->camera_[itmImages][itmRectified][itmLeft].getBinaryData(image_data, NULL);

  ROS_INFO("IMG DATA SIZE %lu", image_data.size());
  return image_data;
}

void applyColor(const pcl::PointCloud<pcl::PointXYZ>& points, const std::vector<unsigned char>& grays,
                pcl::PointCloud<pcl::PointXYZRGB>& color_points)
{
  uint32_t width = points.width;
  uint32_t height = points.height;

  color_points.clear();
  color_points.width = width;
  color_points.height = height;
  color_points.is_dense = points.is_dense;

  for (size_t i = 0; i < points.points.size(); ++i)
  {
    const auto& ref = points.points[i];
    pcl::PointXYZRGB new_pt;
    new_pt.x = ref.x;
    new_pt.y = ref.y;
    new_pt.z = ref.z;

    new_pt.r = grays[i];
    new_pt.g = grays[i];
    new_pt.b = grays[i];

    color_points.points.push_back(new_pt);
  }
}

cv::Mat toGrayImg(const std::vector<unsigned char>& data, int width, int height)
{
//  std::cout << "CREATING RECT IMAGE W/ DIMS " << cloud.height << " " << cloud.width << "\n";
  cv::Mat mat;
  mat.create(height, width, CV_8UC1);

  for (std::size_t i = 0; i < width * height; i++)
  {
    mat.at<unsigned char>(i) = data[i];
//    mat.at<uint16_t>(i, j) = static_cast<uint16_t>(cloud(j, i).z * 1000.0f);
  }

  return mat;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ensenso_publisher");
    ros::NodeHandle nh;

    // Load parameters
    nh.param("camera_frame_id", camera_frame_id, std::string("camera_optical_frame"));

    if (nh.getParam("/ensenso_publisher_node/FlexView", FlexView))
    {
        std::cout << "Flex View: " << FlexView << std::endl;
    }
    
    if (nh.getParam("/ensenso_publisher_node/FlexViewImages", FlexViewImages))
    {
        std::cout << "Flex View Images: " << FlexViewImages << std::endl;
    }

    // Create publisher for depth cloud
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);

    // Publisher for depth image
    image_transport::ImageTransport it (nh);
    image_transport::Publisher depth_image_pub = it.advertise("depth/image", 0);

    ros::Rate loop_rate(1);

    ensenso_ptr.reset(new pcl::EnsensoGrabber);
    ensenso_ptr->openTcpPort();
    ensenso_ptr->openDevice();

    std::cout << "Device open, ready to start streaming\n";

    // Enables Flex View
    if (FlexView && FlexViewImages >= 2 && FlexViewImages <= 8)
    {
        ensenso_ptr->camera_[itmParameters][itmCapture][itmBinning].set(1);
        ensenso_ptr->camera_[itmParameters][itmCapture][itmFlexView].set(FlexViewImages); 
    }
    else
    {   
        ensenso_ptr->camera_[itmParameters][itmCapture][itmFlexView].set(false);
    }


    // Reserve memory for our outputs
    PointCloudXYZ::Ptr cloud_ptr = boost::make_shared<PointCloudXYZ>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_ptr->header.frame_id = camera_frame_id;
    color_cloud_ptr->header.frame_id = camera_frame_id;

//    auto depth_image = cv::Mat::create()

//    ensenso_ptr->start();


    while (ros::ok())
    {
      std::cout << "Grabbing cloud\n";
      ensenso_ptr->grabSingleCloud(*cloud_ptr);

      std::cout << "to image\n";
      cv::Mat depth_img = toImage(*cloud_ptr);

//      std::cout << "cv bridge\n";
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
      msg->header.frame_id = "left_camera";
      msg->header.stamp = ros::Time::now();
      depth_image_pub.publish(msg);


//      auto rect_data = getRectifiedImages();
//      auto rect_img = toGrayImg(rect_data, 1280, 1024);

//      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", rect_img).toImageMsg();
//      msg->header.frame_id = "left_camera";

//      std::cout << "Pub img\n";
//      depth_image_pub.publish(msg);

//      applyColor(*cloud_ptr, rect_data, *color_cloud_ptr);

      std::cout << "Pub cloud\n";
      point_cloud_pub.publish(color_cloud_ptr);


      ros::spinOnce();
      loop_rate.sleep();
    }

    ensenso_ptr->closeDevice ();

    return (0);
}

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
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

/** @brief PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso_ptr;

std::string camera_frame_id;
bool FlexView;
int FlexViewImages;


/**
 * @brief to depth image
 * @param cloud
 * @return
 */
cv::Mat toImage(const PointCloudXYZ& cloud)
{
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

void getRectifiedImage(std::vector<unsigned char>& image_data)
{
  image_data.resize(1280 * 1024);
  ensenso_ptr->camera_[itmImages][itmRectified][itmLeft].getBinaryData(image_data, NULL);
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
  cv::Mat mat;
  mat.create(height, width, CV_8UC1);

  for (std::size_t i = 0; i < width * height; i++)
  {
    mat.at<unsigned char>(i) = data[i];
  }

  return mat;
}

void captureColorCloud(PointCloudXYZ& xyz_cloud, PointCloudXYZRGB& rgb_cloud)
{
  // Take an image with the Ensenso with the projector on
  ROS_INFO("grab single");
  ensenso_ptr->grabSingleCloud(xyz_cloud);

  // Disable the projector and...
  try
  {
    ROS_INFO("projector off");
    ensenso_ptr->camera_[itmParameters][itmCapture][itmProjector].set(false);
    ROS_INFO("flex view off");
    ensenso_ptr->camera_[itmParameters][itmCapture][itmFlexView].set(false);
  }
  catch (const NxLibException& e)
  {
    ROS_ERROR_STREAM(e.getErrorText());
    throw;
  }

  // Take another image & rectify it
  ROS_INFO("capture");
  NxLibCommand (cmdCapture).execute();
  ROS_INFO("rectify");
  NxLibCommand (cmdRectifyImages).execute();

  // restore the projector settings
  ensenso_ptr->camera_[itmParameters][itmCapture][itmProjector].set(true);
  ensenso_ptr->camera_[itmParameters][itmCapture][itmFlexView].set(FlexViewImages);

  // Extract the rectified data
  ROS_INFO("colorize");
  std::vector<unsigned char> grayscale_data;
  getRectifiedImage(grayscale_data);

  // Apply the color to the point cloud
  applyColor(xyz_cloud, grayscale_data, rgb_cloud);
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
//    image_transport::ImageTransport it (nh);
//    image_transport::Publisher depth_image_pub = it.advertise("depth/image", 0);

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

    while (ros::ok())
    {
      ROS_INFO("capture...");
      captureColorCloud(*cloud_ptr, *color_cloud_ptr);

      point_cloud_pub.publish(color_cloud_ptr);

      ros::spinOnce();
      loop_rate.sleep();
    }

    ensenso_ptr->closeDevice ();

    return (0);
}

//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", rect_img).toImageMsg();
//msg->header.frame_id = "left_camera";

#include <iostream>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <godel_msgs/EnsensoCommand.h>

const static std::string ENSENSO_SRV_NAME = "ensenso_manager";

ros::Publisher chatter_pub; 
ros::Publisher chatter_pub2;

/** @brief Convenience typedef */
typedef pcl::visualization::CloudViewer CloudViewer;

/** @brief Convenience typedef for XYZ point clouds */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

/** @brief CloudViewer pointer */
boost::shared_ptr<CloudViewer> viewer_ptr;

/** @brief PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso_ptr;

std::string camera_frame_id;
bool FlexView;
int FlexViewImages;


PointCloudXYZ::Ptr downSampleCloud(const PointCloudXYZ &cloud, int skip)
{
    PointCloudXYZ::Ptr down_sampled_cloud(new PointCloudXYZ);

    for (int i = 0; i < cloud.size(); i+= skip)
    {
        down_sampled_cloud->push_back(cloud[i]);
    }

    return down_sampled_cloud;
}

void grabberCallback(const PointCloudXYZ::Ptr& cloud)
{
    cloud->header.frame_id = camera_frame_id;
    PointCloudXYZ::Ptr ds_cloud = downSampleCloud(*cloud, 6);
    ds_cloud->header.frame_id = camera_frame_id;
    chatter_pub2.publish(ds_cloud);
    chatter_pub.publish(cloud);
}

void stopEnsenso()
{
  ensenso_ptr->stop();
}


void startEnsenso()
{
  ensenso_ptr->start();
}

bool managerCallback(godel_msgs::EnsensoCommandRequest& req, godel_msgs::EnsensoCommandResponse& res)
{
  switch(req.action)
  {

  case godel_msgs::EnsensoCommandRequest::START:
    startEnsenso();
    break;

  case godel_msgs::EnsensoCommandRequest::STOP:
    stopEnsenso();
    break;

  }

  res.result = ensenso_ptr->isRunning();
  return true;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "ensenso_publisher");
    ros::NodeHandle nh;
    ros::ServiceServer manager = nh.advertiseService(ENSENSO_SRV_NAME, &managerCallback);

    nh.param("camera_frame_id", camera_frame_id, std::string("camera_optical_frame"));

    if (nh.getParam("/ensenso_publisher_node/FlexView", FlexView))
    {
        std::cout << "Flex View: " << FlexView << std::endl;
    }
    
    if (nh.getParam("/ensenso_publisher_node/FlexViewImages", FlexViewImages))
    {
        std::cout << "Flex View Images: " << FlexViewImages << std::endl;
    }

    chatter_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 100);
    chatter_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/depth/ds_points", 100);

    ros::Rate loop_rate(10);

    ensenso_ptr.reset (new pcl::EnsensoGrabber);
    ensenso_ptr->openTcpPort ();
    ensenso_ptr->openDevice ();

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

    boost::function<void(const PointCloudXYZ::Ptr&)> f = boost::bind (&grabberCallback, _1);
    ensenso_ptr->registerCallback (f);
    ensenso_ptr->start ();

    ros::spin();

    ensenso_ptr->closeDevice ();

    return (0);
}

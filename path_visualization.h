#ifndef PATH_VISULIZATION_H
#define PATH_VISULIZATION_H

#include "common_headers.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>

#define PI 3.1415926535
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

class VisualPath
{
public:
    VisualPath(ros::NodeHandle* nh);
    void tfCallBack(const tf2_msgs::TFMessageConstPtr tf);

private:
    ros::NodeHandle* nh;
    ros::Subscriber tf_sub;
    cv::Mat mat1;
    double x=234;
    double y=932;
    double theta = 1;
};

#endif // PATH_VISULIZATION_H

#ifndef __DEPTH_CALLBACK
#define __DEPTH_CALLBACK

#include <stdio.h>
#include "OpenNI.h"
#include "ros/ros.h"

#include "sensor_msgs/CameraInfo.h"

class DepthCallback : public openni::VideoStream::NewFrameListener
{
public:
    DepthCallback(ros::NodeHandle aRosNode, std::string camNamespace="camera", bool publishRosMessage = true, bool createCVwin = false, std::string calibrationFile="");
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence, publishRosMessage, createCVWindow;
    std::string m_CameraNamespace;
private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    ros::Publisher        m_RosPublisher;
    ros::Publisher        m_RosCameraInfoPublisher;

    sensor_msgs::CameraInfo     m_CamInfo;

};


#endif

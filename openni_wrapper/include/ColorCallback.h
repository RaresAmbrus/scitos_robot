#ifndef __COLOR_CALLBACK
#define __COLOR_CALLBACK

#include <stdio.h>
#include <string>
#include "OpenNI.h"
#include "ros/ros.h"
#include <pthread.h>

#include "DepthCallback.h"



class ColorCallback : public openni::VideoStream::NewFrameListener
{
public:
    ColorCallback(ros::NodeHandle aRosNode, std::string camNamespace="camera", bool publishRosMessage = true, bool createCVwin = false);
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence,publishRosMessage, createCVWindow;
    std::string m_CameraNamespace;
private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    ros::Publisher        m_RosImagePublisher;
    ros::Publisher        m_RosImagePublisherLowFPS;
    ros::Publisher        m_RosCameraInfoPublisher;
    std::string           m_rgbTopic;
    std::string           m_rgbInfoTopic;


};

#endif

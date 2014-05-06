#ifndef __DEPTH_CALLBACK
#define __DEPTH_CALLBACK

#include <stdio.h>
#include "OpenNI.h"
 #include "ros/ros.h"

#include <image_transport/subscriber_filter.h>

class DepthCallback : public openni::VideoStream::NewFrameListener
{
public:
    DepthCallback(ros::NodeHandle aRosNode, std::string camNamespace="camera", bool publishRosMessage = true, bool createCVwin = false);
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence, publishRosMessage, createCVWindow;
    std::string m_CameraNamespace;
private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    boost::shared_ptr<image_transport::ImageTransport>                          m_ImageTransport;
    image_transport::Publisher                                                  m_RosPublisher;
    image_transport::Publisher                                                  m_RosPublisherLowFPS;
    ros::Publisher                                                              m_RosCameraInfoPublisher;

};


#endif

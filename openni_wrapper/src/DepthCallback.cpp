#include "DepthCallback.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <camera_calibration_parsers/parse.h>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"

using namespace openni;
using namespace cv;
using namespace std;

DepthCallback::DepthCallback(ros::NodeHandle aRosNode, std::string camNamespace, bool publish_in_ros, bool createCVwin, std::string calibrationFile) : publishRosMessage(publish_in_ros), createCVWindow(createCVwin)
{
    if (createCVWindow)
    {
        namedWindow( "DepthWindow", CV_WINDOW_NORMAL );
        cvResizeWindow("DepthWindow", 640, 480);
        cvMoveWindow("DepthWindow",640,0);
    }

    m_CameraNamespace = camNamespace;

    m_RosNode = aRosNode;

    m_RosPublisher = m_RosNode.advertise<sensor_msgs::Image>(string("/") + string (m_CameraNamespace)+string("/")+"depth/image_raw", 1000);
    m_RosCameraInfoPublisher = m_RosNode.advertise<sensor_msgs::CameraInfo>(string("/") + string (m_CameraNamespace)+string("/")+"depth/camera_info", 1000);

    saveOneFrame = false;
    saveFrameSequence = false;

    std::ifstream calibrationFileStream(calibrationFile.c_str());
    if (!calibrationFileStream)
    {
        m_CamInfo.distortion_model = "plumb_bob";
        m_CamInfo.K = {{570.3422241210938, 0.0, 314.5, 0.0, 570.3422241210938, 235.5, 0.0, 0.0, 1.0}};
        m_CamInfo.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
        m_CamInfo.P = {{570.3422241210938, 0.0, 314.5, 0.0, 0.0, 570.3422241210938, 235.5, 0.0, 0.0, 0.0, 1.0, 0.0}};
        double D[5] = {0.0,0.0,0.0,0.0,0.0};
        m_CamInfo.D.assign(&D[0], &D[0]+5);
    } else {
        // calibration file exists -> load it
        calibrationFileStream.close();
        string cameraName;
        camera_calibration_parsers::readCalibration(calibrationFile,cameraName,m_CamInfo);
    }

    std::cout<<"Camera calibration info "<<m_CamInfo<<std::endl;


}

void DepthCallback::onNewFrame(VideoStream& stream)
{
    stream.readFrame(&m_frame);

    analyzeFrame(m_frame);
}

void DepthCallback::analyzeFrame(const VideoFrameRef& frame)
{

    Mat image;
    image.create(frame.getHeight(),frame.getWidth(),CV_16UC1);
    const openni::DepthPixel* pImageRow = (const openni::DepthPixel*)frame.getData();
    memcpy(image.data,pImageRow,frame.getStrideInBytes() * frame.getHeight());
//    image *= 16;
    if (createCVWindow)
    {
        imshow( "DepthWindow", image );
        waitKey(10);
    }

//    cout<<"New depth frame w: "<<frame.getWidth()<<"  h: "<<frame.getHeight()<<endl;

    if (saveOneFrame || saveFrameSequence)
    {
        char buffer[50];
        sprintf(buffer,"depth%lld.png",frame.getTimestamp());
        imwrite(buffer,image);

        saveOneFrame = false;
        std::cout<<"DepthCallback :: saved file "<<buffer<<std::endl;
    }

    if (publishRosMessage)
    {
        cv_bridge::CvImage aBridgeImage;
        aBridgeImage.image = image;
        aBridgeImage.encoding = "mono16";
        cv::flip(aBridgeImage.image,aBridgeImage.image,1);
        sensor_msgs::ImagePtr rosImage = aBridgeImage.toImageMsg();
//        rosImage.get()->header.frame_id="/camera_depth_optical_frame";
        rosImage.get()->header.frame_id=string("/") + string (m_CameraNamespace)+string("_depth_optical_frame");
        rosImage.get()->encoding="16UC1";
        rosImage.get()->header.stamp = ros::Time::now();
        m_RosPublisher.publish(rosImage);

        // camera info
        m_CamInfo.width = frame.getWidth();
        m_CamInfo.height = frame.getHeight();
        m_CamInfo.header.frame_id = string("/") + string (m_CameraNamespace)+string("_depth_optical_frame");
        m_CamInfo.header.stamp = rosImage.get()->header.stamp;
        m_RosCameraInfoPublisher.publish(m_CamInfo);
    }

}

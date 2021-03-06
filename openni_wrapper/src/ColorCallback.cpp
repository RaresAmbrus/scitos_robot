#include "ColorCallback.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include "OpenNI.h"

#include "camera_calibration_parsers/parse.h"

using namespace openni;
using namespace cv;
using namespace std;

ColorCallback::ColorCallback(ros::NodeHandle aRosNode, std::string camNamespace, bool publish_in_ros, bool createCVwin, std::string calibrationFile) : publishRosMessage(publish_in_ros), createCVWindow(createCVwin)
{

    if (createCVWindow)
    {
        namedWindow( "ColorWindow", CV_WINDOW_NORMAL );
        cvResizeWindow("ColorWindow", 640, 480);
    }

    m_CameraNamespace = camNamespace;

    m_RosNode = aRosNode;


    m_RosImagePublisher = m_RosNode.advertise<sensor_msgs::Image>(string("/") + string (m_CameraNamespace)+string("/")+"rgb/image_raw", 1000);
    m_RosCameraInfoPublisher = m_RosNode.advertise<sensor_msgs::CameraInfo>(string("/") + string (m_CameraNamespace)+string("/")+"rgb/camera_info", 1000);

    saveOneFrame = false;
    saveFrameSequence = false;

    // create camera info

    // ros camera parameters
    std::ifstream calibrationFileStream(calibrationFile.c_str());
    if (!calibrationFileStream)
    {
        m_CamInfo.distortion_model = "plumb_bob";
        m_CamInfo.K = {{525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0}};
        m_CamInfo.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
        m_CamInfo.P = {{525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0}};
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

void ColorCallback::onNewFrame(VideoStream& stream)
{
    stream.readFrame(&m_frame);

    analyzeFrame(m_frame);



}

void ColorCallback::analyzeFrame(const VideoFrameRef& frame)
{

    Mat image;
    image.create(frame.getHeight(),frame.getWidth(),CV_8UC3);
    const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)frame.getData();
    memcpy(image.data,pImageRow,frame.getStrideInBytes() * frame.getHeight());
    cvtColor(image, image, CV_RGB2BGR);
    if (createCVWindow)
    {
        imshow( "ColorWindow", image );
        waitKey(10);
    }

//    cout<<"New color frame w: "<<frame.getWidth()<<"  h: "<<frame.getHeight()<<endl;

    if (saveOneFrame || saveFrameSequence)
    {
        char buffer[50];
        sprintf(buffer,"rgb%lld.png",frame.getTimestamp());
        imwrite(buffer,image);
        saveOneFrame = false;
        std::cout<<"ColorCallback :: saved file "<<buffer<<std::endl;
    }

    if (publishRosMessage)
    {
        // ros image
        cv_bridge::CvImage aBridgeImage;
        aBridgeImage.image = image;
        aBridgeImage.encoding = "bgr8";
        cv::flip(aBridgeImage.image,aBridgeImage.image,1);

        sensor_msgs::ImagePtr rosImage = aBridgeImage.toImageMsg();        
        rosImage.get()->header.frame_id=string("/") + string (m_CameraNamespace)+string("_rgb_optical_frame");
        rosImage.get()->header.stamp = ros::Time::now();


        m_RosImagePublisher.publish(rosImage);

        // camera info
        m_CamInfo.width = frame.getWidth();
        m_CamInfo.height = frame.getHeight();
        m_CamInfo.header.frame_id = string("/") + string (m_CameraNamespace)+string("_rgb_optical_frame");
        m_CamInfo.header.stamp = rosImage.get()->header.stamp;
        m_RosCameraInfoPublisher.publish(m_CamInfo);

    }
}

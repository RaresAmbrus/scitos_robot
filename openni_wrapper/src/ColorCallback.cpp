#include "ColorCallback.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <iostream>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/CameraInfo.h"

#include "OpenNI.h"

using namespace openni;
using namespace cv;
using namespace std;

ColorCallback::ColorCallback(ros::NodeHandle aRosNode, std::string camNamespace, bool publish_in_ros, bool createCVwin) : publishRosMessage(publish_in_ros), createCVWindow(createCVwin)
{

    if (createCVWindow)
    {
        namedWindow( "ColorWindow", CV_WINDOW_NORMAL );
        cvResizeWindow("ColorWindow", 640, 480);
    }

    m_CameraNamespace = camNamespace;

    m_RosNode = aRosNode;

    m_ImageTransport = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(m_RosNode));

    m_RosImagePublisher = m_ImageTransport->advertise(string("/") + string (m_CameraNamespace)+string("/")+"rgb/image_raw", 1000);
    m_RosImagePublisherLowFPS = m_ImageTransport->advertise(string("/") + string (m_CameraNamespace)+string("/")+"rgb/image_raw_low_fps", 1000);

    m_RosCameraInfoPublisher = m_RosNode.advertise<sensor_msgs::CameraInfo>(string("/") + string (m_CameraNamespace)+string("/")+"rgb/camera_info", 1000);

    saveOneFrame = false;
    saveFrameSequence = false;


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

        static int fps_counter = 0;

        if (fps_counter == 15)
        {
            fps_counter = 0;
            m_RosImagePublisherLowFPS.publish(rosImage);
        } else {
            fps_counter++;
        }

        // ros camera parameters
        sensor_msgs::CameraInfo camInfo;
        camInfo.width = frame.getWidth();
        camInfo.height = frame.getHeight();
        camInfo.distortion_model = "plumb_bob";
        camInfo.K = {{525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0}};
        camInfo.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
        camInfo.P = {{525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0}};
        double D[5] = {0.0,0.0,0.0,0.0,0.0};
        camInfo.D.assign(&D[0], &D[0]+5);
        camInfo.header.frame_id = string("/") + string (m_CameraNamespace)+string("_rgb_optical_frame");
        camInfo.header.stamp = rosImage.get()->header.stamp;
        m_RosCameraInfoPublisher.publish(camInfo);
    }
}

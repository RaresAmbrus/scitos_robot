#include "OpenniWrapperNode.h"

#include <sstream>
#include <fstream>

using namespace openni;
using namespace std;
using namespace cv;

const int OpenniWrapperNode::openni_wrapper_max_devices = 10;

OpenniWrapperNode::OpenniWrapperNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
//    m_ColorCallback(nh,"rgb/image_raw","rgb/camera_info", true, false),
//    m_DepthCallback(nh, true, false)
{
    m_nodeHandle = nh;
    m_privateNodeHandle = private_nh;


    // check how many cameras have been defined
    m_DevicesDefined = 0;

    for (size_t i=0; i<openni_wrapper_max_devices;i++)
    {
        std::string deviceName;
        ostringstream deviceNameStream;
        deviceNameStream<<"camera";
        if (i>0) {    deviceNameStream<<i; }

        bool found = m_privateNodeHandle.getParam(deviceNameStream.str(),deviceName);
        if (found)
        {
            if (deviceName != "false")
            {
                m_DevicesDefined++;
                m_vCameraNamespace.push_back(deviceName);
            }
        } else {
//            break;
        }
    }

    if (m_vCameraNamespace.size() == 0)
    {
        cout<<"---------------------------- No devices defined. Exitting.-------------------------------"<<endl;
        exit(-1);
    }

    string inverted;
    bool found = m_privateNodeHandle.getParam("inverted",inverted);
    if (found)
    {
        if (inverted == "yes")
        {
            if (m_vCameraNamespace.size()>1)
            {
                cout<<"Namsespace for devices 1 and 2 will be swapped."<<endl;
                string temp = m_vCameraNamespace[0];
                m_vCameraNamespace[0] = m_vCameraNamespace[1];
                m_vCameraNamespace[1] = temp;
            } else{
                cout<<"Inverted argument passed but less than 2 devices detected."<<endl;
            }


        } else {
            cout<<"Namsespace for devices 1 and 2 will NOT be swapped."<<endl;
        }
    } else {
        cout<<"Parameter inverted not defined."<<endl;
    }


    cout<<"Devices defined "<<m_DevicesDefined<<endl;
    for (size_t i=0; i<m_DevicesDefined;i++)
    {
        cout<<m_vCameraNamespace[i]<<endl;
//        ColorCallback* colorCB = new ColorCallback(nh, m_vCameraNamespace[i], true, false);
//        DepthCallback* depthCB = new DepthCallback(nh, m_vCameraNamespace[i], true, false);

//        m_vDepthCallback.push_back(depthCB);
//        m_vColorCallback.push_back(colorCB);
    }


}

OpenniWrapperNode::~OpenniWrapperNode()
{

}


void OpenniWrapperNode::initializeOpenni()
{
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    } else {
        cout<<"Openni initialized"<<endl;
    }



    OpenNI::addDeviceConnectedListener(&m_devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&m_devicePrinter);
    OpenNI::addDeviceStateChangedListener(&m_devicePrinter);

    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for (int i = 0; i < deviceList.getSize(); ++i)
    {
        printf("Device \"%s\" already connected\n", deviceList[i].getUri());
    }

    m_vDevice.resize(deviceList.getSize());
    m_vColor.resize(deviceList.getSize());
    m_vDepth.resize(deviceList.getSize());

    cout<<"Number of devices connected "<<deviceList.getSize()<<endl;

    for (size_t i=0; i<m_DevicesDefined;i++)
    {
        if (i+1>deviceList.getSize())
        {
            cout<<"Name "<<m_vCameraNamespace[i]<<" defined but the device is not connected."<<endl;
            break;
        }

        m_vDevice[i] = new Device();
        rc = m_vDevice[i]->open(deviceList[i].getUri());
        if (rc != STATUS_OK)
        {
            printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
            return 2;
        } else {
            cout<<"Opened device "<<deviceList[0].getUri()<<"   namespace "<<m_vCameraNamespace[i]<<endl;
        }

        cout<<"This device supports the following sensors ";
        bool sensor = m_vDevice[i]->hasSensor(SENSOR_IR); if (sensor) cout <<"IR ";
        sensor = m_vDevice[i]->hasSensor(SENSOR_COLOR); if (sensor) cout <<"COLOR ";
        sensor = m_vDevice[i]->hasSensor(SENSOR_DEPTH); if (sensor) cout <<"DEPTH ";
        cout<<endl;

        ImageRegistrationMode mode = IMAGE_REGISTRATION_DEPTH_TO_COLOR;
        bool registrationSupported = m_vDevice[i]->getImageRegistrationMode();
        if(registrationSupported)
        {
            cout<<"Image registration SUPPORTED"<<endl;
            rc = m_vDevice[i]->setImageRegistrationMode(mode);
            // handle ret
            if (rc != STATUS_OK)
            {
                std::cout<<"Could not set the image registration on. Some error occured  "<<rc<<std::endl;
            }
        } else {
            cout<<"Image registration NOT SUPPORTED"<<endl;
        }

        rc = m_vDevice[i]->setDepthColorSyncEnabled(true);
        // handle rc
        if (rc != STATUS_OK)
        {
            std::cout<<"Could not set the depth-color sync. Some error occured"<<std::endl;
        }

        if (m_vDevice[i]->getSensorInfo(SENSOR_DEPTH) != NULL)
        {
            m_vDepth[i] = new VideoStream;
            rc = m_vDepth[i]->create(*m_vDevice[i], SENSOR_DEPTH);
            if (rc != STATUS_OK)
            {
                printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        VideoMode depthVideoMode = m_vDepth[i]->getVideoMode();
        depthVideoMode.setResolution(640,480);
        rc = m_vDepth[i]->setVideoMode(depthVideoMode);
        if (rc != STATUS_OK)
        {
            printf("Couldn't set increased resolution for depth stream\n%s\n", OpenNI::getExtendedError());
        }

        rc = m_vDepth[i]->start();
        if (rc != STATUS_OK)
        {
            printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        }


        char serialNumber[100];
        m_vDevice[i]->getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER,&serialNumber);
        std::cout<<"Device serial number "<<serialNumber<<std::endl;
        // check if a calibration file exists
        std::string home=getenv("HOME");
        std::string calibrationFileDepth = home+"/.ros/camera_info/depth_"+serialNumber+".yaml";
        std::string calibrationFileRGB = home+"/.ros/camera_info/rgb_"+serialNumber+".yaml";

        ifstream idepthfile(calibrationFileDepth.c_str());
        if (!idepthfile)
        {
            calibrationFileDepth = "";
        }
        idepthfile.close();

        ifstream irgbfile(calibrationFileRGB.c_str());
        if (!irgbfile)
        {
            calibrationFileRGB = "";
        }
        irgbfile.close();

        cout<<"RGB calib "<<calibrationFileRGB<<endl;
        cout<<"Depth calib "<<calibrationFileDepth<<endl;

        ColorCallback* colorCB = new ColorCallback(m_nodeHandle, m_vCameraNamespace[i], true, false,calibrationFileRGB);
        DepthCallback* depthCB = new DepthCallback(m_nodeHandle, m_vCameraNamespace[i], true, false,calibrationFileDepth);

        m_vDepthCallback.push_back(depthCB);
        m_vColorCallback.push_back(colorCB);

        if (m_vDevice[i]->getSensorInfo(SENSOR_COLOR) != NULL)
        {
            m_vColor[i] = new VideoStream;          

            rc = m_vColor[i]->create(*m_vDevice[i], SENSOR_COLOR);
            if (rc != STATUS_OK)
            {
                printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
            }

            CameraSettings* streamSettings = m_vColor[i]->getCameraSettings();

            if (streamSettings->getAutoExposureEnabled())
            {
                std::cout<<"Stream has auto exposure enabled"<<std::endl;
            } else {
                std::cout<<"Stream doesn't have auto exposure enabled"<<std::endl;
            }

            if (streamSettings->getAutoWhiteBalanceEnabled())
            {
                std::cout<<"Stream has white balancing enabled"<<std::endl;
            } else {
                std::cout<<"Stream doesn't have white balancing enabled"<<std::endl;
            }

//            streamSettings->setAutoExposureEnabled(false);
//            streamSettings->setGain(15000);
//            streamSettings->setAutoWhiteBalanceEnabled(false);
        }
        VideoMode colorVideoMode = m_vColor[i]->getVideoMode();
        colorVideoMode.setResolution(640,480);
        rc = m_vColor[i]->setVideoMode(colorVideoMode);
        if (rc != STATUS_OK)
        {
            printf("Couldn't set increased resolution for color stream\n%s\n", OpenNI::getExtendedError());
        }

        rc = m_vColor[i]->start();
        if (rc != STATUS_OK)
        {
            printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        }

        // Register to new frame
        m_vDepth[i]->addNewFrameListener(m_vDepthCallback[i]);
        m_vColor[i]->addNewFrameListener(m_vColorCallback[i]);
    }
}

void OpenniWrapperNode::terminateOpenni()
{
    std::cout<<"Shutting down Openni driver "<<std::endl;




    for (size_t i=0; i<m_DevicesDefined; i++)
    {
        m_vDepth[i]->removeNewFrameListener(m_vDepthCallback[i]);
        m_vColor[i]->removeNewFrameListener(m_vColorCallback[i]);

        m_vDepth[i]->stop();
        m_vDepth[i]->destroy();
        m_vColor[i]->stop();
        m_vColor[i]->destroy();
        m_vDevice[i]->close();

        delete m_vDepth[i];
        delete m_vColor[i];
        delete m_vDevice[i];

        delete m_vColorCallback[i];
        delete m_vDepthCallback[i];
    }

    OpenNI::shutdown();
}

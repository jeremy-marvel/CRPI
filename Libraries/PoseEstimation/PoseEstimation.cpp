// This is the main DLL file.

// *******************************************************************************
//	OpenCVKinect: Provides method to access Kinect Color and Depth Stream        *
//				  in OpenCV Mat format.                                           *
//                                                                                *
//				  Pre-requisites: OpenCV_2.x, OpenNI_2.x, KinectSDK_1.8           *
//                                                                                *
//   Copyright (C) 2013  Muhammad Asad                                            *
//                       Webpage: http://seevisionc.blogspot.co.uk/p/about.html   *
//						 Contact: masad.801@gmail.com                             *
//                                                                                *
//   This program is free software: you can redistribute it and/or modify         *
//   it under the terms of the GNU General Public License as published by         *
//   the Free Software Foundation, either version 3 of the License, or            *
//   (at your option) any later version.                                          *
//                                                                                *
//   This program is distributed in the hope that it will be useful,              *
//   but WITHOUT ANY WARRANTY; without even the implied warranty of               *
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                *
//   GNU General Public License for more details.                                 *
//                                                                                *
//   You should have received a copy of the GNU General Public License            *
//   along with this program.  If not, see <http://www.gnu.org/licenses/>.        *
//                                                                                *
//  This file has been modified by Nithyananda Bhat Kumbla for an internal        *
//  library on 12/8/2016                                                          *
//                                                                                *
//                                                                                *
//                                                                                *
// *******************************************************************************

#include "PoseEstimation.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <OpenNI.h>
#include <vector>
#include <math.h>


using namespace cv;
using namespace aruco;
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
Mat RelativeTrans, T, R;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;

Mat m_colorImageCopy;
PoseEstimation::PoseEstimation(void)
{
	m_depthTimeStamp = 0;
	m_colorTimeStamp = 0;

	m_alignedStreamStatus = false;
	m_colorStreamStatus = false;
	m_depthStreamStatus = false;
}

bool PoseEstimation::setMode(int inMode)
{
	if((inMode&C_MODE_COLOR) == C_MODE_COLOR && (inMode&C_MODE_DEPTH) == C_MODE_DEPTH )
	{
		// std::cout << "COLOR + DEPTH" << std::endl;
		m_colorStreamStatus = true;
		m_depthStreamStatus = true;


		if((inMode&C_MODE_ALIGNED) == C_MODE_ALIGNED)
		{
			std::cout << "+ ALIGNED" << std::endl;
			m_alignedStreamStatus = true;
		}

		return true;

	}
	else if((inMode&C_MODE_COLOR) == C_MODE_COLOR)
	{
		std::cout << "COLOR" << std::endl;
		m_colorStreamStatus = true;
		m_depthStreamStatus = false;
		m_alignedStreamStatus = false;
		return true;
	}
	else if((inMode&C_MODE_DEPTH) == C_MODE_DEPTH)
	{
		std::cout << "DEPTH" << std::endl;
		m_colorStreamStatus = false;
		m_depthStreamStatus = true;
		m_alignedStreamStatus = false;
		return true;
	}
	else
	{
		std::cout << "NOTHING" << std::endl;
		return false;

	}
	return false;
}

bool PoseEstimation::init()

{

	m_status = openni::STATUS_OK;
	const char* deviceURI = openni::ANY_DEVICE;

	m_status = openni::OpenNI::initialize();

	std::cout << "After initialization: " << std::endl; 
	std::cout << openni::OpenNI::getExtendedError() << std::endl;

	// open the device
	m_status = m_device.open(deviceURI);
	if(m_status != openni::STATUS_OK)
	{
		std::cout << "OpenCVKinect: Device open failseed: " << std::endl;
		std::cout << openni::OpenNI::getExtendedError() << std::endl;
		openni::OpenNI::shutdown();
		return false;
	}

	if(m_depthStreamStatus)
	{
		// create a depth object
		m_status = m_depth.create(m_device, openni::SENSOR_DEPTH);
		if(m_status == openni::STATUS_OK)
		{
			m_status = m_depth.start();
			if(m_status != openni::STATUS_OK)
			{
				std::cout << "OpenCVKinect: Couldn't start depth stream: " << std::endl;
				std::cout << openni::OpenNI::getExtendedError() << std::endl;
				m_depth.destroy();
				return false;
			}
		}
		else
		{
			std::cout << "OpenCVKinect: Couldn't find depth stream: " << std::endl;
			std::cout << openni::OpenNI::getExtendedError() << std::endl;
			return false;
		}
	}

	if(m_colorStreamStatus)
	{
		// create a color object
		m_status = m_color.create(m_device, openni::SENSOR_COLOR);
		if(m_status == openni::STATUS_OK)
		{
			m_status = m_color.start();
			if(m_status != openni::STATUS_OK)
			{

				std::cout << "OpenCVKinect: Couldn't start color stream: " << std::endl;
				std::cout << openni::OpenNI::getExtendedError() << std::endl;
				m_color.destroy();
				return false;
			}
		}
		else
		{
			std::cout << "OpenCVKinect: Couldn't find color stream: " << std::endl;
			std::cout << openni::OpenNI::getExtendedError() << std::endl;
			return false;
		}

	}

	if(m_alignedStreamStatus)
	{
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}


	if(!m_depth.isValid() && !m_color.isValid())
	{
		std::cout << "OpenCVKinect: No valid streams. Exiting" << std::endl;
		openni::OpenNI::shutdown();
		return false;
	}

	this->m_streams = new openni::VideoStream*[C_NUM_STREAMS];
	m_streams[C_DEPTH_STREAM] = &m_depth;
	m_streams[C_COLOR_STREAM] = &m_color;

	return true;
}


cv::vector<cv::Mat> PoseEstimation::getData()
{
	cv::vector<cv::Mat> returnVec;
	cv::Mat bufferImage;
	bool depthCaptured = false, colorCaptured = false;

	if(this->m_colorStreamStatus & this->m_depthStreamStatus)
	{
		while( !depthCaptured || !colorCaptured || m_depthTimeStamp != m_colorTimeStamp)
		{
			m_status = openni::OpenNI::waitForAnyStream(m_streams, C_NUM_STREAMS, &m_currentStream, C_STREAM_TIMEOUT);
			if(m_status != openni::STATUS_OK)
			{
				std::cout << "OpenCVKinect: Unable to wait for streams. Exiting" << std::endl;
				exit(EXIT_FAILURE);
			}

			switch(m_currentStream)
			{
			case C_DEPTH_STREAM:
				m_depth.readFrame(&m_depthFrame);
				m_depthImage.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
				m_depthImage.data = (uchar*)m_depthFrame.getData();
				this->m_depthTimeStamp = m_depthFrame.getTimestamp() >> 16;
				// std::cout << "Depth Timestamp: " << this->m_depthTimeStamp << std::endl;
				depthCaptured = true;
				break;
			case C_COLOR_STREAM:
				m_color.readFrame(&m_colorFrame);
				m_colorImage.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
				bufferImage.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
				bufferImage.data = (uchar*)m_colorFrame.getData();
				this->m_colorTimeStamp = m_colorFrame.getTimestamp() >> 16;
				// std::cout << "Color Timestamp: " << m_colorTimeStamp << std::endl;
				colorCaptured = true;
				cv::cvtColor(bufferImage, m_colorImage, CV_BGR2RGB);
				break;
			default:
				break;
			}

		}
	}
	else if(this->m_colorStreamStatus & !this->m_depthStreamStatus)
	{
		while(!colorCaptured)
		{
			m_status = openni::OpenNI::waitForAnyStream(m_streams, C_NUM_STREAMS, &m_currentStream, C_STREAM_TIMEOUT);
			if(m_status != openni::STATUS_OK)
			{
				std::cout << "OpenCVKinect: Unable to wait for streams. Exiting" << std::endl;
				exit(EXIT_FAILURE);
			}

			switch(m_currentStream)
			{
			case C_COLOR_STREAM:
				m_color.readFrame(&m_colorFrame);
				m_colorImage.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
				bufferImage.create(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3);
				bufferImage.data = (uchar*)m_colorFrame.getData();
				this->m_colorTimeStamp = m_colorFrame.getTimestamp() >> 16;
				// std::cout << "Color Timestamp: " << m_colorTimeStamp << std::endl;
				colorCaptured = true;
				cv::cvtColor(bufferImage, m_colorImage, CV_BGR2RGB); 
				break;
			default:
				break;
			}

		}
		m_depthImage = cv::Mat::zeros(m_colorImage.size(), CV_16UC1);
		returnVec.push_back(m_depthImage);
		returnVec.push_back(m_colorImage);
		bufferImage.release();
		return returnVec;

	}
	else if(!this->m_colorStreamStatus & this->m_depthStreamStatus)
	{
		while(!depthCaptured)
		{
			m_status = openni::OpenNI::waitForAnyStream(m_streams, C_NUM_STREAMS, &m_currentStream, C_STREAM_TIMEOUT);
			if(m_status != openni::STATUS_OK)
			{
				std::cout << "OpenCVKinect: Unable to wait for streams. Exiting" << std::endl;
				exit(EXIT_FAILURE);
			}

			switch(m_currentStream)
			{
			case C_DEPTH_STREAM:
				m_depth.readFrame(&m_depthFrame);
				m_depthImage.create(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1);
				m_depthImage.data = (uchar*)m_depthFrame.getData();
				this->m_depthTimeStamp = m_depthFrame.getTimestamp() >> 16;
				// std::cout << "Depth Timestamp: " << this->m_depthTimeStamp << std::endl;
				depthCaptured = true;
				break;
			default:
				break;
			}

		}
		m_colorImage = cv::Mat::zeros(m_depthImage.size(), CV_8UC3);
		returnVec.push_back(m_depthImage);
		returnVec.push_back(m_colorImage);
		bufferImage.release();
		return returnVec;
	}
	else
	{

	m_colorImage = cv::Mat::zeros(10, 10, CV_8UC1);
	m_depthImage = cv::Mat::zeros(10, 10, CV_8UC1);

	/*returnVec.push_back(m_depthImage);
	returnVec.push_back(m_colorImage);
	bufferImage.release();
	return returnVec;*/
	}
	returnVec.push_back(m_depthImage);
	returnVec.push_back(m_colorImage);
	bufferImage.release();
	return returnVec;
}

void PoseEstimation::registerDepthAndImage()
{
	m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

cv::Mat PoseEstimation::getColor()
{
	return m_colorImage;
}

cv::Mat PoseEstimation::getDepth()
{
	return m_depthImage;
}

PoseEstimation::~PoseEstimation(void)
{
	this->m_depthFrame.release();
	this->m_colorFrame.release();
	this->m_depth.stop();
	this->m_color.stop();
	openni::OpenNI::shutdown();
	this->m_device.close();
}


// Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
    cv::Mat M;
    cv::Mat R,T;
    R_.copyTo ( R );
    T_.copyTo ( T );
    if ( R.type() ==CV_64F ) {
        assert ( T.type() ==CV_64F );
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );

        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R64;
            R.convertTo ( R64,CV_64F );
            R.copyTo ( R33 );
        }
        for ( int i=0; i<3; i++ )
            Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
        M=Matrix;
    } else if ( R.depth() ==CV_32F ) {
        cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
        cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
        if ( R.total() ==3 ) {
            cv::Rodrigues ( R,R33 );
        } else if ( R.total() ==9 ) {
            cv::Mat R32;
            R.convertTo ( R32,CV_32F );
            R.copyTo ( R33 );
        }

        for ( int i=0; i<3; i++ )
            Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
        M=Matrix;
    }

    if ( forceType==-1 ) return M;
    else {
        cv::Mat MTyped;
        M.convertTo ( MTyped,forceType );
        return MTyped;
    }
}

// This Matrix Inverses a Transformation Matrix
Mat getT_InverseMatrix(Mat Rxyz, Mat Trans)
{
	Mat T = getRTMatrix(Rxyz,Trans,CV_32F);// getTMatrix(Rxyz, Trans);
	Mat Temp= (Mat_<float>(4,4) <<
               1,       0,              0,		0,
               0,       1,				0,		0,
               0,       0,				1,		0,
			   0,       0,				0,		1);
	Temp.at<float>(0,0)=T.at<float>(0,0);
	Temp.at<float>(0,1)=T.at<float>(1,0);
	Temp.at<float>(0,2)=T.at<float>(2,0);
	Temp.at<float>(1,0)=T.at<float>(0,1);
	Temp.at<float>(1,1)=T.at<float>(1,1);
	Temp.at<float>(1,2)=T.at<float>(2,1);
	Temp.at<float>(2,0)=T.at<float>(0,2);
	Temp.at<float>(2,1)=T.at<float>(1,2);
	Temp.at<float>(2,2)=T.at<float>(2,2);

	cout << T.at<float>(0,3) <<  " , "<< T.at<float>(0,3) << " , "<<  T.at<float>(0,3) << " , "<<  T.at<float>(0,3) <<  " , "<< T.at<float>(0,3) << " , "<<  T.at<float>(0,3) <<endl;
	Temp.at<float>(0,3)= -(T.at<float>(0,0) * T.at<float>(0,3)+T.at<float>(1,0)*T.at<float>(1,3)+T.at<float>(2,0)*T.at<float>(2,3));
	cout << Temp.at<float>(0,3) << endl;
	Temp.at<float>(1,3)= -(T.at<float>(0,1) * T.at<float>(0,3)+T.at<float>(1,1)*T.at<float>(1,3)+T.at<float>(2,1)*T.at<float>(2,3));
	Temp.at<float>(2,3)= -(T.at<float>(0,2) * T.at<float>(0,3)+T.at<float>(1,2)*T.at<float>(1,3)+T.at<float>(2,2)*T.at<float>(2,3));

	return Temp;
}


int  PoseEstimation::MarkerPose(float size)
{
	ofstream myfile;
	TheCameraParameters.readFromXMLFile("camera214.yml");   // Camera calibration file obtained from the calibration routine (case 1)
	TheMarkerSize=size;										// Physical Marker size (Do not include the white border) 
	myfile.open("pose.txt");								//File to ouput pose information
	
	cv::vector<cv::Mat> dataStream;
	PoseEstimation myDataCap;
	myDataCap.setMode(C_MODE_COLOR);
	if(!myDataCap.init()){							
		std::cout << "Error initializing" << std::endl;
		return 0; 
	}
				
	char ch = ' ';
	int flag=1, DONE=0;

	while(ch != 27  && DONE==0)								// Keep searching for the markers until ESC key is pressed or a single set is obtained
	{
		dataStream = myDataCap.getData();
		dataStream[C_COLOR_STREAM].copyTo(TheInputImage);	// Capture an Image from the video stream
		
		if (flag == 1) {
			TheCameraParameters.resize(TheInputImage.size());//read camera parameters
			flag=0;

			MDetector.getThresholdParams(ThresParam1, ThresParam2);
			MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
			
			if (ThePyrDownLevel>0)
				MDetector.pyrDown(ThePyrDownLevel);		
		}
		double tick = (double)getTickCount();				// Time for pose estimation
		MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize); //Detect markers in the image
		
		TheInputImage.copyTo(TheInputImageCopy);			// Print marker info and draw the markers in image
		for (unsigned int i=0;i<TheMarkers.size();i++) {
			cout<<TheMarkers[i]<<endl;
			TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
		}

		if (  TheCameraParameters.isValid()){				// Draw a 3d cube and coordinate axes for each marker
			for (unsigned int i=0;i<TheMarkers.size();i++) {
				// CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
				CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);					
			}
		}

		if(TheMarkers.size()>1){							// Compute Relative Transformation
			RelativeTrans=  getT_InverseMatrix(TheMarkers[1].Rvec, TheMarkers[1].Tvec) * getRTMatrix(TheMarkers[0].Rvec,TheMarkers[0].Tvec,CV_32F);
			DONE=1;
			cout<<"Relative Translation Matrix"<< endl << RelativeTrans <<endl;
		}
		cout<<endl<<endl<<endl;
		cv::imshow("in", TheInputImageCopy);
		ch = cv::waitKey(150);	
	
	}		
	for(int row=0;row<3;row++)
		myfile<<RelativeTrans.at<float>(row,0)<<" "<<RelativeTrans.at<float>(row,1)<<" "<<RelativeTrans.at<float>(row,2)<<" "<<RelativeTrans.at<float>(row,3)<<endl;
	myfile<<"0 0 0 1"<<endl;
	myfile.close();
	return 1; 
}


#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <sstream>
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int _tmain( int argc, _TCHAR* argv[] )
{
	cv::setUseOptimized( true );

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open( );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 1920
	pDescription->get_Height( &height ); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof( unsigned char );

	cv::Mat bufferMat( height, width, CV_8UC4 );
	cv::Mat bodyMat( height / 2, width / 2, CV_8UC4 );
	cv::namedWindow("Body");
	cv::Mat YPRMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("YPR");
	cv::Mat XYZMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("XYZ");

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );
	color[1] = cv::Vec3b(   0, 255,   0 );
	color[2] = cv::Vec3b(   0,   0, 255 );
	color[3] = cv::Vec3b( 255, 255,   0 );
	color[4] = cv::Vec3b( 255,   0, 255 );
	color[5] = cv::Vec3b(   0, 255, 255 );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	while( 1 ){
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
		}
		//SafeRelease( pColorFrame );

		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
		if (SUCCEEDED(hResult)){
			cv::Mat YPRbufferMat(height, width, CV_8UC4);
			cv::Mat XYZbufferMat(height, width, CV_8UC4);
			cv::resize(YPRbufferMat, YPRMat, cv::Size(), 0.5, 0.5);
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
			if( SUCCEEDED( hResult ) ){
				for( int count = 0; count < BODY_COUNT; count++ ){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked( &bTracked );
					if( SUCCEEDED( hResult ) && bTracked ){
						Joint joint[JointType::JointType_Count];
						hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );
						if( SUCCEEDED( hResult ) ){
							// Joint
							double X_90[2] = { 0, 0 };
							double Y_90[2] = { 0, 0 };
							double X_3[2] = { 0, 0 };
							double Y_3[2] = { 0, 0 };
							double Z_3[2] = { 0, 0 };
							for( int type = 0; type < JointType::JointType_Count; type++ ){
								if (type == 9 || type == 10){
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									if (type == 9){
										X_90[0] = x;
										Y_90[0] = y;
										X_3[0] = joint[type].Position.X;
										Y_3[0] = joint[type].Position.Y;
										Z_3[0] = joint[type].Position.Z;
									}
									else if (type == 10){
										X_90[1] = x;
										Y_90[1] = y;
										X_3[1] = joint[type].Position.X;
										Y_3[1] = joint[type].Position.Y;
										Z_3[1] = joint[type].Position.Z;
									}
									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)){
										cv::circle(bufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
										if (type == 10){
											ColorSpacePoint CSP = { 0 };
											CameraSpacePoint newpt;
											newpt.X = (X_3[1]);
											newpt.Y = (Y_3[1]);
											newpt.Z = (Z_3[1]);
											pCoordinateMapper->MapCameraPointToColorSpace(newpt, &CSP);
											int x1 = static_cast<int>(CSP.X);
											int y1 = static_cast<int>(CSP.Y);

											cv::circle(YPRbufferMat, cv::Point(X_90[1] - X_90[0] + width / 3, Y_90[1] - Y_90[0] + height / 3 * 2), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
											cv::circle(YPRbufferMat, cv::Point(width / 3, height / 3 * 2), 20, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
											cv::arrowedLine(YPRbufferMat, cv::Point(width / 3, height / 3 * 2), cv::Point(X_90[1] - X_90[0] + width / 3, Y_90[1] - Y_90[0] + height / 3 * 2), static_cast<cv::Scalar>(color[5 - count]), 3);
											cv::arrowedLine(YPRbufferMat, cv::Point(width / 3, height / 3 * 2), cv::Point(width, height / 3 * 2), static_cast<cv::Scalar>(color[3]), 3);
											cv::arrowedLine(YPRbufferMat, cv::Point(width / 3, height / 3 * 2), cv::Point(width / 3, 0), static_cast<cv::Scalar>(color[3]), 3);
											cv::arrowedLine(YPRbufferMat, cv::Point(width / 3, height / 3 * 2), cv::Point(0, height), static_cast<cv::Scalar>(color[3]), 3);
											double pitch = 180 * atan((X_3[1] - X_3[0]) / sqrt((Y_3[1] - Y_3[0])*(Y_3[1] - Y_3[0]) + (Z_3[1] - Z_3[0])*(Z_3[1] - Z_3[0]))) / 3.1415926;
											double roll = 180 * atan((Y_3[1] - Y_3[0]) / sqrt((X_3[1] - X_3[0])*(X_3[1] - X_3[0]) + (Z_3[1] - Z_3[0])*(Z_3[1] - Z_3[0]))) / 3.1415926;
											double yaw = 180 * atan((Z_3[1] - Z_3[0]) / sqrt((X_3[1] - X_3[0])*(X_3[1] - X_3[0]) + (Y_3[1] - Y_3[0])*(Y_3[1] - Y_3[0]))) / 3.1415926;
											std::stringstream syaw;
											syaw << "Yaw: " << yaw;
											std::string Yaw = syaw.str();
											std::stringstream sroll;
											sroll << "Roll: " << roll;
											std::string Roll = sroll.str();
											std::stringstream spitch;
											spitch << "Pitch: " << pitch;
											std::string Pitch = spitch.str();

											cv::putText(YPRbufferMat, Yaw, cv::Point(width / 2 * 1.6, height / 2 * 1.6), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);
											cv::putText(YPRbufferMat, Roll, cv::Point(width / 2 * 1.6, height / 2 * 1.75), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);
											cv::putText(YPRbufferMat, Pitch, cv::Point(width / 2 * 1.6, height / 2 * 1.9), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);



											cv::circle(XYZbufferMat, cv::Point(X_90[0]/2 + width / 2, Y_90[0]/2 + height / 2), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
											cv::circle(XYZbufferMat, cv::Point(width / 2, height / 2), 20, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
											cv::arrowedLine(XYZbufferMat, cv::Point(width / 2, height / 2), cv::Point(X_90[0]/2 + width / 2, Y_90[0]/2 + height / 2), static_cast<cv::Scalar>(color[5 - count]), 3);
											cv::arrowedLine(XYZbufferMat, cv::Point(width / 2, height / 2), cv::Point(width, height / 2), static_cast<cv::Scalar>(color[3]), 3);
											cv::arrowedLine(XYZbufferMat, cv::Point(width / 2, height / 2), cv::Point(width / 2, 0), static_cast<cv::Scalar>(color[3]), 3);
											cv::arrowedLine(XYZbufferMat, cv::Point(width / 2, height / 2), cv::Point(0, height), static_cast<cv::Scalar>(color[3]), 3);
											std::stringstream sX;
											sX << "X: " << X_3[0];
											std::string ssX = sX.str();
											std::stringstream sY;
											sY << "Y: " << Y_3[0];
											std::string ssY = sY.str();
											std::stringstream sZ;
											sZ << "Z: " << Z_3[0];
											std::string ssZ = sZ.str();

											cv::putText(XYZbufferMat, ssX, cv::Point(width / 2 * 1.6, height / 2 * 1.6), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);
											cv::putText(XYZbufferMat, ssY, cv::Point(width / 2 * 1.6, height / 2 * 1.75), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);
											cv::putText(XYZbufferMat, ssZ, cv::Point(width / 2 * 1.6, height / 2 * 1.9), cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0, 255, 0), 2.0);
										}
									}
								}
							}
						}
						// Lean
						PointF amount;
						hResult = pBody[count]->get_Lean( &amount );
						if( SUCCEEDED( hResult ) ){
							//std::cout << "amount : " << amount.X << ", " << amount.Y << std::endl;
						}
					}
				}
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
				cv::resize(YPRbufferMat, YPRMat, cv::Size(), 0.5, 0.5);
				cv::resize(XYZbufferMat, XYZMat, cv::Size(), 0.5, 0.5);
			}
			for( int count = 0; count < BODY_COUNT; count++ ){
				SafeRelease( pBody[count] );
			}
		}
		//SafeRelease( pBodyFrame );

		SafeRelease( pColorFrame );
		SafeRelease( pBodyFrame );

		cv::imshow("Body", bodyMat);
		cv::imshow("YPR", YPRMat);
		cv::imshow("XYZ", XYZMat);
		//YPRbufferMat.release();

		if( cv::waitKey( 10 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pColorSource );
	SafeRelease( pBodySource );
	SafeRelease( pColorReader );
	SafeRelease( pBodyReader );
	SafeRelease( pDescription );
	SafeRelease( pCoordinateMapper );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}


#pragma once
#include "RSCamera.h"
#include <vector>
#include "Aruco.h"
//#include <wtypes.h>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

using namespace rs2;
using std::vector;
using std::string;

std::atomic<bool> RSCamera::isRunning;

void RSCamera::Init()
{
	isRunning = false;
	Camera_DESC desc;
	desc.frameRate = 30;   // desc is completely useless. its better to replace it by predefined instances of Camera_DESC class
	desc.resW = 1280;
	desc.resH = 720;
	descs.push_back(desc);
	desc.frameRate = 30;
	desc.resW = 640;
	desc.resH = 480;
	descs.push_back(desc);
	desc.frameRate = 60;
	desc.resW = 640;
	desc.resH = 480;
	descs.push_back(desc);
	desc.frameRate = 90;
	desc.resW = 256;
	desc.resH = 144;
	descs.push_back(desc);
	desc.frameRate = 30;
	desc.resW = 320;
	desc.resH = 240;
	descs.push_back(desc);
	desc.frameRate = 30;
	desc.resW = 424;
	desc.resH = 240;
	descs.push_back(desc);
}

void RSCamera::GetRGBImage(cv::Mat& image, bool detectAruco = true)
{
	std::cout<< "Recieving RGB data..." << std::endl;
	//cv::Mat image;
	if (m_rgb_frame != NULL)
	{
		const int w = m_rgb_frame.as<video_frame>().get_width();
		const int h = m_rgb_frame.as<video_frame>().get_height();
		image = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)m_rgb_frame.get_data(), cv::Mat::AUTO_STEP);
		if (detectAruco)
        	Aruco::Detect(image, image);
	}
	//TODO: Zkusit jen kopirovat data a nevytvaret novy objekt memcpy
}

void RSCamera::GetDepthImage(Image<float>& image)
{
	std::lock_guard<std::mutex> guard(depth_lock);
	if (m_depth_frame != NULL)
		image.FillFromCamera((void*)(m_depth_frame.get_data()), m_scale);
	else
		printf("COULD NOT READ FROM CAMERA !");
}

void RSCamera::GetAlignedDepthImage(Image<float>& image)
{
	std::lock_guard<std::mutex> guard(depth_lock);
	if (m_depth_frame != NULL)
		image.FillFromCamera((void*)(alignedDepthFrame.get_data()), m_scale);
	else
		printf("COULD NOT READ FROM CAMERA !");
}

sensor_msgs::CameraInfo RSCamera::GetCameraInfo()
{
	// from https://github.com/IntelRealSense/realsense-ros/blob/f400d682beee6c216052a419f419e95b797255ad/realsense2_camera/src/base_realsense_node.cpp#L1883
	sensor_msgs::CameraInfo cameraInfo;
	cameraInfo.width = m_intrin.width;
	cameraInfo.height = m_intrin.height;
	cameraInfo.header.frame_id = "camera_frame";

	cameraInfo.K.at(0) = m_intrin.fx;
	cameraInfo.K.at(2) = m_intrin.ppx;
	cameraInfo.K.at(4) = m_intrin.fy;
	cameraInfo.K.at(5) = m_intrin.ppy;
	cameraInfo.K.at(8) = 1;

	cameraInfo.P.at(0) = cameraInfo.K.at(0);
	cameraInfo.P.at(1) = 0;
	cameraInfo.P.at(2) = cameraInfo.K.at(2);
	cameraInfo.P.at(3) = 0;
	cameraInfo.P.at(4) = 0;
	cameraInfo.P.at(5) = cameraInfo.K.at(4);
	cameraInfo.P.at(6) = cameraInfo.K.at(5);
	cameraInfo.P.at(7) = 0;
	cameraInfo.P.at(8) = 0;
	cameraInfo.P.at(9) = 0;
	cameraInfo.P.at(10) = 1;
	cameraInfo.P.at(11) = 0;

	// set R (rotation matrix) values to identity matrix
	cameraInfo.R.at(0) = 1.0;
	cameraInfo.R.at(1) = 0.0;
	cameraInfo.R.at(2) = 0.0;
	cameraInfo.R.at(3) = 0.0;
	cameraInfo.R.at(4) = 1.0;
	cameraInfo.R.at(5) = 0.0;
	cameraInfo.R.at(6) = 0.0;
	cameraInfo.R.at(7) = 0.0;
	cameraInfo.R.at(8) = 1.0;

	int coeff_size(5);
	cameraInfo.D.resize(coeff_size);
	for (int i = 0; i < coeff_size; i++)
	{
		cameraInfo.D.at(i) = m_intrin.coeffs[i];
	}

	if (m_intrin.model == RS2_DISTORTION_KANNALA_BRANDT4)
	{
		cameraInfo.distortion_model = "equidistant";
		coeff_size = 4;
	} 
	else 
	{
		cameraInfo.distortion_model = "plumb_bob";
	}

	//for (int i=0; i<5; i++)
	//	printf("%9.6f", m_intrin.coeffs[i]);
	return cameraInfo;
}

void RSCamera::Start(CAM_DESC rgb, CAM_DESC depth)
{
	rgb_desc = descs[(int)rgb];
	depth_desc = descs[(int)depth];
	vector<string> serial_numbers;
	context ctx = context();
	for (auto&& device : ctx.query_devices())
		serial_numbers.push_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

	m_connected_camera = false;
	if (serial_numbers.size() > 0)
	{
		m_connected_camera = true;
		pipe = pipeline(ctx);
		config cfg;
		cfg.enable_stream(RS2_STREAM_COLOR, -1, rgb_desc.resW, rgb_desc.resH, RS2_FORMAT_RGB8, rgb_desc.frameRate);
		cfg.enable_stream(RS2_STREAM_DEPTH, depth_desc.resW, depth_desc.resH, RS2_FORMAT_Z16, depth_desc.frameRate);
		cfg.enable_device(serial_numbers[0]);
		profile = pipe.start(cfg);
		device dev = profile.get_device();
		depth_sensor ds = dev.query_sensors().front().as<depth_sensor>();
		m_scale = ds.get_depth_scale();

		m_frame = pipe.wait_for_frames();

		framesAlignment = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

		SetDepthFrameWithLock();
		m_intrin = rs2::video_stream_profile(m_depth_frame.get_profile()).get_intrinsics();
		
		hFov =2* std::atan(m_intrin.width/( 2 * m_intrin.fx));
		vFov =2* std::atan(m_intrin.height / (2 * m_intrin.fy));
		dFov = 2 * std::atan(std::sqrt(m_intrin.height * m_intrin.height + m_intrin.width * m_intrin.width) /(m_intrin.fy + m_intrin.fx));

		auto exintrin = rs2::video_stream_profile(m_depth_frame.get_profile()).get_extrinsics_to(m_rgb_frame.get_profile());

		depth_to_rgb = Matrix(
			(float)exintrin.rotation[0], (float)exintrin.rotation[3], (float)exintrin.rotation[6], 0.0f,
			(float)exintrin.rotation[1], (float)exintrin.rotation[4], (float)exintrin.rotation[7], 0.0f,
			(float)exintrin.rotation[2], (float)exintrin.rotation[5], (float)exintrin.rotation[8], 0.0f,
						  0,			       0, 			    0, 1.0f);	
		depth_to_rgb.Transpose();
		depth_to_rgb.SetPosition({(float)exintrin.translation[0], (float)exintrin.translation[1], (float)exintrin.translation[2]});
		
	}
	else
	{
		std::cout << "Could not find any camera";
	}
	
	
	isRunning = true;
	//m_frame = pipe.wait_for_frames();
	m_thread_wait_for_frame = std::thread(Task);
}

void RSCamera::Joint()
{
	isRunning = false;
	m_thread_wait_for_frame.join();
	m_thread_rgb_processing.join();
	pipe.stop();
	//m_thread_depth_processing.join();
}

void RSCamera::ProjectDepthToPointCloud(const Image<float>& depth, const Image<bool>& mask, vector<Vec3>& pc)
{
	pc.clear();
	int resH, resW;
	depth.FillResolution(resH, resW);
	pc.reserve(resH * resW);
	for (int i = 0; i < resH; i++)
	{
		for (int j = 0; j < resW; j++)
		{
			if (mask.At(j, i))
			{
				float pixel[] = { j, i };
				Vec3 p(0, 0, 0);

				rs2_deproject_pixel_to_point(p, &m_intrin, pixel, depth.At(j, i));
				pc.push_back(p);
			}
		}
	}
}

void RSCamera::Task()
{
	while (isRunning && m_connected_camera)
	{
		m_frame = pipe.wait_for_frames();
		SetDepthFrameWithLock();
	}
}

void RSCamera::Lock(cv::Mat& outputImage)
{
	std::lock_guard<std::mutex> guard(rgb_lock);
	const int w = m_rgb_frame.as<video_frame>().get_width();
	const int h = m_rgb_frame.as<video_frame>().get_height();

	rgb_image = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)m_rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        Aruco::Detect(rgb_image, outputImage);
}

inline void RSCamera::SetDepthFrameWithLock()
{
	{
		std::lock_guard<std::mutex> guard(depth_lock);
		m_depth_frame = m_frame.get_depth_frame();
		m_frame = framesAlignment->process(m_frame);
		alignedDepthFrame = m_frame.get_depth_frame();
	}
	std::lock_guard<std::mutex> guard_rgb(rgb_lock);
	m_rgb_frame = m_frame.get_color_frame();
}

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
	desc.frameRate = 30;
	desc.resH = 720;
	desc.resW = 1280;
	descs.push_back(desc);
	desc.frameRate = 30;
	desc.resH = 480;
	desc.resW = 640;
	descs.push_back(desc);
	desc.frameRate = 60;
	desc.resH = 480;
	desc.resW = 640;
	descs.push_back(desc);
	desc.frameRate = 90;
	desc.resH = 144;
	desc.resW = 256;
	descs.push_back(desc);
	desc.frameRate = 30;
	desc.resH = 240;
	desc.resW = 320;
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
        //m_thread_rgb_processing = std::thread(RgbTask);
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
                //cv::Mat img;
                //GetRGBImage(img);
		m_frame = pipe.wait_for_frames();
		SetDepthFrameWithLock();
	}
}


void RSCamera::RgbTask()
{
	//std::cout<< "rgb task start   " << std::endl;
	while (isRunning)
	{
		//std::cout<< "rgb task  "<< m_connected_camera << std::endl;
		if (m_connected_camera and m_rgb_frame != NULL)
		{
			//std::cout<< "rgb lock  " << std::endl;
			//Lock();
			//Sleep(10);
                        std::this_thread::sleep_for(1000ms);
		}
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
	}
	std::lock_guard<std::mutex> guard_rgb(rgb_lock);
	m_rgb_frame = m_frame.get_color_frame();
}

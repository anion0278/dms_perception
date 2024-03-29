#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include "Image.h"
#include <librealsense2/rsutil.h>
#include "Math.h"
#include <vector>
#include <atomic>
#include <sensor_msgs/CameraInfo.h>

using std::vector;

struct Camera_DESC
{
	int resH = 0;
	int resW = 0;
	int frameRate = 0;
};

class RSCamera
{

public: // Why everything is static???, why the whole class is used as static?? this is very wrong
	static void stop();
	static void Init();
	static void GetColorImage(cv::Mat& image, bool detectAruco);
	static void GetDepthImage(Image<float>& image);
	static void GetAlignedDepthImage(Image<float>& image);
	static void Start(Camera_DESC cameraStreamSettings);

	static void Joint();

	static void ProjectDepthToPointCloud(const Image<float>& depth,const Image<bool>& mask, vector<Vec3>& pc);

	static Camera_DESC GetDepthDesc() { return cameraSettings; } 
	static void Lock(cv::Mat& outputImage);
	static float GetVFov() { return vFov;}

	static float GetHFov() { return hFov;}
	static Matrix GetTFDepthToRgb() {return depth_to_rgb;} 

	static float GetScale() {return m_scale;}

	static sensor_msgs::CameraInfo GetCameraInfo();

private:
	static void Task();
	//static void Lock();

	//static void DepthTask();
	inline static float hFov = 60.f;
	inline static float vFov = 60.f;
	inline static float dFov = 60.f;

	inline static Matrix depth_to_rgb;


	inline static void SetDepthFrameWithLock();
	static std::atomic<bool> isRunning;
	inline static bool m_connected_camera = false;
	inline static rs2::pipeline_profile profile;
	inline static rs2::pipeline pipe;
	inline static rs2::frameset m_frame;
	inline static std::shared_ptr<rs2::align> framesAlignment;
	inline static std::shared_ptr<rs2::hole_filling_filter> hf_filter;
	inline static rs2::frame m_depth_frame; // Why snake_case?
	inline static rs2::frame alignedDepthFrame;
	inline static rs2::frame m_rgb_frame;
	inline static cv::Mat rgb_image;
	inline static float m_scale = 0;

	inline static std::thread m_thread_wait_for_frame;
	inline static std::thread m_thread_rgb_processing;
	inline static std::thread m_thread_depth_processing;
	inline static std::mutex rgb_lock;
	inline static std::mutex depth_lock;

	inline static rs2_intrinsics m_intrin;
	inline static rs2_intrinsics colorIntrinsics;

	inline static Camera_DESC cameraSettings;

	inline static vector<Camera_DESC> descs;
};


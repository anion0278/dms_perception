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

using std::vector;

struct Camera_DESC
{
	int resH = 0;
	int resW = 0;
	int frameRate = 0;
};

enum CAM_DESC
{
	RS_30_1280_720,
	RS_30_640_480,
	RS_60_640_480,
	RS_90_256_144,
	RS_30_320_240,
};


class RSCamera
{

public:
	static void Init();
	static void GetRGBImage(cv::Mat& image, bool detectAruco);
	static void GetDepthImage(Image<float>& image);
	static void Start(CAM_DESC rgb, CAM_DESC depth);

	static void Joint();

	static void ProjectDepthToPointCloud(const Image<float>& depth,const Image<bool>& mask, vector<Vec3>& pc);

	static Camera_DESC GetDepthDesc() { return depth_desc; }
	static Camera_DESC GetRGBDesc() { return rgb_desc; }
	static void Lock(cv::Mat& outputImage);
	static float GetVFov() { return vFov;}

	static float GetHFov() { return hFov;}
	static Matrix GetTFDepthToRgb() {return depth_to_rgb;}


private:
	static void Task();
	static void RgbTask();
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
	inline static rs2::frame m_depth_frame;
	inline static rs2::frame m_rgb_frame;
	inline static cv::Mat rgb_image;
	inline static float m_scale = 0;

	inline static std::thread m_thread_wait_for_frame;
	inline static std::thread m_thread_rgb_processing;
	inline static std::thread m_thread_depth_processing;
	inline static std::mutex rgb_lock;
	inline static std::mutex depth_lock;

	inline static rs2_intrinsics m_intrin;

	inline static Camera_DESC depth_desc;
	inline static Camera_DESC rgb_desc;

	inline static vector<Camera_DESC> descs;
};


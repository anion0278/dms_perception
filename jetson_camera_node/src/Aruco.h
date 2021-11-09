#pragma once

#include <opencv2/aruco.hpp>
#include <unordered_map>
//#include "RoboMathGL.h"
#include "Math.h"

using std::unordered_map;
//using namespace RoboMathGL;

class Aruco
{
public:
	static void Init();
	static void Detect(const cv::Mat& inputImage, cv::Mat& outputImage);
	static Matrix GetViewMatrix() { std::lock_guard<std::mutex> guard(lock); return view; };
	static void ManualCalibMatrix(const Matrix& mat){manualCalib = mat;};
	static Matrix GetManualMatrix () {return manualCalib;}
	static void BaseToGridboard(const Matrix& mat){baseToGridboard = mat;};
	static Matrix GetBaseToGridboard(){ return baseToGridboard;};
private:
	inline static cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
	inline static cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 902.92930085f, 0, 634.76689785f, 0, 902.32670633f, 354.34797514f, 0, 0, 1);
	inline static cv::Mat distCoeffs = (cv::Mat1d(1, 5) << 0.16792601f, -0.49010724f, -0.00327827f, -0.00113529f, 0.41018407f);
	inline static auto dparam = cv::aruco::DetectorParameters::create();
	inline static float markerLength = 0.07f;
	inline static unordered_map<int,Matrix> gb_def;
	inline static Matrix view = Matrix(-0.991634, -0.116853, -0.0548485, 0, -0.0145268, 0.523223, -0.852072, 0, 0.128265, -0.844146, -0.520543, 0, -0.10944, 0.359958, 1.84452, 1);
	inline static Matrix savedMat = Matrix(-0.991634, -0.116853, -0.0548485, 0, -0.0145268, 0.523223, -0.852072, 0, 0.128265, -0.844146, -0.520543, 0, -0.10944, 0.359958, 1.84452, 1);
	inline static Matrix manualCalib = Matrix(1.0);
	inline static Matrix baseToGridboard = Matrix(-0.f, -0.3f, -0.065f);
	inline static std::mutex lock;
};

#pragma once
#include "Aruco.h"
#include <vector>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "Translator.h"
#include "Filter.h"
#include "Math.h"
#include "RSCamera.h"

void PrintMatrix(Matrix m, int id)
{
	//for (int i = 0; i < 4; i++) {
	//	for (int j = 0; j < 4; j++) {

	//	}
	//}

	std::cout << id << ":  " << std::endl <<
		m.m[0][0] << "," << m.m[0][1] << "," << m.m[0][2] << "," << m.m[0][3] << "," <<std::endl<<
		m.m[1][0] << "," << m.m[1][1] << "," << m.m[1][2] << "," << m.m[1][3] << "," <<std::endl<<
		m.m[2][0] << "," << m.m[2][1] << "," << m.m[2][2] << "," << m.m[2][3] << "," <<std::endl<<
		m.m[3][0] << "," << m.m[3][1] << "," << m.m[3][2] << "," << m.m[3][3] << "," << std::endl;

	std::cout << std::endl;

	std::cout << std::endl;
}

void Aruco::Init(std::string pathToMatrixCalibration)
{
	//dparam->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG;
	//dparam->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIXEL;
	//gb_def.resize(1000, Matrix::Identity);
	gb_def[10] = Matrix::Identity;
	gb_def[0] = Matrix::Identity;

	Matrix m11 = Matrix::BuildTranslation(-0.05, 0, 0);
	Matrix::Multiply(m11, m11, Matrix::BuildRotationY(-15_deg));
	Matrix::Multiply(m11, m11, Matrix::BuildTranslation(-0.05, 0, 0));
	m11.Invert();
	m11.Transpose();
	gb_def[11] = m11;
	gb_def[1] = m11;


	Matrix m12 = Matrix::BuildRotationZ(90_deg);
	Matrix::Multiply(m12, m12, Matrix::BuildTranslation(-0.05, 0, 0));
	Matrix::Multiply(m12, m12, Matrix::BuildRotationY(-15_deg));
	Matrix::Multiply(m12, m12, Matrix::BuildTranslation(-0.05, 0, 0));
	m12.Transpose();
	gb_def[12] = m12;
	gb_def[2] = m12;

	Matrix m13 = Matrix::BuildRotationZ(180_deg);
	Matrix::Multiply(m13, m13, Matrix::BuildTranslation(0.05, 0, 0));
	Matrix::Multiply(m13, m13, Matrix::BuildRotationY(15_deg));
	Matrix::Multiply(m13, m13, Matrix::BuildTranslation(0.05, 0, 0));
	m13.Transpose();
	gb_def[13] = m13;
	gb_def[3] = m13;

	Matrix m14 = Matrix::BuildRotationZ(-90_deg);
	Matrix::Multiply(m14, m14, Matrix::BuildTranslation(-0.05, 0, 0));
	Matrix::Multiply(m14, m14, Matrix::BuildRotationY(-15_deg));
	Matrix::Multiply(m14, m14, Matrix::BuildTranslation(-0.05, 0, 0));
	m14.Transpose();
	gb_def[14] = m14;
	gb_def[4] = m14;


	auto flag = savedMat.LoadFromFile(pathToMatrixCalibration);

	if(flag)
	{
		std::lock_guard<std::mutex> guard(lock);
		view = savedMat;
	}

}

void Aruco::Detect(const cv::Mat& inputImage, cv::Mat& outputImage)
{
	//std::cout<< "ARUCO DETECTION" << std::endl;
	//cv::Mat outputImage;
	inputImage.copyTo(outputImage);
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

	cv::Mat gray;
	cv::cvtColor(inputImage, gray, cv::COLOR_RGB2GRAY);

	cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, dparam, rejectedCandidates);
	if (markerIds.size() > 0)
		cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	if (markerIds.size() == 0)
	{
		std::cout<< "NO ARUCO DETECTED" << std::endl;
		std::lock_guard<std::mutex> guard(lock);
		view = savedMat;
		return;
	}
	std::vector<cv::Vec3d> rvecs, tvecs;
	cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);
	cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
	vector<Filter::SS> vec_ss;

	vec_ss.reserve(tvecs.size());
	for (int i = 0; i < rvecs.size(); ++i)
	{
		//if(markerIds[i] == 14)
		{
			const auto& rvec = rvecs[i];
			const auto& tvec = tvecs[i];
			//cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.02);
			cv::Rodrigues(rvec, rot_mat);
			Matrix rot = Translator::MatrixFromMat(rot_mat);

			rot._14 = (float)tvec[0];
			rot._24 = (float)tvec[1];
			rot._34 = (float)tvec[2];

			if (auto f = gb_def.find(markerIds[i]); f != gb_def.end())
				Matrix::Multiply(rot, rot, f->second);
			else
				Matrix::Multiply(rot, rot, Matrix::Identity);

			vec_ss.push_back({ {rot._14,rot._24,rot._34},Translator::EulerFromMatrix(rot), 0 });
		}
	}

	for (int i = 0; i < vec_ss.size(); i++)
	{
		for (int j = 0; j < vec_ss.size(); j++)
		{
			if (i != j)
			{
				vec_ss[i].dist += Filter::Distance(vec_ss[i].pos, vec_ss[j].pos);
			}
		}
	}
	Filter::Sort(vec_ss);
	Filter::CheckDir(vec_ss);
	Filter::SS estimated = Filter::Estimate(vec_ss);
	Matrix rotMat = Translator::MatrixFromEuler(estimated.ori);
	cv::Mat rMat = Translator::MatFromMatrix(rotMat);
	cv::Vec3d rvec;
	cv::Rodrigues(rMat, rvec);
	cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, Translator::Vec3dFromVec3(estimated.pos), 0.05);

	rotMat.Transpose();
	//rotMat.Invert();

	rotMat.SetPosition(estimated.pos);


	/*Matrix camRGBToBase;

	Matrix::Multiply(camRGBToBase, baseToGridboard, rotMat);
	Matrix rgbToDepthCalibration = RSCamera::GetTFDepthToRgb(); //(0.04, -0.0, 0.0042);//0.0042
	//rgbToDepthCalibration.Invert();

	Matrix camDepthToBase;
	//camDepthToBase = camRGBToBase;
	Matrix::Multiply(camDepthToBase, camRGBToBase, rgbToDepthCalibration);
	Matrix::Multiply(camDepthToBase, camDepthToBase, manualCalib);
	
	camDepthToBase.SaveToFile("/home/jetson/Desktop/DMS_01/camPose.bmat");
	
	std::lock_guard<std::mutex> guard(lock);
	view = camDepthToBase;
	//PrintMatrix(view,0);
	//PrintMatrix(rgbToDepthCalibration,-1000);
	*/
	std::lock_guard<std::mutex> guard(lock);
	view = rotMat;
}

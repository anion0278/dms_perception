#pragma once
#include <opencv2/opencv.hpp>
//#include "RoboMathGL.h"
#include "Math.h"
//using namespace RoboMathGL;

class Translator {

public:

	static Matrix MatrixFromMat(const cv::Mat& mat) {
		Matrix rot;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				rot.m[i][j] = (float)mat.at<double>(i, j);
		}
		return rot;
	}

	static cv::Mat MatFromMatrix(const Matrix& matrix) {
		cv::Mat mat = (cv::Mat1d(3, 3) << matrix._11, matrix._12, matrix._13, matrix._21, matrix._22, matrix._23, matrix._31, matrix._32, matrix._33);
		return mat;
	}

	static Vec3 EulerFromMatrix(const Matrix& mat) {
		Vec3 euler;
		euler.x = atan2(mat._23, mat._33);
		euler.y = atan2(-mat._13, sqrt(mat._11 * mat._11 + mat._12 * mat._12));
		euler.z = atan2(mat._12, mat._11);
		return euler;
	}

	static Matrix MatrixFromEuler(const Vec3& euler) {
		Matrix res;
		Matrix rX = Matrix::BuildRotationX(euler.x);
		Matrix rY = Matrix::BuildRotationY(euler.y);
		Matrix rZ = Matrix::BuildRotationZ(euler.z);

		Matrix::Multiply(rY, rY, rZ);
		Matrix::Multiply(res, rX, rY);

		return res;
	}

	static Vec3 Vec3FromVec3d(const cv::Vec3d& v) {
		return { (float)v[0], (float)v[1], (float)v[2] };
	}

	static cv::Vec3d Vec3dFromVec3(const Vec3& v) {
		return { v.x, v.y, v.z };
	}

};
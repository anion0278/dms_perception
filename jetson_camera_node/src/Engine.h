#pragma once
#include "Object.h"
#include <opencv2/opencv.hpp>
#include "Math.h"
#include "Image.h"

class Engine
{

public:
	static void TransformMesh(Mesh& mesh, const  Matrix& transformationMatrix);
	static void TransformMeshPerspectiveDivide(Mesh& mesh, const  Matrix& viewProjMatrix);
	static void BackFaceCull(Mesh& mesh);
	static void ViewportScale(Mesh& mesh, int resH, int resW);

	static void BuildDepthMap(const Mesh& mesh, int resH, int resW, Image<float>& image);

private:
	static void WritePixel(int x, int y, Image<float>& image, float value);
	static void HorniHrana(const vector< Vec4>& tri, int resH, int resW, Image<float>& image);
	static void SpodniHrana(const vector< Vec4>& tri, int resH, int resW, Image<float>& image);
	static void SortTriangle(vector< Vec4>& tri);
};


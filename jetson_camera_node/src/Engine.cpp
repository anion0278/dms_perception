#include "Engine.h"
#include "Math.h"
#include "Aruco.h"
bool AtBackViw(Vec3 point)
{

	if (point.z < 0.01f)
		return true;
	return false;
}
void Engine::TransformMesh(Mesh& mesh, const Matrix& transformationMatrix)
{
	Matrix camMat = Aruco::GetViewMatrix();
	for (auto& triangle : mesh.triangles)
	{
		triangle.points[0].Transform(transformationMatrix);
		triangle.points[1].Transform(transformationMatrix);
		triangle.points[2].Transform(transformationMatrix);
		if (AtBackViw(triangle.points[0]) || AtBackViw(triangle.points[1]) || AtBackViw(triangle.points[2]))
			triangle.visible = false;
	}
}

void Engine::TransformMeshPerspectiveDivide(Mesh& mesh, const Matrix& viewProjMatrix)
{
	for (auto& triangle : mesh.triangles) {

		for (auto& point : triangle.points) {

			point.Transform(viewProjMatrix);
			if (point.w != 0) {
				point.x /= point.w;
				point.y /= point.w;
				//point.z /= point.w;
				//TODO: prehodit na nasobeni
			}
		}
	}
}

void Engine::BackFaceCull(Mesh& mesh)
{
	for (auto& triangle : mesh.triangles) {
		float l1x = triangle.points[1].x - triangle.points[0].x;
		float l1y = triangle.points[1].y - triangle.points[0].y;

		float l2x = triangle.points[2].x - triangle.points[0].x;
		float l2y = triangle.points[2].y - triangle.points[0].y;

		float n = l1x * l2y - l1y * l2x;
		if (n > 0)
			triangle.visible = false;
	}
}

void Engine::ViewportScale(Mesh& mesh,int resH, int resW)
{
	int wpul = resW / 2;
	int hpul = resH / 2;
	for (auto& triangle : mesh.triangles) {
		if (triangle.visible) {
			triangle.points[0].x = int(triangle.points[0].x * wpul) + wpul;
			triangle.points[0].y = int(triangle.points[0].y * hpul) + hpul;

			triangle.points[1].x = int(triangle.points[1].x * wpul) + wpul;
			triangle.points[1].y = int(triangle.points[1].y * hpul) + hpul;

			triangle.points[2].x = int(triangle.points[2].x * wpul) + wpul;
			triangle.points[2].y = int(triangle.points[2].y * hpul) + hpul;
		}
	}
}

void Engine::BuildDepthMap(const Mesh& mesh, int resH, int resW, Image<float>& image)
{
	for (auto& triangle : mesh.triangles) {
		if (!triangle.visible)
			continue;

		vector< Vec4> tri;
		tri.reserve(3);

		tri.push_back(triangle.points[0]);
		tri.push_back(triangle.points[1]);
		tri.push_back(triangle.points[2]);

		SortTriangle(tri);
		if (tri[0].y == tri[1].y) // P1
		{
			HorniHrana(tri, resH, resW, image);
		}
		else { //P2

			if (tri[1].y == tri[2].y) // Neni treba delit trojuhelnik
			{
				SpodniHrana(tri, resH, resW, image);
			}
			else
			{
				float aZ = tri[0].z;
				float cZ = tri[2].z;
				float dY = tri[2].y - tri[0].y;

				float dZ = (aZ - cZ) / dY;
				int dX = (int)(((tri[0].x - tri[2].x) / dY) * (tri[1].y - tri[0].y));

				Vec4 newPoint(tri[0].x - dX, tri[1].y, tri[0].z - (dZ * (tri[1].y - tri[0].y)), 1);

				vector< Vec4> triUP;
				triUP.reserve(3);
				triUP.push_back(tri[0]);
				triUP.push_back(tri[1]);
				triUP.push_back(newPoint);
				SortTriangle(triUP);
				SpodniHrana(triUP, resH, resW, image);

				vector< Vec4> triBottom;
				triBottom.reserve(3);
				triBottom.push_back(tri[1]);
				triBottom.push_back(newPoint);
				triBottom.push_back(tri[2]);
				SortTriangle(triBottom);
				HorniHrana(triBottom, resH, resW, image);
			}
		}
	}
}


void Engine::WritePixel(int x, int y, Image<float>& image, float value)
{
	//int actualDepth = image.at<uchar>(cv::Point(x, y));
	//if (uchar d = (uchar)(value * 255); actualDepth > d)
	//	image.at<uchar>(cv::Point(x, y)) = d;
}

void Engine::HorniHrana(const vector< Vec4>& tri, int resH, int resW, Image<float>& image)
{
	float dy = (float)(tri[2].y - tri[0].y);

	float dxAC = (tri[0].x - tri[2].x) / dy; // (tri[2].y - tri[0].y);
	float dxBC = (tri[1].x - tri[2].x) / dy; //(tri[2].y - tri[0].y);
	float dzAC = (tri[0].z - tri[2].z) / dy; //(tri[2].y - tri[0].y);
	float dzBC = (tri[1].z - tri[2].z) / dy; //(tri[2].y - tri[0].y);
	int y = tri[0].y;
	int xA = tri[0].x;
	int xB = tri[1].x;

	for (int i = tri[0].y; i < tri[2].y; i++)
	{
		if (i >= resH or i < 0)
			continue;
		int startLine = (int)(xA - dxAC * (i - tri[0].y));
		int endLine = (int)(xB - dxBC * (i - tri[0].y));

		float dZStart = tri[0].z - ((i - tri[0].y) * dzAC);
		float dZEnd = tri[1].z - ((i - tri[1].y) * dzBC);
		float dz_over_line = 0;
		if (endLine - startLine == 0)
			dz_over_line = -(dZStart - dZEnd);
		else
			dz_over_line = -(dZStart - dZEnd) / (endLine - startLine);

		for (int j = startLine; j <= endLine; j++)
		{
			if (j >= resW or j < 0)
				continue;
			float value = dZStart + dz_over_line * (j - startLine);
			//if (value > 1)
			//	value = 1;
			if (value < 0)
				continue;
			image.StoreSmaller(value, j, i);
			//WritePixel((int)j, (int)i, image, value);
		}
	}
}

void Engine::SpodniHrana(const vector< Vec4>& tri, int resH, int resW, Image<float>& image)
{
	float dLine = tri[2].y - tri[0].y;
	float dzAB = (tri[0].z - tri[1].z) / dLine;
	float dzAC = (tri[0].z - tri[2].z) / dLine;

	float dxAB = (tri[0].x - tri[1].x) / dLine;
	float dxAC = (tri[0].x - tri[2].x) / dLine;

	for (int line = tri[0].y; line <= tri[2].y; line++) {

		if (line >= resH or line < 0)
			continue;

		int startLine = (int)(tri[0].x - (line - tri[0].y) * dxAB);
		int endLine = (int)(tri[0].x - (line - tri[0].y) * dxAC);


		float dZStart = tri[0].z - ((line - tri[0].y) * dzAB);
		float dZEnd = tri[0].z - ((line - tri[0].y) * dzAC);


		float dz_over_line = 0;
		if (startLine - endLine == 0)
			dz_over_line = -(dZStart - dZEnd);
		else
			dz_over_line = -(dZStart - dZEnd) / (startLine - endLine);

		for (int j = startLine; j <= endLine; j++)
		{
			if (j >= resW or j < 0)
				continue;
			float value = dZStart - dz_over_line * (j - startLine);
			//if (value > 1)
			//	value = 1;
			if (value < 0)
				continue;
			//WritePixel((int)j, (int)line, image, value);
			image.StoreSmaller(value, j, line);
		}
	}
}

void Engine::SortTriangle(vector< Vec4>& tri)
{
	std::sort(tri.begin(), tri.end(), [](const  Vec4& p1, const  Vec4& p2)
		{
			if (p1.y == p2.y)
				return p1.x < p2.x;
			return p1.y < p2.y;
		});
}

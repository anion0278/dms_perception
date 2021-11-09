#include "Object.h"
#include <iostream>
#include <fstream>
#include <algorithm>
//#include "RoboMathGL.h"
#include "Math.h"
//#include <filesystem>
#include <experimental/filesystem>


void Object::Load(const string& path, float scale)
{
	//std::wstring full_path = std::filesystem::absolute(path);
	std::ifstream file(path, std::ios::in | std::ios::binary);

	char header[80];
	file.read(header, 80);

	int numFaces = 0;
	file.read((char*)&numFaces, 4);
	mesh.triangles.reserve(numFaces);


	int i = 0;
	for (i = 0; i < numFaces && !file.eof(); i++) {

		Vec4 p1; Vec4 p2; Vec4 p3;

		file.ignore(12);

		file.read((char*)&(p1), 12);
		file.read((char*)&(p2), 12);
		file.read((char*)&(p3), 12);

		p1 *= scale;
		p2 *= scale;
		p3 *= scale;
		//p1.Scale(scale);
		//p2.Scale(scale);
		//p3.Scale(scale);

		p1.w = 1.0f; p2.w = 1.0f;	p3.w = 1.0f;

		mesh.triangles.push_back(Triangle(p1, p2, p3));

		file.ignore(2);
	}
	file.close();
}

Triangle::Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
{
	points[0] = Vec4(a.x, a.y, a.z, 1);
	points[1] = Vec4(b.x, b.y, b.z, 1);
	points[2] = Vec4(c.x, c.y, c.z, 1);
}

Triangle::Triangle(const Vec4& a, const Vec4& b, const Vec4& c)
{
	//points[0] = a;
	//points[1] = b;
	//points[2] = c;

	points[0] = Vec4(a.x, a.y, a.z, a.w);
	points[1] = Vec4(b.x, b.y, b.z, b.w);
	points[2] = Vec4(c.x, c.y, c.z, c.w);
}

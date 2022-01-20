#pragma once
#include <vector>
#include <string>
#include "Math.h"
//#include "RoboMathGL.h"

//using namespace RoboMathGL;
using std::vector;
using std::string;
//using namespace RoboMathSpec;



struct Triangle {
	Vec4 points[3];
	bool visible = true;
	Triangle(const Vec3& a, const Vec3& b, const Vec3& c);
	Triangle(const Vec4& a, const Vec4& b, const Vec4& c);
};
struct Mesh {
	vector<Triangle> triangles;
};


class Object
{
public:
	void Load(const string& path, float scale);
	const Mesh& GetMesh() const { return mesh; }

private:
	Mesh mesh;
};


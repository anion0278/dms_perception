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

	inline static const string path_cube = "D:\\Dokumenty\\SW\\src\\camera\\GraphicsPkg\\STLDatabase\\test\\cube_1m.stl";
	inline static const string path_smart = "D:\\Dokumenty\\SW\\src\\camera\\GraphicsPkg\\STLDatabase\\test\\Smart.STL";
	inline static const string path_ws = "/home/jetson/Desktop/DMS_01/ur10/workspace2.STL";
	inline static const string path_ws_ur3 = "/home/k354jn1/catkin_ws/src/dms_perception/jetson_camera_node/src/ur10/ws2.STL";

private:
	Mesh mesh;
};


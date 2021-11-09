#include "Robot.h"
#include <iostream>
#define pi 3.141592654

void Robot::Init()
{
	Object j0;
	Object j1;
	Object j2;
	Object j3;
	Object j4;
	Object j5;
	Object j6;
        string source = "/home/jetson/Desktop/DMS_01/ur3_poly/";// "D:\\Dokumenty\\SW\\NAS4\\cameraPkg\\src\\camera\\GraphicsPkg\\STLDatabase\\ur10\\";
	//string source = "/home/jetson/Desktop/DMS_01/ur10_low/";
/*
        j0.Load(source + "ur10_joint0.stl",0.001);
	joints.push_back(j0);

        j1.Load(source + "ur10_joint1.stl", 0.001);
	joints.push_back(j1);

        j2.Load(source + "ur10_joint2.stl", 0.001);
	joints.push_back(j2);

        j3.Load(source + "ur10_joint3.stl", 0.001);
	joints.push_back(j3);

        j4.Load(source + "ur10_joint4.stl", 0.001);
	joints.push_back(j4);

        j5.Load(source + "ur10_joint5.stl", 0.001);
	joints.push_back(j5);

        j6.Load(source + "ur10_joint6.stl", 0.001);
	joints.push_back(j6);

	//dh.push_back({ 0,0,0,0 });
	//dh.push_back({ 0.128,0,0,-pi / 2.f });
	//dh.push_back({ 0,0.612f,-pi / 2,0 });
	//dh.push_back({ 0,-0.572f,pi / 2,0 });
	//dh.push_back({ 0.164f,0,-pi / 2,pi / 2 });
	//dh.push_back({ 0.116f,0,0,pi / 2 });
	//dh.push_back({ 0.092f, 0, 0, 0});


	dh.push_back({ 0,			0,			-pi/2,				0 });
	dh.push_back({ 0.128,		0,			-pi/2,				-pi / 2.f });
	dh.push_back({ 0,			0.612f,		0,				0 });
	dh.push_back({ 0,			-0.572f,	pi,				0 });
	dh.push_back({ 0.164f,		0,			0,				pi / 2 });
	dh.push_back({ 0.116f,		0,			pi,				pi / 2 });
	dh.push_back({ 0.092f,		0,			0,				0 });
*/
        j0.Load(source + "joint_0.stl",0.001);

	std::cout<< j0.GetMesh().triangles.size() << std::endl;
	joints.push_back(j0);

        j1.Load(source + "joint_1.stl", 0.001);
	joints.push_back(j1);

        j2.Load(source + "joint_2.stl", 0.001);
	joints.push_back(j2);

        j3.Load(source + "joint_3.stl", 0.001);
	joints.push_back(j3);

        j4.Load(source + "joint_4.stl", 0.001);
	joints.push_back(j4);

        j5.Load(source + "joint_5.stl", 0.001);
	joints.push_back(j5);

        j6.Load(source + "joint_6.stl", 0.001);
	joints.push_back(j6);

	//dh.push_back({ 0,0,0,0 });
	//dh.push_back({ 0.128,0,0,-pi / 2.f });
	//dh.push_back({ 0,0.612f,-pi / 2,0 });
	//dh.push_back({ 0,-0.572f,pi / 2,0 });
	//dh.push_back({ 0.164f,0,-pi / 2,pi / 2 });
	//dh.push_back({ 0.116f,0,0,pi / 2 });
	//dh.push_back({ 0.092f, 0, 0, 0});

	/*DHj0   0   0   0   0
	DHj1   0.150   0   0   -pi/2
	DHj2   0   0.244   pi/2   0
	DHj3   0   -0.213   pi/2   0
	DHj4   0.11   0   -pi/2   pi/2
	DHj5   0.083   0   0   pi/2
	DHj6   0.082   0   0   0*/

/*
	dh.push_back({ 0,			0,		-pi/2,				0 });
	dh.push_back({ 0.15,			0,		0,				-pi / 2.f });
	dh.push_back({ 0,			0.244f,		pi/2.f,				0 });
	dh.push_back({ 0,			-0.213f,	pi/2.f,				0 });
	dh.push_back({ 0.11f,		0,			-pi/2.f,			pi / 2 });
	dh.push_back({ 0.083f,		0,			0,				pi / 2 });
	dh.push_back({ 0.082f,		0,			0,				0 });
*/
	dh.push_back({ 0,		0,		-pi/2,				0 });
	dh.push_back({ 0.15185,		0,		0,				-pi / 2.f });
	dh.push_back({ 0,		0.24355f,		0,				0 });
	dh.push_back({ 0,		-0.2132f,	pi,				0 });
	dh.push_back({ 0.13105f,		0,			0,			pi / 2 });
	dh.push_back({ 0.08535f,		0,			pi,				pi / 2 });
	dh.push_back({ 0.0921f,		0,			0,				0 });
}

void Robot::BuilMat(Matrix& source, float d, float a, float fi, float alfa)
{
	source.m[0][0] = cos(fi);
	source.m[1][0] = sin(fi);
	source.m[2][0] = 0;
	source.m[3][0] = 0;

	source.m[0][1] = -sin(fi) * cos(alfa);
	source.m[1][1] = cos(fi) * cos(alfa);
	source.m[2][1] = sin(alfa);
	source.m[3][1] = 0;

	source.m[0][2] = sin(fi) * sin(alfa);
	source.m[1][2] = -cos(fi) * sin(alfa);
	source.m[2][2] = cos(alfa);
	source.m[3][2] = 0;


	source.m[0][3] = a*cos(fi);
	source.m[1][3] = a*sin(fi);
	source.m[2][3] = d;
	source.m[3][3] = 1;

}

void Robot::BuilMat(Matrix& source, DHParam dh,float actual)
{
	source.m[0][0] = cos(dh.fi + actual);
	source.m[1][0] = sin(dh.fi + actual);
	source.m[2][0] = 0;
	source.m[3][0] = 0;

	source.m[0][1] = -sin(dh.fi + actual) * cos(dh.alfa);
	source.m[1][1] = cos(dh.fi + actual) * cos(dh.alfa);
	source.m[2][1] = sin(dh.alfa);
	source.m[3][1] = 0;

	source.m[0][2] = sin(dh.fi + actual) * sin(dh.alfa);
	source.m[1][2] = -cos(dh.fi + actual) * sin(dh.alfa);
	source.m[2][2] = cos(dh.alfa);
	source.m[3][2] = 0;


	source.m[0][3] = dh.a * cos(dh.fi + actual);
	source.m[1][3] = dh.a * sin(dh.fi + actual);
	source.m[2][3] = dh.d;
	source.m[3][3] = 1;

}

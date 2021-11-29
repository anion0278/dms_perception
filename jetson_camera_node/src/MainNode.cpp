
#if _DEBUG
#pragma comment(lib, "opencv_world452d.lib")
#pragma comment(lib, "opencv_aruco452d.lib")
#else
#pragma comment(lib, "opencv_world452.lib")
#pragma comment(lib, "opencv_aruco452.lib")
#endif
#pragma comment(lib, "realsense2.lib")

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "RSCamera.h"
#include "Aruco.h"
#include "Object.h"
#include "Engine.h"
#include "Camera.h"
#include "Clock.h"
#include "Math.h"
#include "Robot.h"
#include <array>
#include "Image.h"
#include "UdpServer.h"
#include "OctoMap.h"
#include <unordered_set>
#include "ros/ros.h"
#include "jetson_camera_node/pcSubscribe.h"
#include "jetson_camera_node/PointCloud.h"
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/String.h"
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include "std_msgs/Int32.h"
#include <boost/array.hpp>

#include "jetson_camera_node/CameraData.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

constexpr float pi = 3.141592654;
float voxelSize = 2.f;
float percentageVoxelOccupancy = 1.0f;
int pointPerVoxel = 50;

using namespace cv;
using std::unordered_set;
std::string id = "PetannnJetson";
std::unordered_map<uint32_t, float> voxelTimeStamp;

float deltaTimeFilter = 0.0f;

bool SaveToFile(const std::string& name, float value) // again, why all the methods are "static" ? why there is no encapsulating object
{
	std::ofstream file(name, std::ios::binary);
	if(file.is_open())
	{
		//file.write((const char*)value, sizeof(float));
		file.write(reinterpret_cast<const char*>(&value),sizeof(float));
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}


bool LoadFromFile(const std::string& name, float& value)
{
	std::ifstream file(name, std::ios::binary);
	if(file.is_open())
	{
		//file.read((char*)value, sizeof(float));
		file.read(reinterpret_cast<char*>(&value), sizeof(float));	
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

void GetIP()
{
    char ip_address[15];
    int fd;
    struct ifreq ifr;
     
    /*AF_INET - to define network interface IPv4*/
    /*Creating soket for it.*/
    fd = socket(AF_INET, SOCK_DGRAM, 0);
     
    /*AF_INET - to define IPv4 Address type.*/
    ifr.ifr_addr.sa_family = AF_INET;
     
    /*eth0 - define the ifr_name - port name
    where network attached.*/
    memcpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);
     
    /*Accessing network interface information by
    passing address using ioctl.*/
    ioctl(fd, SIOCGIFADDR, &ifr);
    /*closing fd*/
    close(fd);
     
    /*Extract IP Address*/

    strcpy(ip_address,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
  

    std::string ip = ip_address;
    std::replace(ip.begin(),ip.end(),'.','_');
    printf("System IP Address is: %s\n",ip_address);
    id = ip;

}
void NewPointPerVoxel(const std_msgs::Int32::ConstPtr& msg)
{
	pointPerVoxel = (int)msg->data;
	std::cout << "new pointPerVoxel " << pointPerVoxel << std::endl;
}

bool manual_calibration = false;
bool initCalib = false;
std::string calibWinName = "calib";
int maxValue = 1000;
int maxValUhel = 360;

void on_trackbar(int state, void* userdata)
{
	//std::cout <<"sstate "<< state << std::endl;
	float derivator = getTrackbarPos("scale",calibWinName);
	float x = getTrackbarPos("x",calibWinName)/derivator ;
	float y = getTrackbarPos("y",calibWinName)/derivator;
	float z = getTrackbarPos("z",calibWinName)/derivator;
	Matrix mat(x,y,z);
	Aruco::ManualCalibMatrix(mat);
	std::cout <<"x "<< x <<"   y "<< y <<"   z "<< z << std::endl;

	Matrix resMat;
	Matrix rx = Matrix::BuildRotationX(getTrackbarPos("alfa_base",calibWinName)*pi/180.0);
	Matrix ry = Matrix::BuildRotationY(getTrackbarPos("beta_base",calibWinName)*pi/180.0);
	Matrix rz = Matrix::BuildRotationZ(getTrackbarPos("gama_base",calibWinName)*pi/180.0);

	Matrix::Multiply(ry,ry,rz);
	Matrix::Multiply(resMat,rx,ry);

	float x_base = 0.0f + getTrackbarPos("x_base",calibWinName)/derivator ;
	float y_base = -0.35f + getTrackbarPos("y_base",calibWinName)/derivator;
	float z_base = -0.07f + getTrackbarPos("z_base",calibWinName)/derivator;

	Matrix mat_base(x_base,y_base,z_base);
	Matrix::Multiply(mat_base,resMat,mat_base);
	Aruco::BaseToGridboard(mat_base);

}

void InitCalibTool()
{
	initCalib = false;
	namedWindow(calibWinName);
	createTrackbar("arucoDetect", calibWinName, 0,1,on_trackbar, nullptr);
	createTrackbar("deltaFilter", calibWinName, 0,2000,on_trackbar, nullptr);
	createTrackbar("timeStamp", calibWinName, 0,1000,on_trackbar, nullptr);
	createTrackbar("start_joint", calibWinName, 0,6,on_trackbar, nullptr);
	createTrackbar("end_joint", calibWinName, 0,6,on_trackbar, nullptr);
	createTrackbar("ws", calibWinName, 0,1,on_trackbar, nullptr);
	createTrackbar("scale", calibWinName, 0,maxValue,on_trackbar, nullptr);
	createTrackbar("fov", calibWinName, 0,20,on_trackbar, nullptr);
	createTrackbar("x", calibWinName, 0,maxValue/2,on_trackbar, nullptr);
	createTrackbar("y", calibWinName, 0,maxValue/2,on_trackbar, nullptr);
	createTrackbar("z", calibWinName, 0,maxValue/2,on_trackbar, nullptr);

	createTrackbar("alfa", calibWinName, 0,maxValUhel,on_trackbar, nullptr);
	createTrackbar("beta", calibWinName, 0,maxValUhel,on_trackbar, nullptr);
	createTrackbar("gama", calibWinName, 0,maxValUhel,on_trackbar, nullptr);

	createTrackbar("alfa_base", calibWinName, 0,maxValUhel,on_trackbar, nullptr);
	createTrackbar("beta_base", calibWinName, 0,maxValUhel,on_trackbar, nullptr);
	createTrackbar("gama_base", calibWinName, 0,maxValUhel,on_trackbar, nullptr);

	createTrackbar("x_base", calibWinName, 0,maxValue/2,on_trackbar, nullptr);
	createTrackbar("y_base", calibWinName, 0,maxValue/2,on_trackbar, nullptr);
	createTrackbar("z_base", calibWinName, 0,maxValue/2,on_trackbar, nullptr);

	setTrackbarMax("scale",calibWinName, maxValue*10);

	setTrackbarMin("scale",calibWinName, maxValue/2);
	setTrackbarMax("fov",calibWinName, 20);
	setTrackbarMin("fov",calibWinName, -20);

	setTrackbarMin("x",calibWinName, -maxValue/2);
	setTrackbarMin("y",calibWinName, -maxValue/2);
	setTrackbarMin("z",calibWinName, -maxValue/2);
	setTrackbarMin("x_base",calibWinName, -maxValue/2);
	setTrackbarMin("y_base",calibWinName, -maxValue/2);
	setTrackbarMin("z_base",calibWinName, -maxValue/2);

	setTrackbarMin("alfa_base",calibWinName,-maxValUhel);
	setTrackbarMin("beta_base",calibWinName,-maxValUhel);
	setTrackbarMin("gama_base",calibWinName,-maxValUhel);

	setTrackbarMin("alfa",calibWinName,-maxValUhel);
	setTrackbarMin("beta",calibWinName,-maxValUhel);
	setTrackbarMin("gama",calibWinName,-maxValUhel);
	
	setTrackbarPos("arucoDetect",calibWinName, 0);
	setTrackbarPos("x",calibWinName, 0);
	setTrackbarPos("y",calibWinName, 0);
	setTrackbarPos("z",calibWinName, 0);
	setTrackbarPos("x_base",calibWinName, 0);
	setTrackbarPos("y_base",calibWinName, 0);
	setTrackbarPos("z_base",calibWinName, 0);
	setTrackbarPos("fov",calibWinName, 0);
	setTrackbarPos("end_joint", calibWinName,6);
	setTrackbarPos("end_joint", calibWinName,6);

	setTrackbarPos("alfa",calibWinName, 0);
	setTrackbarPos("beta",calibWinName, 0);
	setTrackbarPos("gama",calibWinName, 0);

	setTrackbarPos("alfa_base",calibWinName, 0); 
	setTrackbarPos("beta_base",calibWinName, 0);
	setTrackbarPos("deltaFilter",calibWinName, 1000);
	
}

jetson_camera_node::CameraData CreateCameraDataMsg(
	Mat cvRgbImage, Mat cvDepthImage, 
	Matrix cameraToRobotExtrinsics, 
	sensor_msgs::CameraInfo camInfo, 
	float depthScale)
{
	jetson_camera_node::CameraData camData;
	camData.depth = *cv_bridge::CvImage(std_msgs::Header(), "mono8", cvDepthImage).toImageMsg();
	camData.color = *cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvRgbImage).toImageMsg();
	camData.cameraInfo = camInfo;
	camData.depthScale = depthScale;

	Matrix m = cameraToRobotExtrinsics; // just for comfort of use
	std::vector<float> rotation{ m.m[0][0], m.m[0][1], m.m[0][2], m.m[1][0], m.m[1][1], m.m[1][2], m.m[2][0], m.m[2][1], m.m[2][2] };
	camData.extRotationMatrix = rotation;

	std::vector<float> translation{ m.m[3][0], m.m[3][1], m.m[3][2]};
	camData.extTranslationVector = translation;

	return camData;
}

void PublishCameraData(ros::Publisher rosCameraDataPublisher, Matrix cameraToRobotExtrinsics, bool isVisualizationEnabled = true)
{
	Mat cvRgbImg;
	RSCamera::GetRGBImage(cvRgbImg, false); // TODO check if image already has been recieved during "manual_calibration"
	Image<float> alignedDepth(RSCamera::GetDepthDesc().resW, RSCamera::GetDepthDesc().resH, 0);
	RSCamera::GetAlignedDepthImage(alignedDepth); 
	Mat cvDepthImg = alignedDepth.ToOpenCV();
	auto msg = CreateCameraDataMsg(cvRgbImg, cvDepthImg, cameraToRobotExtrinsics, RSCamera::GetCameraInfo(), RSCamera::GetScale());
	rosCameraDataPublisher.publish(msg);
	if (isVisualizationEnabled)
	{
		imshow("Aligned Depth", cvDepthImg);
		imshow("RGB", cvRgbImg);
	}
}

int main(int argc, char** argv) // TODO Petr, please, divide this god-method into some methods
{
	GetIP();
   	ros::init(argc, argv, "cam_data_processor_" + id);
	ros::NodeHandle n;
	ros::ServiceClient clientInit = n.serviceClient<jetson_camera_node::pcSubscribe>("sub");
	ros::ServiceClient clientData = n.serviceClient<jetson_camera_node::PointCloud>("data");
	//ros::Subscriber sub = n.subscribe("pointPerVoxel",1000, NewPointPerVoxel);
	UdpServer server;
	
	ros::Publisher cameraDataPublisher = n.advertise<jetson_camera_node::CameraData>("camera_data", 1);

	RSCamera::Init();
    RSCamera::Start(CAM_DESC::RS_30_424_240, CAM_DESC::RS_30_424_240); // RS_30_640_480

	InitCalibTool();
	Mat calibWin(10,500,CV_8UC3,Scalar(0,0,0));

	BoundingBox boundingBox;
	boundingBox.max = Vec3(0.5f, 0.2f, 1.5f);
	boundingBox.min = Vec3(-0.5f, -0.6f, 0.0f);

	float fNear = 0.1f;
	float fFar = 100.f;
	float fFov = RSCamera::GetVFov();//58_deg; //65_deg;// 55_deg;//41_deg;// 50_deg;
	float fAspectRatio = (float)RSCamera::GetDepthDesc().resW / (float)RSCamera::GetDepthDesc().resH;

	Image<bool> mask(RSCamera::GetDepthDesc().resW, RSCamera::GetDepthDesc().resH, false);
	Image<float> depth(RSCamera::GetDepthDesc().resW, RSCamera::GetDepthDesc().resH, 20.f);
	Image<float> source(RSCamera::GetDepthDesc().resW, RSCamera::GetDepthDesc().resH, 20.f);

	Aruco::Init();
	Clock::init();
	OrbitCamera cam;
	cam.SetProjectionParams(fAspectRatio, fFov, fNear, fFar);
	cam.SetViewParams(0, 0, 0.3f);
	Object obj;
	//Object obj_cube;
	obj.Load(obj.path_ws_ur3, 0.001f);
	//obj_cube.Load("/home/jetson/Desktop/DMS_01/ur3_poly/joint_0", 0.00005f);
	Matrix view = Matrix(-0.991634, -0.116853, -0.0548485, 0, -0.0145268, 0.523223, -0.852072, 0, 0.128265, -0.844146, -0.520543, 0, -0.10944, 0.359958, 1.84452, 1);
	Robot rob;
	rob.Init();
	float j = 0;
	int index = 1;
	//std::array<float, resH* resW> dept_mask;
	//server.StartServer(50001);
	//server.ProcessIncoming(jointsValues);
	float jointsValues[] = { 0,0,0,0,0,0 };
	jetson_camera_node::pcSubscribe message;
	message.request.id = id;

	if(clientInit.call(message))
	{
        std::cout <<"it was OK -> " << message.response.voxelSize << std::endl;
		voxelSize = message.response.voxelSize;
	}
	else
	{
		std::cout <<"service call error " << std::endl;
	}
    ros::Rate loop_rate(100);
	Mat rgb_img;
	//cam.SetView(Aruco::GetViewMatrix());
	int keyPress = waitKey(1);
	Matrix resMat;
	resMat.LoadFromFile("/home/jetson/Desktop/DMS_01/camPose.bmat");
	cam.SetView(resMat);
	float deltaFilter = 0.12f;
	LoadFromFile("/home/jetson/Desktop/DMS_01/deltaFilter.bfloat", deltaFilter);
	while (ros::ok() && keyPress !=27)
	{
		//RSCamera::Lock(rgb_img);
		//cam.SetView(Aruco::GetViewMatrix());
		//std::cout <<  " fov   " << RSCamera::GetVFov() * 180.0/pi << std::endl;
		RSCamera::GetDepthImage(source);
		Clock::update();
        //std::cout << (int)(Clock::getDeltaSec() * 1000) << " [ms], ->    " << 1.0 / Clock::getDeltaSec() << " [fps]  calib>" << manual_calibration << std::endl; //; // << " ,    " << 1.f / delta << " " << std::endl;

		depth.Fill(10.f);
		mask.Fill(true);
		int start_j = 0;
		int end_j = 6;

		if(manual_calibration)
		{
			cam.SetProjectionParams(fAspectRatio, fFov + (getTrackbarPos("fov",calibWinName)*pi/180.0), fNear, fFar);

			if((bool)getTrackbarPos("arucoDetect",calibWinName))
			{
				RSCamera::Lock(rgb_img);
				//imshow("rgb", rgb_img);
			}
			auto matGBtoBase = Aruco::GetBaseToGridboard();
			auto mat = Aruco::GetViewMatrix();
			auto calibManualMat = Aruco::GetManualMatrix();

			Matrix rx = Matrix::BuildRotationX(getTrackbarPos("alfa",calibWinName)*pi/180.0);
			Matrix ry = Matrix::BuildRotationY(getTrackbarPos("beta",calibWinName)*pi/180.0);
			Matrix rz = Matrix::BuildRotationZ(getTrackbarPos("gama",calibWinName)*pi/180.0);

			Matrix::Multiply(ry,ry,rz);
			Matrix::Multiply(rx,rx,ry);
			Matrix::Multiply(mat,mat,rx);

			Matrix::Multiply(mat,matGBtoBase,mat);

			Matrix rgbToDepthCalibration = RSCamera::GetTFDepthToRgb(); 

			rgbToDepthCalibration.Invert();
			Matrix::Multiply(mat, mat, rgbToDepthCalibration);

			Matrix::Multiply(resMat,mat,calibManualMat);
			resMat.SaveToFile("/home/jetson/Desktop/DMS_01/camPose.bmat");
			cam.SetView(resMat);

			start_j = getTrackbarPos("start_joint",calibWinName);
			end_j = getTrackbarPos("end_joint",calibWinName);
			deltaFilter = ( getTrackbarPos("deltaFilter",calibWinName) - 1000)/1000.0;
			SaveToFile("/home/jetson/Desktop/DMS_01/deltaFilter.bfloat", deltaFilter);
		}

		Matrix actual(1.f);

		//server.ProcessIncoming(jointsValues);
		for (int i = 0; i < rob.joints.size(); i++)		
		{

			Mesh mesh = rob.joints[i].GetMesh();
			Matrix m;
			float act = 0;

			if (i > 0)
				act = jointsValues[i - 1];

			rob.BuilMat(m, rob.dh[i], act);
			m.Transpose();
			Matrix::Multiply(actual, m, actual);
			Matrix full;
			Matrix::Multiply(full, actual, cam.GetViewProj());
			Engine::TransformMeshPerspectiveDivide(mesh, full);

			Engine::BackFaceCull(mesh);
			Engine::ViewportScale(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW);
			if(i >= start_j && i <= end_j  )
				Engine::BuildDepthMap(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW, depth);

		}
		if(manual_calibration)
		{
			if((bool)getTrackbarPos("ws",calibWinName))
			{
				Mesh mesh = obj.GetMesh();
				Engine::TransformMesh(mesh, cam.GetView());
				Engine::TransformMeshPerspectiveDivide(mesh, cam.GetProj());
				Engine::BackFaceCull(mesh);
				Engine::ViewportScale(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW);
				Engine::BuildDepthMap(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW, depth);
			}
			//voxelTimeStampTreshold = getTrackbarPos("timeStamp",calibWinName)/1000.0;
		}
		else
		{	
			Mesh mesh = obj.GetMesh();
			Engine::TransformMesh(mesh, cam.GetView());
			Engine::TransformMeshPerspectiveDivide(mesh, cam.GetProj());
			Engine::BackFaceCull(mesh);
			Engine::ViewportScale(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW);
            Engine::BuildDepthMap(mesh, RSCamera::GetDepthDesc().resH, RSCamera::GetDepthDesc().resW, depth);
		 }
		std::cout << "delta filter  " << deltaFilter << std::endl;
		depth.Compare(source.GetPtr(), mask, 1.0f + deltaFilter);
		//mask.Fill(true);
		vector<Vec3> voxels;

		RSCamera::ProjectDepthToPointCloud(source, mask, voxels);

		Matrix m = cam.GetView(); //Aruco::GetViewMatrix();
		m.Invert();
		m.Print();

		Matrix::Multiply(m, m, Matrix::BuildRotationZ(-180_deg));
		vector<Vec3> voxelsInScene;
		voxelsInScene.reserve(voxels.size());

		PublishCameraData(cameraDataPublisher, m, true);

		OctoMap::TransformAndCheckWithBoundingBox(voxels, voxelsInScene, boundingBox, m);

		unordered_set<uint32_t> indexedVoxelsHesh;

		Vec3 camPos = Vec3(m._41,m._42,m._43);
		//float percentileTreshold = 0.5;
		OctoMap::AlignToMap3(voxelsInScene, indexedVoxelsHesh, voxelSize, percentageVoxelOccupancy, camPos, m.GetUnitZ());

		jetson_camera_node::PointCloud data_message;
		//data_message.request.array = indexedVoxelsHesh;

		for(auto& voxel : voxelTimeStamp)
		{
			voxel.second += Clock::getDeltaSec();
		}
		for(auto& voxel : indexedVoxelsHesh)
		{
			auto result = voxelTimeStamp.insert({ voxel, 0 });
			if(!result.second)
				result.first->second = 0;
		}

		std::unordered_set<uint32_t> cleanVoxel;
		for(auto& voxel : voxelTimeStamp)
		{
			
			if (voxel.second > deltaTimeFilter)
			{
				cleanVoxel.insert(voxel.first);
			}
		}
		for(auto& voxel : cleanVoxel)
		{
			voxelTimeStamp.erase(voxel);
		}
		
		
		int index = 0;
		data_message.request.array.resize(voxelTimeStamp.size());
		for(auto& voxel : voxelTimeStamp)
		{
			data_message.request.array[index] = voxel.first;
			index = index + 1;
		}
	
		data_message.request.id = id;
		data_message.request.voxelCount = indexedVoxelsHesh.size();// 
		data_message.request.voxelSize = voxelSize;

		if(clientData.call(data_message))
		{
			//std::cout <<"it was OK -> " << data_message.response.voxelSize << std::endl;
			voxelSize = data_message.response.voxelSize;
			percentageVoxelOccupancy = data_message.response.percentageVoxelOccupancy;
			deltaTimeFilter = data_message.response.deltaTimeFilter;
			//std::cout << "delta filter  " <<  data_message.response.deltaTimeFilter << std::endl;
			jointsValues[0] = data_message.response.joints[0];
			jointsValues[1] = data_message.response.joints[1];
			jointsValues[2] = data_message.response.joints[2];
			jointsValues[3] = data_message.response.joints[3];
			jointsValues[4] = data_message.response.joints[4];
			jointsValues[5] = data_message.response.joints[5];
		}
		else
		{
			std::cout <<"service call error " << std::endl;
		}

		auto depth_cv = mask.ToOpenCV();
		//imshow("Depth mask", depth_cv);


		if(manual_calibration)
		{
			if(initCalib) 
			{
				InitCalibTool();
			}
			imshow(calibWinName,calibWin);
		}
		else
		{
			if(!initCalib)
				destroyWindow(calibWinName);
			initCalib = true;	
		}

 		keyPress = waitKey(1);
		if(keyPress == 99)
		{
			manual_calibration = !manual_calibration;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	//RSCamera::Joint();	
	//destroyAllWindows();

}

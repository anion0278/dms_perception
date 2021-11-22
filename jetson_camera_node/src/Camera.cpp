#include <algorithm>
#include "Camera.h"
#include <opencv2/opencv.hpp>
//#include "RoboMathGL.h"
#include "Math.h"

#define WHEEL_DELTA 1

//using namespace RoboMathSpec;


void PerspectiveCamera::BuildProj()
{

	if (m_rightHanded)
		m_proj.FillPerspectiveFovRH(m_fov, m_aspect, m_near, m_far);
	else
	{
		m_proj.FillPerspectiveFovLH(m_fov, m_aspect, m_near, m_far);
	}
		ComputeViewProj();
}


void OrbitCamera::BuildView()
{
	if (m_rightHanded) {
		// za��n�me s kamerou d�vaj�c� se ve sm�ru osy Y:
		m_eyePosition = Vec3(0.0, -m_distance, 0.0);
		Vec3 vUp(0.0, 0.0, 1.0f);
		// oto��me kameru kolem po��tku sou�adnic podle po�adovan�ho nato�en�:
		Matrix m, m1;
		m.FillRotationX(-m_pitch);
		m1.FillRotationZ(m_yaw + m_parentYaw);
		Matrix::Multiply(m, m, m1);
		Vec3::TransformCoord(m_eyePosition, m_eyePosition, m);
		Vec3::TransformCoord(vUp, vUp, m);
		Vec3 target = m_target + m_parentTarget;
		//Vec3::Count(target, m_target, m_parentTarget);
		m_eyePosition = m_eyePosition + target;
		//Vec3::Count(m_eyePosition, m_eyePosition, target);

		m_view.FillLookAtRH(m_eyePosition, target, vUp);
	}
	else {
		// za��n�me s kamerou d�vaj�c� se ve sm�ru osy Z:
		m_eyePosition = Vec3(0.0, 0.0, -m_distance);
		Vec3 vUp(0.0, 1.0f, 0.0);
		// oto��me kameru kolem po��tku sou�adnic podle po�adovan�ho nato�en�:
		Matrix m = Matrix::Identity;
		Matrix m1 = Matrix::Identity;
		m.FillRotationX(m_pitch);
		m1.FillRotationZ(-(m_yaw + m_parentYaw));
		Matrix::Multiply(m, m, m1);
		Vec3::TransformCoord(m_eyePosition, m_eyePosition, m);
		Vec3::TransformCoord(vUp, vUp, m);
		Vec3 target = m_target + m_parentTarget;
		//Vec3::Count(target, m_target, m_parentTarget);
		//Vec3::Count(m_eyePosition, m_eyePosition, target);
		m_eyePosition += target;
		m_view.FillLookAtLH(m_eyePosition, target, vUp);


		//RoboMath::Matrix m_test, m_tes1, m_view_test;
		//RoboMath::Vec3 m_eyePosition_test(0.0, 0.0, -m_distance);
		//RoboMath::Vec3 m_vUp_test(0.0, 1.0, 0);
		//RoboMath::Vec3 target_test;
		//m_test.FillRotationX(m_pitch);
		//m_tes1.FillRotationZ(-(m_yaw + m_parentYaw));
		//RoboMath::Matrix::Multiply(m_test, m_test, m_tes1);
		//RoboMath::Vec3::TransformCoord(m_eyePosition_test, m_eyePosition_test, m_test);
		//RoboMath::Vec3::TransformCoord(m_vUp_test, m_vUp_test, m_test);

		//target_test.x = m_target.x + m_parentTarget.x;
		//target_test.y = m_target.y + m_parentTarget.y;
		//target_test.z = m_target.z + m_parentTarget.z;

		//m_eyePosition_test += target_test;

		//m_view_test.FillLookAtLH(m_eyePosition_test, target_test, m_vUp_test);
		//std::cout << m_view_test.m[0][0] << std::endl;

	}
	ComputeViewProj();
}








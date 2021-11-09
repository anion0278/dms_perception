#pragma once

#include "Object.h"

class Camera
{
protected:
	Matrix m_view;
	Matrix m_proj;
	Matrix m_viewProj;
	Vec3   m_eyePosition = Vec3(0, 0, 0);

	bool   m_rightHanded = false; // D3D výchozí je LH (false) s osou Y nahoru a Z dopøedu; RH (true) má osu Z nahoru a Y dopøedu

	virtual void BuildView() = 0;
	virtual void BuildProj() = 0;


	void ComputeViewProj() { Matrix::Multiply(m_viewProj, m_view, m_proj); };

public:
	Camera() {};

	const Matrix& GetProj() const { return m_proj; };
	const Matrix& GetView() const { return m_view; };
	const Matrix& GetViewProj() const { return m_viewProj; };
	const Vec3& GetEyePosition() const { return m_eyePosition; };

	bool IsRightHanded() const { return m_rightHanded; };
	void SetRightHanded() { m_rightHanded = true; };
	void SetProj(const Matrix& proj) { m_proj = proj; };
	void SetView(const Matrix& view) { m_view = view; ComputeViewProj(); };

};

class PerspectiveCamera : public Camera
{
private:
	//bool OnDeviceEvent(Event const& e);

protected:
	float  m_aspect = 4.f / 3.f;
	float  m_fov = 0.7854f;
	float  m_near = 0.01f;
	float  m_far = 100.0f;

	void BuildProj() override;

public:
	PerspectiveCamera() { BuildProj(); };

	float GetFov() const { return m_fov; };

	void SetProjectionParams(float aspect, float fov, float nearClip, float farClip) { m_aspect = aspect; m_fov = fov;  m_near = nearClip; m_far = farClip; BuildProj(); };
	void SetAspectRatio(float aspect) { m_aspect = aspect; BuildProj(); };
	void SetAspectRatio(int width, int height) { m_aspect = (float)width / (float)height; BuildProj(); };
	void SetFOV(float fov) { m_fov = fov; BuildProj(); };
	void SetClip(float nearClip, float farClip) { m_near = nearClip; m_far = farClip; BuildProj(); };
};

class OrbitCamera : public PerspectiveCamera
{
private:
	Vec3    m_target = Vec3(0, 0, 0);
	Vec3    m_parentTarget = Vec3(0, 0, 0);
	float   m_parentYaw = 0;
	float   m_yaw = 0.1f;
	float   m_pitch = 0.4f;
	float   m_distance = 5.0f;
	float   m_rotationSpeed = 0.01f;
	float   m_translationSpeed = 0.005f;
	float   m_zoomSpeed = 0.1f;
	float   m_minDistance = 0.02f;
	float   m_maxDistance = 100.0f;
	bool    m_movementStarting = false;
	//Mouse::Button m_rotationButton = Mouse::ButtonRight;
	//Mouse::Button m_translationButton = Mouse::ButtonMiddle;

private:
	void BuildView() override;

public:
	OrbitCamera() { BuildView(); };
	// Automaticky registruje zachytávání všech událostí, které kamera potøebuje pro své ovládání uživatelem.
	//void RegisterCameraControllerEvents(bool allowZoom, Mouse::Button rotationButton = Mouse::ButtonRight, Mouse::Button translationButton = Mouse::ButtonMiddle);

	// Pøi ruèním ovládání kamery myší (bez použití automatického zachytávání) je tuto funkci potøeba volat pøi mousedown události tlaèítka, které ovládá rotaci kamery.
	void StartMouseManipulation(void) { m_movementStarting = true; };
	// Pøi ruèním ovládání kamery myší (bez použití automatického zachytávání) je tuto funkci potøeba volat pøi mousemove události, pokud se stále drženo tlaèítko ovládající rotaci kamery.

	// Nastaví rychlost rotace kamery pomocí myší. Hodnota je spoleèná pro oba smìry rotace a má jednotku [rad/pixel]. Výchozí je 0.01.
	void SetRotationSpeed(float radPerPixel) { m_rotationSpeed = radPerPixel; };
	// Nastaví parametry zoomování (resp. pøibližování a oddalování kamery).
	void SetZoomParams(float speed, float minDistance, float maxDistance) { m_zoomSpeed = speed; m_minDistance = minDistance; m_maxDistance = maxDistance; };

	void SetViewParams(const Vec3 &target, float yaw, float pitch, float distance) { m_target = target; m_yaw = yaw; m_pitch = pitch; m_distance = distance; BuildView(); };
	void SetViewParams(float yaw, float pitch, float distance)                     { m_yaw = yaw; m_pitch = pitch; m_distance = distance; BuildView(); };
	void SetTarget(const Vec3 &target)                                             { m_target = target; BuildView(); };
	void SetRotation(float yaw, float pitch)                                       { m_yaw = yaw; m_pitch = pitch; BuildView();  };
	void SetDistance(float distance)                                               { m_distance = distance; BuildView();  };
	void SetParentYaw(float yaw)                                                   { m_parentYaw = yaw; BuildView(); };
	void SetParentTarget(const Vec3 &target)                                       { m_parentTarget = target; BuildView(); };
	void SetParent(float yaw, const Vec3 &target)                                  { m_parentYaw = yaw; m_parentTarget = target; BuildView(); };

	const Vec3& GetTarget() const { return m_target; };
	float GetDistance() const { return m_distance; };
	float GetYaw() const { return m_yaw; };
	float GetPitch() const { return m_pitch; };
};





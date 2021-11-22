#include "RoboMathSpec.h"
#include <memory.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace RoboMathSpec;

////////////////////////////////////////////////////////////////////////////////////
//    Vec3
////////////////////////////////////////////////////////////////////////////////////

void Vec3::Normalize()
{
	float s = Length();
	if (s > 0) {
		s = 1.0f / s;
		x *= s; y *= s; z *= s;
	}
}

void Vec3::TransformNormal(const Matrix& m)
{
	float v[3];
	for (int c = 0; c < 3; c++) {
		v[c] = x * m.m[0][c] + y * m.m[1][c] + z * m.m[2][c];
	}
	memcpy(this, v, sizeof(v));
}

void Vec3::TransformCoord(const Matrix& m)
{
	float v[3];
	for (int c = 0; c < 3; c++) {
		v[c] = x * m.m[0][c] + y * m.m[1][c] + z * m.m[2][c] + 1.0f * m.m[3][c];
	}
	memcpy(this, v, sizeof(v));
}

// static
void Vec3::TransformNormal(Vec3& out, const Vec3& v, const Matrix& m)
{
	for (int c = 0; c < 3; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c];
	}
}

// static
Vec3 Vec3::TransformNormal(const Vec3& v, const Matrix& m)
{
	Vec3 out;
	for (int c = 0; c < 3; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c];
	}
	return out;
}

// static
void Vec3::TransformCoord(Vec3& out, const Vec3& v, const Matrix& m)
{
	for (int c = 0; c < 3; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c] + 1.0f * m.m[3][c];
	}
}

// static
Vec3 Vec3::TransformCoord(const Vec3& v, const Matrix& m)
{
	Vec3 out;
	for (int c = 0; c < 3; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c] + 1.0f * m.m[3][c];
	}
	return out;
}






////////////////////////////////////////////////////////////////////////////////////
//    Vec4
////////////////////////////////////////////////////////////////////////////////////

void Vec4::Normalize()
{
	float s = Length();
	if (s > 0) {
		s = 1.f / s;
		x *= s; y *= s; z *= s; w *= s;
	}
};

void Vec4::Transform(const Matrix& m)
{
	float v[4];
	for (int c = 0; c < 4; c++) {
		v[c] = x * m.m[0][c] + y * m.m[1][c] + z * m.m[2][c] + w * m.m[3][c];
	}
	memcpy(this, v, sizeof(v));
}

// static
void Vec4::Transform(Vec4& out, const Vec4& v, const Matrix& m)
{
	for (int c = 0; c < 4; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c] + v.w * m.m[3][c];
	}
}

// static
Vec4 Vec4::Transform(const Vec4& v, const Matrix& m)
{
	Vec4 out;
	for (int c = 0; c < 4; c++) {
		out[c] = v.x * m.m[0][c] + v.y * m.m[1][c] + v.z * m.m[2][c] + v.w * m.m[3][c];
	}
	return out;
}



////////////////////////////////////////////////////////////////////////////////////
//    Matrix
////////////////////////////////////////////////////////////////////////////////////

inline constexpr Matrix& Matrix::operator += (const Matrix& mat)
{
	_11 += mat._11; _12 += mat._12; _13 += mat._13; _14 += mat._14;
	_21 += mat._21; _22 += mat._22; _23 += mat._23; _24 += mat._24;
	_31 += mat._31; _32 += mat._32; _33 += mat._33; _34 += mat._34;
	_41 += mat._41; _42 += mat._42; _43 += mat._43; _44 += mat._44;
	return *this;
}

inline constexpr Matrix& Matrix::operator -= (const Matrix& mat)
{
	_11 -= mat._11; _12 -= mat._12; _13 -= mat._13; _14 -= mat._14;
	_21 -= mat._21; _22 -= mat._22; _23 -= mat._23; _24 -= mat._24;
	_31 -= mat._31; _32 -= mat._32; _33 -= mat._33; _34 -= mat._34;
	_41 -= mat._41; _42 -= mat._42; _43 -= mat._43; _44 -= mat._44;
	return *this;
}

inline constexpr Matrix& Matrix::operator *= (float f)
{
	_11 *= f; _12 *= f; _13 *= f; _14 *= f;
	_21 *= f; _22 *= f; _23 *= f; _24 *= f;
	_31 *= f; _32 *= f; _33 *= f; _34 *= f;
	_41 *= f; _42 *= f; _43 *= f; _44 *= f;
	return *this;
}

inline constexpr Matrix& Matrix::operator /= (float f)
{
	float fInv = 1.0f / f;
	_11 *= fInv; _12 *= fInv; _13 *= fInv; _14 *= fInv;
	_21 *= fInv; _22 *= fInv; _23 *= fInv; _24 *= fInv;
	_31 *= fInv; _32 *= fInv; _33 *= fInv; _34 *= fInv;
	_41 *= fInv; _42 *= fInv; _43 *= fInv; _44 *= fInv;
	return *this;
}

void Matrix::Transpose()
{
	Matrix M{ _11, _21, _31, _41, _12, _22, _32, _42, _13, _23, _33, _43, _14, _24, _34, _44 };
	memcpy(m, M.m, sizeof(Matrix));
}

void Matrix::GetCfactor(const Matrix& M, Matrix& t, int p, int q, int n)
{
	int i = 0, j = 0;
	for (int r = 0; r < n; r++) {
		for (int c = 0; c < n; c++) {
			if (r != p && c != q) {
				t.m[i][j++] = M.m[r][c]; //If row is filled increase r index and reset c index
				if (j == n - 1) {
					j = 0; i++;
				}
			}
		}
	}
}

float Matrix::Determinant(const Matrix& M, int n)
{
	float D = 0;
	if (n == 1)
		return M.m[0][0];
	Matrix t; //store cofactors
	float s = 1; //store sign multiplier //
	//To Iterate each element of first row
	for (int f = 0; f < n; f++) {
		//For Getting Cofactor of M[0][f] do
		GetCfactor(M, t, 0, f, n);
		D += s * M.m[0][f] * Determinant(t, n - 1);
		s = -s;
	}
	return D;
}

void RoboMathSpec::Matrix::Adj(const Matrix& M, Matrix& adj)
{
	float s = 1;
	Matrix t;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			//To get cofactor of M[i][j]
			GetCfactor(M, t, i, j, 4);
			s = ((i + j) % 2 == 0) ? 1 : -1; //sign of adj[j][i] positive if sum of row and column indexes is even.
			adj.m[j][i] = (s) * (Determinant(t, 4 - 1)); //Interchange rows and columns to get the transpose of the cofactor matrix
		}
	}
}

bool Matrix::Invert()
{

	//float det = Determinant(*this, 4);
	//if (det == 0)
	//	return false;
	//Matrix adj;
	//Adj(*this, adj);
	//Matrix inv;
	//for (int i = 0; i < 4; i++)
	//	for (int j = 0; j < 4; j++)
	//		inv.m[i][j] = adj.m[i][j] / float(det);
	//Transpose();


	Matrix Tinv{ 1.0f, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 1.0f, 0, -_41, -_42, -_43, 1.0f };
	Matrix Rinv = *this;
	Rinv._41 = 0; Rinv._42 = 0; Rinv._43 = 0;
	Rinv.Transpose();
	*this = Matrix::Multiply(Tinv, Rinv);
	return true;
}

bool Matrix::SaveToFile(const std::string& name)
{
	std::ofstream file(name, std::ios::binary);
	if(file.is_open())
	{
		file.write((const char*)this, sizeof(Matrix));
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}


bool Matrix::LoadFromFile(const std::string& name)
{
	std::ifstream file(name, std::ios::binary);
	if(file.is_open())
	{
		file.read((char*)this, sizeof(Matrix));
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

void Matrix::FillRotationX(float angle)
{
	FillIdentity();
	float s = sin(angle);
	float c = cos(angle);
	_22 = c;   _23 = s;
	_32 = -s;  _33 = c;
}

void Matrix::FillRotationY(float angle)
{
	FillIdentity();
	float s = sin(angle);
	float c = cos(angle);
	_11 = c;   _13 = -s;
	_31 = s;   _33 = c;
}

void Matrix::FillRotationZ(float angle)
{
	FillIdentity();
	float s = sin(angle);
	float c = cos(angle);
	_11 = c;   _12 = s;
	_21 = -s;  _22 = c;
}

void Matrix::FillRotationX(float angle, const Vec3& translation)
{
	FillTranslation(translation);
	float s = sin(angle);
	float c = cos(angle);
	_22 = c;   _23 = s;
	_32 = -s;  _33 = c;
}

void Matrix::FillRotationY(float angle, const Vec3& translation)
{
	FillTranslation(translation);
	float s = sin(angle);
	float c = cos(angle);
	_11 = c;   _13 = -s;
	_31 = s;   _33 = c;
}

void Matrix::FillRotationZ(float angle, const Vec3& translation)
{
	FillTranslation(translation);
	float s = sin(angle);
	float c = cos(angle);
	_11 = c;   _12 = s;
	_21 = -s;  _22 = c;
}

void Matrix::FillRotationAxis(const Vec3& axis, float angle)
{
	Vec3 axisN = Vec3::Normalize(axis);
	float s = sin(angle);
	float c = cos(angle);
	float c2 = 1.0f - c;

	Vec4 V0{ c2 * axis.y * axis.z, c2 * axis.z * axis.x, c2 * axis.x * axis.y, 0 };

	Vec4 V1{ s * axis.z + V0.z, V0.y - s * axis.y, V0.z - s * axis.z, s * axis.x + V0.x };
	Vec4 V2{ s * axis.y + V0.y, V0.x - s * axis.x, s * axis.y + V0.y, V0.x - s * axis.x };

	V0 = Vec4(c2 * axis.x * axis.x + c, c2 * axis.y * axis.y + c, c2 * axis.z * axis.z + c, 0);

	_11 = V0.x; _12 = V1.x; _13 = V1.y; _14 = V0.w;
	_21 = V1.z; _22 = V0.y; _23 = V1.w; _24 = V0.w;
	_31 = V2.x; _32 = V2.y; _33 = V0.z; _34 = V0.w;
	_41 = 0; _42 = 0; _43 = 0; _44 = 1.0f;
}

void Matrix::FillPerspectiveFovLH(float fov, float aspectRatio, float nearPlane, float farPlane)
{
	float yScale = (float)(1.0 / tan(fov / 2.0));
	float xScale = yScale / aspectRatio;
	FillZeros();
	_11 = xScale;
	_22 = yScale;
	_33 = farPlane / (farPlane - nearPlane);
	_34 = 1.0f;
	_43 = -nearPlane * farPlane / (farPlane - nearPlane); ;
}

void Matrix::FillPerspectiveFovRH(float fov, float aspectRatio, float nearPlane, float farPlane)
{
	float yScale = (float)(1.0 / tan(fov / 2.0));
	float xScale = yScale / aspectRatio;
	FillZeros();
	_11 = xScale;
	_22 = yScale;
	_33 = farPlane / (nearPlane - farPlane);
	_34 = -1.0f;
	_43 = nearPlane * farPlane / (nearPlane - farPlane); ;
}

void Matrix::FillOrthographicLH(float width, float height, float zNearPlane, float zFarPlane)
{
	FillIdentity();
	_11 = 2.0f / width;
	_22 = 2.0f / height;
	_33 = 1.0f / (zFarPlane - zNearPlane);
	_43 = zNearPlane / (zNearPlane - zFarPlane);
}

void Matrix::FillOrthographicRH(float width, float height, float zNearPlane, float zFarPlane)
{
	FillIdentity();
	_11 = 2.0f / width;
	_22 = 2.0f / height;
	_33 = 1.0f / (zNearPlane - zFarPlane);
	_43 = zNearPlane / (zNearPlane - zFarPlane);
}

void Matrix::FillLookAtLH(const Vec3& eye, const Vec3& target, const Vec3& up)
{
	Vec3 zaxis = (target - eye);
	zaxis.Normalize();
	Vec3 xaxis = Vec3::Cross(up, zaxis);
	xaxis.Normalize();
	Vec3 yaxis = Vec3::Cross(zaxis, xaxis);

	_11 = xaxis.x; _12 = yaxis.x; _13 = zaxis.x; _41 = 0;
	_21 = xaxis.y; _22 = yaxis.y; _23 = zaxis.y; _42 = 0;
	_31 = xaxis.z; _32 = yaxis.z; _33 = zaxis.z; _43 = 0;
	_41 = -Vec3::Dot(xaxis, eye);
	_42 = -Vec3::Dot(yaxis, eye);
	_43 = -Vec3::Dot(zaxis, eye);
	_44 = 1.0f;
}

void Matrix::FillLookAtRH(const Vec3& eye, const Vec3& target, const Vec3& up)
{
	Vec3 zaxis = (eye - target);
	zaxis.Normalize();
	Vec3 xaxis = Vec3::Cross(up, zaxis);
	xaxis.Normalize();
	Vec3 yaxis = Vec3::Cross(zaxis, xaxis);

	_11 = xaxis.x; _12 = yaxis.x; _13 = zaxis.x; _41 = 0;
	_21 = xaxis.y; _22 = yaxis.y; _23 = zaxis.y; _42 = 0;
	_31 = xaxis.z; _32 = yaxis.z; _33 = zaxis.z; _43 = 0;
	_41 = Vec3::Dot(xaxis, eye);
	_42 = Vec3::Dot(yaxis, eye);
	_43 = Vec3::Dot(zaxis, eye);
	_44 = 1.0f;
}

// static
void Matrix::Multiply(Matrix& out, const Matrix& m1, const Matrix& m2)
{
	Matrix temp{ m1._11 * m2._11 + m1._12 * m2._21 + m1._13 * m2._31 + m1._14 * m2._41,
			   m1._11 * m2._12 + m1._12 * m2._22 + m1._13 * m2._32 + m1._14 * m2._42,
			   m1._11 * m2._13 + m1._12 * m2._23 + m1._13 * m2._33 + m1._14 * m2._43,
			   m1._11 * m2._14 + m1._12 * m2._24 + m1._13 * m2._34 + m1._14 * m2._44,
			   m1._21 * m2._11 + m1._22 * m2._21 + m1._23 * m2._31 + m1._24 * m2._41,
			   m1._21 * m2._12 + m1._22 * m2._22 + m1._23 * m2._32 + m1._24 * m2._42,
			   m1._21 * m2._13 + m1._22 * m2._23 + m1._23 * m2._33 + m1._24 * m2._43,
			   m1._21 * m2._14 + m1._22 * m2._24 + m1._23 * m2._34 + m1._24 * m2._44,
			   m1._31 * m2._11 + m1._32 * m2._21 + m1._33 * m2._31 + m1._34 * m2._41,
			   m1._31 * m2._12 + m1._32 * m2._22 + m1._33 * m2._32 + m1._34 * m2._42,
			   m1._31 * m2._13 + m1._32 * m2._23 + m1._33 * m2._33 + m1._34 * m2._43,
			   m1._31 * m2._14 + m1._32 * m2._24 + m1._33 * m2._34 + m1._34 * m2._44,
			   m1._41 * m2._11 + m1._42 * m2._21 + m1._43 * m2._31 + m1._44 * m2._41,
			   m1._41 * m2._12 + m1._42 * m2._22 + m1._43 * m2._32 + m1._44 * m2._42,
			   m1._41 * m2._13 + m1._42 * m2._23 + m1._43 * m2._33 + m1._44 * m2._43,
			   m1._41 * m2._14 + m1._42 * m2._24 + m1._43 * m2._34 + m1._44 * m2._44 };
	out = temp;
}

// static
void Matrix::Transpose(Matrix& out, const Matrix& m)
{
	out = Matrix{ m._11, m._21, m._31, m._41, m._12, m._22, m._32, m._42, m._13, m._23, m._33, m._43, m._14, m._24, m._34, m._44 };
}

// static
void Matrix::Invert(Matrix& out, const Matrix& m)
{
	out = m;
	out.Invert();
}

// static
Matrix Matrix::Multiply(const Matrix& m1, const Matrix& m2)
{
	Matrix out{ m1._11 * m2._11 + m1._12 * m2._21 + m1._13 * m2._31 + m1._14 * m2._41,
			   m1._11 * m2._12 + m1._12 * m2._22 + m1._13 * m2._32 + m1._14 * m2._42,
			   m1._11 * m2._13 + m1._12 * m2._23 + m1._13 * m2._33 + m1._14 * m2._43,
			   m1._11 * m2._14 + m1._12 * m2._24 + m1._13 * m2._34 + m1._14 * m2._44,
			   m1._21 * m2._11 + m1._22 * m2._21 + m1._23 * m2._31 + m1._24 * m2._41,
			   m1._21 * m2._12 + m1._22 * m2._22 + m1._23 * m2._32 + m1._24 * m2._42,
			   m1._21 * m2._13 + m1._22 * m2._23 + m1._23 * m2._33 + m1._24 * m2._43,
			   m1._21 * m2._14 + m1._22 * m2._24 + m1._23 * m2._34 + m1._24 * m2._44,
			   m1._31 * m2._11 + m1._32 * m2._21 + m1._33 * m2._31 + m1._34 * m2._41,
			   m1._31 * m2._12 + m1._32 * m2._22 + m1._33 * m2._32 + m1._34 * m2._42,
			   m1._31 * m2._13 + m1._32 * m2._23 + m1._33 * m2._33 + m1._34 * m2._43,
			   m1._31 * m2._14 + m1._32 * m2._24 + m1._33 * m2._34 + m1._34 * m2._44,
			   m1._41 * m2._11 + m1._42 * m2._21 + m1._43 * m2._31 + m1._44 * m2._41,
			   m1._41 * m2._12 + m1._42 * m2._22 + m1._43 * m2._32 + m1._44 * m2._42,
			   m1._41 * m2._13 + m1._42 * m2._23 + m1._43 * m2._33 + m1._44 * m2._43,
			   m1._41 * m2._14 + m1._42 * m2._24 + m1._43 * m2._34 + m1._44 * m2._44 };
	return out;
}

// static
Matrix Matrix::Transpose(const Matrix& m)
{
	return Matrix{ m._11, m._21, m._31, m._41, m._12, m._22, m._32, m._42, m._13, m._23, m._33, m._43, m._14, m._24, m._34, m._44 };
}

// static
Matrix Matrix::Invert(const Matrix& m)
{
	Matrix out = m;
	out.Invert();
	return out;
}

// static
Matrix Matrix::BuildRotationX(float angle)
{
	Matrix M;
	M.FillRotationX(angle);
	return M;
}

// static
Matrix Matrix::BuildRotationY(float angle)
{
	Matrix M;
	M.FillRotationY(angle);
	return M;
}

// static
Matrix Matrix::BuildRotationZ(float angle)
{
	Matrix M;
	M.FillRotationZ(angle);
	return M;
}

// static
Matrix Matrix::BuildRotationX(float angle, const Vec3& translation)
{
	Matrix M;
	M.FillRotationX(angle, translation);
	return M;
}

// static
Matrix Matrix::BuildRotationY(float angle, const Vec3& translation)
{
	Matrix M;
	M.FillRotationY(angle, translation);
	return M;
}

// static
Matrix Matrix::BuildRotationZ(float angle, const Vec3& translation)
{
	Matrix M;
	M.FillRotationZ(angle, translation);
	return M;
}

// static
Matrix Matrix::BuildRotationAxis(const Vec3& axis, float angle)
{
	Matrix M;
	M.FillRotationAxis(axis, angle);
	return M;
}

// static
Matrix Matrix::BuildPerspectiveFovLH(float fov, float aspectRatio, float nearPlane, float farPlane)
{
	Matrix M;
	M.FillPerspectiveFovLH(fov, aspectRatio, nearPlane, farPlane);
	return M;
}

// static
Matrix Matrix::BuildPerspectiveFovRH(float fov, float aspectRatio, float nearPlane, float farPlane)
{
	Matrix M;
	M.FillPerspectiveFovRH(fov, aspectRatio, nearPlane, farPlane);
	return M;
}

// static
Matrix Matrix::BuildOrthographicLH(float width, float height, float zNearPlane, float zFarPlane)
{
	Matrix M;
	M.FillOrthographicLH(width, height, zNearPlane, zFarPlane);
	return M;
}

// static
Matrix Matrix::BuildOrthographicRH(float width, float height, float zNearPlane, float zFarPlane)
{
	Matrix M;
	M.FillOrthographicRH(width, height, zNearPlane, zFarPlane);
	return M;
}

// static
Matrix Matrix::BuildLookAtLH(const Vec3& eye, const Vec3& target, const Vec3& up)
{
	Matrix M;
	M.FillLookAtLH(eye, target, up);
	return M;
}

// static
Matrix Matrix::BuildLookAtRH(const Vec3& eye, const Vec3& target, const Vec3& up)
{
	Matrix M;
	M.FillLookAtRH(eye, target, up);
	return M;
}


Matrix RoboMathSpec::operator* (const Matrix& M1, const Matrix& M2)
{
	Matrix M = Matrix::Multiply(M1, M2);
	return M;
}

constexpr Matrix RoboMathSpec::operator + (const Matrix& m1, const Matrix& m2)
{
	return Matrix(m1._11 + m2._11, m1._12 + m2._12, m1._13 + m2._13, m1._14 + m2._14,
		m1._21 + m2._21, m1._22 + m2._22, m1._23 + m2._23, m1._24 + m2._24,
		m1._31 + m2._31, m1._32 + m2._32, m1._33 + m2._33, m1._34 + m2._34,
		m1._41 + m2._41, m1._42 + m2._42, m1._43 + m2._43, m1._44 + m2._44);
}

constexpr Matrix RoboMathSpec::operator - (const Matrix& m1, const Matrix& m2)
{
	return Matrix(m1._11 - m2._11, m1._12 - m2._12, m1._13 - m2._13, m1._14 - m2._14,
		m1._21 - m2._21, m1._22 - m2._22, m1._23 - m2._23, m1._24 - m2._24,
		m1._31 - m2._31, m1._32 - m2._32, m1._33 - m2._33, m1._34 - m2._34,
		m1._41 - m2._41, m1._42 - m2._42, m1._43 - m2._43, m1._44 - m2._44);
}

constexpr Matrix RoboMathSpec::operator * (const Matrix& m, float f)
{
	return Matrix(m._11 * f, m._12 * f, m._13 * f, m._14 * f,
		m._21 * f, m._22 * f, m._23 * f, m._24 * f,
		m._31 * f, m._32 * f, m._33 * f, m._34 * f,
		m._41 * f, m._42 * f, m._43 * f, m._44 * f);
}

constexpr Matrix RoboMathSpec::operator / (const Matrix& m, float f)
{
	float fInv = 1.0f / f;
	return Matrix(m._11 * fInv, m._12 * fInv, m._13 * fInv, m._14 * fInv,
		m._21 * fInv, m._22 * fInv, m._23 * fInv, m._24 * fInv,
		m._31 * fInv, m._32 * fInv, m._33 * fInv, m._34 * fInv,
		m._41 * fInv, m._42 * fInv, m._43 * fInv, m._44 * fInv);
}







////////////////////////////////////////////////////////////////////////////////////
//    Constants
////////////////////////////////////////////////////////////////////////////////////

const Vec3 Vec3::Zero = { 0.f, 0.f, 0.f };
const Vec3 Vec3::One = { 1.f, 1.f, 1.f };
const Vec3 Vec3::UnitX = { 1.f, 0.f, 0.f };
const Vec3 Vec3::UnitY = { 0.f, 1.f, 0.f };
const Vec3 Vec3::UnitZ = { 0.f, 0.f, 1.f };

const Vec4 Vec4::Zero = { 0.f, 0.f, 0.f, 0.f };
const Vec4 Vec4::One = { 1.f, 1.f, 1.f, 1.f };
const Vec4 Vec4::UnitX = { 1.f, 0.f, 0.f, 0.f };
const Vec4 Vec4::UnitY = { 0.f, 1.f, 0.f, 0.f };
const Vec4 Vec4::UnitZ = { 0.f, 0.f, 1.f, 0.f };
const Vec4 Vec4::UnitW = { 0.f, 0.f, 0.f, 1.f };

const Matrix Matrix::Identity = { 1.f, 0.f, 0.f, 0.f,
								  0.f, 1.f, 0.f, 0.f,
								  0.f, 0.f, 1.f, 0.f,
								  0.f, 0.f, 0.f, 1.f };




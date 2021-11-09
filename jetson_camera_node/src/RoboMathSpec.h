#pragma once
#include <memory.h>
#include <cmath>
#include <string>
#include <iostream>
//#include "common.h"


float constexpr operator ""_deg(long double d) {
	return (float)(d * 3.141592654 / 180.0);
}

float constexpr operator ""_deg(unsigned long long int d) {
	return (float)d * 3.141592654f / 180.0f;
}

namespace RoboMathSpec
{

	class Matrix;

	////////////////////////////////////////////////////////////////////////////////////
	//  Vec3
	////////////////////////////////////////////////////////////////////////////////////

	class Vec3
	{
		public:
			float x;
			float y;
			float z;

			constexpr Vec3() : x(0.f), y(0.f), z(0.f) {};
			constexpr Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
			//explicit Vec3(_In_reads_(3) const float* pf) : x(pf[0]), y(pf[1]), z(pf[2]) {};
			constexpr Vec3(const Vec3& copy) = default;

			// Casting
			constexpr inline operator float* () { return (float *)&x; };
			constexpr inline operator const float* () const { return (float *)&x; };

			// Comparison operators
			constexpr inline bool operator == (const Vec3& v) const { return x == v.x && y == v.y && z == v.z; };
			constexpr inline bool operator != (const Vec3& v) const { return x != v.x || y != v.y || z != v.z; };

			// Assignment operators
			constexpr inline Vec3& operator=(const Vec3& copy) = default;
			constexpr inline Vec3& operator += (const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; };
			constexpr inline Vec3& operator -= (const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; };
			constexpr inline Vec3& operator *= (float f) { x *= f;   y *= f;   z *= f; return *this; };
			constexpr inline Vec3& operator /= (float f) { float fInv = 1.0f / f; x *= fInv; y *= fInv; z *= fInv; return *this; };

			// Unary operators
			constexpr inline Vec3 operator + () const { return *this; };
			constexpr inline Vec3 operator - () const { return Vec3(-x, -y, -z); };

			// Functions
			float Length() const { return sqrt(x*x + y*y + z*z); };
			float LengthSquared() const { return (x*x + y*y + z*z); };
			void Normalize();
			void TransformNormal(const Matrix& m);
			void TransformCoord(const Matrix& m);

			// Static functions
			static void Normalize(Vec3& out, const Vec3& a) { out = a; out.Normalize(); };
			static Vec3 Normalize(const Vec3& a) { Vec3 out = a; out.Normalize(); return out; };
			static float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
			static void Cross(Vec3& out, const Vec3& a, const Vec3& b) { out.x = a.y * b.z - a.z * b.y; out.y = a.z * b.x - a.x * b.z; out.z = a.x * b.y - a.y * b.x; };
			static Vec3 Cross(const Vec3& a, const Vec3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; };
			static void TransformNormal(Vec3& out, const Vec3& v, const Matrix& m);
			static Vec3 TransformNormal(const Vec3& v, const Matrix& m);
			static void TransformCoord(Vec3& out, const Vec3& v, const Matrix& m);
			static Vec3 TransformCoord(const Vec3& v, const Matrix& m);
			//static void Lerp(Vec3& out, const Vec3& a, const Vec3& b, float s);
			//static Vec3 Lerp(const Vec3& a, const Vec3& b, float s);

			// Constants
			static const Vec3 Zero;
			static const Vec3 One;
			static const Vec3 UnitX;
			static const Vec3 UnitY;
			static const Vec3 UnitZ;
	};

	// Binary operators
	inline constexpr Vec3 operator + (const Vec3& v1, const Vec3& v2) { return Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z); };
	inline constexpr Vec3 operator - (const Vec3& v1, const Vec3& v2) { return Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z); };
	inline constexpr Vec3 operator * (const Vec3& v, float f) { return Vec3(v.x * f, v.y * f, v.z * f); };
	inline constexpr Vec3 operator / (const Vec3& v, float f) { float fInv = 1.0f / f; return Vec3(v.x * fInv, v.y * fInv, v.z * fInv); };
	inline constexpr Vec3 operator * (float f, const Vec3& v) { return Vec3(f * v.x, f * v.y, f * v.z); };


	////////////////////////////////////////////////////////////////////////////////////
	//  Vec4
	////////////////////////////////////////////////////////////////////////////////////

	class Vec4
	{
		public:
			float x;
			float y;
			float z;
			float w;

			constexpr Vec4() : x(0.f), y(0.f), z(0.f), w(0.f) {};
			constexpr Vec4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {};
			//explicit Vec4(_In_reads_(4) const float *pf) : x(pf[0]), y(pf[1]), z(pf[2]), w(pf[3]){}
			constexpr Vec4(const Vec4& copy) = default;

			// Casting
			inline constexpr operator float* () { return (float *)&x; };
			inline constexpr operator const float* () const { return (float *)&x; };
			inline constexpr operator Vec3() const { return Vec3(x, y, z); }

			// Comparison operators
			inline constexpr bool operator == (const Vec4& v) const { return x == v.x && y == v.y && z == v.z && w == v.w; };
			inline constexpr bool operator != (const Vec4& v) const { return x != v.x || y != v.y || z != v.z || w != v.w; };

			// Assignment operators
			inline constexpr Vec4& operator=(const Vec4& copy) = default;
			//inline Vec4& operator=(Vec4&& move) = default;
			inline constexpr Vec4& operator += (const Vec4& v) { x += v.x; y += v.y; z += v.z; w += v.w; return *this; };
			inline constexpr Vec4& operator -= (const Vec4& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this; };
			inline constexpr Vec4& operator *= (float f) { x *= f;   y *= f;   z *= f;  w *= f; return *this; };
			inline constexpr Vec4& operator /= (float f) { float fInv = 1.0f / f; x *= fInv; y *= fInv; z *= fInv; w *= fInv;  return *this; };

			// Unary operators
			inline constexpr Vec4 operator + () const { return *this; };
			inline constexpr Vec4 operator - () const { return Vec4(-x, -y, -z, -w); };

			// Functions
			float Length() const { return sqrt(x*x + y*y + z*z + w*w); };
			constexpr float LengthSquared() const { return (x*x + y*y + z*z + w*w); };
			void Normalize();
			void Transform(const Matrix& m);

			// Static functions
			static void Normalize(Vec4& out, const Vec4& a) { out = a; out.Normalize(); };
			static Vec4 Normalize(const Vec4& a) { Vec4 out = a; out.Normalize(); return out; };
			constexpr static float Dot(const Vec4& a, const Vec4& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; };
			static void Transform(Vec4& out, const Vec4& v, const Matrix& m);
			static Vec4 Transform(const Vec4& v, const Matrix& m);

			// Constants
			static const Vec4 Zero;
			static const Vec4 One;
			static const Vec4 UnitX;
			static const Vec4 UnitY;
			static const Vec4 UnitZ;
			static const Vec4 UnitW;
	};

	// Binary operators
	inline constexpr Vec4 operator + (const Vec4& v1, const Vec4& v2) { return Vec4(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w); };
	inline constexpr Vec4 operator - (const Vec4& v1, const Vec4& v2) { return Vec4(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w); };
	inline constexpr Vec4 operator * (const Vec4& v, float f) { return Vec4(v.x * f, v.y * f, v.z * f, v.w * f); };
	inline constexpr Vec4 operator / (const Vec4& v, float f) { float fInv = 1.0f / f; return Vec4(v.x * fInv, v.y * fInv, v.z * fInv, v.w * fInv); };
	inline constexpr Vec4 operator * (float f, const Vec4& v) { return Vec4(f * v.x, f * v.y, f * v.z, f * v.w); };



	////////////////////////////////////////////////////////////////////////////////////
	//  Matrix
	////////////////////////////////////////////////////////////////////////////////////

	class Matrix
	{
		public:
			union {
				struct {
					float _11, _12, _13, _14;
					float _21, _22, _23, _24;
					float _31, _32, _33, _34;
					float _41, _42, _43, _44;
				};
				float m[4][4];
			};

			constexpr Matrix() : _11(0), _12(0), _13(0), _14(0),
				                 _21(0), _22(0), _23(0), _24(0),
				                 _31(0), _32(0), _33(0), _34(0),
				                 _41(0), _42(0), _43(0), _44(0) {};
			constexpr Matrix(float identityOrScale) : _11(identityOrScale), _12(0), _13(0), _14(0),
				                                      _21(0), _22(identityOrScale), _23(0), _24(0),
				                                      _31(0), _32(0), _33(identityOrScale), _34(0),
				                                      _41(0), _42(0), _43(0), _44(identityOrScale) {};
			constexpr Matrix(float m00, float m01, float m02, float m03,
				float m10, float m11, float m12, float m13,
				float m20, float m21, float m22, float m23,
				float m30, float m31, float m32, float m33) : _11(m00), _12(m01), _13(m02), _14(m03),
				                                              _21(m10), _22(m11), _23(m12), _24(m13),
				                                              _31(m20), _32(m21), _33(m22), _34(m23),
				                                              _41(m30), _42(m31), _43(m32), _44(m33) {};
			constexpr Matrix(const Matrix& copy) = default;
			constexpr Matrix(Matrix&& move) = default;
			//constexpr explicit Matrix(const Vec4& r0, const Vec4& r1, const Vec4& r2, const Vec4& r3) : XMFLOAT4X4(r0.x, r0.y, r0.z, r0.w, r1.x, r1.y, r1.z, r1.w, r2.x, r2.y, r2.z, r2.w, r3.x, r3.y, r3.z, r3.w) {};
			//explicit Matrix(_In_reads_(16) const float *pArray) : XMFLOAT4X4(pArray) {}
			constexpr explicit Matrix(const Vec3& translation) : _11(1.f), _12(0), _13(0), _14(0),
				                                                 _21(0), _22(1.f), _23(0), _24(0),
				                                                 _31(0), _32(0), _33(1.f), _34(0),
				                                                 _41(translation.x), _42(translation.y), _43(translation.z), _44(1.f) {};
			constexpr explicit Matrix(float translationX, float translationY, float translationZ) : _11(1.f), _12(0), _13(0), _14(0),
				                                                                                    _21(0), _22(1.f), _23(0), _24(0),
				                                                                                    _31(0), _32(0), _33(1.f), _34(0),
				                                                                                    _41(translationX), _42(translationY), _43(translationZ), _44(1.f) {};

			// Comparison operators
			bool operator == (const Matrix& M) const { return 0 == memcmp(this, &M, sizeof(Matrix)); };
			bool operator != (const Matrix& M) const { return 0 != memcmp(this, &M, sizeof(Matrix)); };

			// Assignment operators
			constexpr Matrix& operator=(const Matrix& copy) = default;
			constexpr Matrix& operator=(Matrix&& move) = default;
			constexpr Matrix& operator+= (const Matrix& M);
			constexpr Matrix& operator-= (const Matrix& M);
			constexpr Matrix& operator*= (float S);
			constexpr Matrix& operator/= (float S);

			// Unary operators
			inline constexpr Matrix operator + () const { return *this; };
			inline constexpr Matrix operator - () const { return Matrix(-_11, -_12, -_13, -_14, -_21, -_22, -_23, -_24, -_31, -_32, -_33, -_34, -_41, -_42, -_43, -_44); };

			// Get special vectors
			constexpr Vec3 GetPosition() const { return { _41, _42, _43 }; };
			constexpr void GetPosition(Vec3& outVec) const { outVec.x = _41; outVec.y = _42; outVec.z = _43; };
			constexpr Vec3 GetUnitX() const { return { _11, _12, _13 }; };
			constexpr void GetUnitX(Vec3& outVec) const { outVec.x = _11; outVec.y = _12; outVec.z = _13; };
			constexpr Vec3 GetUnitY() const { return { _21, _22, _23 }; };
			constexpr void GetUnitY(Vec3& outVec) const { outVec.x = _21; outVec.y = _22; outVec.z = _23; };
			constexpr Vec3 GetUnitZ() const { return { _31, _32, _33 }; };
			constexpr void GetUnitZ(Vec3& outVec) const { outVec.x = _31; outVec.y = _32; outVec.z = _33; };

			// Functions
			void Transpose();
			bool Invert();

			inline constexpr void FillZeros() { _11 = 0; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = 0; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = 0; _34 = 0; _41 = 0; _42 = 0; _43 = 0; _44 = 0; };
			inline constexpr void FillIdentity() { _11 = 1.f; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = 1.f; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = 1.f; _34 = 0; _41 = 0; _42 = 0; _43 = 0; _44 = 1.f; };

			inline constexpr void FillTranslation(const Vec3& v)             { _11 = 1.f; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = 1.f; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = 1.f; _34 = 0; _41 = v.x; _42 = v.y; _43 = v.z; _44 = 1.f; };
			inline constexpr void FillTranslation(float x, float y, float z) { _11 = 1.f; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = 1.f; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = 1.f; _34 = 0; _41 = x, _42 = y, _43 = z; _44 = 1.0f; };

			inline constexpr void FillScale(const Vec3& s)                { _11 = s.x; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = s.y; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = s.z; _34 = 0; _41 = 0; _42 = 0; _43 = 0; _44 = 1.f; };
			inline constexpr void FillScale(float sx, float sy, float sz) { _11 = sx; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = sy; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = sz; _34 = 0; _41 = 0; _42 = 0; _43 = 0; _44 = 1.f; };
			inline constexpr void FillScale(float s)                      { _11 = s; _12 = 0; _13 = 0; _14 = 0; _21 = 0; _22 = s; _23 = 0; _24 = 0; _31 = 0; _32 = 0; _33 = s; _34 = 0; _41 = 0; _42 = 0; _43 = 0; _44 = 1.f; };

			void FillRotationX(float angle);
			void FillRotationY(float angle);
			void FillRotationZ(float angle);
			void FillRotationX(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			void FillRotationY(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			void FillRotationZ(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			void FillRotationAxis(const Vec3& axis, float angle);
			//void FillRotationYawPitchRoll(float yaw, float pitch, float roll);

			void FillPerspectiveFovLH(float fov, float aspectRatio, float nearPlane, float farPlane);
			void FillPerspectiveFovRH(float fov, float aspectRatio, float nearPlane, float farPlane);
			void FillOrthographicLH(float width, float height, float zNearPlane, float zFarPlane);
			void FillOrthographicRH(float width, float height, float zNearPlane, float zFarPlane);
			void FillLookAtLH(const Vec3& eye, const Vec3& target, const Vec3& up);
			void FillLookAtRH(const Vec3& eye, const Vec3& target, const Vec3& up);

			void SetPosition(const Vec3& pos) { _41 = pos.x; _42 = pos.y; _43 = pos.z; };
			void SetPositionT(const Vec3& pos) { _14 = pos.x; _24 = pos.y; _34 = pos.z; };
			bool SaveToFile(const std::string& name);
			bool LoadFromFile(const std::string& name);

			void Print()
			{
				//for (int i = 0; i < 4; i++) {
				//	for (int j = 0; j < 4; j++) {

				//	}
				//}

				std::cout << 
					m[0][0] << "," << m[0][1] << "," << m[0][2] << "," << m[0][3] << "," <<std::endl<<
					m[1][0] << "," << m[1][1] << "," << m[1][2] << "," << m[1][3] << "," <<std::endl<<
					m[2][0] << "," << m[2][1] << "," << m[2][2] << "," << m[2][3] << "," <<std::endl<<
					m[3][0] << "," << m[3][1] << "," << m[3][2] << "," << m[3][3] << "," << std::endl;

				std::cout << std::endl;

			}
			

			// Static function
			static void Multiply(Matrix& out, const Matrix& a, const Matrix& b);
			static void Transpose(Matrix& out, const Matrix& m);
			static void Invert(Matrix& out, const Matrix& m);
			static Matrix Multiply(const Matrix& a, const Matrix& b);
			static Matrix Transpose(const Matrix& m);
			static Matrix Invert(const Matrix& m);

			inline constexpr static Matrix BuildTranslation(const Vec3& v)             { return { 1.0f, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 1.0f, 0, v.x, v.y, v.z, 1.0f }; };
			inline constexpr static Matrix BuildTranslation(float x, float y, float z) { return { 1.0f, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0, 1.0f, 0, x, y, z, 1.0f }; };

			inline constexpr static Matrix BuildScale(const Vec3& s) { return { s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, 0, 0, 0, 1.0f }; };
			inline constexpr static Matrix BuildScale(float sx, float sy, float sz) { return { sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1.0f }; };
			inline constexpr static Matrix BuildScale(float s) { return { s, 0, 0, 0, 0, s, 0, 0, 0, 0, s, 0, 0, 0, 0, 1.0f }; };

			static Matrix BuildRotationX(float angle);
			static Matrix BuildRotationY(float angle);
			static Matrix BuildRotationZ(float angle);
			static Matrix BuildRotationX(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			static Matrix BuildRotationY(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			static Matrix BuildRotationZ(float angle, const Vec3& translation); // rotace n\E1sledovan\E1 translac\ED (zrychlen\E9 vytvo\F8en\ED oproti n\E1soben\ED matic)
			static Matrix BuildRotationAxis(const Vec3& axis, float angle);
			static Matrix BuildRotationYawPitchRoll(float yaw, float pitch, float roll);

			static Matrix BuildPerspectiveFovLH(float fov, float aspectRatio, float nearPlane, float farPlane);
			static Matrix BuildPerspectiveFovRH(float fov, float aspectRatio, float nearPlane, float farPlane);
			static Matrix BuildOrthographicLH(float width, float height, float zNearPlane, float zFarPlane);
			static Matrix BuildOrthographicRH(float width, float height, float zNearPlane, float zFarPlane);
			static Matrix BuildLookAtLH(const Vec3& eye, const Vec3& target, const Vec3& up);
			static Matrix BuildLookAtRH(const Vec3& eye, const Vec3& target, const Vec3& up);

			// Constants
			static const Matrix Identity;
		private:
			static float Determinant(const Matrix& m, int n);
			static void GetCfactor(const Matrix& M, Matrix& t, int p, int q, int n);
			static void Adj(const Matrix& M, Matrix& adj);

	};

	Matrix operator * (const Matrix& m1, const Matrix& m2);
	constexpr Matrix operator + (const Matrix& m1, const Matrix& m2);
	constexpr Matrix operator - (const Matrix& m1, const Matrix& m2);
	constexpr Matrix operator * (const Matrix& m, float f);
	constexpr Matrix operator / (const Matrix& m, float f);


	////////////////////////////////////////////////////////////////////////////////////
	//  Plane
	////////////////////////////////////////////////////////////////////////////////////

	/*class Plane : public XMFLOAT4
	{
		public:
		constexpr Plane() : XMFLOAT4(0.f, 0.f, 0.f, 0.f) {};
		constexpr Plane(float _x, float _y, float _z, float _w) : XMFLOAT4(_x, _y, _z, _w) {};
		constexpr Plane(const Vec3& normal, float d) : XMFLOAT4(normal.x, normal.y, normal.z, d) {};
		Plane(const Vec3& point1, const Vec3& point2, const Vec3& point3);
		Plane(const Vec3& point, const Vec3& normal);
		constexpr explicit Plane(const Vec4& v) : XMFLOAT4(v.x, v.y, v.z, v.w) {};
		explicit Plane(_In_reads_(4) const float *pf) : XMFLOAT4(pf) {}
		Plane(FXMVECTOR V) { XMStoreFloat4(this, V); };
		Plane(const XMFLOAT4& V) { this->x = V.x; this->y = V.y; this->z = V.z; this->w = V.w; };

		// Casting
		inline operator XMVECTOR() const { return XMLoadFloat4(this); }
		inline constexpr operator float* () { return (float *)&x; };
		inline constexpr operator const float* () const { return (float *)&x; };

		// Comparison operators
		inline constexpr bool operator == (const Plane& v) const { return x == v.x && y == v.y && z == v.z && w == v.w; };
		inline constexpr bool operator != (const Plane& v) const { return x != v.x || y != v.y || z != v.z || w != v.w; };

		// Assignment operators
		inline constexpr Plane& operator= (const Plane& v) { x = v.x; y = v.y; z = v.z; w = v.w; return *this; }
		inline constexpr Plane& operator= (const XMFLOAT4& v) { x = v.x; y = v.y; z = v.z; w = v.w; return *this; }

		// Properties
		inline constexpr Vec3 GetNormal() const { return Vec3(x, y, z); }
		inline constexpr void SetNormal(const Vec3& normal) { x = normal.x; y = normal.y; z = normal.z; }
		inline constexpr float GetD() const { return w; }
		inline constexpr void SetD(float d) { w = d; }

		// Functions
		void Normalize();
		void Transform(const Matrix& m);

		float Dot(const Vec4& v) const;                  // Plane's relationship with a homogeneous coordinate (coordinate is on a particular plane, or on which side of a particular plane a particular coordinate lies?)
		float DotCoord(const Vec3& position) const;      // Finds the signed distance from a point to a plane
		float DotNormal(const Vec3& normal) const;       // Calculates the angle between the normal vector of the plane and another normal vector

		float GetDistanceToPoint(const Vec3& point) const { return DotCoord(point); };
		Vec3 GetIntersectionWithLine(const Vec3& point1, const Vec3& point2) const;
		void GetIntersectionWithPlane(const Plane& otherPlane, Vec3& outPoint1, Vec3& outPoint2) const;

		// Static functions
		static void Transform(Plane& out, const Plane& p, const Matrix& m);

	};*/



	////////////////////////////////////////////////////////////////////////////////////
	//  Color
	////////////////////////////////////////////////////////////////////////////////////

	/*class Color
	{
		public:
		float r;
		float g;
		float b;
		float a;

		Color() : r(0.f), g(0.f), b(0.f), a(0.f) {}
		Color(float _r, float _g, float _b, float _a) : r(_r), g(_g), b(_b), a(_a) {}
		Color(float _r, float _g, float _b) : r(_r), g(_g), b(_b), a(1.f) {}
		Color(float _rgb, float _a) : r(_rgb), g(_rgb), b(_rgb), a(_a) {}
		explicit Color(float _rgb) : r(_rgb), g(_rgb), b(_rgb), a(1.0f) {}
		Color(DWORD argb);
		explicit Color(const Vec3& c) : r(c.x), g(c.y), b(c.z), a(1.f) {}
		explicit Color(const Vec4& c) : r(c.x), g(c.y), b(c.z), a(c.w) {}
		explicit Color(_In_reads_(4) const float *pf) : r(pf[0]), g(pf[1]), b(pf[2]), a(pf[3]) {}
		Color(FXMVECTOR V) { XMStoreFloat4((XMFLOAT4*)this, V); }
		Color(const XMFLOAT4& c) : r(c.x), g(c.y), b(c.z), a(c.w) {}
		Color(const Color& copy) = default;
		Color(Color&& move) = default;

		operator DWORD() const;
		operator XMVECTOR() const { return XMLoadFloat4((XMFLOAT4*)this); }
		operator XMFLOAT4() const { return *((XMFLOAT4*)this); }
		operator Vec3() const { return Vec3(r, g, b); }
		operator const float*() const { return reinterpret_cast<const float*>(this); }

		// Comparison operators
		inline bool operator == (const Color& v) const { return r == v.r && g == v.g && b == v.b && a == v.a; };
		inline bool operator != (const Color& v) const { return r != v.r || g != v.g || b != v.b || a != v.a; };

		// Assignment operators
		Color& operator=(const Color& copy) = default;
		Color& operator=(Color&& move) = default;
		Color& operator= (const XMFLOAT4& c) { r = c.x; g = c.y; b = c.z; a = c.w; return *this; }
		Color& operator= (const XMVECTORF32& F) { r = F.f[0]; g = F.f[1]; b = F.f[2]; a = F.f[3]; return *this; }
		Color& operator+= (const Color& c) { r += c.r; g += c.g; b += c.b; a += c.a; return *this; };
		Color& operator-= (const Color& c) { r -= c.r; g -= c.g; b -= c.b; a -= c.a; return *this; };
		Color& operator*= (const Color& c) { r *= c.r; g *= c.g; b *= c.b; a *= c.a; return *this; };
		Color& operator*= (float s) { r *= s; g *= s; b *= s; a *= s; return *this; };
		Color& operator/= (const Color& c) { r /= c.r; g /= c.g; b /= c.b; a /= c.a; return *this; };

		// Unary operators
		Color operator+ () const { return *this; }
		Color operator- () const { return Color(-r, -g, -b, -a); };

		// Functions
		BYTE GetByteR() const { return r >= 1.0f ? 0xff : r <= 0.0f ? 0x00 : (BYTE)(r * 255.0f + 0.5f); };
		BYTE GetByteG() const { return g >= 1.0f ? 0xff : g <= 0.0f ? 0x00 : (BYTE)(g * 255.0f + 0.5f); };
		BYTE GetByteB() const { return b >= 1.0f ? 0xff : b <= 0.0f ? 0x00 : (BYTE)(b * 255.0f + 0.5f); };
		void Saturate();
		void AdjustSaturation(float sat); // returns gray-scale color for sat = 0 and original color for sat = 1

		// Static functions
		static void Modulate(Color& out, const Color& c1, const Color& c2);
		static void Lerp(Color& out, const Color& c1, const Color& c2, float s);
		static void Lerp3(Color& out, const Color& c1, const Color& c2, const Color& c3, float s);
		static Color Modulate(const Color& c1, const Color& c2);
		static Color Lerp(const Color& c1, const Color& c2, float s);
		static Color Lerp3(const Color& c1, const Color& c2, const Color& c3, float s);

		// Constants
		static const Color Empty;
		static const Color Black;
		static const Color White;
		static const Color Red;
		static const Color Green;
		static const Color Blue;
		static const Color Yellow;
		static const Color Cyan;
		static const Color Magenta;
		static const Color Grey25;
		static const Color Grey50;
		static const Color Grey75;
	};

	// Binary operators
	inline Color operator + (const Color& c1, const Color& c2) { return Color(c1.r + c2.r, c1.g + c2.g, c1.b + c2.b, c1.a + c2.a); };
	inline Color operator - (const Color& c1, const Color& c2) { return Color(c1.r - c2.r, c1.g - c2.g, c1.b - c2.b, c1.a - c2.a); };
	inline Color operator * (const Color& c1, const Color& c2) { return Color(c1.r * c2.r, c1.g * c2.g, c1.b * c2.b, c1.a * c2.a); };
	inline Color operator * (const Color& c, float f) { return Color(c.r * f, c.g * f, c.b * f, c.a * f); };
	inline Color operator / (const Color& c, float f) { float fInv = 1.0f / f; return Color(c.r * fInv, c.g * fInv, c.b * fInv, c.a * fInv); };
	*/

}

#pragma once
#include <vector>
#include <algorithm>
//#include "RoboMathGL.h"
#include "Math.h"

using std::vector;




class Filter
{
public:
	struct SS{
		Vec3 pos;
		Vec3 ori;
		float dist;
	};
	static float Distance(const Vec3& pointA, const Vec3& pointB) {
		Vec3 diff = pointA - pointB;
		return diff.Length();
	}

	static void Sort(vector<SS>& vect) {
		std::sort(vect.begin(), vect.end(), [](const SS& ss1, const SS& ss2)
			{
				return ss1.dist < ss2.dist;
			});
	}

	static SS Estimate(const vector<SS>& vect) {
		Vec3 pos;
		Vec3 ori;
		int cull = 0;
		if (vect.size() > 2)
			cull = 1;
		for (int i = 0; i < vect.size()- cull; i++) {

			pos += vect[i].pos;
			ori += vect[i].ori;
		}
		pos /= (vect.size() - cull);
		ori /= (vect.size() - cull);


		SS result;
		result.pos = pos;
		result.ori = ori;
		return result;
	}

	static void CheckDir(vector<SS>& vect) {
		int dirX = 0;
		int dirY = 0;
		int dirZ = 0;
		int X = 1;
		int Y = 1;
		int Z = 1;


		for (auto& ss : vect)
		{
			if (ss.ori.x < 0)
				dirX--;
			else
				dirX++;
			if (ss.ori.y < 0)
				dirY--;
			else
				dirY++;
			if (ss.ori.z < 0)
				dirZ--;
			else
				dirZ++;
		}

		if (dirX < 0)
			X = -1;
		if (dirY < 0)
			Y = -1;
		if (dirZ < 0)
			Z = -1;

		for (auto& ss : vect)
		{
			ss.ori.x = X * abs(ss.ori.x);
			ss.ori.y = Y * abs(ss.ori.y);
			ss.ori.z = Z * abs(ss.ori.z);
		}
	}


};


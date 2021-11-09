#pragma once
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "Math.h"
#include <cassert>
#include "RSCamera.h"

using std::vector;
using std::unordered_set;
using std::unordered_map;

struct BoundingBox {
	Vec3 min;
	Vec3 max;
};

struct Int3
{
	int x = 0;
	int y = 0;
	int z = 0;

	Int3() {};
	Int3(int _x, int _y, int _z) : x(_x), y(_y), z(_z)
	{
		assert(_x < 512 && _x> -512);
		assert(_y < 512 && _y> -512);
		assert(_z < 512 && _z> -512);
	}
	inline bool operator ==(const Int3& other) { return x == other.x && y == other.y && z == other.z; };
	inline operator uint32_t () const
	{
		return Hash();
	};

	inline uint32_t Hash() const
	{
		return (uint32_t)(x + 512) + ((uint32_t)(y + 512) << 10) + ((uint32_t)(z + 512) << 20);
	}

	inline void UnHash(uint32_t hash_code)
	{
		x = (hash_code & 0b1111111111) - 512;
		y = ((hash_code & 0b11111111110000000000) >> 10) - 512;
		z = ((hash_code & 0b111111111100000000000000000000) >> 20) - 512;
	}

};

struct Filter
{
	int count = 1;
	float distance = 0;
};


class OctoMap
{
public:
	static void AlignToMap(const vector<Vec3>& input, unordered_set<uint32_t>& output, float voxelSize)
	{
		for (auto& voxel : input)
		{
			Int3 indexedVoxel(GetIndexOfMap(voxel.x, voxelSize), GetIndexOfMap(voxel.y, voxelSize), GetIndexOfMap(voxel.z, voxelSize));

			output.insert(indexedVoxel);
		}
	}

	static void AlignToMap2(const vector<Vec3>& input, unordered_set<uint32_t>& output, float voxelSize, int threshold = 5)
	{
		unordered_map<uint32_t, int> numerator;
		for (auto& voxel : input)
		{
			Int3 indexedVoxel(GetIndexOfMap(voxel.x, voxelSize), GetIndexOfMap(voxel.y, voxelSize), GetIndexOfMap(voxel.z, voxelSize));


			auto result = numerator.insert({ indexedVoxel, 1 });
			if (!result.second)
				result.first->second++;


			/*auto iter = numerator.find(indexedVoxel);
			if (iter != numerator.end()) {
				iter->second++;
			}
			else {
				numerator.insert({ indexedVoxel, 1 });
			}*/
		}
		for (auto& voxel : numerator)
		{
			if (voxel.second >= threshold)
				output.insert((uint32_t)voxel.first);
		}

	}

	static void AlignToMap3(const vector<Vec3>& input, unordered_set<uint32_t>& output, float voxelSize, float threshold, Vec3 camPos, Vec3 camZDirr)
	{
		unordered_map<uint32_t, Filter> numerator;
		for (auto& voxel : input)
		{
			Int3 indexedVoxel(GetIndexOfMap(voxel.x, voxelSize), GetIndexOfMap(voxel.y, voxelSize), GetIndexOfMap(voxel.z, voxelSize));

			auto filter = Filter();
			auto result = numerator.insert({ indexedVoxel, filter });
			if (!result.second)
				result.first->second.count++;
			else
			{
				Vec3 voxelPos(indexedVoxel.x * voxelSize, indexedVoxel.y * voxelSize, indexedVoxel.z * voxelSize);
				Vec3 diff = voxelPos - camPos;
				result.first->second.distance = Vec3::Dot(diff, camZDirr);
			}


			/*auto iter = numerator.find(indexedVoxel);
			if (iter != numerator.end()) {
				iter->second++;
			}
			else {
				numerator.insert({ indexedVoxel, 1 });
			}*/
		}
		float hFov = RSCamera::GetHFov();
		float vFov = RSCamera::GetHFov();
		auto cam_params = RSCamera::GetDepthDesc();

		int resH = cam_params.resH;
		int resW = cam_params.resW;

		float deltahFov = hFov / resW;
		float deltavFov = hFov / resW;
		for (auto& voxel : numerator)
		{

			
			float alfa = atan(voxelSize/voxel.second.distance);

			int cout_per_row = int( alfa / deltahFov);
			int cout_per_column= int( alfa / deltavFov);
			int count_per_voxel = cout_per_row * cout_per_column;


			//std::cout << "count per voxel" << count_per_voxel<< std::endl;

			if (voxel.second.count >= count_per_voxel * threshold)
				output.insert((uint32_t)voxel.first);
		}

	}

	static void TransformAndCheckWithBoundingBox(vector<Vec3>& input, vector<Vec3>& output, const BoundingBox box, const Matrix m) {
		for (auto& point : input)
		{
			point.TransformCoord(m);
			if (point.x < box.max.x && point.x > box.min.x && point.y < box.max.y && point.y > box.min.y && point.z > box.min.z)
				output.push_back(point);
		}
	}

private:
	static int GetIndexOfMap(float value, float voxelSize)
	{
		return floor(value / voxelSize + 0.5 * voxelSize);
	}

};

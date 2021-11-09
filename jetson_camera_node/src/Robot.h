#pragma once
#include "Object.h"
#include <vector>

using std::vector;

struct DHParam
{
	float d;
	float a;
	float fi;
	float alfa;

	DHParam(float _d, float _a, float _fi, float _alfa) { d = _d; a = _a; fi = _fi; alfa = _alfa; }

};

class Robot
{

public:
	void Init();
	void BuilMat(Matrix& source, float d, float a, float fi, float alfa);
	void BuilMat(Matrix& source, DHParam dh, float actual);

	vector<DHParam> dh;
	vector<Object> joints;
	vector<Matrix> source_mat;



};


#pragma once
#include "Vec3.h"
#include <vector>
#include <math.h>

using namespace std;

class Triangle
{
public:
	int point[3];
	vec3 minPos;
	vec3 maxPos;

public:
	Triangle()
	{}

	Triangle(int index0, int index1, int index2)
	{
		point[0] = index0;
		point[1] = index1;
		point[2] = index2;
	}

	void UpdateBoundPos(vector<vec3> _pos1)
	{
		vec3 pos0 = _pos1[point[0]];
		vec3 pos1 = _pos1[point[1]];
		vec3 pos2 = _pos1[point[2]];

		double minX = (pos0.x() < pos1.x()) ? (pos0.x() < pos2.x() ? pos0.x() : pos2.x()) : (pos1.x() < pos2.x() ? pos1.x() : pos2.x());
		double minY = (pos0.y() < pos1.y()) ? (pos0.y() < pos2.y() ? pos0.y() : pos2.y()) : (pos1.y() < pos2.y() ? pos1.y() : pos2.y());
		double minZ = (pos0.z() < pos1.z()) ? (pos0.z() < pos2.z() ? pos0.z() : pos2.z()) : (pos1.z() < pos2.z() ? pos1.z() : pos2.z());

		double maxX = (pos0.x() > pos1.x()) ? (pos0.x() > pos2.x() ? pos0.x() : pos2.x()) : (pos1.x() > pos2.x() ? pos1.x() : pos2.x());
		double maxY = (pos0.y() > pos1.y()) ? (pos0.y() > pos2.y() ? pos0.y() : pos2.y()) : (pos1.y() > pos2.y() ? pos1.y() : pos2.y());
		double maxZ = (pos0.z() > pos1.z()) ? (pos0.z() > pos2.z() ? pos0.z() : pos2.z()) : (pos1.z() > pos2.z() ? pos1.z() : pos2.z());

		minPos = vec3(minX, minY, minZ);
		maxPos = vec3(maxX, maxY, maxZ);
	}
};
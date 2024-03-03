#pragma once
#include "Vec3.h"
#include "Triangle.h"
#include <vector>
#include <math.h>

using namespace std;

class Hash
{
public:
	double spacing;
	int tableSize;
	int querySize;
	int TquerySize;
	int numParticles;
	int numOfTri;

	vector<int> cellStart; //cell Index ��ŸƮ �迭
	vector<int> cellEntries; //��ƼŬ �迭
	vector<int> queryIds; //���� ���� �迭
	vector<int> firstAdjId; //queryAll �Լ��� ��ŸƮ �迭 (Dense Grid Representation)
	vector<int> adjIds; //queryAll �Լ��� ���� ���� �迭

	vector<int> TqueryIds; //���� ���� �迭
	vector<int> TfirstAdjId; //queryAll �Լ��� ��ŸƮ �迭 (Dense Grid Representation)
	vector<int> TadjIds; //queryAll �Լ��� ���� ���� �迭

public:
	Hash();
	Hash(double _spacing, int _numParticles, int _numOfTri);
	~Hash();

public:
	int intCoord(double coord);
	int hashCoords(int xi, int yi, int zi);
	int hashPos(vec3 pos);
	void create(vector<vec3> _pos);
	void query(vector<vec3> _pos, int index, double maxDist);
	void queryAll(vector<vec3> _pos, double maxDist);
	void triQuery(vector<Triangle> _tri, int index);
	void triQueryAll(vector<Triangle> _tri);
};
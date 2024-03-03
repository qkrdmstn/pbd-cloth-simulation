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

	vector<int> cellStart; //cell Index 스타트 배열
	vector<int> cellEntries; //파티클 배열
	vector<int> queryIds; //인접 정점 배열
	vector<int> firstAdjId; //queryAll 함수의 스타트 배열 (Dense Grid Representation)
	vector<int> adjIds; //queryAll 함수의 인접 정점 배열

	vector<int> TqueryIds; //인접 정점 배열
	vector<int> TfirstAdjId; //queryAll 함수의 스타트 배열 (Dense Grid Representation)
	vector<int> TadjIds; //queryAll 함수의 인접 정점 배열

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
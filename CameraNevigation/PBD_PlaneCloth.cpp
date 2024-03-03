#include "PBD_PlaneCloth.h"
#include "Hash.h"
#include "GL\glut.h"
#include <Windows.h>
#include <iostream>
#include <cmath>
using namespace std;
//#define BEND_DIHEDRAL_ANGLE 
#define SELF_COLLISION 0
//Collision Method
#define PROJECTED_GRADIENT_DESCENT 0
#define FRANK_WOLFE 1
//천 Init 형태
#define INIT_HORIZONTAL_CLOTH 1
#define INIT_VERTICAL_CLOTH 0

//sdf calculate
//#define SPHERE 
#define PLANE
//#define BOX

Hash* _hash;

PBD_PlaneCloth::PBD_PlaneCloth()
{
}

PBD_PlaneCloth::PBD_PlaneCloth(int width, int height, double spacing, double _thickness)
{
	_res[0] = width;
	_res[1] = height;

	int size = _res[0] * _res[1];
	numOfTri = 2 * (_res[0] - 1) * (_res[1] - 1);
	_restPos.resize(size);
	_pos.resize(size);
	_pos1.resize(size);
	_vel.resize(size);
	_invMass.resize(size);
	_bendingForce.resize(size);

	_hash = new Hash(spacing, size, numOfTri);
	thickness = _thickness;

	_tri.resize(numOfTri);
	InitTriangle();

	init();
	computeRestLength();
	computeDihedralAngle();
}

PBD_PlaneCloth::~PBD_PlaneCloth()
{
}

void PBD_PlaneCloth::computeDihedralAngle(void)
{
	// m_DihedralAngle
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			int index0 = (j + 1) * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + (i + 1);
			int index3 = j * _res[0] + i;

			auto p0 = _pos[index0];
			auto p1 = _pos[index1];
			auto p2 = _pos[index2];
			auto p3 = _pos[index3];

			vec3 e = p3 - p2;
			double length = e.length();
			if (length < 1e-6) {
				return;
			}
			double invlength = 1.0 / length;
			vec3 n1 = (p2 - p0).cross(p3 - p0);
			vec3 n2 = (p3 - p1).cross(p2 - p1);
			n1 /= n1.lengthSquared();
			n2 /= n2.lengthSquared();

			vec3 d0 = n1 * length;
			vec3 d1 = n2 * length;
			vec3 d2 = n1 * ((p0 - p3).dot(e) * invlength) + n2 * ((p1 - p3).dot(e) * invlength);
			vec3 d3 = n1 * ((p2 - p0).dot(e) * invlength) + n2 * ((p2 - p1).dot(e) * invlength);

			n1.normalize();
			n2.normalize();
			double dot = n1.dot(n2);

			if (dot < -1.0) {
				dot = -1.0;
			}
			if (dot > 1.0) {
				dot = 1.0;
			}
			double restAngle = acos(dot);
			_dihedralAngle.push_back(restAngle);
		}
	}
}

void PBD_PlaneCloth::computeRestLength(void)
{
	_structuralLength0Horiz = 2.0 / (double)_res[0];
	_structuralLength0Verti = 2.0 / (double)_res[1];
	_shearLength0Horiz = sqrt(2.0) * _structuralLength0Horiz;
	_shearLength0Verti = sqrt(2.0) * _structuralLength0Verti;
	_bendLength0Horiz = _structuralLength0Horiz * 2.0;
	_bendLength0Verti = _structuralLength0Verti * 2.0;
}

void PBD_PlaneCloth::init(void)
{
	for (int i = 0; i < _res[0]; i++) {
		for (int j = 0; j < _res[1]; j++) {
			int index = j * _res[0] + i;
			_invMass[index] = 1.0;

#if INIT_HORIZONTAL_CLOTH
			_pos[index].set(2.0 * i / (double)_res[0] + 0.5, 2.0, 2.0 * j / (double)_res[1]);
			_restPos[index].set(2.0 * i / (double)_res[0] + 0.5, 2.0, 2.0 * j / (double)_res[1]);
#endif

#if INIT_VERTICAL_CLOTH
			_pos[index].set(2.0 * i / (double)_res[0] + 0.5, 2.0 * j / (double)_res[1], 2.0);
			_restPos[index].set(2.0 * i / (double)_res[0] + 0.5, 2.0 * j / (double)_res[1], 2.0);
#endif

			_vel[index].clear();
		}
	}
}

void PBD_PlaneCloth::solveDistanceConstraint(int index0, int index1, double restlength)
{
	double c_p1p2 = (_pos1[index0] - _pos1[index1]).length() - restlength;
	vec3 dp1 = (_pos1[index0] - _pos1[index1]);
	vec3 dp2 = (_pos1[index0] - _pos1[index1]);
	dp1.normalize();
	dp2.normalize();
	dp1 *= -_invMass[index0] / (_invMass[index0] + _invMass[index1]) * c_p1p2;
	dp2 *= _invMass[index1] / (_invMass[index0] + _invMass[index1]) * c_p1p2;
	_pos1[index0] += dp1;
	_pos1[index1] += dp2;
}

void PBD_PlaneCloth::applyExtForces(double dt)
{
	vec3 gravity(0.0, -9.8, 0.0);
	double damping = 0.99;
	for (int i = 0; i < _res[0]; i++) {
		for (int j = 0; j < _res[1]; j++) {
			int index = j * _res[0] + i;
			_vel[index] += gravity * dt *  _invMass[index];
			_vel[index] *= damping;
			_pos1[index] = _pos[index] + (_vel[index] * dt);
		}
	}
}

void PBD_PlaneCloth::updateStructuralSprings(void)
{
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + i;
			solveDistanceConstraint(index0, index1, _structuralLength0Horiz);
		}
	}
	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			solveDistanceConstraint(index0, index1, _structuralLength0Verti);
		}
	}
}

void PBD_PlaneCloth::updateShearSprings(void)
{
	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + i;
			int index3 = j * _res[0] + (i + 1);
			solveDistanceConstraint(index0, index1, _shearLength0Horiz);
			solveDistanceConstraint(index2, index3, _shearLength0Verti);
		}
	}
}

void PBD_PlaneCloth::updateBendSprings(void)
{
#ifdef BEND_DIHEDRAL_ANGLE
	for (int i = 0; i < _bendingForce.size(); i++) {
		_bendingForce[i] = 0.0;
	}
	int id = 0;
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			int index0 = (j + 1) * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + (i + 1);
			int index3 = j * _res[0] + i;
			solveDihedralConstraint(index0, index1, index2, index3, _dihedralAngle[id++]);
		}
	}
#else
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 2; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 2) * _res[0] + i;
			solveDistanceConstraint(index0, index1, _bendLength0Horiz);
		}
	}
	for (int i = 0; i < _res[1] - 2; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 2);
			solveDistanceConstraint(index0, index1, _bendLength0Verti);
		}
	}
#endif
}

void PBD_PlaneCloth::integrate(double dt)
{
	
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index = j * _res[0] + i;

			////회전
			//if ((i == _res[1] - 1 && j == 0) || (i == _res[1] - 1 && j == _res[0] - 1) || (i == 0 && j == _res[0] - 1) || (i == 0 && j == 0))
			//{
			//	if ((i == 0 && j == 0) || (i == _res[1]-1 && j == 0))
			//	{
			//		double x1 = (_pos[index].x() - 1) * cos(0.3 * PI / 180) - (_pos[index].z() - 1) * sin(0.3 * PI / 180) + 1;
			//		double z1 = (_pos[index].x() - 1) * sin(0.3 * PI / 180) + (_pos[index].z() - 1) * cos(0.3 * PI / 180) + 1;
			//		_pos1[index] = vec3(x1, _pos[index].y(), z1);
			//		_vel[index] = (_pos1[index] - _pos[index]) / dt;
			//		_pos[index] = _pos1[index];
			//	}
			//	
			//	continue;
			//}
			
			_vel[index] = (_pos1[index] - _pos[index]) / dt;

			//double v = sqrt(_vel[index].lengthSquared()); //최대 속도 제한
			//double maxV = thickness / dt;
			//if (v > maxV)
			//	_vel[index] *= (maxV / v);

			_pos[index] = _pos1[index];
			//_pos[index] += _vel[index] * dt;
		}
	}
}

void PBD_PlaneCloth::simulation(double dt, int numSubStep)
{
	//SelfCollision
	double subDt = dt / numSubStep;
	double maxVelocity =  thickness / subDt;

#if SELF_COLLISION
	_hash->create(_pos1);
	double maxTravelDist = maxVelocity * dt;
	_hash->queryAll(_pos1, maxTravelDist);

	updateTriBoundBox(_pos1);
	_hash->triQueryAll(_tri);
#endif
	//SubStep
	for (int step = 0; step < numSubStep; step++)
	{
		applyExtForces(subDt);
		updateStructuralSprings();
		updateShearSprings();
		updateBendSprings();
#if SELF_COLLISION
		//updatePPSelfCollision(subDt);
		//updatePTSelfCollision(subDt);
		updateEESelfCollision(subDt);
#endif

		double magSum = 0;
		for (int i = 0; i < _res[1]; i++) {
			for (int j = 0; j < _res[0]; j++) {
				int index = j * _res[0] + i;

				magSum += (_pos[index] - _pos1[index]).getNorm();
			}
		}
		//printf("%f\n", magSum / (_res[1] * _res[0]));

		integrate(subDt);
		SdfCollision(subDt);

		//HistoryBasedCollision();
		//LevelSetCollision();
	}

	//applyExtForces(dt);

	////iterator
	//for (int i = 0; i < 5; i++)
	//{
	//	updateStructuralSprings();
	//	updateShearSprings();
	//	updateBendSprings();

	//}

	//integrate(dt);
	//SdfCollision(dt);
}

void PBD_PlaneCloth::computeWindForTriangle(vec3 wind, int index0, int index1, int index2)
{
	auto p0 = _pos1[index0];
	auto p1 = _pos1[index1];
	auto p2 = _pos1[index2];
	auto normal = (p1 - p0).cross(p2 - p0);
	normal.normalize();
	auto force = normal * (normal.dot(wind));
	_vel[index0] += force;
	_vel[index1] += force;
	_vel[index2] += force;
}

void PBD_PlaneCloth::solveDihedralConstraint(int index0, int index1, int index2, int index3, double restAngle)
{
	double stiffness = 0.05;
	auto p0 = _pos1[index0];
	auto p1 = _pos1[index1];
	auto p2 = _pos1[index2];
	auto p3 = _pos1[index3];

	vec3 e = p3 - p2;
	double length = e.length();
	if (length < 0.001) {
		return;
	}
	double invlength = 1.0 / length;
	vec3 n1 = (p2 - p0).cross(p3 - p0);
	vec3 n2 = (p3 - p1).cross(p2 - p1);
	n1 /= n1.lengthSquared();
	n2 /= n2.lengthSquared();
	
	vec3 d0 = n1 * length;
	vec3 d1 = n2 * length;
	vec3 d2 = n1 * ((p0 - p3).dot(e) * invlength) + n2 * ((p1 - p3).dot(e) * invlength);
	vec3 d3 = n1 * ((p2 - p0).dot(e) * invlength) + n2 * ((p2 - p1).dot(e) * invlength);

	n1.normalize();
	n2.normalize();
	double dot = n1.dot(n2);
	
	if (dot < -1.0) {
		dot = -1.0;
	}
	if (dot > 1.0) {
		dot = 1.0;
	}
	double phi = acos(dot);

	double lambda = _invMass[index0] * d0.lengthSquared() +
		_invMass[index1] * d1.lengthSquared() +
		_invMass[index2] * d2.lengthSquared() +
		_invMass[index3] * d3.lengthSquared();

	if (lambda == 0.0) {
		return;
	}

	lambda = (phi - restAngle) / lambda * stiffness;

	if (n1.cross(n2).dot(e) > 0.0) {
		lambda = -lambda;
	}

	_pos1[index0] += d0 * (-_invMass[index0] * lambda);
	_pos1[index1] += d1 * (-_invMass[index1] * lambda);
	_pos1[index2] += d2 * (-_invMass[index2] * lambda);
	_pos1[index3] += d3 * (-_invMass[index3] * lambda);

	double visLambda = fabs(lambda*1000000.0);
	_bendingForce[index0] += visLambda;
	_bendingForce[index1] += visLambda;
	_bendingForce[index2] += visLambda;
	_bendingForce[index3] += visLambda;
}

void PBD_PlaneCloth::applyWind(vec3 wind)
{
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			computeWindForTriangle(wind, j * _res[0] + i, (j + 1) * _res[0] + (i + 1), (j+1) * _res[0] + i);
			computeWindForTriangle(wind, j * _res[0] + i, (j + 1) * _res[0] + (i + 1), j * _res[0] + (i + 1));
		}
	}
}

void PBD_PlaneCloth::drawSpring(void)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < _res[1]; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {
			int index0 = j * _res[0] + i;
			int index1 = (j + 1) * _res[0] + i;
			auto p1 = _pos[index0];
			auto p2 = _pos[index1];
			glBegin(GL_LINES);
			glVertex3f(p1.x(), p1.y(), p1.z());
			glVertex3f(p2.x(), p2.y(), p2.z());
			glEnd();
		}
	}

	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0]; j++) {
			int index0 = j * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			auto p1 = _pos[index0];
			auto p2 = _pos[index1];
			glBegin(GL_LINES);
			glVertex3f(p1.x(), p1.y(), p1.z());
			glVertex3f(p2.x(), p2.y(), p2.z());
			glEnd();
		}
	}
	glEnable(GL_LIGHTING);
}

void PBD_PlaneCloth::draw(void)
{
	glDisable(GL_LIGHTING);
	for (int i = 0; i < _res[0] - 1; i++) {
		for (int j = 0; j < _res[1] - 1; j++) {
			auto p00 = pos(i, j);
			auto p10 = pos(i + 1, j);
			auto p11 = pos(i + 1, j + 1);
			auto p01 = pos(i, j + 1);
			int c = ((((i & 0x8) == 0) ^ ((j & 0x8)) == 0)) * 255;
			glColor3f((float)c, (float)c, (float)c);
			glBegin(GL_QUADS);
			glVertex3f(p00.x(), p00.y(), p00.z());
			glVertex3f(p10.x(), p10.y(), p10.z());
			glVertex3f(p11.x(), p11.y(), p11.z());
			glVertex3f(p01.x(), p01.y(), p01.z());
			glEnd();
		}
	}
	drawOutline();

	

}

void PBD_PlaneCloth::drawCollisionSphere(void)
{
#if defined(SPHERE)
	//구 그리기
	glTranslatef(1.5f, -1.5f, 1.0f);
	glColor3f(0, 0, 0);
	glutWireSphere(1, 10, 8);
#elif defined(BOX)
	glColor3f(0, 0, 0);
	double x = 1.5f, y = -1.5f, z = 1.0f; //box 중심
	double rx = 1.0, ry = 1.0, rz = 1.0; //box 길이

	glBegin(GL_QUADS);
	glColor3f(1, 0, 0); //back
	glVertex3f(x + rx, y + ry, z - rz);
	glVertex3f(x + rx, y - ry, z - rz);
	glVertex3f(x - rx, y - ry, z - rz);
	glVertex3f(x - rx, y + ry, z - rz);
	
	glColor3f(0, 1, 0); //front
	glVertex3f(x + rx, y + ry, z + rz);
	glVertex3f(x + rx, y - ry, z + rz);
	glVertex3f(x - rx, y - ry, z + rz);
	glVertex3f(x - rx, y + ry, z + rz);
	
	glColor3f(0, 0, 1);
	glVertex3f(x + rx, y + ry, z + rz);
	glVertex3f(x + rx, y - ry, z + rz);
	glVertex3f(x + rx, y - ry, z - rz);
	glVertex3f(x + rx, y + ry, z - rz);

	glColor3f(0, 1, 1);
	glVertex3f(x - rx, y + ry, z + rz);
	glVertex3f(x - rx, y - ry, z + rz);
	glVertex3f(x - rx, y - ry, z - rz);
	glVertex3f(x - rx, y + ry, z - rz);

	glColor3f(1, 1, 0);
	glVertex3f(x + rx, y + ry, z + rz);
	glVertex3f(x - rx, y + ry, z + rz);
	glVertex3f(x - rx, y + ry, z - rz);
	glVertex3f(x + rx, y + ry, z - rz);

	glColor3f(1, 0, 1);
	glVertex3f(x + rx, y - ry, z + rz);
	glVertex3f(x - rx, y - ry, z + rz);
	glVertex3f(x - rx, y - ry, z - rz);
	glVertex3f(x + rx, y - ry, z - rz);

	glEnd();
#endif
}

void PBD_PlaneCloth::drawOutline(void)
{
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (int i = 0; i < _res[0] - 1; i++) {
		auto p0 = pos(i, 0);
		auto p1 = pos(i + 1, 0);
		auto p2 = pos(i, _res[1] - 1);
		auto p3 = pos(i + 1, _res[1] - 1);
		auto p4 = pos(0, i);
		auto p5 = pos(0, i + 1);
		auto p6 = pos(_res[1] - 1, i);
		auto p7 = pos(_res[1] - 1, i + 1);
		glVertex3f(p0.x(), p0.y(), p0.z());
		glVertex3f(p1.x(), p1.y(), p1.z());
		glVertex3f(p2.x(), p2.y(), p2.z());
		glVertex3f(p3.x(), p3.y(), p3.z());
		glVertex3f(p4.x(), p4.y(), p4.z());
		glVertex3f(p5.x(), p5.y(), p5.z());
		glVertex3f(p6.x(), p6.y(), p6.z());
		glVertex3f(p7.x(), p7.y(), p7.z());
	}
	glEnd();
	glLineWidth(1.0f);
}

void PBD_PlaneCloth::LevelSetCollision(void) //구는 정지 상태
{
	double deltaT = 0.01f;
	double h = 0.1f;
	double coefficientFriction = 0.1f; //마찰 계수

	for (int i = 0; i < _res[0]; i++) { //width
		for (int j = 0; j < _res[1]; j++) { //height 모든 정점에 대하여 실행
			int index = j * _res[0] + i;

			double x = _pos[index].x();
			double y = _pos[index].y();
			double z = _pos[index].z();

			vec3 N(SDFCalculate(x + h, y, z) - SDFCalculate(x, y, z),
				SDFCalculate(x, y + h, z) - SDFCalculate(x, y, z),
				SDFCalculate(x, y, z + h) - SDFCalculate(x, y, z));
			N /= h; //법선 벡터 계산 (오일러 방법 이용) = Gradient PI
			N.normalize();

			double pi = SDFCalculate(x, y, z); //PI, newPI 계산
			double newPI = pi + (_vel[index] * deltaT).dot(N);

			if (newPI < 0)
			{
				double vpN = _vel[index].dot(N); //원래의 법선 방향 속력
				vec3 vpNN = N * vpN; //원래의 법선 방향 속도
				vec3 vpT = _vel[index] - vpNN; //원래의 접선 방향 속도

				double newVpN = vpN - (newPI / deltaT); //새로운 법선 방향 속력
				vec3 newVpNN = N * newVpN; // 새로운 법선 방향 속도

				double friction = (coefficientFriction * (newVpN - vpN) / vpT.getNorm());
				vec3 newVpT = vpT * (1 - friction);

				if (1 - friction < 0)
					newVpT.set(0, 0, 0);

				_vel[index] = newVpNN + newVpT; //속도 업데이트
			}
		}
	}
}

double	PBD_PlaneCloth::SDFCalculate(double x, double y, double z)
{
#if defined(SPHERE)
	double x0 = 1.5f, y0 = -1.5f, z0 = 1.0f; //sphere 중심 좌표
	double r = 3.0f;
	return sqrt(pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2)) - r; //구 방정식
#elif defined(PLANE)
	//ax + by + cz + d = 0 평면 방정식
	double a = 0.0;
	double b = 1;
	double c = 0.0;
	double d = 2;

	return (a * x + b * y + c * z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
#elif defined(BOX)
	vec3 b = vec3(1.0,1.0,1.0);
	double x0 = 1.5f, y0 = -1.5f, z0 = 1.0f; //box 중심 좌표

	vec3 p = vec3(x - x0, y - y0, z - z0);
	vec3 q = p.abs() - b;
	vec3 a = vec3(max(q.x(), 0), max(q.y(), 0), max(q.z(), 0));
	double d = a.length() + min(max(q.x(), max(q.y(), q.z())), 0);
	return d;
#endif
}

double	PBD_PlaneCloth::SDFCalculate(vec3 v)
{
#if defined(SPHERE)
	double x0 = 1.5f, y0 = -1.5f, z0 = 1.0f; //sphere 중심 좌표
	double r = 3.0f;
	return sqrt(pow(v.x() - x0, 2) + pow(v.y() - y0, 2) + pow(v.z() - z0, 2)) - r; //구 방정식
#elif defined(PLANE)
	//ax + by + cz + d = 0 평면 방정식
	double a = 0.0;
	double b = 1;
	double c = 0.0;
	double d = 2;
	return (a * v.x() + b * v.y() + c * v.z() + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
#elif defined(BOX)
	vec3 b = vec3(1.0,1.0,1.0);
	double x0 = 1.5f, y0 = -1.5f, z0 = 1.0f;//box 중심 좌표

	vec3 p = vec3(v.x() - x0, v.y() - y0, v.z() - z0);
	vec3 q = p.abs() - b;
	vec3 a = vec3(max(q.x(), 0), max(q.y(), 0), max(q.z(), 0));
	double d = a.length() + min(max(q.x(), max(q.y(), q.z())), 0);
	return d;
#endif
}

void PBD_PlaneCloth::HistoryBasedCollision(void) //평면은 정지 상태
{
	double deltaT = 0.01f;
	double h = 0.1f;
	double coefficientFriction = 0.2f; //마찰 계수

	for (int i = 0; i < _res[0]; i++) { //width
		for (int j = 0; j < _res[1]; j++) { //height 모든 정점에 대하여 실행
			int index = j * _res[0] + i;

			double x = _pos[index].x();
			double y = _pos[index].y();
			double z = _pos[index].z();

			vec3 N(SDFCalculate(x + h, y, z) - SDFCalculate(x, y, z),
				SDFCalculate(x, y + h, z) - SDFCalculate(x, y, z),
				SDFCalculate(x, y, z + h) - SDFCalculate(x, y, z));
			N /= h; //법선 벡터 계산 (오일러 방법 이용) = Gradient PI
			N.normalize();

			double pi = SDFCalculate(x, y, z); //PI, newPI 계산
			vec3 newPos = _pos[index] + (_vel[index] * deltaT);
			double newPI = SDFCalculate(newPos);

			if (newPI < 0)
			{
				double weight = SDFCalculate(_pos[index]) / (SDFCalculate(_pos[index]) - SDFCalculate(newPos));

				vec3 beforeV = _vel[index] * weight; //현재 위치 ~ 접촉면까지의 속도 (v~n+1/2)
				vec3 xc = _pos[index] + beforeV * deltaT;
				_vel[index] = beforeV; //업데이트
				_pos[index] = xc;

				// V star n+1/2 계산
				double vpN = _vel[index].dot(N); //원래의 법선 방향 속력
				vec3 vpNN = N * vpN; //원래의 법선 방향 속도
				vec3 vpT = _vel[index] - vpNN; //원래의 접선 방향 속도

				double newVpN = max(vpN, 0); //새로운 법선 방향 속력 (평면 속도 = 0 정지)
				vec3 newVpNN = N * newVpN; // 새로운 법선 방향 속도

				double friction = (coefficientFriction * (newVpN - vpN) / vpT.getNorm()); //마찰 계산
				vec3 newVpT = vpT * (1 - friction);

				if (1 - friction < 0)
					newVpT.set(0, 0, 0);

				vec3 Vnew = newVpNN + newVpT;
				vec3 afterV = Vnew * (1 - weight);

				_vel[index] = afterV; //업데이트
			}
		}
	}
}

void PBD_PlaneCloth::updatePPSelfCollision(double dt)
{
	double thickness2 = thickness * thickness;

	for (int i = 0; i < _res[0]; i++) { //width
		for (int j = 0; j < _res[1]; j++) { //height 모든 정점에 대하여 실행
			int index = j * _res[0] + i;

			int id0 = index;
			int first = _hash->firstAdjId[index];
			int last = _hash->firstAdjId[index + 1];
			
			for (int j = first; j < last; j++)
			{
				int id1 = _hash->adjIds[j];

				vec3 diffPos = (_pos1[id1] - _pos1[id0]);
				double dist2 = diffPos.lengthSquared();
				
				if (dist2 > thickness2 || dist2 == 0) //거리가 두께보다 멀면 처리 X
					continue;

				double restDist2 = (_restPos[id1] - _restPos[id0]).lengthSquared();
				double minDist = thickness; //restDistance 와 thickness 중 작은 값을 minDist로 설정
				if (dist2 > restDist2)
					continue;
				if (restDist2 < thickness2)
					minDist = sqrt(restDist2);

				//position correction
				double dist = sqrt(dist2);
				diffPos = diffPos * ((minDist - dist) / dist);

				_pos1[id0] += diffPos * -0.5 ;
				_pos1[id1] += diffPos * 0.5 ;

				//friction
				vec3 v0 = (_pos1[id0] - _pos[id0]);
				vec3 v1 = (_pos1[id1] - _pos[id1]);

				vec3 Vavg = (v0 + v1) * 0.5;

				double damping = 0.3;
				_pos1[id0] = _pos1[id0] + (Vavg - v0) * damping ;
				_pos1[id1] = _pos1[id1] + (Vavg - v1) * damping;
			}
		}
	}	
}

void PBD_PlaneCloth::InitTriangle()
{
	int k = 0;
	for (int i = 0; i < _res[1] - 1; i++) {
		for (int j = 0; j < _res[0] - 1; j++) {

			int index0 = (j + 1) * _res[0] + i;
			int index1 = j * _res[0] + (i + 1);
			int index2 = (j + 1) * _res[0] + (i + 1);
			int index3 = j * _res[0] + i;

			_tri[k] = Triangle(index3, index1, index2);
			k++;
			_tri[k] = Triangle(index3, index0, index2);
			k++;
		}
	}
}

void PBD_PlaneCloth::updatePTSelfCollision(double dt)
{
	for (int i = 0; i < _hash->numOfTri; i++)
	{
		int index0 = _tri[i].point[0]; //삼각형 꼭짓점
		int index1 = _tri[i].point[1]; 
		int index2 = _tri[i].point[2];

		auto p0 = _pos1[index0];
		auto p1 = _pos1[index1];
		auto p2 = _pos1[index2];

		int first = _hash->TfirstAdjId[i];
		int last = _hash->TfirstAdjId[i + 1];

		double u, v, w;
		for (int j = first; j < last; j++)
		{
			int index = _hash->TadjIds[j]; //점
			auto p = _pos1[index]; //충돌 점

			//제약 조건 투영
			vec3 v0 = p1 - p0;
			vec3 v1 = p2 - p0;
			vec3 v2 = p - p0;

			double dot00 = v0.dot(v0);
			double dot01 = v0.dot(v1);
			double dot02 = v0.dot(v2);
			double dot11 = v1.dot(v1);
			double dot12 = v1.dot(v2);

			double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

			double tempU = (dot11 * dot02 - dot01 * dot12) * invDenom;
			double tempV = (dot00 * dot12 - dot01 * dot02) * invDenom;

			if (!(tempU >= 0 && tempV >= 0 && tempU + tempV <= 1))
				continue;

			u = tempU;
			v = tempV;
			w = 1 - u - v;

			vec3 pc = p0 + v0 * u + v1 * v;

			double distance = (p - pc).length();
			if (distance > thickness) // 설정 거리보다 멀리 있을 경우 투영 X
				continue;

			vec3 n = (p - pc);
			n.normalize();

			//barycentric coord 이용 보간
			double c = n.dot(p - p0) - thickness;

			double a0 = w;
			double a1 = u;
			double a2 = v;
			double scale = c / (pow(a0, 2) + pow(a1, 2) + pow(a2, 2) + 1); //정점의 무게가 다 같다고 가정

			vec3 gP = n;
			vec3 gP0 = n * a0 * (-1);
			vec3 gP1 = n * a1 * (-1);
			vec3 gP2 = n * a2 * (-1);

			vec3 dP = gP * scale * (-1);
			vec3 dP0 = gP0 * scale * (-1);
			vec3 dP1 = gP1 * scale * (-1);
			vec3 dP2 = gP2 * scale * (-1);

			double damping = 0.9;
			_pos1[index] += dP * damping;
			_pos1[index0] += dP0 * damping;
			_pos1[index1] += dP1 * damping;
			_pos1[index2] += dP2 * damping;

			/*//단순 0.5씩 이동
			vec3 cor = n * (thickness - distance);
			double damping = 1;
			_pos1[index] += cor * 0.5 * damping;
			_pos1[index0] += -cor * 0.5 * damping;
			_pos1[index1] += -cor * 0.5 * damping;
			_pos1[index2] += -cor * 0.5 * damping;*/
		}
	}

}

void PBD_PlaneCloth::updateTriBoundBox(vector<vec3> _pos1)
{
	for (int i = 0; i < _tri.size(); i++)
	{
		_tri[i].UpdateBoundPos(_pos1);
	}
}

void PBD_PlaneCloth::updateEESelfCollision(double dt)
{
	for (int i = 0; i < _hash->numOfTri; i++)
	{
		int index0 = _tri[i].point[0]; //삼각형 꼭짓점
		int index1 = _tri[i].point[1];
		int index2 = _tri[i].point[2];

		auto p0 = _pos1[index0];
		auto p1 = _pos1[index1];
		auto p2 = _pos1[index2];

		int first = _hash->TfirstAdjId[i];
		int last = _hash->TfirstAdjId[i + 1];

		for (int j = first; j < last; j++)
		{
			int index = _hash->TadjIds[j]; //점
			
			int nxtIdx[6] = { -1, -1 * (_res[0] + 1), -1 * _res[0], 1, _res[0] + 1, _res[0] };
			for (int k = 0; k < 6; k++)
			{
				if (index % _res[0] == 0)
				{
					if (k == 0 || k == 1)
						continue;
				}
				else if (index % _res[0] == _res[0]-1)
				{
					if (k == 3 || k == 4)
						continue;
				}

				if (index / _res[0] == 0)
				{
					if (k == 1 || k == 2)
						continue;
				}
				else if (index / _res[0] == _res[1]-1)
				{
					if (k == 4 || k == 5)
						continue;
				}

				solveEEConstraints(index0, index1, index, index + nxtIdx[k]);
				solveEEConstraints(index0, index2, index, index + nxtIdx[k]);
				solveEEConstraints(index1, index2, index, index + nxtIdx[k]);
			}	
		}
	}
}

float PBD_PlaneCloth::Clamp(float n, float min, float max)
{
	if (n < min) return min;
	if (n > max) return max;
	return n;
}

void PBD_PlaneCloth::solveEEConstraints(int indexA, int indexB, int indexC, int indexD)
{
	if (indexA == indexC || indexA == indexD)
		return;
	if (indexB == indexC || indexB == indexD)
		return;

	auto pa = _pos1[indexA];
	auto pb = _pos1[indexB];
	auto pc = _pos1[indexC];
	auto pd = _pos1[indexD];

	//최단점 계산
	vec3 d1 = pb - pa;
	vec3 d2 = pd - pc;
	vec3 r = pa - pc;
	
	double a = d1.dot(d1);
	double b = d2.dot(d1);
	double e = d2.dot(d2);
	double c = d1.dot(r);
	double f = d2.dot(r);
	double denom = a * e - b * b;

	double s;
	if (denom != 0) //d=0 일 때 edge는 평행
	{
		s = Clamp((b * f - c * e) / denom, 0.0, 1.0);
	}
	else
	{
		s = 0.0f;
	}

	double t = (b * s + f) / e;

	if (t < 0)
	{
		t = 0.0f;
		s = Clamp(-1 * c / a, 0.0f, 1.0f);
	}
	else if (t > 1)
	{
		t = 1.0f;
		s = Clamp((b - c) / a, 0.0f, 1.0f);
	}

	vec3 Palpha = pa + d1 * s;
	vec3 Pbeta = pc + d2 * t;
	
	double dist = (Palpha - Pbeta).length();

	if (dist > thickness)
		return;

	vec3 n = (Pbeta - Palpha);
	n.normalize();

	//position 수정
	double C = dist - thickness; //음수

	vec3 GradPa = n * (1 - s) * (-1);
	vec3 GradPb = n * s * (-1);
	vec3 GradPc = n * (1 - t);
	vec3 GradPd = n * t;

	double scale = C / (GradPa.lengthSquared() + GradPb.lengthSquared() + GradPc.lengthSquared() + GradPd.lengthSquared());

	vec3 deltaPa = GradPa * scale * (-1);
	vec3 deltaPb = GradPb * scale * (-1);
	vec3 deltaPc = GradPc * scale * (-1);
	vec3 deltaPd = GradPd * scale * (-1);

	double damping = 0.01;
	_pos1[indexA] += deltaPa * damping;
	_pos1[indexB] += deltaPb * damping;
	_pos1[indexC] += deltaPc * damping;
	_pos1[indexD] += deltaPd * damping;

	/*cout << ",,,,,,,,,,,,," << endl;
	Palpha = pos[indexA] + (pos[indexB] - pos[indexA]) * s;
	Pbeta = pos[indexC] + (pos[indexD] - pos[indexC]) * t;

	Palpha.print();
	Pbeta.print();

	cout << (Palpha - Pbeta).length() << endl;*/
}

void PBD_PlaneCloth::SdfCollision(double dt)
{
	double deltaT = 0.01f;
	double h = 0.01f;
	for (int i = 0; i < numOfTri; i++)
	{
		int p0 = _tri[i].point[0]; //Triangle 꼭짓점 접근
		int p1 = _tri[i].point[1];
		int p2 = _tri[i].point[2];

		vec3 pos0 = _pos[p0]; //꼭짓점의 위치 대입
		vec3 pos1 = _pos[p1];
		vec3 pos2 = _pos[p2];

		vec3 midPos = (pos0 + pos1 + pos2) / 3; //삼각형의 중간점
		double boundRadius = max(max((pos0 - midPos).length(), (pos1 - midPos).length()), (pos2 - midPos).length()); //Bounding Sphere의 반지름

		if (SDFCalculate(midPos) > boundRadius) //Bounding Sphere의 반지름보다 중심으로부터 충돌체와의 거리가 멀 경우
			continue;

		vec3 p = midPos; //Iterate 시작지점
		double u, v, w;

		for (int iterate = 0; iterate < 15; iterate++)
		{
			vec3 N(SDFCalculate(p.x() + h, p.y(), p.z()) - SDFCalculate(p.x(), p.y(), p.z()),
				SDFCalculate(p.x(), p.y() + h, p.z()) - SDFCalculate(p.x(), p.y(), p.z()),
				SDFCalculate(p.x(), p.y(), p.z() + h) - SDFCalculate(p.x(), p.y(), p.z()));
			N /= h; //법선 벡터 계산 (오일러 방법 이용) = Gradient PI
			N.normalize();
#if PROJECTED_GRADIENT_DESCENT
			p -= N; //Gradient Descent
			p = barycentricCoord(pos0, pos1, pos2, p, w, u, v); //Projection
#endif
#if FRANK_WOLFE
			float da = pos0.dot(N); //각 꼭짓점과 법선 벡터 내적
			float db = pos1.dot(N);
			float dc = pos2.dot(N);

			vec3 s; //내적 값이 가장 작은 꼭짓점을 s로 설정
			if (da < db)
			{
				if (dc < da)
					s = pos2;
				else
					s = pos0;
			}
			else
			{
				if (dc < db)
					s = pos2;
				else
					s = pos1;
			}

			float gamma = 0.3 * 2.0 / (float(iterate) + 2.0); //보간 비율 gamma
			p = p * (1 - gamma) + s * gamma; //현재 위치로부터 s 위치로 이동
#endif

		}
		double d = SDFCalculate(p);
		if (d > 0) //thickness = contact_dist
			continue;

#if FRANK_WOLFE
		barycentricCoord(pos0, pos1, pos2, p, w, u, v);
#endif

		//solve contraints
		double a0 = w;
		double a1 = u;
		double a2 = v;

		double c = d;
		double scale = c / (pow(a0, 2) + pow(a1, 2) + pow(a2, 2)); //정점의 무게가 다 같다고 가정

		vec3 n(SDFCalculate(p.x() + h, p.y(), p.z()) - SDFCalculate(p.x(), p.y(), p.z()),
			SDFCalculate(p.x(), p.y() + h, p.z()) - SDFCalculate(p.x(), p.y(), p.z()),
			SDFCalculate(p.x(), p.y(), p.z() + h) - SDFCalculate(p.x(), p.y(), p.z()));
		n /= h; //법선 벡터 계산 (오일러 방법 이용) = Gradient PI
		n.normalize();

		vec3 gP0 = n * a0;
		vec3 gP1 = n * a1;
		vec3 gP2 = n * a2;

		vec3 dP0 = gP0 * scale * (-1);
		vec3 dP1 = gP1 * scale * (-1);
		vec3 dP2 = gP2 * scale * (-1);

		////개선 전1
		//_vel[p0] += dP0 / dt;
		//_vel[p1] += dP1 / dt;
		//_vel[p2] += dP2 / dt;

		//////개선 후
		vec3 vN0 = dP0 / dt;
		vec3 vN1 = dP1 / dt;
		vec3 vN2 = dP2 / dt;

		vec3 vT0 = _vel[p0] - n * _vel[p0].dot(n);
		vec3 vT1 = _vel[p1] - n * _vel[p1].dot(n);
		vec3 vT2 = _vel[p2] - n * _vel[p2].dot(n);

		_vel[p0] = vN0 + vT0;
		_vel[p1] = vN1 + vT1;
		_vel[p2] = vN2 + vT2;
	}
}

vec3 PBD_PlaneCloth::barycentricCoord(vec3 pos0, vec3 pos1, vec3 pos2, vec3 p, double& w, double& u, double& v)
{
	vec3 v0 = pos1 - pos0;
	vec3 v1 = pos2 - pos0;
	vec3 v2 = p - pos0;

	double dot00 = v0.dot(v0);
	double dot01 = v0.dot(v1);
	double dot02 = v0.dot(v2);
	double dot11 = v1.dot(v1);
	double dot12 = v1.dot(v2);

	double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

	double tempU = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double tempV = (dot00 * dot12 - dot01 * dot02) * invDenom;

	if (!(tempU >= 0 && tempV >= 0 && tempU + tempV <= 1))
	{
		tempU = min(tempU, 1.0);
		tempU = max(tempU, 0.0);

		tempV = min(tempV, 1.0);
		tempV = max(tempV, 0.0);
		//continue;
	}

	u = tempU;
	v = tempV;
	w = 1 - u - v;

	vec3 pc = pos0 + v0 * u + v1 * v;
	return pc;
}
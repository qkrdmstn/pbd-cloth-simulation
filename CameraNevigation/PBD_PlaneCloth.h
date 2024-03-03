#ifndef __PBD_PLANE_CLOTH_H__
#define __PBD_PLANE_CLOTH_H__

#pragma once
#include "Vec3.h"
#include "Triangle.h"
#include <vector>

using namespace std;

class PBD_PlaneCloth
{
public:
	int				_res[2]; // width and height
	vector<double>	_bendingForce;
	vector<double>	_invMass;
	vector<double>	_dihedralAngle;
	vector<vec3>	_restPos;
	vector<vec3>	_pos;
	vector<vec3>	_pos1; // new pos
	vector<vec3>	_vel;
	vector<Triangle> _tri; //trinangle set
	int numOfTri;

public:
	double			_structuralLength0Horiz; // horizontal structural
	double			_structuralLength0Verti; // vertical structural
	double			_bendLength0Horiz; // horizontal bend
	double			_bendLength0Verti; // vertical bend
	double			_shearLength0Horiz;	// horizontal shear
	double			_shearLength0Verti;	// Vertical shear 
	double			thickness;
public:
	PBD_PlaneCloth();
	PBD_PlaneCloth(int width, int height, double spacing, double _thickness);
	~PBD_PlaneCloth();
public:
	inline vec3	pos(int i, int j) { return _pos[j * _res[0] + i]; }
public:
	void	init(void);
	void	integrate(double dt);
	void	simulation(double dt, int numSubStep);
	void	computeRestLength(void);
	void	computeDihedralAngle(void);
	void	updateBendSprings(void);
	void	updateShearSprings(void);
	void	updateStructuralSprings(void);
	void	applyWind(vec3 wind);
	void	computeWindForTriangle(vec3 wind, int index0, int index1, int index2);
	void	applyExtForces(double dt);
	void	solveDistanceConstraint(int index0, int index1, double restLength);
	void	solveDihedralConstraint(int index0, int index1, int index2, int index3, double restAngle);
	void	LevelSetCollision(void); //LevelSetCollision
	void	HistoryBasedCollision(void); //HistoryBasedCollision - Sphere
	double	SDFCalculate(double x0, double y0, double z0);
	double	SDFCalculate(vec3 v);
	void	updatePPSelfCollision(double dt); //SelfCollision - Point to Point
	void	updateTriBoundBox(vector<vec3> _pos1); //Update - Triangle AABB
	void	InitTriangle(); //Init triangle set
	void	updatePTSelfCollision(double dt); //SelfCollision - Point to Triangle
	void	updateEESelfCollision(double dt); //SelfCollision - Edge to Edge
	void	solveEEConstraints(int indexA, int indexB, int indexC, int indexD); // Solve edge-edge constraints
	float	Clamp(float n, float min, float max); 
	void	SdfCollision(double dt);
	vec3	barycentricCoord(vec3 pos0, vec3 pos1, vec3 pos2, vec3 p, double& w, double& u, double& v);
	
public:
	void	draw(void);
	void	drawSpring(void);
	void	drawOutline(void);
	void	drawCollisionSphere(void);
};

#endif

#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//todo: complete this header
#include "RigidBodySystem.h"

#include "util/quaternion.h"
#include <DirectXMath.h>

#define single_box 0
#define two_boxes 1

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {

	int  fMass;

	Vec3 vPosition;

	Vec3 vVelocity;

	//Vec3 force;

	//Vec3 centerM;
	Vec3 vPositionBody;

	Vec3 vAngularVelocity;

	Vec3 angularMomentum;

	GamePhysics::Quat qOrientation;

	XMMATRIX tensor;

	Vec3 size;

	Vec3 vTorque;  //total torque on body
};


class RigidBodySystemSimulator:public Simulator{

public:

	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);


	//for simulation
	void precomputing();

	void updateOrientationAndMomentum(float timestep);

	void updateInertiaTensorAndAngu();

	//void updateAngularMomentum(float timestep);

	//void updateObjectPos();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	Vec3 m_cm; //world position for mass center

	XMFLOAT3X3 covMatrix;

	int totalMass;

	XMMATRIX I0;

	vector<RigidBody> m_pRigidBodySystem;

	int m_iIntegrator;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif
#include "RigidBodySystemSimulator.h"


Vec3 crossProduct(Vec3 a, Vec3 b);

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iIntegrator = 0;

	m_externalForce = Vec3();

	m_cm = Vec3();

	totalMass = 0;

}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "single_box, two_boxes";
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	//TwAddVarRW(DUC->g_pTweakBar, "")

	/*
	switch (m_iIntegrator)
	{
	case 0:

		break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}
	*/
}



void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

	//this->precomputing();

	this->updateOrientationAndMomentum(timeStep);

	this->updateInertiaTensorAndAngu();

	//this->collisionHandeling();

	this->updateLinear(timeStep);

	this->collisionHandeling();
	/*
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		// rotate the teapot
		
		break;
	default:
		break;
	}
	*/
}


void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	/*
	this->m_iIntegrator;
	cout << "single box !\n";
	this->m_pRigidBodySystem.clear();
	this->addRigidBody(Vec3(0, 0, 0), 0.5f, 5);
	*/
	//break;
	this->m_iIntegrator = testCase;
	//this->m_pRigidBodySystem.swap(vector<RigidBody>());
	switch (m_iIntegrator) 
	{
	case 0:
		cout << "single box !\n";
		//this->m_pRigidBodySystem.clear();
		this->m_pRigidBodySystem.swap(vector<RigidBody>());
		this->addRigidBody(Vec3(-1.0f, -0.2f, 0.1f), Vec3(0.4f,0.2f,0.2f), 10.0f);
		//this->applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));
		break;
	case 1:
		cout << "two boxes!\n";
		//this->m_pRigidBodySystem.clear();
		this->m_pRigidBodySystem.swap(vector<RigidBody>());
		this->addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.3f, 0.2f, 0.2f), 100.0f);
		this->addRigidBody(Vec3(0.0f, 0.2f,0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		this->setOrientationOf(1, GamePhysics::Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		this->setVelocityOf(1, Vec3(0.0f,-0.1f,0.05f));
		break;
	case 2:
		cout << " !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.01f;
		//inputWorld = inputWorld * inputScale;
		
		if (m_iIntegrator == 0) {
			this->applyForceOnBody(0, inputWorld, (-1) * inputWorld * inputScale);
		}
		
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){

	drawRigid();
	//DUC->drawRigidBody();
}




void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_pRigidBodySystem.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return m_pRigidBodySystem[i].vPosition;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return m_pRigidBodySystem[i].vVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return m_pRigidBodySystem[i].vAngularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_pRigidBodySystem[i].vTorque += Vec3(loc.y*force.z - force.y*loc.z, force.x*loc.z - loc.x * force.z, loc.x * force.y - force.x * loc.y);
	m_pRigidBodySystem[i].acc += force / m_pRigidBodySystem[i].fMass;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	RigidBody tmp;
	tmp.fMass = mass;
	tmp.vPosition = position;
	tmp.size = size;
	m_pRigidBodySystem.push_back(tmp);

	this->precomputing();

	//Vec3 tmpSum(0,0,0);
	
	//Vec3 tmp2 = m_cm * totalMass + position * mass;

	//totalMass += mass;

	//m_cm = tmp2 / totalMass;
	/*
	for (int i = 0; i < m_pRigidBodySystem.size();i++) {
		m_pRigidBodySystem[i].vPositionBody = m_pRigidBodySystem[i].vPosition - m_cm;
		//m_pRigidBodySystem[i].tMass += mass;
	}
	*/
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	m_pRigidBodySystem[i].qOrientation = orientation;
}


void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	m_pRigidBodySystem[i].vVelocity = velocity;
}


void RigidBodySystemSimulator::updateOrientationAndMomentum(float timestep) {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		GamePhysics::Quat tmp = GamePhysics::Quat(m_pRigidBodySystem[i].vAngularVelocity.x, m_pRigidBodySystem[i].vAngularVelocity.y, m_pRigidBodySystem[i].vAngularVelocity.z, 0).operator*(m_pRigidBodySystem[i].qOrientation);
		m_pRigidBodySystem[i].qOrientation += tmp.operator*=(timestep * 0.5);
		m_pRigidBodySystem[i].angularMomentum += m_pRigidBodySystem[i].vTorque * timestep;
	}
}

/*
void RigidBodySystemSimulator::updateObjectPos() {

}

void RigidBodySystemSimulator::updateAngularMomentum(float timestep) {
	for () {

	}
}
*/

void RigidBodySystemSimulator::drawRigid() {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		XMMATRIX scale = XMMatrixScalingFromVector(m_pRigidBodySystem[i].size.toDirectXVector());
		XMMATRIX transl = XMMatrixTranslationFromVector(m_pRigidBodySystem[i].vPosition.toDirectXVector());
		DUC->drawRigidBody(scale * transl);
		//DUC->drawSphere(m_pRigidBodySystem[i].vPosition, m_pRigidBodySystem[i].size);
	}
}

//don't need to compute every time
void RigidBodySystemSimulator::precomputing() {
	
	/*
	Vec3 tmp;
	
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		tmp += m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPosition;
	}

	m_cm = tmp / totalMass;
	*/

	for (int i = 0; i < m_pRigidBodySystem.size();i++) {
		//m_pRigidBodySystem[i].vPositionBody = m_pRigidBodySystem[i].vPosition - m_cm;
		m_pRigidBodySystem[i].vPositionBody = m_pRigidBodySystem[i].vPosition;

		covMatrix._11 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.x * m_pRigidBodySystem[i].vPositionBody.x;
		covMatrix._12 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.x * m_pRigidBodySystem[i].vPositionBody.y;
		covMatrix._13 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.x * m_pRigidBodySystem[i].vPositionBody.z;
		covMatrix._21 = covMatrix._12;
		covMatrix._22 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.y * m_pRigidBodySystem[i].vPositionBody.y;
		covMatrix._23 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.y * m_pRigidBodySystem[i].vPositionBody.z;
		covMatrix._31 = covMatrix._13;
		covMatrix._32 = covMatrix._23;
		covMatrix._33 = m_pRigidBodySystem[i].fMass * m_pRigidBodySystem[i].vPositionBody.z * m_pRigidBodySystem[i].vPositionBody.z;
	}

	float trace = covMatrix._11 + covMatrix._22 + covMatrix._33;

	I0 = XMMatrixIdentity() * trace - XMLoadFloat3x3(&covMatrix);

}

void RigidBodySystemSimulator::updateInertiaTensorAndAngu() {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		//update the inverse inertia tensor
		m_pRigidBodySystem[i].tensor =  m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix() * XMMatrixInverse(nullptr, I0) * XMMatrixTranspose(m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix());

		//update the angualr velocity
		m_pRigidBodySystem[i].vAngularVelocity = (GamePhysics::Mat4f(m_pRigidBodySystem[i].tensor) * GamePhysics::Vec3(m_pRigidBodySystem[i].angularMomentum).toDirectXVector()).toDirectXVector();

		//update the position based on new orientation
		m_pRigidBodySystem[i].vPosition = m_cm + m_pRigidBodySystem[i].qOrientation.getRotMat() * m_pRigidBodySystem[i].vPositionBody;
	}
}

void RigidBodySystemSimulator::updateLinear(float timestep) {
	for (int i = 0; i < m_pRigidBodySystem.size();i++) {
		m_pRigidBodySystem[i].vPosition += timestep * m_pRigidBodySystem[i].vVelocity;
		m_pRigidBodySystem[i].vVelocity += timestep * m_pRigidBodySystem[i].acc;
	}
}

void RigidBodySystemSimulator::collisionHandeling() {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		for (int j = i; j < m_pRigidBodySystem.size();j++) {
			if (j == i) {
				continue;
			}
			CollisionInfo info = collisionTools::checkCollisionSATHelper(XMMatrixTranslationFromVector(m_pRigidBodySystem[i].vPosition.toDirectXVector()),
				XMMatrixTranslationFromVector(m_pRigidBodySystem[j].vPosition.toDirectXVector()), 
				m_pRigidBodySystem[i].size.toDirectXVector(),
				m_pRigidBodySystem[j].size.toDirectXVector());

			if (!info.isValid) {
				continue;
			}
			else {

				//
				Vec3 relV = m_pRigidBodySystem[j].vVelocity + crossProduct(m_pRigidBodySystem[i].vAngularVelocity, m_pRigidBodySystem[i].vPositionBody)- m_pRigidBodySystem[i].vVelocity - crossProduct(m_pRigidBodySystem[j].vAngularVelocity, m_pRigidBodySystem[j].vPositionBody);
				float c = 1;
				auto tmp = GamePhysics::Mat4f(m_pRigidBodySystem[i].tensor) * GamePhysics::Vec3((GamePhysics::cross(info.collisionPointWorld - m_pRigidBodySystem[i].vPosition, info.normalWorld))).toDirectXVector();
				auto tmp2 = GamePhysics::Mat4f(m_pRigidBodySystem[j].tensor) * GamePhysics::Vec3((GamePhysics::cross(info.collisionPointWorld - m_pRigidBodySystem[j].vPosition, info.normalWorld))).toDirectXVector();
				Vec3 impuls = -(1 + c) * relV * info.normalWorld /
					(1/m_pRigidBodySystem[i].fMass + 1/m_pRigidBodySystem[j].fMass + 
						(crossProduct(Vec3(tmp.x, tmp.y,tmp.z), info.collisionPointWorld - m_pRigidBodySystem[i].vPosition) +
						crossProduct(Vec3(tmp2.x,tmp2.y,tmp2.z) , info.collisionPointWorld - m_pRigidBodySystem[j].vPosition)) * info.normalWorld
						);

				//velocity changes  
				m_pRigidBodySystem[i].vVelocity += impuls / m_pRigidBodySystem[i].fMass;
				m_pRigidBodySystem[j].vVelocity -= impuls / m_pRigidBodySystem[j].fMass;

				//todo: change angular velocity here
				m_pRigidBodySystem[i].angularMomentum += GamePhysics::cross(info.collisionPointWorld - m_pRigidBodySystem[i].vPosition, impuls);
				m_pRigidBodySystem[j].angularMomentum -= GamePhysics::cross(info.collisionPointWorld - m_pRigidBodySystem[j].vPosition, impuls);

			}
		}
	}
}

Vec3 crossProduct(Vec3 a, Vec3 b) {
	return Vec3(a.y * b.z - b.y * a.z, b.x * a.z - a.x * b.z, a.x * b.y - b.x * a.y);
}
#include "RigidBodySystemSimulator.h"


Vec3 crossProduct(Vec3 a, Vec3 b);

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iIntegrator = 0;

	m_externalForce = Vec3();

	//m_cm = Vec3();

	//totalMass = 0;

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
}



void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

	//this->precomputing();
	//printf("%s\n", m_pRigidBodySystem[0].vPosition.toString());

	//printf("%s\n", m_pRigidBodySystem[0].vVelocity.toString());

    //printf("%s\n", m_pRigidBodySystem[0].vAngularVelocity.toString());
	this->updateTorq();
	
	this->updateOrientationAndMomentum(timeStep);
	
	//there is something wrong here
	this->updateInertiaTensorAndAngu();

	//printf("%s\n", m_pRigidBodySystem[0].vPosition.toString());
	//here
	/*
	printf("Position:\n");

	printf("x = %g\n", (float)m_pRigidBodySystem[0].vPosition.x);

	printf("y = %g\n", (float)m_pRigidBodySystem[0].vPosition.y);

	printf("z = %g\n", (float)m_pRigidBodySystem[0].vPosition.z);
	*/
	if (m_pRigidBodySystem[0].vAngularVelocity.squaredDistanceTo(Vec3(0,0,0)) != 0) {
		auto euler = m_pRigidBodySystem[0].qOrientation.getAxis();
		printf("angualr velocity:\n");
		printf("x  = %g\n", (float)m_pRigidBodySystem[0].vAngularVelocity.x);
		printf("y  = %g\n", (float)m_pRigidBodySystem[0].vAngularVelocity.y);
		printf("z  = %g\n", (float)m_pRigidBodySystem[0].vAngularVelocity.z);
		printf("Orientation:\n");
		printf("x  = %g\n y = %g\n z = %g\n", (float)euler.x, (float)euler.y, (float)euler.z);
	}

	if (m_pRigidBodySystem[0].angularMomentum.squaredDistanceTo(Vec3(0, 0, 0)) != 0) {
		printf("angular momentum:\n");
		printf("x = %g\n, y = %g\n, z = %g\n", m_pRigidBodySystem[0].angularMomentum.x, m_pRigidBodySystem[0].angularMomentum.y, m_pRigidBodySystem[0].angularMomentum.z);
	}




	//this->collisionHandeling();

	
	this->updateLinear(timeStep);

	//collision handeling still need to modify
	this->collisionHandeling();

	this->removeForce();
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
		this->addRigidBody(Vec3(-1.0f, -0.2f, 0.1f), Vec3(0.4f,0.2f,0.2f), 1.0f);
		this->setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
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
		float inputScale = 0.1f;
		//inputWorld = inputWorld * inputScale;
		
		if (m_iIntegrator == 0) {
			
			//maybe a better way to deal with the input
			this->applyForceOnBody(0, inputWorld, inputWorld * inputScale);
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
	
	Force newF;
	newF.loc = loc;
	newF.force = force;
	m_pRigidBodySystem[i].forces.push_back(newF);
	/*
	m_pRigidBodySystem[i].vTorque += Vec3((loc.y - m_pRigidBodySystem[i].vPosition.y)*force.z - force.y* (loc.z - m_pRigidBodySystem[i].vPosition.z), force.x* (loc.z - m_pRigidBodySystem[i].vPosition.z) - (loc.x - m_pRigidBodySystem[i].vPosition.x) * force.z, (loc.x - m_pRigidBodySystem[i].vPosition.x) * force.y - force.x * (loc.y - m_pRigidBodySystem[i].vPosition.y));
	m_pRigidBodySystem[i].acc += force / m_pRigidBodySystem[i].fMass;
	*/
}

void RigidBodySystemSimulator::removeForce() {
	//remove forces after a timestep
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		m_pRigidBodySystem[i].forces.swap(vector<Force>());
		m_pRigidBodySystem[i].acc = Vec3();
		m_pRigidBodySystem[i].vTorque = Vec3();
	}
}

void RigidBodySystemSimulator::updateTorq() {

	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		if (m_pRigidBodySystem[i].forces.empty()) {
			continue;
		}
		for (int j = 0; j < m_pRigidBodySystem[i].forces.size();j++) {
			m_pRigidBodySystem[i].vTorque += GamePhysics::cross(m_pRigidBodySystem[i].forces[j].loc - m_pRigidBodySystem[i].vPosition, m_pRigidBodySystem[i].forces[j].force);
			m_pRigidBodySystem[i].acc += m_pRigidBodySystem[i].forces[j].force;
		}
	}
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	RigidBody tmp;
	tmp.fMass = mass;
	tmp.vPosition = position;
	tmp.size = size;
	tmp.vAngularVelocity = Vec3(0.0, 0.0, 0.0);
	//computing I0
	precomputing(tmp);
	m_pRigidBodySystem.push_back(tmp);

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	m_pRigidBodySystem[i].qOrientation = orientation;
}


void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	m_pRigidBodySystem[i].vVelocity = velocity;
}


void RigidBodySystemSimulator::updateOrientationAndMomentum(float timestep) {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		//update the orientation
		//maybe the problem woul be here
		GamePhysics::Quat tmp = GamePhysics::Quat( m_pRigidBodySystem[i].vAngularVelocity.x, m_pRigidBodySystem[i].vAngularVelocity.y, m_pRigidBodySystem[i].vAngularVelocity.z, 0.0) 
			                                      * (m_pRigidBodySystem[i].qOrientation);
		tmp *= (timestep * 0.5);
		//it failed afer adding this line, changing orientation so to say
		m_pRigidBodySystem[i].qOrientation += tmp;

		//perhaps still need to normalize orientation?
		if (m_pRigidBodySystem[i].qOrientation.norm() != 0) {
			//m_pRigidBodySystem[i].qOrientation = m_pRigidBodySystem[i].qOrientation.unit();
		}

		//update the angular momentum
		m_pRigidBodySystem[i].angularMomentum += m_pRigidBodySystem[i].vTorque * timestep;
	}
}



void RigidBodySystemSimulator::drawRigid() {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		XMMATRIX scale = XMMatrixScalingFromVector(m_pRigidBodySystem[i].size.toDirectXVector());
		XMMATRIX transl = XMMatrixTranslationFromVector(m_pRigidBodySystem[i].vPosition.toDirectXVector());
		//XMMATRIX rot = XMMatrixRotationQuaternion(m_pRigidBodySystem[i].qOrientation.toDirectXQuat());
		XMMATRIX rot = m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix();
		DUC->drawRigidBody(scale * rot * transl);
		//DUC->drawSphere(m_pRigidBodySystem[i].vPosition, m_pRigidBodySystem[i].size);
	}
}

//don't need to compute every time
void RigidBodySystemSimulator::precomputing(RigidBody rb) {
	
	float  y = (1 / 12) * rb.fMass * (pow(rb.size.x,2) + pow(rb.size.z, 2));
	float  x = (1 / 12) * rb.fMass * (pow(rb.size.z, 2) + pow(rb.size.y,2));
	float  z = (1 / 12) * rb.fMass * (pow(rb.size.y,2) + pow(rb.size.x, 2));

	rb.I0 = XMMatrixInverse(nullptr, XMMatrixScaling(x,y,z));
}

void RigidBodySystemSimulator::updateInertiaTensorAndAngu() {
	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
		//update the inverse inertia tensor
		//m_pRigidBodySystem[i].tensor =  m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix() * XMMatrixInverse(nullptr, m_pRigidBodySystem[i].I0) * XMMatrixTranspose(m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix());
		m_pRigidBodySystem[i].tensor = XMMatrixTranspose(m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix()) * m_pRigidBodySystem[i].I0 * m_pRigidBodySystem[i].qOrientation.getRotMat().toDirectXMatrix();
		
		
		//update the angualr velocity using I (inertia tensor)and L (angular momentum)(problem occurs here)
		//m_pRigidBodySystem[i].vAngularVelocity = (GamePhysics::Mat4f(m_pRigidBodySystem[i].tensor) * GamePhysics::Vec3(m_pRigidBodySystem[i].angularMomentum).toDirectXVector()).toDirectXVector();
		//m_pRigidBodySystem[i].vAngularVelocity = m_pRigidBodySystem[i].tensor * m_pRigidBodySystem[i].angularMomentum.toDirectXVector();
		
		auto tmp = m_pRigidBodySystem[i].tensor.transformVector(m_pRigidBodySystem[i].angularMomentum);
	    m_pRigidBodySystem[i].vAngularVelocity = tmp;


		//update the position based on new orientation (irrelevant)
		//m_pRigidBodySystem[i].vPosition += m_pRigidBodySystem[i].qOrientation.getRotMat() * m_pRigidBodySystem[i].vPosition;
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
				Vec3 relV = m_pRigidBodySystem[j].vVelocity + crossProduct(m_pRigidBodySystem[i].vAngularVelocity, info.collisionPointWorld)- m_pRigidBodySystem[i].vVelocity - crossProduct(m_pRigidBodySystem[j].vAngularVelocity, info.collisionPointWorld);
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

#include "MassSpringSystemSimulator.h"
/*
struct MassPoint {
	Vec3 position;
	Vec3 Velocity;
	bool isFixed;
	Vec3 Acc;
};

struct Spring {
	int masspoint1;
	int masspoint2;
	float initialLength;
};
*/

float distance(Vec3 first, Vec3 sec);
//int euler(float timestep, float stiffness, float mass);
//int calculateAcc(float stiffness, float mass, vector<MassPoint> points);
//void midpoint(float timestep, float stiffness, float mass);
//void leapFrog(float timestep, float stiffness, float mass);


/*
vector<MassPoint> points;

vector<Spring> springs;
*/

//constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_iIntegrator = 0;
	m_fDamping = 0;
	m_externalForce = Vec3();
}


//UI
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler, Leap Frog, Midpoint";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}


void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	this->m_iIntegrator = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Euler !\n";
		
		break;
	case 1:
		cout << "Leap Frog!\n";		
		break;
	case 2:
		cout << "Midpoint !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//todo
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}


void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	int testcase2 = m_iTestCase;
	int testcase = this->m_iIntegrator;
	//determine which case tobe simulated
	/*
	if (testcase == 0) {
		int eulerRE = euler(timeStep, this->m_fStiffness, this->m_fMass);
	}else if (testcase == 1) {
		leapFrog(timeStep, this->m_fStiffness, this->m_fMass);
	}else if(testcase == 2)
	{
		midpoint(timeStep, this->m_fStiffness, this->m_fMass);
	}
	*/
	switch (testcase)
	{// handling different cases
	//EULER
	   case 0:
	   {
		  //todo
		  euler(timeStep, this->m_fStiffness, this->m_fMass);
		  break;
	    }

		//LEAPFROG
	case 1:
		//todo
		leapFrog(timeStep, this->m_fStiffness, this->m_fMass);
		break;
		//MIDPOINT
	case 2:
		//todo
		midpoint(timeStep, this->m_fStiffness, this->m_fMass);
		break;
     //default:
     //break;
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	
	switch (this->m_iIntegrator)
	{
	case 0: 
		for (int i = 0; i < points.size(); i++) {
			DUC->drawSphere(points[i].position, Vec3(0.5, 0.5, 0.5));
		}
		break;
	case 1: 
		break;
	case 2: 
		break;
	}
	
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
		m_trackmouse.x = x;
		m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}



//simulation


void MassSpringSystemSimulator::setMass(float mass){
	this->m_fMass = mass;
}


void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}

int  MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	MassPoint tmp;
	tmp.position = position;
	tmp.Velocity = Velocity;
	tmp.isFixed = isFixed;
	points.push_back(tmp);
	return points.size() - 1;
}

void  MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (points.size() >=2 && masspoint1 < points.size() && masspoint2 < points.size()) {
		Spring tmp;
		tmp.masspoint1 = masspoint1;
		tmp.masspoint2 = masspoint2;
		tmp.initialLength = initialLength;
		springs.push_back(tmp);
	}

}

int  MassSpringSystemSimulator::getNumberOfMassPoints() {
	return points.size();
}

int  MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3  MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return points[index].position;
}


Vec3  MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return points[index].Velocity;
}


void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	for (MassPoint m : points) {
		m.Acc = force / this->m_fMass;
	}
}


void MassSpringSystemSimulator::euler(float timestep, float stiffness, float mass) {
	//first calculate the acceleration
	calculateAcc(stiffness, mass, this->points);

	for (MassPoint m : this->points) {
		if (!m.isFixed) {
			m.position = m.position + timestep * m.Velocity;
		    m.Velocity = m.Velocity + timestep * m.Acc;
		}
	}
    //return 0;
}

//could also return float or double, may cause the difference
float distance(Vec3 first, Vec3 sec) {
	return sqrtf((sec.x - first.x) * (sec.x - first.x) + (sec.y - first.y) * (sec.y - first.y) + (sec.z - first.z) * (sec.z - first.z));
}

int MassSpringSystemSimulator::calculateAcc(float stiffness, float mass, vector<MassPoint> &points) {
	for (Spring& spr : springs) {
		//MassPoint first = points[spr.masspoint1];
		//MassPoint sec = points[spr.masspoint2];
		float dis = distance(points[spr.masspoint1].position, points[spr.masspoint2].position);
		//Vec3 direc1 = (sec.position - first.position) / dis;
		//Vec3 direc2 = (-1) * direc1;
		//nead to make it to Vec3

		Vec3 tmp = Vec3(points[spr.masspoint2].position.x - points[spr.masspoint1].position.x, points[spr.masspoint2].position.y - points[spr.masspoint1].position.y, points[spr.masspoint2].position.z - points[spr.masspoint1].position.z);

		tmp.operator*=((dis - spr.initialLength) * stiffness / mass / dis);

		//points[spr.masspoint1].Acc = Vec3(((dis - spr.initialLength) * stiffness / mass) * (points[spr.masspoint2].position.operator-=(points[spr.masspoint1].position)) / dis);
		points[spr.masspoint1].Acc = Vec3(tmp);
		//points[spr.masspoint1].Acc.x = tmp.x;
		//points[spr.masspoint1].Acc.y = tmp.y;
		//points[spr.masspoint1].Acc.z = tmp.z;

		points[spr.masspoint2].Acc = points[spr.masspoint1].Acc.operator-();
	}
	return 0;
}

void MassSpringSystemSimulator::midpoint(float timestep, float stiffness, float mass) {
	int len = this->points.size();
	vector<MassPoint> midPoints(points);
	calculateAcc(stiffness, mass, this->points);
	for (int i = 0; i < points.size(); i++) {
		MassPoint tmp;
		tmp.position = points[i].position + 0.5 * timestep * points[i].Velocity;
		tmp.Velocity = points[i].Velocity + 0.5 * timestep * points[i].Acc;
		midPoints[i] = tmp;
	}
	calculateAcc(stiffness, mass, midPoints);

	for (int i = 0; i < points.size(); i++) {
		points[i].position += timestep * midPoints[i].Velocity;
		points[i].Velocity += timestep * midPoints[i].Acc;
	}

	//return 0;
}

void MassSpringSystemSimulator::leapFrog(float timestep, float stiffness, float mass) {
	calculateAcc(stiffness, mass, points);
	for (MassPoint p : points) {
		p.Velocity += timestep * p.Acc;
		p.position += timestep * p.Velocity;
	}

	//return 0;
}


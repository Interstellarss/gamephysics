
#include "MassSpringSystemSimulator.h"


struct MassPoint {
	Vec3 position;
	Vec3 Velocity;
	bool isFixed;

};

struct Spring {
	int masspoint1;
	int masspoint2;
	float initialLength;
};

vector<MassPoint> points;

vector<Spring> springs;

void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
}


void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}




int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	MassPoint tmp;
	tmp.position = position;
	tmp.Velocity = Velocity;
	tmp.isFixed = isFixed;
	points.push_back(tmp);
	return points.size() - 1;
}

void addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring tmp;
	tmp.masspoint1 = masspoint1;
	tmp.masspoint2 = masspoint2;
	tmp.initialLength = initialLength;
	springs.push_back(tmp);
}

int getNumberOfMassPoints() {
	return points.size();
}

int getNumberOfSprings() {
	return springs.size();
}

Vec3 getPositionOfMassPoint(int index) {
	return points[index].position;
}


Vec3 getVelocityOfMassPoint(int index) {
	return points[index].Velocity;
}

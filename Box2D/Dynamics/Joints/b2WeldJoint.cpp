/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Dynamics/Joints/b2WeldJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2WeldJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor));
	this->referenceAngle = this->bodyB->GetAngle() - this->bodyA->GetAngle();
}

b2WeldJoint::b2WeldJoint(const b2WeldJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);
	this->m_referenceAngle = def->referenceAngle;
	this->m_frequencyHz = def->frequencyHz;
	this->m_dampingRatio = def->dampingRatio;

	this->m_impulse.SetZero();
}

void b2WeldJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexA = this->m_bodyA->m_islandIndex;
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_localCenterA.Assign(this->m_bodyA->m_sweep.localCenter);
	this->m_localCenterB.Assign(this->m_bodyB->m_sweep.localCenter);
	this->m_invMassA = this->m_bodyA->m_invMass;
	this->m_invMassB = this->m_bodyB->m_invMass;
	this->m_invIA = this->m_bodyA->m_invI;
	this->m_invIB = this->m_bodyB->m_invI;

	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;

	float32 aB = data.positions[this->m_indexB].a;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	b2Rot qA(aA), qB(aB);

	this->m_rA.Assign(b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA)));
	this->m_rB.Assign(b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB)));

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	b2Mat33 K;
	K.ex.x = mA + mB + this->m_rA.y * this->m_rA.y * iA + this->m_rB.y * this->m_rB.y * iB;
	K.ey.x = -this->m_rA.y * this->m_rA.x * iA - this->m_rB.y * this->m_rB.x * iB;
	K.ez.x = -this->m_rA.y * iA - this->m_rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + this->m_rA.x * this->m_rA.x * iA + this->m_rB.x * this->m_rB.x * iB;
	K.ez.y = this->m_rA.x * iA + this->m_rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;

	if (this->m_frequencyHz > 0.0)
	{
		K.GetInverse22(&this->m_mass);

		float32 invM = iA + iB;
		float32 m = invM > 0.0 ? 1.0 / invM : 0.0;

		float32 C = aB - aA - this->m_referenceAngle;

		// Frequency
		float32 omega = 2.0 * b2_pi * this->m_frequencyHz;

		// Damping coefficient
		float32 d = 2.0 * m * this->m_dampingRatio * omega;

		// Spring stiffness
		float32 k = m * omega * omega;

		// magic formulas
		float32 h = data.step.dt;
		this->m_gamma = h * (d + h * k);
		this->m_gamma = this->m_gamma != 0.0 ? 1.0 / this->m_gamma : 0.0;
		this->m_bias = C * h * k * this->m_gamma;

		invM += this->m_gamma;
		this->m_mass.ez.z = invM != 0.0 ? 1.0 / invM : 0.0;
	}
	else
	{
		K.GetSymInverse33(&this->m_mass);
		this->m_gamma = 0.0;
		this->m_bias = 0.0;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		this->m_impulse.Multiply(data.step.dtRatio);

		b2Vec2 P(this->m_impulse.x, this->m_impulse.y);

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * (b2Cross_v2_v2(this->m_rA, P) + this->m_impulse.z);

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * (b2Cross_v2_v2(this->m_rB, P) + this->m_impulse.z);
	}
	else
	{
		this->m_impulse.SetZero();
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2WeldJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	if (this->m_frequencyHz > 0.0)
	{
		float32 Cdot2 = wB - wA;

		float32 impulse2 = -this->m_mass.ez.z * (Cdot2 + this->m_bias + this->m_gamma * this->m_impulse.z);
		this->m_impulse.z += impulse2;

		wA -= iA * impulse2;
		wB += iB * impulse2;

		b2Vec2 Cdot1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB)), vA), b2Cross_f_v2(wA, this->m_rA));

		b2Vec2 impulse1 = b2Mul22_m33_v2(this->m_mass, Cdot1).Negate();
		this->m_impulse.x += impulse1.x;
		this->m_impulse.y += impulse1.y;

		b2Vec2 P = impulse1;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * b2Cross_v2_v2(this->m_rA, P);

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * b2Cross_v2_v2(this->m_rB, P);
	}
	else
	{
		b2Vec2 Cdot1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB)), vA), b2Cross_f_v2(wA, this->m_rA));
		float32 Cdot2 = wB - wA;
		b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

		b2Vec3 impulse = b2Mul_m33_v3(this->m_mass, Cdot).Negate();
		this->m_impulse.Add(impulse);

		b2Vec2 P(impulse.x, impulse.y);

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * (b2Cross_v2_v2(this->m_rA, P) + impulse.z);

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * (b2Cross_v2_v2(this->m_rB, P) + impulse.z);
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

bool b2WeldJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));

	float32 positionError, angularError;

	b2Mat33 K;
	K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.ez.x = -rA.y * iA - rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.ez.y = rA.x * iA + rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;

	if (this->m_frequencyHz > 0.0)
	{
		b2Vec2 C1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);

		positionError = C1.Length();
		angularError = 0.0;

		b2Vec2 P = K.Solve22(C1).Negate();

		cA.Subtract(b2Vec2::Multiply(mA, P));
		aA -= iA * b2Cross_v2_v2(rA, P);

		cB.Add(b2Vec2::Multiply(mB, P));
		aB += iB * b2Cross_v2_v2(rB, P);
	}
	else
	{
		b2Vec2 C1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);
		float32 C2 = aB - aA - this->m_referenceAngle;

		positionError = C1.Length();
		angularError = b2Abs(C2);

		b2Vec3 C(C1.x, C1.y, C2);
	
		b2Vec3 impulse = K.Solve33(C).Negate();
		b2Vec2 P(impulse.x, impulse.y);

		cA.Subtract(b2Vec2::Multiply(mA, P));
		aA -= iA * (b2Cross_v2_v2(rA, P) + impulse.z);

		cB.Add(b2Vec2::Multiply(mB, P));
		aB += iB * (b2Cross_v2_v2(rB, P) + impulse.z);
	}

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2WeldJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2WeldJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2WeldJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P(this->m_impulse.x, this->m_impulse.y);
	return b2Vec2::Multiply(inv_dt, P);
}

float32 b2WeldJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * this->m_impulse.z;
}

void b2WeldJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2WeldJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", this->m_referenceAngle);
	b2Log("  jd.frequencyHz = %.15lef;\n", this->m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", this->m_dampingRatio);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

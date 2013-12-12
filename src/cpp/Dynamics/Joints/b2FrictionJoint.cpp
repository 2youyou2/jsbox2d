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

#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2FrictionJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor));
}

b2FrictionJoint::b2FrictionJoint(const b2FrictionJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);

	this->m_linearImpulse.SetZero();
	this->m_angularImpulse = 0.0;

	this->m_maxForce = def->maxForce;
	this->m_maxTorque = def->maxTorque;
}

void b2FrictionJoint::InitVelocityConstraints(const b2SolverData& data)
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

	// Compute the effective mass matrix.
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

	b2Mat22 K;
	K.ex.x = mA + mB + iA * this->m_rA.y * this->m_rA.y + iB * this->m_rB.y * this->m_rB.y;
	K.ex.y = -iA * this->m_rA.x * this->m_rA.y - iB * this->m_rB.x * this->m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = mA + mB + iA * this->m_rA.x * this->m_rA.x + iB * this->m_rB.x * this->m_rB.x;

	this->m_linearMass.Assign(K.GetInverse());

	this->m_angularMass = iA + iB;
	if (this->m_angularMass > 0.0)
	{
		this->m_angularMass = 1.0 / this->m_angularMass;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		this->m_linearImpulse.Multiply(data.step.dtRatio);
		this->m_angularImpulse *= data.step.dtRatio;

		b2Vec2 P(this->m_linearImpulse.x, this->m_linearImpulse.y);
		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * (b2Cross_v2_v2(this->m_rA, P) + this->m_angularImpulse);
		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * (b2Cross_v2_v2(this->m_rB, P) + this->m_angularImpulse);
	}
	else
	{
		this->m_linearImpulse.SetZero();
		this->m_angularImpulse = 0.0;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2FrictionJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	float32 h = data.step.dt;

	// Solve angular friction
	{
		float32 Cdot = wB - wA;
		float32 impulse = -this->m_angularMass * Cdot;

		float32 oldImpulse = this->m_angularImpulse;
		float32 maxImpulse = h * this->m_maxTorque;
		this->m_angularImpulse = b2Clamp(this->m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = this->m_angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve linear friction
	{
		b2Vec2 Cdot = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB)), vA), b2Cross_f_v2(wA, this->m_rA));

		b2Vec2 impulse = b2Mul_m22_v2(this->m_linearMass, Cdot).Negate();
		b2Vec2 oldImpulse = this->m_linearImpulse;
		this->m_linearImpulse.Add(impulse);

		float32 maxImpulse = h * this->m_maxForce;

		if (this->m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			this->m_linearImpulse.Normalize();
			this->m_linearImpulse.Multiply(maxImpulse);
		}

		impulse.Assign(b2Vec2::Subtract(this->m_linearImpulse, oldImpulse));

		vA.Subtract(b2Vec2::Multiply(mA, impulse));
		wA -= iA * b2Cross_v2_v2(this->m_rA, impulse);

		vB.Add(b2Vec2::Multiply(mB, impulse));
		wB += iB * b2Cross_v2_v2(this->m_rB, impulse);
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

bool b2FrictionJoint::SolvePositionConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);

	return true;
}

b2Vec2 b2FrictionJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2FrictionJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2FrictionJoint::GetReactionForce(float32 inv_dt) const
{
	return b2Vec2::Multiply(inv_dt, this->m_linearImpulse);
}

float32 b2FrictionJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * this->m_angularImpulse;
}

void b2FrictionJoint::SetMaxForce(float32 force)
{
	b2Assert(b2IsValid(force) && force >= 0.0);
	this->m_maxForce = force;
}

float32 b2FrictionJoint::GetMaxForce() const
{
	return this->m_maxForce;
}

void b2FrictionJoint::SetMaxTorque(float32 torque)
{
	b2Assert(b2IsValid(torque) && torque >= 0.0);
	this->m_maxTorque = torque;
}

float32 b2FrictionJoint::GetMaxTorque() const
{
	return this->m_maxTorque;
}

void b2FrictionJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2FrictionJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.maxForce = %.15lef;\n", this->m_maxForce);
	b2Log("  jd.maxTorque = %.15lef;\n", this->m_maxTorque);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

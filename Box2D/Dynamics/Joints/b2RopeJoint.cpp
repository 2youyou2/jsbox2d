/*
* Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2RopeJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>


// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

b2RopeJoint::b2RopeJoint(const b2RopeJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA = def->localAnchorA;
	this->m_localAnchorB = def->localAnchorB;

	this->m_maxLength = def->maxLength;

	this->m_mass = 0.0;
	this->m_impulse = 0.0;
	this->m_state = b2Joint::e_inactiveLimit;
	this->m_length = 0.0;
}

void b2RopeJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexA = this->m_bodyA->m_islandIndex;
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_localCenterA = this->m_bodyA->m_sweep.localCenter;
	this->m_localCenterB = this->m_bodyB->m_sweep.localCenter;
	this->m_invMassA = this->m_bodyA->m_invMass;
	this->m_invMassB = this->m_bodyB->m_invMass;
	this->m_invIA = this->m_bodyA->m_invI;
	this->m_invIB = this->m_bodyB->m_invI;

	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;

	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	b2Rot qA(aA), qB(aB);

	this->m_rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	this->m_rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	this->m_u = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, this->m_rB), cA), this->m_rA);

	this->m_length = this->m_u.Length();

	float32 C = this->m_length - this->m_maxLength;
	if (C > 0.0)
	{
		this->m_state = b2Joint::e_atUpperLimit;
	}
	else
	{
		this->m_state = b2Joint::e_inactiveLimit;
	}

	if (this->m_length > b2_linearSlop)
	{
		this->m_u.Multiply(1.0 / this->m_length);
	}
	else
	{
		this->m_u.SetZero();
		this->m_mass = 0.0;
		this->m_impulse = 0.0;
		return;
	}

	// Compute effective mass.
	float32 crA = b2Cross_v2_v2(this->m_rA, this->m_u);
	float32 crB = b2Cross_v2_v2(this->m_rB, this->m_u);
	float32 invMass = this->m_invMassA + this->m_invIA * crA * crA + this->m_invMassB + this->m_invIB * crB * crB;

	this->m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

	if (data.step.warmStarting)
	{
		// Scale the impulse to support a variable time step.
		this->m_impulse *= data.step.dtRatio;

		b2Vec2 P = b2Vec2::Multiply(this->m_impulse, this->m_u);
		vA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
		wA -= this->m_invIA * b2Cross_v2_v2(this->m_rA, P);
		vB.Add(b2Vec2::Multiply(this->m_invMassB, P));
		wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, P);
	}
	else
	{
		this->m_impulse = 0.0;
	}

	data.velocities[this->m_indexA].v = vA;
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v = vB;
	data.velocities[this->m_indexB].w = wB;
}

void b2RopeJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	b2Vec2 vpA = b2Vec2::Add(vA, b2Cross_f_v2(wA, this->m_rA));
	b2Vec2 vpB = b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB));
	float32 C = this->m_length - this->m_maxLength;
	float32 Cdot = b2Dot_v2_v2(this->m_u, b2Vec2::Subtract(vpB, vpA));

	// Predictive constraint.
	if (C < 0.0)
	{
		Cdot += data.step.inv_dt * C;
	}

	float32 impulse = -this->m_mass * Cdot;
	float32 oldImpulse = this->m_impulse;
	this->m_impulse = b2Min(0.0, this->m_impulse + impulse);
	impulse = this->m_impulse - oldImpulse;

	b2Vec2 P = b2Vec2::Multiply(impulse, this->m_u);
	vA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
	wA -= this->m_invIA * b2Cross_v2_v2(this->m_rA, P);
	vB.Add(b2Vec2::Multiply(this->m_invMassB, P));
	wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, P);

	data.velocities[this->m_indexA].v = vA;
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v = vB;
	data.velocities[this->m_indexB].w = wB;
}

bool b2RopeJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	b2Vec2 u = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);

	float32 length = u.Normalize();
	float32 C = length - this->m_maxLength;

	C = b2Clamp(C, 0.0, b2_maxLinearCorrection);

	float32 impulse = -this->m_mass * C;
	b2Vec2 P = b2Vec2::Multiply(impulse, u);

	cA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
	aA -= this->m_invIA * b2Cross_v2_v2(rA, P);
	cB.Add(b2Vec2::Multiply(this->m_invMassB, P));
	aB += this->m_invIB * b2Cross_v2_v2(rB, P);

	data.positions[this->m_indexA].c = cA;
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c = cB;
	data.positions[this->m_indexB].a = aB;

	return length - this->m_maxLength < b2_linearSlop;
}

b2Vec2 b2RopeJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2RopeJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2RopeJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 F = b2Vec2::Multiply((inv_dt * this->m_impulse), this->m_u);
	return F;
}

float32 b2RopeJoint::GetReactionTorque(float32 inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return 0.0;
}

float32 b2RopeJoint::GetMaxLength() const
{
	return this->m_maxLength;
}

int b2RopeJoint::GetLimitState() const
{
	return this->m_state;
}

void b2RopeJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2RopeJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.maxLength = %.15lef;\n", this->m_maxLength);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

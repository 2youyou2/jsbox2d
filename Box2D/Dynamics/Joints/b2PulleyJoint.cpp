/*
* Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

void b2PulleyJointDef::Initialize(b2Body* bA, b2Body* bB,
				const b2Vec2& groundA, const b2Vec2& groundB,
				const b2Vec2& anchorA, const b2Vec2& anchorB,
				float32 r)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->groundAnchorA.Assign(groundA);
	this->groundAnchorB.Assign(groundB);
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchorA));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchorB));
	b2Vec2 dA = b2Vec2::Subtract(anchorA, groundA);
	this->lengthA = dA.Length();
	b2Vec2 dB = b2Vec2::Subtract(anchorB, groundB);
	this->lengthB = dB.Length();
	this->ratio = r;
	b2Assert(this->ratio > b2_epsilon);
}

b2PulleyJoint::b2PulleyJoint(const b2PulleyJointDef* def)
: b2Joint(def)
{
	this->m_groundAnchorA.Assign(def->groundAnchorA);
	this->m_groundAnchorB.Assign(def->groundAnchorB);
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);

	this->m_lengthA = def->lengthA;
	this->m_lengthB = def->lengthB;

	b2Assert(def->ratio != 0.0);
	this->m_ratio = def->ratio;

	this->m_constant = def->lengthA + this->m_ratio * def->lengthB;

	this->m_impulse = 0.0;
}

void b2PulleyJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexA = this->m_bodyA->m_islandIndex;
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_localCenterA.Assign(this->m_bodyA->m_sweep.localCenter);
	this->m_localCenterB.Assign(this->m_bodyB->m_sweep.localCenter);
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

	this->m_rA.Assign(b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA)));
	this->m_rB.Assign(b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB)));

	// Get the pulley axes.
	this->m_uA.Assign(b2Vec2::Add(cA, b2Vec2::Subtract(this->m_rA, this->m_groundAnchorA)));
	this->m_uB.Assign(b2Vec2::Add(cB, b2Vec2::Subtract(this->m_rB, this->m_groundAnchorB)));

	float32 lengthA = this->m_uA.Length();
	float32 lengthB = this->m_uB.Length();

	if (lengthA > 10.0 * b2_linearSlop)
	{
		this->m_uA.Multiply(1.0 / lengthA);
	}
	else
	{
		this->m_uA.SetZero();
	}

	if (lengthB > 10.0 * b2_linearSlop)
	{
		this->m_uB.Multiply(1.0 / lengthB);
	}
	else
	{
		this->m_uB.SetZero();
	}

	// Compute effective mass.
	float32 ruA = b2Cross_v2_v2(this->m_rA, this->m_uA);
	float32 ruB = b2Cross_v2_v2(this->m_rB, this->m_uB);

	float32 mA = this->m_invMassA + this->m_invIA * ruA * ruA;
	float32 mB = this->m_invMassB + this->m_invIB * ruB * ruB;

	this->m_mass = mA + this->m_ratio * this->m_ratio * mB;

	if (this->m_mass > 0.0)
	{
		this->m_mass = 1.0 / this->m_mass;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support variable time steps.
		this->m_impulse *= data.step.dtRatio;

		// Warm starting.
		b2Vec2 PA = b2Vec2::Multiply(-(this->m_impulse), this->m_uA);
		b2Vec2 PB = b2Vec2::Multiply((-this->m_ratio * this->m_impulse), this->m_uB);

		vA.Add(b2Vec2::Multiply(this->m_invMassA, PA));
		wA += this->m_invIA * b2Cross_v2_v2(this->m_rA, PA);
		vB.Add(b2Vec2::Multiply(this->m_invMassB, PB));
		wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, PB);
	}
	else
	{
		this->m_impulse = 0.0;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2PulleyJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	b2Vec2 vpA = b2Vec2::Add(vA, b2Cross_f_v2(wA, this->m_rA));
	b2Vec2 vpB = b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB));

	float32 Cdot = -b2Dot_v2_v2(this->m_uA, vpA) - this->m_ratio * b2Dot_v2_v2(this->m_uB, vpB);
	float32 impulse = -this->m_mass * Cdot;
	this->m_impulse += impulse;

	b2Vec2 PA = b2Vec2::Multiply(-impulse, this->m_uA);
	b2Vec2 PB = b2Vec2::Multiply(-this->m_ratio, b2Vec2::Multiply(impulse, this->m_uB));
	vA.Add(b2Vec2::Multiply(this->m_invMassA, PA));
	wA += this->m_invIA * b2Cross_v2_v2(this->m_rA, PA);
	vB.Add(b2Vec2::Multiply(this->m_invMassB, PB));
	wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, PB);

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

bool b2PulleyJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));

	// Get the pulley axes.
	b2Vec2 uA = b2Vec2::Add(cA, b2Vec2::Subtract(rA, this->m_groundAnchorA));
	b2Vec2 uB = b2Vec2::Add(cB, b2Vec2::Subtract(rB, this->m_groundAnchorB));

	float32 lengthA = uA.Length();
	float32 lengthB = uB.Length();

	if (lengthA > 10.0 * b2_linearSlop)
	{
		uA.Multiply(1.0 / lengthA);
	}
	else
	{
		uA.SetZero();
	}

	if (lengthB > 10.0 * b2_linearSlop)
	{
		uB.Multiply(1.0 / lengthB);
	}
	else
	{
		uB.SetZero();
	}

	// Compute effective mass.
	float32 ruA = b2Cross_v2_v2(rA, uA);
	float32 ruB = b2Cross_v2_v2(rB, uB);

	float32 mA = this->m_invMassA + this->m_invIA * ruA * ruA;
	float32 mB = this->m_invMassB + this->m_invIB * ruB * ruB;

	float32 mass = mA + this->m_ratio * this->m_ratio * mB;

	if (mass > 0.0)
	{
		mass = 1.0 / mass;
	}

	float32 C = this->m_constant - lengthA - this->m_ratio * lengthB;
	float32 linearError = b2Abs(C);

	float32 impulse = -mass * C;

	b2Vec2 PA = b2Vec2::Multiply(-impulse, uA);
	b2Vec2 PB = b2Vec2::Multiply(-this->m_ratio, b2Vec2::Multiply(impulse, uB));

	cA.Add(b2Vec2::Multiply(this->m_invMassA, PA));
	aA += this->m_invIA * b2Cross_v2_v2(rA, PA);
	cB.Add(b2Vec2::Multiply(this->m_invMassB, PB));
	aB += this->m_invIB * b2Cross_v2_v2(rB, PB);

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;

	return linearError < b2_linearSlop;
}

b2Vec2 b2PulleyJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2PulleyJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2PulleyJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P = b2Vec2::Multiply(this->m_impulse, this->m_uB);
	return b2Vec2::Multiply(inv_dt, P);
}

float32 b2PulleyJoint::GetReactionTorque(float32 inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return 0.0;
}

b2Vec2 b2PulleyJoint::GetGroundAnchorA() const
{
	return this->m_groundAnchorA;
}

b2Vec2 b2PulleyJoint::GetGroundAnchorB() const
{
	return this->m_groundAnchorB;
}

float32 b2PulleyJoint::GetLengthA() const
{
	return this->m_lengthA;
}

float32 b2PulleyJoint::GetLengthB() const
{
	return this->m_lengthB;
}

float32 b2PulleyJoint::GetRatio() const
{
	return this->m_ratio;
}

float32 b2PulleyJoint::GetCurrentLengthA() const
{
	b2Vec2 p = this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
	b2Vec2 s = this->m_groundAnchorA;
	b2Vec2 d = b2Vec2::Subtract(p, s);
	return d.Length();
}

float32 b2PulleyJoint::GetCurrentLengthB() const
{
	b2Vec2 p = this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
	b2Vec2 s = this->m_groundAnchorB;
	b2Vec2 d = b2Vec2::Subtract(p, s);
	return d.Length();
}

void b2PulleyJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2PulleyJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.groundAnchorA.Set(%.15lef, %.15lef);\n", this->m_groundAnchorA.x, this->m_groundAnchorA.y);
	b2Log("  jd.groundAnchorB.Set(%.15lef, %.15lef);\n", this->m_groundAnchorB.x, this->m_groundAnchorB.y);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.lengthA = %.15lef;\n", this->m_lengthA);
	b2Log("  jd.lengthB = %.15lef;\n", this->m_lengthB);
	b2Log("  jd.ratio = %.15lef;\n", this->m_ratio);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

void b2PulleyJoint::ShiftOrigin(const b2Vec2& newOrigin)
{
	this->m_groundAnchorA.Subtract(newOrigin);
	this->m_groundAnchorB.Subtract(newOrigin);
}

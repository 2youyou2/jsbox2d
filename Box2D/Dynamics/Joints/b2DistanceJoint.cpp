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

#include <Box2D/Dynamics/Joints/b2DistanceJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void b2DistanceJointDef::Initialize(b2Body* b1, b2Body* b2,
									const b2Vec2& anchor1, const b2Vec2& anchor2)
{
	this->bodyA = b1;
	this->bodyB = b2;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor1));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor2));
	b2Vec2 d = b2Vec2::Subtract(anchor2, anchor1);
	this->length = d.Length();
}

b2DistanceJoint::b2DistanceJoint(const b2DistanceJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);
	this->m_length = def->length;
	this->m_frequencyHz = def->frequencyHz;
	this->m_dampingRatio = def->dampingRatio;
	this->m_impulse = 0.0;
	this->m_gamma = 0.0;
	this->m_bias = 0.0;
}

void b2DistanceJoint::InitVelocityConstraints(const b2SolverData& data)
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
	this->m_u.Assign(b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, this->m_rB), cA), this->m_rA));

	// Handle singularity.
	float32 length = this->m_u.Length();
	if (length > b2_linearSlop)
	{
		this->m_u.Multiply(1.0 / length);
	}
	else
	{
		this->m_u.Set(0.0, 0.0);
	}

	float32 crAu = b2Cross_v2_v2(this->m_rA, this->m_u);
	float32 crBu = b2Cross_v2_v2(this->m_rB, this->m_u);
	float32 invMass = this->m_invMassA + this->m_invIA * crAu * crAu + this->m_invMassB + this->m_invIB * crBu * crBu;

	// Compute the effective mass matrix.
	this->m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

	if (this->m_frequencyHz > 0.0)
	{
		float32 C = length - this->m_length;

		// Frequency
		float32 omega = 2.0 * b2_pi * this->m_frequencyHz;

		// Damping coefficient
		float32 d = 2.0 * this->m_mass * this->m_dampingRatio * omega;

		// Spring stiffness
		float32 k = this->m_mass * omega * omega;

		// magic formulas
		float32 h = data.step.dt;
		this->m_gamma = h * (d + h * k);
		this->m_gamma = this->m_gamma != 0.0 ? 1.0 / this->m_gamma : 0.0;
		this->m_bias = C * h * k * this->m_gamma;

		invMass += this->m_gamma;
		this->m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
	}
	else
	{
		this->m_gamma = 0.0;
		this->m_bias = 0.0;
	}

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

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2DistanceJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	b2Vec2 vpA = b2Vec2::Add(vA, b2Cross_f_v2(wA, this->m_rA));
	b2Vec2 vpB = b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB));
	float32 Cdot = b2Dot_v2_v2(this->m_u, b2Vec2::Subtract(vpB, vpA));

	float32 impulse = -this->m_mass * (Cdot + this->m_bias + this->m_gamma * this->m_impulse);
	this->m_impulse += impulse;

	b2Vec2 P = b2Vec2::Multiply(impulse, this->m_u);
	vA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
	wA -= this->m_invIA * b2Cross_v2_v2(this->m_rA, P);
	vB.Add(b2Vec2::Multiply(this->m_invMassB, P));
	wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, P);

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

bool b2DistanceJoint::SolvePositionConstraints(const b2SolverData& data)
{
	if (this->m_frequencyHz > 0.0)
	{
		// There is no position correction for soft distance constraints.
		return true;
	}

	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	b2Vec2 u = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);

	float32 length = u.Normalize();
	float32 C = length - this->m_length;
	C = b2Clamp(C, -b2_maxLinearCorrection, b2_maxLinearCorrection);

	float32 impulse = -this->m_mass * C;
	b2Vec2 P = b2Vec2::Multiply(impulse, u);

	cA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
	aA -= this->m_invIA * b2Cross_v2_v2(rA, P);
	cB.Add(b2Vec2::Multiply(this->m_invMassB, P));
	aB += this->m_invIB * b2Cross_v2_v2(rB, P);

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;

	return b2Abs(C) < b2_linearSlop;
}

b2Vec2 b2DistanceJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2DistanceJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2DistanceJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 F = b2Vec2::Multiply((inv_dt * this->m_impulse), this->m_u);
	return F;
}

float32 b2DistanceJoint::GetReactionTorque(float32 inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return 0.0;
}

void b2DistanceJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2DistanceJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.length = %.15lef;\n", this->m_length);
	b2Log("  jd.frequencyHz = %.15lef;\n", this->m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", this->m_dampingRatio);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

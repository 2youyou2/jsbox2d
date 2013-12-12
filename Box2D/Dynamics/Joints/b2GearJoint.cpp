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

#include <Box2D/Dynamics/Joints/b2GearJoint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

b2GearJoint::b2GearJoint(const b2GearJointDef* def)
: b2Joint(def)
{
	this->m_joint1 = def->joint1;
	this->m_joint2 = def->joint2;

	this->m_typeA = this->m_joint1->GetType();
	this->m_typeB = this->m_joint2->GetType();

	b2Assert(this->m_typeA == b2Joint::e_revoluteJoint || this->m_typeA == b2Joint::e_prismaticJoint);
	b2Assert(this->m_typeB == b2Joint::e_revoluteJoint || this->m_typeB == b2Joint::e_prismaticJoint);

	float32 coordinateA, coordinateB;

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	this->m_bodyC = this->m_joint1->GetBodyA();
	this->m_bodyA = this->m_joint1->GetBodyB();

	// Get geometry of joint1
	b2Transform xfA = this->m_bodyA->m_xf;
	float32 aA = this->m_bodyA->m_sweep.a;
	b2Transform xfC = this->m_bodyC->m_xf;
	float32 aC = this->m_bodyC->m_sweep.a;

	if (this->m_typeA == b2Joint::e_revoluteJoint)
	{
		b2RevoluteJoint* revolute = (b2RevoluteJoint*)def->joint1;
		this->m_localAnchorC.Assign(revolute->m_localAnchorA);
		this->m_localAnchorA.Assign(revolute->m_localAnchorB);
		this->m_referenceAngleA = revolute->m_referenceAngle;
		this->m_localAxisC.SetZero();

		coordinateA = aA - aC - this->m_referenceAngleA;
	}
	else
	{
		b2PrismaticJoint* prismatic = (b2PrismaticJoint*)def->joint1;
		this->m_localAnchorC.Assign(prismatic->m_localAnchorA);
		this->m_localAnchorA.Assign(prismatic->m_localAnchorB);
		this->m_referenceAngleA = prismatic->m_referenceAngle;
		this->m_localAxisC.Assign(prismatic->m_localXAxisA);

		b2Vec2 pC = this->m_localAnchorC;
		b2Vec2 pA = b2MulT_r_v2(xfC.q, b2Vec2::Add(b2Mul_r_v2(xfA.q, this->m_localAnchorA), b2Vec2::Subtract(xfA.p, xfC.p)));
		coordinateA = b2Dot_v2_v2(b2Vec2::Subtract(pA, pC), this->m_localAxisC);
	}

	this->m_bodyD = this->m_joint2->GetBodyA();
	this->m_bodyB = this->m_joint2->GetBodyB();

	// Get geometry of joint2
	b2Transform xfB = this->m_bodyB->m_xf;
	float32 aB = this->m_bodyB->m_sweep.a;
	b2Transform xfD = this->m_bodyD->m_xf;
	float32 aD = this->m_bodyD->m_sweep.a;

	if (this->m_typeB == b2Joint::e_revoluteJoint)
	{
		b2RevoluteJoint* revolute = (b2RevoluteJoint*)def->joint2;
		this->m_localAnchorD.Assign(revolute->m_localAnchorA);
		this->m_localAnchorB.Assign(revolute->m_localAnchorB);
		this->m_referenceAngleB = revolute->m_referenceAngle;
		this->m_localAxisD.SetZero();

		coordinateB = aB - aD - this->m_referenceAngleB;
	}
	else
	{
		b2PrismaticJoint* prismatic = (b2PrismaticJoint*)def->joint2;
		this->m_localAnchorD.Assign(prismatic->m_localAnchorA);
		this->m_localAnchorB.Assign(prismatic->m_localAnchorB);
		this->m_referenceAngleB = prismatic->m_referenceAngle;
		this->m_localAxisD.Assign(prismatic->m_localXAxisA);

		b2Vec2 pD = this->m_localAnchorD;
		b2Vec2 pB = b2MulT_r_v2(xfD.q, b2Vec2::Add(b2Mul_r_v2(xfB.q, this->m_localAnchorB), b2Vec2::Subtract(xfB.p, xfD.p)));
		coordinateB = b2Dot_v2_v2(b2Vec2::Subtract(pB, pD), this->m_localAxisD);
	}

	this->m_ratio = def->ratio;

	this->m_constant = coordinateA + this->m_ratio * coordinateB;

	this->m_impulse = 0.0;
}

void b2GearJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexA = this->m_bodyA->m_islandIndex;
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_indexC = this->m_bodyC->m_islandIndex;
	this->m_indexD = this->m_bodyD->m_islandIndex;
	this->m_lcA.Assign(this->m_bodyA->m_sweep.localCenter);
	this->m_lcB.Assign(this->m_bodyB->m_sweep.localCenter);
	this->m_lcC.Assign(this->m_bodyC->m_sweep.localCenter);
	this->m_lcD.Assign(this->m_bodyD->m_sweep.localCenter);
	this->m_mA = this->m_bodyA->m_invMass;
	this->m_mB = this->m_bodyB->m_invMass;
	this->m_mC = this->m_bodyC->m_invMass;
	this->m_mD = this->m_bodyD->m_invMass;
	this->m_iA = this->m_bodyA->m_invI;
	this->m_iB = this->m_bodyB->m_invI;
	this->m_iC = this->m_bodyC->m_invI;
	this->m_iD = this->m_bodyD->m_invI;

	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;

	float32 aB = data.positions[this->m_indexB].a;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	float32 aC = data.positions[this->m_indexC].a;
	b2Vec2 vC = data.velocities[this->m_indexC].v;
	float32 wC = data.velocities[this->m_indexC].w;

	float32 aD = data.positions[this->m_indexD].a;
	b2Vec2 vD = data.velocities[this->m_indexD].v;
	float32 wD = data.velocities[this->m_indexD].w;

	b2Rot qA(aA), qB(aB), qC(aC), qD(aD);

	this->m_mass = 0.0;

	if (this->m_typeA == b2Joint::e_revoluteJoint)
	{
		this->m_JvAC.SetZero();
		this->m_JwA = 1.0;
		this->m_JwC = 1.0;
		this->m_mass += this->m_iA + this->m_iC;
	}
	else
	{
		b2Vec2 u = b2Mul_r_v2(qC, this->m_localAxisC);
		b2Vec2 rC = b2Mul_r_v2(qC, b2Vec2::Subtract(this->m_localAnchorC, this->m_lcC));
		b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_lcA));
		this->m_JvAC.Assign(u);
		this->m_JwC = b2Cross_v2_v2(rC, u);
		this->m_JwA = b2Cross_v2_v2(rA, u);
		this->m_mass += this->m_mC + this->m_mA + this->m_iC * this->m_JwC * this->m_JwC + this->m_iA * this->m_JwA * this->m_JwA;
	}

	if (this->m_typeB == b2Joint::e_revoluteJoint)
	{
		this->m_JvBD.SetZero();
		this->m_JwB = this->m_ratio;
		this->m_JwD = this->m_ratio;
		this->m_mass += this->m_ratio * this->m_ratio * (this->m_iB + this->m_iD);
	}
	else
	{
		b2Vec2 u = b2Mul_r_v2(qD, this->m_localAxisD);
		b2Vec2 rD = b2Mul_r_v2(qD, b2Vec2::Subtract(this->m_localAnchorD, this->m_lcD));
		b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_lcB));
		this->m_JvBD.Assign(b2Vec2::Multiply(this->m_ratio, u));
		this->m_JwD = this->m_ratio * b2Cross_v2_v2(rD, u);
		this->m_JwB = this->m_ratio * b2Cross_v2_v2(rB, u);
		this->m_mass += this->m_ratio * this->m_ratio * (this->m_mD + this->m_mB) + this->m_iD * this->m_JwD * this->m_JwD + this->m_iB * this->m_JwB * this->m_JwB;
	}

	// Compute effective mass.
	this->m_mass = this->m_mass > 0.0 ? 1.0 / this->m_mass : 0.0;

	if (data.step.warmStarting)
	{
		vA.Add(b2Vec2::Multiply((this->m_mA * this->m_impulse), this->m_JvAC));
		wA += this->m_iA * this->m_impulse * this->m_JwA;
		vB.Add(b2Vec2::Multiply((this->m_mB * this->m_impulse), this->m_JvBD));
		wB += this->m_iB * this->m_impulse * this->m_JwB;
		vC.Subtract(b2Vec2::Multiply((this->m_mC * this->m_impulse), this->m_JvAC));
		wC -= this->m_iC * this->m_impulse * this->m_JwC;
		vD.Subtract(b2Vec2::Multiply((this->m_mD * this->m_impulse), this->m_JvBD));
		wD -= this->m_iD * this->m_impulse * this->m_JwD;
	}
	else
	{
		this->m_impulse = 0.0;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
	data.velocities[this->m_indexC].v.Assign(vC);
	data.velocities[this->m_indexC].w = wC;
	data.velocities[this->m_indexD].v.Assign(vD);
	data.velocities[this->m_indexD].w = wD;
}

void b2GearJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;
	b2Vec2 vC = data.velocities[this->m_indexC].v;
	float32 wC = data.velocities[this->m_indexC].w;
	b2Vec2 vD = data.velocities[this->m_indexD].v;
	float32 wD = data.velocities[this->m_indexD].w;

	float32 Cdot = b2Dot_v2_v2(this->m_JvAC, b2Vec2::Subtract(vA, vC)) + b2Dot_v2_v2(this->m_JvBD, b2Vec2::Subtract(vB, vD));
	Cdot += (this->m_JwA * wA - this->m_JwC * wC) + (this->m_JwB * wB - this->m_JwD * wD);

	float32 impulse = -this->m_mass * Cdot;
	this->m_impulse += impulse;

	vA.Add(b2Vec2::Multiply((this->m_mA * impulse), this->m_JvAC));
	wA += this->m_iA * impulse * this->m_JwA;
	vB.Add(b2Vec2::Multiply((this->m_mB * impulse), this->m_JvBD));
	wB += this->m_iB * impulse * this->m_JwB;
	vC.Subtract(b2Vec2::Multiply((this->m_mC * impulse), this->m_JvAC));
	wC -= this->m_iC * impulse * this->m_JwC;
	vD.Subtract(b2Vec2::Multiply((this->m_mD * impulse), this->m_JvBD));
	wD -= this->m_iD * impulse * this->m_JwD;

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
	data.velocities[this->m_indexC].v.Assign(vC);
	data.velocities[this->m_indexC].w = wC;
	data.velocities[this->m_indexD].v.Assign(vD);
	data.velocities[this->m_indexD].w = wD;
}

bool b2GearJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;
	b2Vec2 cC = data.positions[this->m_indexC].c;
	float32 aC = data.positions[this->m_indexC].a;
	b2Vec2 cD = data.positions[this->m_indexD].c;
	float32 aD = data.positions[this->m_indexD].a;

	b2Rot qA(aA), qB(aB), qC(aC), qD(aD);

	float32 linearError = 0.0;

	float32 coordinateA, coordinateB;

	b2Vec2 JvAC, JvBD;
	float32 JwA, JwB, JwC, JwD;
	float32 mass = 0.0;

	if (this->m_typeA == b2Joint::e_revoluteJoint)
	{
		JvAC.SetZero();
		JwA = 1.0;
		JwC = 1.0;
		mass += this->m_iA + this->m_iC;

		coordinateA = aA - aC - this->m_referenceAngleA;
	}
	else
	{
		b2Vec2 u = b2Mul_r_v2(qC, this->m_localAxisC);
		b2Vec2 rC = b2Mul_r_v2(qC, b2Vec2::Subtract(this->m_localAnchorC, this->m_lcC));
		b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_lcA));
		JvAC.Assign(u);
		JwC = b2Cross_v2_v2(rC, u);
		JwA = b2Cross_v2_v2(rA, u);
		mass += this->m_mC + this->m_mA + this->m_iC * JwC * JwC + this->m_iA * JwA * JwA;

		b2Vec2 pC = b2Vec2::Subtract(this->m_localAnchorC, this->m_lcC);
		b2Vec2 pA = b2MulT_r_v2(qC, b2Vec2::Add(rA, b2Vec2::Subtract(cA, cC)));
		coordinateA = b2Dot_v2_v2(b2Vec2::Subtract(pA, pC), this->m_localAxisC);
	}

	if (this->m_typeB == b2Joint::e_revoluteJoint)
	{
		JvBD.SetZero();
		JwB = this->m_ratio;
		JwD = this->m_ratio;
		mass += this->m_ratio * this->m_ratio * (this->m_iB + this->m_iD);

		coordinateB = aB - aD - this->m_referenceAngleB;
	}
	else
	{
		b2Vec2 u = b2Mul_r_v2(qD, this->m_localAxisD);
		b2Vec2 rD = b2Mul_r_v2(qD, b2Vec2::Subtract(this->m_localAnchorD, this->m_lcD));
		b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_lcB));
		JvBD.Assign(b2Vec2::Multiply(this->m_ratio, u));
		JwD = this->m_ratio * b2Cross_v2_v2(rD, u);
		JwB = this->m_ratio * b2Cross_v2_v2(rB, u);
		mass += this->m_ratio * this->m_ratio * (this->m_mD + this->m_mB) + this->m_iD * JwD * JwD + this->m_iB * JwB * JwB;

		b2Vec2 pD = b2Vec2::Subtract(this->m_localAnchorD, this->m_lcD);
		b2Vec2 pB = b2MulT_r_v2(qD, b2Vec2::Add(rB, b2Vec2::Subtract(cB, cD)));
		coordinateB = b2Dot_v2_v2(b2Vec2::Subtract(pB, pD), this->m_localAxisD);
	}

	float32 C = (coordinateA + this->m_ratio * coordinateB) - this->m_constant;

	float32 impulse = 0.0;
	if (mass > 0.0)
	{
		impulse = -C / mass;
	}

	cA.Add(b2Vec2::Multiply(this->m_mA, b2Vec2::Multiply(impulse, JvAC)));
	aA += this->m_iA * impulse * JwA;
	cB.Add(b2Vec2::Multiply(this->m_mB, b2Vec2::Multiply(impulse, JvBD)));
	aB += this->m_iB * impulse * JwB;
	cC.Subtract(b2Vec2::Multiply(this->m_mC, b2Vec2::Multiply(impulse, JvAC)));
	aC -= this->m_iC * impulse * JwC;
	cD.Subtract(b2Vec2::Multiply(this->m_mD, b2Vec2::Multiply(impulse, JvBD)));
	aD -= this->m_iD * impulse * JwD;

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;
	data.positions[this->m_indexC].c.Assign(cC);
	data.positions[this->m_indexC].a = aC;
	data.positions[this->m_indexD].c.Assign(cD);
	data.positions[this->m_indexD].a = aD;

	// TODO_ERIN not implemented
	return linearError < b2_linearSlop;
}

b2Vec2 b2GearJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2GearJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2GearJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P = b2Vec2::Multiply(this->m_impulse, this->m_JvAC);
	return b2Vec2::Multiply(inv_dt, P);
}

float32 b2GearJoint::GetReactionTorque(float32 inv_dt) const
{
	float32 L = this->m_impulse * this->m_JwA;
	return inv_dt * L;
}

void b2GearJoint::SetRatio(float32 ratio)
{
	b2Assert(b2IsValid(ratio));
	this->m_ratio = ratio;
}

float32 b2GearJoint::GetRatio() const
{
	return this->m_ratio;
}

void b2GearJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	int32 index1 = this->m_joint1->m_index;
	int32 index2 = this->m_joint2->m_index;

	b2Log("  b2GearJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.joint1 = joints[%d];\n", index1);
	b2Log("  jd.joint2 = joints[%d];\n", index2);
	b2Log("  jd.ratio = %.15lef;\n", this->m_ratio);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

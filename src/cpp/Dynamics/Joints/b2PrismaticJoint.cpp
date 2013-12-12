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

#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

void b2PrismaticJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor));
	this->localAxisA.Assign(this->bodyA->GetLocalVector(axis));
	this->referenceAngle = this->bodyB->GetAngle() - this->bodyA->GetAngle();
}

b2PrismaticJoint::b2PrismaticJoint(const b2PrismaticJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);
	this->m_localXAxisA.Assign(def->localAxisA);
	this->m_localXAxisA.Normalize();
	this->m_localYAxisA.Assign(b2Cross_f_v2(1.0, this->m_localXAxisA));
	this->m_referenceAngle = def->referenceAngle;

	this->m_impulse.SetZero();
	this->m_motorMass = 0.0;
	this->m_motorImpulse = 0.0;

	this->m_lowerTranslation = def->lowerTranslation;
	this->m_upperTranslation = def->upperTranslation;
	this->m_maxMotorForce = def->maxMotorForce;
	this->m_motorSpeed = def->motorSpeed;
	this->m_enableLimit = def->enableLimit;
	this->m_enableMotor = def->enableMotor;
	this->m_limitState = b2Joint::e_inactiveLimit;

	this->m_axis.SetZero();
	this->m_perp.SetZero();
}

void b2PrismaticJoint::InitVelocityConstraints(const b2SolverData& data)
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

	// Compute the effective masses.
	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	b2Vec2 d = b2Vec2::Add(b2Vec2::Subtract(cB, cA), b2Vec2::Subtract(rB, rA));

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	// Compute motor Jacobian and effective mass.
	{
		this->m_axis.Assign(b2Mul_r_v2(qA, this->m_localXAxisA));
		this->m_a1 = b2Cross_v2_v2(b2Vec2::Add(d, rA), this->m_axis);
		this->m_a2 = b2Cross_v2_v2(rB, this->m_axis);

		this->m_motorMass = mA + mB + iA * this->m_a1 * this->m_a1 + iB * this->m_a2 * this->m_a2;
		if (this->m_motorMass > 0.0)
		{
			this->m_motorMass = 1.0 / this->m_motorMass;
		}
	}

	// Prismatic constraint.
	{
		this->m_perp.Assign(b2Mul_r_v2(qA, this->m_localYAxisA));

		this->m_s1 = b2Cross_v2_v2(b2Vec2::Add(d, rA), this->m_perp);
		this->m_s2 = b2Cross_v2_v2(rB, this->m_perp);

		float32 k11 = mA + mB + iA * this->m_s1 * this->m_s1 + iB * this->m_s2 * this->m_s2;
		float32 k12 = iA * this->m_s1 + iB * this->m_s2;
		float32 k13 = iA * this->m_s1 * this->m_a1 + iB * this->m_s2 * this->m_a2;
		float32 k22 = iA + iB;
		if (k22 == 0.0)
		{
			// For bodies with fixed rotation.
			k22 = 1.0;
		}
		float32 k23 = iA * this->m_a1 + iB * this->m_a2;
		float32 k33 = mA + mB + iA * this->m_a1 * this->m_a1 + iB * this->m_a2 * this->m_a2;

		this->m_K.ex.Set(k11, k12, k13);
		this->m_K.ey.Set(k12, k22, k23);
		this->m_K.ez.Set(k13, k23, k33);
	}

	// Compute motor and limit terms.
	if (this->m_enableLimit)
	{
		float32 jointTranslation = b2Dot_v2_v2(this->m_axis, d);
		if (b2Abs(this->m_upperTranslation - this->m_lowerTranslation) < 2.0 * b2_linearSlop)
		{
			this->m_limitState = b2Joint::e_equalLimits;
		}
		else if (jointTranslation <= this->m_lowerTranslation)
		{
			if (this->m_limitState != b2Joint::e_atLowerLimit)
			{
				this->m_limitState = b2Joint::e_atLowerLimit;
				this->m_impulse.z = 0.0;
			}
		}
		else if (jointTranslation >= this->m_upperTranslation)
		{
			if (this->m_limitState != b2Joint::e_atUpperLimit)
			{
				this->m_limitState = b2Joint::e_atUpperLimit;
				this->m_impulse.z = 0.0;
			}
		}
		else
		{
			this->m_limitState = b2Joint::e_inactiveLimit;
			this->m_impulse.z = 0.0;
		}
	}
	else
	{
		this->m_limitState = b2Joint::e_inactiveLimit;
		this->m_impulse.z = 0.0;
	}

	if (this->m_enableMotor == false)
	{
		this->m_motorImpulse = 0.0;
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		this->m_impulse.Multiply(data.step.dtRatio);
		this->m_motorImpulse *= data.step.dtRatio;

		b2Vec2 P = b2Vec2::Add(b2Vec2::Multiply(this->m_impulse.x, this->m_perp), b2Vec2::Multiply((this->m_motorImpulse + this->m_impulse.z), this->m_axis));
		float32 LA = this->m_impulse.x * this->m_s1 + this->m_impulse.y + (this->m_motorImpulse + this->m_impulse.z) * this->m_a1;
		float32 LB = this->m_impulse.x * this->m_s2 + this->m_impulse.y + (this->m_motorImpulse + this->m_impulse.z) * this->m_a2;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * LA;

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * LB;
	}
	else
	{
		this->m_impulse.SetZero();
		this->m_motorImpulse = 0.0;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2PrismaticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	// Solve linear motor constraint.
	if (this->m_enableMotor && this->m_limitState != b2Joint::e_equalLimits)
	{
		float32 Cdot = b2Dot_v2_v2(this->m_axis, b2Vec2::Subtract(vB, vA)) + this->m_a2 * wB - this->m_a1 * wA;
		float32 impulse = this->m_motorMass * (this->m_motorSpeed - Cdot);
		float32 oldImpulse = this->m_motorImpulse;
		float32 maxImpulse = data.step.dt * this->m_maxMotorForce;
		this->m_motorImpulse = b2Clamp(this->m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = this->m_motorImpulse - oldImpulse;

		b2Vec2 P = b2Vec2::Multiply(impulse, this->m_axis);
		float32 LA = impulse * this->m_a1;
		float32 LB = impulse * this->m_a2;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * LA;

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * LB;
	}

	b2Vec2 Cdot1;
	Cdot1.x = b2Dot_v2_v2(this->m_perp, b2Vec2::Subtract(vB, vA)) + this->m_s2 * wB - this->m_s1 * wA;
	Cdot1.y = wB - wA;

	if (this->m_enableLimit && this->m_limitState != b2Joint::e_inactiveLimit)
	{
		// Solve prismatic and limit constraint in block form.
		float32 Cdot2;
		Cdot2 = b2Dot_v2_v2(this->m_axis, b2Vec2::Subtract(vB, vA)) + this->m_a2 * wB - this->m_a1 * wA;
		b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

		b2Vec3 f1 = this->m_impulse;
		b2Vec3 df =  this->m_K.Solve33(Cdot.Negate());
		this->m_impulse.Add(df);

		if (this->m_limitState == b2Joint::e_atLowerLimit)
		{
			this->m_impulse.z = b2Max(this->m_impulse.z, 0.0);
		}
		else if (this->m_limitState == b2Joint::e_atUpperLimit)
		{
			this->m_impulse.z = b2Min(this->m_impulse.z, 0.0);
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		b2Vec2 b = b2Vec2::Subtract(Cdot1.Negate(), b2Vec2::Multiply((this->m_impulse.z - f1.z), b2Vec2(this->m_K.ez.x, this->m_K.ez.y)));
		b2Vec2 f2r = b2Vec2::Add(this->m_K.Solve22(b), b2Vec2(f1.x, f1.y));
		this->m_impulse.x = f2r.x;
		this->m_impulse.y = f2r.y;

		df.Assign(b2Vec3::Subtract(this->m_impulse, f1));

		b2Vec2 P = b2Vec2::Add(b2Vec2::Multiply(df.x, this->m_perp), b2Vec2::Multiply(df.z, this->m_axis));
		float32 LA = df.x * this->m_s1 + df.y + df.z * this->m_a1;
		float32 LB = df.x * this->m_s2 + df.y + df.z * this->m_a2;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * LA;

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * LB;
	}
	else
	{
		// Limit is inactive, just solve the prismatic constraint in block form.
		b2Vec2 df = this->m_K.Solve22(Cdot1.Negate());
		this->m_impulse.x += df.x;
		this->m_impulse.y += df.y;

		b2Vec2 P = b2Vec2::Multiply(df.x, this->m_perp);
		float32 LA = df.x * this->m_s1 + df.y;
		float32 LB = df.x * this->m_s2 + df.y;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * LA;

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * LB;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

bool b2PrismaticJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	// Compute fresh Jacobians
	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	b2Vec2 d = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);

	b2Vec2 axis = b2Mul_r_v2(qA, this->m_localXAxisA);
	float32 a1 = b2Cross_v2_v2(b2Vec2::Add(d, rA), axis);
	float32 a2 = b2Cross_v2_v2(rB, axis);
	b2Vec2 perp = b2Mul_r_v2(qA, this->m_localYAxisA);

	float32 s1 = b2Cross_v2_v2(b2Vec2::Add(d, rA), perp);
	float32 s2 = b2Cross_v2_v2(rB, perp);

	b2Vec3 impulse;
	b2Vec2 C1;
	C1.x = b2Dot_v2_v2(perp, d);
	C1.y = aB - aA - this->m_referenceAngle;

	float32 linearError = b2Abs(C1.x);
	float32 angularError = b2Abs(C1.y);

	bool active = false;
	float32 C2 = 0.0;
	if (this->m_enableLimit)
	{
		float32 translation = b2Dot_v2_v2(axis, d);
		if (b2Abs(this->m_upperTranslation - this->m_lowerTranslation) < 2.0 * b2_linearSlop)
		{
			// Prevent large angular corrections
			C2 = b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
			linearError = b2Max(linearError, b2Abs(translation));
			active = true;
		}
		else if (translation <= this->m_lowerTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = b2Clamp(translation - this->m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
			linearError = b2Max(linearError, this->m_lowerTranslation - translation);
			active = true;
		}
		else if (translation >= this->m_upperTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = b2Clamp(translation - this->m_upperTranslation - b2_linearSlop, 0.0, b2_maxLinearCorrection);
			linearError = b2Max(linearError, translation - this->m_upperTranslation);
			active = true;
		}
	}

	if (active)
	{
		float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float32 k12 = iA * s1 + iB * s2;
		float32 k13 = iA * s1 * a1 + iB * s2 * a2;
		float32 k22 = iA + iB;
		if (k22 == 0.0)
		{
			// For fixed rotation
			k22 = 1.0;
		}
		float32 k23 = iA * a1 + iB * a2;
		float32 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

		b2Mat33 K;
		K.ex.Set(k11, k12, k13);
		K.ey.Set(k12, k22, k23);
		K.ez.Set(k13, k23, k33);

		b2Vec3 C;
		C.x = C1.x;
		C.y = C1.y;
		C.z = C2;

		impulse.Assign(K.Solve33(C.Negate()));
	}
	else
	{
		float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float32 k12 = iA * s1 + iB * s2;
		float32 k22 = iA + iB;
		if (k22 == 0.0)
		{
			k22 = 1.0;
		}

		b2Mat22 K;
		K.ex.Set(k11, k12);
		K.ey.Set(k12, k22);

		b2Vec2 impulse1 = K.Solve(C1.Negate());
		impulse.x = impulse1.x;
		impulse.y = impulse1.y;
		impulse.z = 0.0;
	}

	b2Vec2 P = b2Vec2::Add(b2Vec2::Multiply(impulse.x, perp), b2Vec2::Multiply(impulse.z, axis));
	float32 LA = impulse.x * s1 + impulse.y + impulse.z * a1;
	float32 LB = impulse.x * s2 + impulse.y + impulse.z * a2;

	cA.Subtract(b2Vec2::Multiply(mA, P));
	aA -= iA * LA;
	cB.Add(b2Vec2::Multiply(mB, P));
	aB += iB * LB;

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;

	return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2PrismaticJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2PrismaticJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2PrismaticJoint::GetReactionForce(float32 inv_dt) const
{
	return b2Vec2::Multiply(inv_dt, b2Vec2::Add(b2Vec2::Multiply(this->m_impulse.x, this->m_perp), b2Vec2::Multiply((this->m_motorImpulse + this->m_impulse.z), this->m_axis)));
}

float32 b2PrismaticJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * this->m_impulse.y;
}

float32 b2PrismaticJoint::GetJointTranslation() const
{
	b2Vec2 pA = this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
	b2Vec2 pB = this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
	b2Vec2 d = b2Vec2::Subtract(pB, pA);
	b2Vec2 axis = this->m_bodyA->GetWorldVector(this->m_localXAxisA);

	float32 translation = b2Dot_v2_v2(d, axis);
	return translation;
}

float32 b2PrismaticJoint::GetJointSpeed() const
{
	b2Body* bA = this->m_bodyA;
	b2Body* bB = this->m_bodyB;

	b2Vec2 rA = b2Mul_r_v2(bA->m_xf.q, b2Vec2::Subtract(this->m_localAnchorA, bA->m_sweep.localCenter));
	b2Vec2 rB = b2Mul_r_v2(bB->m_xf.q, b2Vec2::Subtract(this->m_localAnchorB, bB->m_sweep.localCenter));
	b2Vec2 p1 = b2Vec2::Add(bA->m_sweep.c, rA);
	b2Vec2 p2 = b2Vec2::Add(bB->m_sweep.c, rB);
	b2Vec2 d = b2Vec2::Subtract(p2, p1);
	b2Vec2 axis = b2Mul_r_v2(bA->m_xf.q, this->m_localXAxisA);

	b2Vec2 vA = bA->m_linearVelocity;
	b2Vec2 vB = bB->m_linearVelocity;
	float32 wA = bA->m_angularVelocity;
	float32 wB = bB->m_angularVelocity;

	float32 speed = b2Dot_v2_v2(d, b2Cross_f_v2(wA, axis)) + b2Dot_v2_v2(axis, b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, rB)), vA), b2Cross_f_v2(wA, rA)));
	return speed;
}

bool b2PrismaticJoint::IsLimitEnabled() const
{
	return this->m_enableLimit;
}

void b2PrismaticJoint::EnableLimit(bool flag)
{
	if (flag != this->m_enableLimit)
	{
		this->m_bodyA->SetAwake(true);
		this->m_bodyB->SetAwake(true);
		this->m_enableLimit = flag;
		this->m_impulse.z = 0.0;
	}
}

float32 b2PrismaticJoint::GetLowerLimit() const
{
	return this->m_lowerTranslation;
}

float32 b2PrismaticJoint::GetUpperLimit() const
{
	return this->m_upperTranslation;
}

void b2PrismaticJoint::SetLimits(float32 lower, float32 upper)
{
	b2Assert(lower <= upper);
	if (lower != this->m_lowerTranslation || upper != this->m_upperTranslation)
	{
		this->m_bodyA->SetAwake(true);
		this->m_bodyB->SetAwake(true);
		this->m_lowerTranslation = lower;
		this->m_upperTranslation = upper;
		this->m_impulse.z = 0.0;
	}
}

bool b2PrismaticJoint::IsMotorEnabled() const
{
	return this->m_enableMotor;
}

void b2PrismaticJoint::EnableMotor(bool flag)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_enableMotor = flag;
}

void b2PrismaticJoint::SetMotorSpeed(float32 speed)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_motorSpeed = speed;
}

void b2PrismaticJoint::SetMaxMotorForce(float32 force)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_maxMotorForce = force;
}

float32 b2PrismaticJoint::GetMotorForce(float32 inv_dt) const
{
	return inv_dt * this->m_motorImpulse;
}

void b2PrismaticJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2PrismaticJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", this->m_localXAxisA.x, this->m_localXAxisA.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", this->m_referenceAngle);
	b2Log("  jd.enableLimit = bool(%d);\n", this->m_enableLimit);
	b2Log("  jd.lowerTranslation = %.15lef;\n", this->m_lowerTranslation);
	b2Log("  jd.upperTranslation = %.15lef;\n", this->m_upperTranslation);
	b2Log("  jd.enableMotor = bool(%d);\n", this->m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", this->m_motorSpeed);
	b2Log("  jd.maxMotorForce = %.15lef;\n", this->m_maxMotorForce);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

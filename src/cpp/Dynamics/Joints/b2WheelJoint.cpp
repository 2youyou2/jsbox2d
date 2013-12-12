/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor));
	this->localAxisA.Assign(this->bodyA->GetLocalVector(axis));
}

b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);
	this->m_localXAxisA.Assign(def->localAxisA);
	this->m_localYAxisA.Assign(b2Cross_f_v2(1.0, this->m_localXAxisA));

	this->m_mass = 0.0;
	this->m_impulse = 0.0;
	this->m_motorMass = 0.0;
	this->m_motorImpulse = 0.0;
	this->m_springMass = 0.0;
	this->m_springImpulse = 0.0;

	this->m_maxMotorTorque = def->maxMotorTorque;
	this->m_motorSpeed = def->motorSpeed;
	this->m_enableMotor = def->enableMotor;

	this->m_frequencyHz = def->frequencyHz;
	this->m_dampingRatio = def->dampingRatio;

	this->m_bias = 0.0;
	this->m_gamma = 0.0;

	this->m_ax.SetZero();
	this->m_ay.SetZero();
}

void b2WheelJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexA = this->m_bodyA->m_islandIndex;
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_localCenterA.Assign(this->m_bodyA->m_sweep.localCenter);
	this->m_localCenterB.Assign(this->m_bodyB->m_sweep.localCenter);
	this->m_invMassA = this->m_bodyA->m_invMass;
	this->m_invMassB = this->m_bodyB->m_invMass;
	this->m_invIA = this->m_bodyA->m_invI;
	this->m_invIB = this->m_bodyB->m_invI;

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

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
	b2Vec2 d = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);

	// Point to line constraint
	{
		this->m_ay.Assign(b2Mul_r_v2(qA, this->m_localYAxisA));
		this->m_sAy = b2Cross_v2_v2(b2Vec2::Add(d, rA), this->m_ay);
		this->m_sBy = b2Cross_v2_v2(rB, this->m_ay);

		this->m_mass = mA + mB + iA * this->m_sAy * this->m_sAy + iB * this->m_sBy * this->m_sBy;

		if (this->m_mass > 0.0)
		{
			this->m_mass = 1.0 / this->m_mass;
		}
	}

	// Spring constraint
	this->m_springMass = 0.0;
	this->m_bias = 0.0;
	this->m_gamma = 0.0;
	if (this->m_frequencyHz > 0.0)
	{
		this->m_ax.Assign(b2Mul_r_v2(qA, this->m_localXAxisA));
		this->m_sAx = b2Cross_v2_v2(b2Vec2::Add(d, rA), this->m_ax);
		this->m_sBx = b2Cross_v2_v2(rB, this->m_ax);

		float32 invMass = mA + mB + iA * this->m_sAx * this->m_sAx + iB * this->m_sBx * this->m_sBx;

		if (invMass > 0.0)
		{
			this->m_springMass = 1.0 / invMass;

			float32 C = b2Dot_v2_v2(d, this->m_ax);

			// Frequency
			float32 omega = 2.0 * b2_pi * this->m_frequencyHz;

			// Damping coefficient
			float32 d = 2.0 * this->m_springMass * this->m_dampingRatio * omega;

			// Spring stiffness
			float32 k = this->m_springMass * omega * omega;

			// magic formulas
			float32 h = data.step.dt;
			this->m_gamma = h * (d + h * k);
			if (this->m_gamma > 0.0)
			{
				this->m_gamma = 1.0 / this->m_gamma;
			}

			this->m_bias = C * h * k * this->m_gamma;

			this->m_springMass = invMass + this->m_gamma;
			if (this->m_springMass > 0.0)
			{
				this->m_springMass = 1.0 / this->m_springMass;
			}
		}
	}
	else
	{
		this->m_springImpulse = 0.0;
	}

	// Rotational motor
	if (this->m_enableMotor)
	{
		this->m_motorMass = iA + iB;
		if (this->m_motorMass > 0.0)
		{
			this->m_motorMass = 1.0 / this->m_motorMass;
		}
	}
	else
	{
		this->m_motorMass = 0.0;
		this->m_motorImpulse = 0.0;
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		this->m_impulse *= data.step.dtRatio;
		this->m_springImpulse *= data.step.dtRatio;
		this->m_motorImpulse *= data.step.dtRatio;

		b2Vec2 P = b2Vec2::Add(b2Vec2::Multiply(this->m_impulse, this->m_ay), b2Vec2::Multiply(this->m_springImpulse, this->m_ax));
		float32 LA = this->m_impulse * this->m_sAy + this->m_springImpulse * this->m_sAx + this->m_motorImpulse;
		float32 LB = this->m_impulse * this->m_sBy + this->m_springImpulse * this->m_sBx + this->m_motorImpulse;

		vA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
		wA -= this->m_invIA * LA;

		vB.Add(b2Vec2::Multiply(this->m_invMassB, P));
		wB += this->m_invIB * LB;
	}
	else
	{
		this->m_impulse = 0.0;
		this->m_springImpulse = 0.0;
		this->m_motorImpulse = 0.0;
	}

	data.velocities[this->m_indexA].v.Assign(vA);
	data.velocities[this->m_indexA].w = wA;
	data.velocities[this->m_indexB].v.Assign(vB);
	data.velocities[this->m_indexB].w = wB;
}

void b2WheelJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	// Solve spring constraint
	{
		float32 Cdot = b2Dot_v2_v2(this->m_ax, b2Vec2::Subtract(vB, vA)) + this->m_sBx * wB - this->m_sAx * wA;
		float32 impulse = -this->m_springMass * (Cdot + this->m_bias + this->m_gamma * this->m_springImpulse);
		this->m_springImpulse += impulse;

		b2Vec2 P = b2Vec2::Multiply(impulse, this->m_ax);
		float32 LA = impulse * this->m_sAx;
		float32 LB = impulse * this->m_sBx;

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * LA;

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * LB;
	}

	// Solve rotational motor constraint
	{
		float32 Cdot = wB - wA - this->m_motorSpeed;
		float32 impulse = -this->m_motorMass * Cdot;

		float32 oldImpulse = this->m_motorImpulse;
		float32 maxImpulse = data.step.dt * this->m_maxMotorTorque;
		this->m_motorImpulse = b2Clamp(this->m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = this->m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve point to line constraint
	{
		float32 Cdot = b2Dot_v2_v2(this->m_ay, b2Vec2::Subtract(vB, vA)) + this->m_sBy * wB - this->m_sAy * wA;
		float32 impulse = -this->m_mass * Cdot;
		this->m_impulse += impulse;

		b2Vec2 P = b2Vec2::Multiply(impulse, this->m_ay);
		float32 LA = impulse * this->m_sAy;
		float32 LB = impulse * this->m_sBy;

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

bool b2WheelJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
	b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));
	b2Vec2 d = b2Vec2::Add(b2Vec2::Subtract(cB, cA), b2Vec2::Subtract(rB, rA));

	b2Vec2 ay = b2Mul_r_v2(qA, this->m_localYAxisA);

	float32 sAy = b2Cross_v2_v2(b2Vec2::Add(d, rA), ay);
	float32 sBy = b2Cross_v2_v2(rB, ay);

	float32 C = b2Dot_v2_v2(d, ay);

	float32 k = this->m_invMassA + this->m_invMassB + this->m_invIA * this->m_sAy * this->m_sAy + this->m_invIB * this->m_sBy * this->m_sBy;

	float32 impulse;
	if (k != 0.0)
	{
		impulse = - C / k;
	}
	else
	{
		impulse = 0.0;
	}

	b2Vec2 P = b2Vec2::Multiply(impulse, ay);
	float32 LA = impulse * sAy;
	float32 LB = impulse * sBy;

	cA.Subtract(b2Vec2::Multiply(this->m_invMassA, P));
	aA -= this->m_invIA * LA;
	cB.Add(b2Vec2::Multiply(this->m_invMassB, P));
	aB += this->m_invIB * LB;

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;

	return b2Abs(C) <= b2_linearSlop;
}

b2Vec2 b2WheelJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2WheelJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2WheelJoint::GetReactionForce(float32 inv_dt) const
{
	return b2Vec2::Multiply(inv_dt, b2Vec2::Add(b2Vec2::Multiply(this->m_impulse, this->m_ay), b2Vec2::Multiply(this->m_springImpulse, this->m_ax)));
}

float32 b2WheelJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * this->m_motorImpulse;
}

float32 b2WheelJoint::GetJointTranslation() const
{
	b2Body* bA = this->m_bodyA;
	b2Body* bB = this->m_bodyB;

	b2Vec2 pA = bA->GetWorldPoint(this->m_localAnchorA);
	b2Vec2 pB = bB->GetWorldPoint(this->m_localAnchorB);
	b2Vec2 d = b2Vec2::Subtract(pB, pA);
	b2Vec2 axis = bA->GetWorldVector(this->m_localXAxisA);

	float32 translation = b2Dot_v2_v2(d, axis);
	return translation;
}

float32 b2WheelJoint::GetJointSpeed() const
{
	float32 wA = this->m_bodyA->m_angularVelocity;
	float32 wB = this->m_bodyB->m_angularVelocity;
	return wB - wA;
}

bool b2WheelJoint::IsMotorEnabled() const
{
	return this->m_enableMotor;
}

void b2WheelJoint::EnableMotor(bool flag)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_enableMotor = flag;
}

void b2WheelJoint::SetMotorSpeed(float32 speed)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_motorSpeed = speed;
}

void b2WheelJoint::SetMaxMotorTorque(float32 torque)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_maxMotorTorque = torque;
}

float32 b2WheelJoint::GetMotorTorque(float32 inv_dt) const
{
	return inv_dt * this->m_motorImpulse;
}

void b2WheelJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2WheelJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", this->m_localXAxisA.x, this->m_localXAxisA.y);
	b2Log("  jd.enableMotor = bool(%d);\n", this->m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", this->m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", this->m_maxMotorTorque);
	b2Log("  jd.frequencyHz = %.15lef;\n", this->m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", this->m_dampingRatio);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

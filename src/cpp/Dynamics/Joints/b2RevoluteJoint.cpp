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

#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2RevoluteJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	this->bodyA = bA;
	this->bodyB = bB;
	this->localAnchorA.Assign(this->bodyA->GetLocalPoint(anchor));
	this->localAnchorB.Assign(this->bodyB->GetLocalPoint(anchor));
	this->referenceAngle = this->bodyB->GetAngle() - this->bodyA->GetAngle();
}

b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def)
: b2Joint(def)
{
	this->m_localAnchorA.Assign(def->localAnchorA);
	this->m_localAnchorB.Assign(def->localAnchorB);
	this->m_referenceAngle = def->referenceAngle;

	this->m_impulse.SetZero();
	this->m_motorImpulse = 0.0;

	this->m_lowerAngle = def->lowerAngle;
	this->m_upperAngle = def->upperAngle;
	this->m_maxMotorTorque = def->maxMotorTorque;
	this->m_motorSpeed = def->motorSpeed;
	this->m_enableLimit = def->enableLimit;
	this->m_enableMotor = def->enableMotor;
	this->m_limitState = b2Joint::e_inactiveLimit;
}

void b2RevoluteJoint::InitVelocityConstraints(const b2SolverData& data)
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

	bool fixedRotation = (iA + iB == 0.0);

	this->m_mass.ex.x = mA + mB + this->m_rA.y * this->m_rA.y * iA + this->m_rB.y * this->m_rB.y * iB;
	this->m_mass.ey.x = -this->m_rA.y * this->m_rA.x * iA - this->m_rB.y * this->m_rB.x * iB;
	this->m_mass.ez.x = -this->m_rA.y * iA - this->m_rB.y * iB;
	this->m_mass.ex.y = this->m_mass.ey.x;
	this->m_mass.ey.y = mA + mB + this->m_rA.x * this->m_rA.x * iA + this->m_rB.x * this->m_rB.x * iB;
	this->m_mass.ez.y = this->m_rA.x * iA + this->m_rB.x * iB;
	this->m_mass.ex.z = this->m_mass.ez.x;
	this->m_mass.ey.z = this->m_mass.ez.y;
	this->m_mass.ez.z = iA + iB;

	this->m_motorMass = iA + iB;
	if (this->m_motorMass > 0.0)
	{
		this->m_motorMass = 1.0 / this->m_motorMass;
	}

	if (this->m_enableMotor == false || fixedRotation)
	{
		this->m_motorImpulse = 0.0;
	}

	if (this->m_enableLimit && fixedRotation == false)
	{
		float32 jointAngle = aB - aA - this->m_referenceAngle;
		if (b2Abs(this->m_upperAngle - this->m_lowerAngle) < 2.0 * b2_angularSlop)
		{
			this->m_limitState = b2Joint::e_equalLimits;
		}
		else if (jointAngle <= this->m_lowerAngle)
		{
			if (this->m_limitState != b2Joint::e_atLowerLimit)
			{
				this->m_impulse.z = 0.0;
			}
			this->m_limitState = b2Joint::e_atLowerLimit;
		}
		else if (jointAngle >= this->m_upperAngle)
		{
			if (this->m_limitState != b2Joint::e_atUpperLimit)
			{
				this->m_impulse.z = 0.0;
			}
			this->m_limitState = b2Joint::e_atUpperLimit;
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
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		this->m_impulse.Multiply(data.step.dtRatio);
		this->m_motorImpulse *= data.step.dtRatio;

		b2Vec2 P(this->m_impulse.x, this->m_impulse.y);

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * (b2Cross_v2_v2(this->m_rA, P) + this->m_motorImpulse + this->m_impulse.z);

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * (b2Cross_v2_v2(this->m_rB, P) + this->m_motorImpulse + this->m_impulse.z);
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

void b2RevoluteJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[this->m_indexA].v;
	float32 wA = data.velocities[this->m_indexA].w;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	float32 mA = this->m_invMassA, mB = this->m_invMassB;
	float32 iA = this->m_invIA, iB = this->m_invIB;

	bool fixedRotation = (iA + iB == 0.0);

	// Solve motor constraint.
	if (this->m_enableMotor && this->m_limitState != b2Joint::e_equalLimits && fixedRotation == false)
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

	// Solve limit constraint.
	if (this->m_enableLimit && this->m_limitState != b2Joint::e_inactiveLimit && fixedRotation == false)
	{
		b2Vec2 Cdot1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB)), vA), b2Cross_f_v2(wA, this->m_rA));
		float32 Cdot2 = wB - wA;
		b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

		b2Vec3 impulse = this->m_mass.Solve33(Cdot).Negate();

		if (this->m_limitState == b2Joint::e_equalLimits)
		{
			this->m_impulse.Add(impulse);
		}
		else if (this->m_limitState == b2Joint::e_atLowerLimit)
		{
			float32 newImpulse = this->m_impulse.z + impulse.z;
			if (newImpulse < 0.0)
			{
				b2Vec2 rhs = b2Vec2::Add(Cdot1.Negate(), b2Vec2::Multiply(this->m_impulse.z, b2Vec2(this->m_mass.ez.x, this->m_mass.ez.y)));
				b2Vec2 reduced = this->m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -this->m_impulse.z;
				this->m_impulse.x += reduced.x;
				this->m_impulse.y += reduced.y;
				this->m_impulse.z = 0.0;
			}
			else
			{
				this->m_impulse.Add(impulse);
			}
		}
		else if (this->m_limitState == b2Joint::e_atUpperLimit)
		{
			float32 newImpulse = this->m_impulse.z + impulse.z;
			if (newImpulse > 0.0)
			{
				b2Vec2 rhs = b2Vec2::Add(Cdot1.Negate(), b2Vec2::Multiply(this->m_impulse.z, b2Vec2(this->m_mass.ez.x, this->m_mass.ez.y)));
				b2Vec2 reduced = this->m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -this->m_impulse.z;
				this->m_impulse.x += reduced.x;
				this->m_impulse.y += reduced.y;
				this->m_impulse.z = 0.0;
			}
			else
			{
				this->m_impulse.Add(impulse);
			}
		}

		b2Vec2 P(impulse.x, impulse.y);

		vA.Subtract(b2Vec2::Multiply(mA, P));
		wA -= iA * (b2Cross_v2_v2(this->m_rA, P) + impulse.z);

		vB.Add(b2Vec2::Multiply(mB, P));
		wB += iB * (b2Cross_v2_v2(this->m_rB, P) + impulse.z);
	}
	else
	{
		// Solve point-to-point constraint
		b2Vec2 Cdot = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB)), vA), b2Cross_f_v2(wA, this->m_rA));
		b2Vec2 impulse = this->m_mass.Solve22(Cdot.Negate());

		this->m_impulse.x += impulse.x;
		this->m_impulse.y += impulse.y;

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

bool b2RevoluteJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[this->m_indexA].c;
	float32 aA = data.positions[this->m_indexA].a;
	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;

	b2Rot qA(aA), qB(aB);

	float32 angularError = 0.0;
	float32 positionError = 0.0;

	bool fixedRotation = (this->m_invIA + this->m_invIB == 0.0);

	// Solve angular limit constraint.
	if (this->m_enableLimit && this->m_limitState != b2Joint::e_inactiveLimit && fixedRotation == false)
	{
		float32 angle = aB - aA - this->m_referenceAngle;
		float32 limitImpulse = 0.0;

		if (this->m_limitState == b2Joint::e_equalLimits)
		{
			// Prevent large angular corrections
			float32 C = b2Clamp(angle - this->m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -this->m_motorMass * C;
			angularError = b2Abs(C);
		}
		else if (this->m_limitState == b2Joint::e_atLowerLimit)
		{
			float32 C = angle - this->m_lowerAngle;
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0.0);
			limitImpulse = -this->m_motorMass * C;
		}
		else if (this->m_limitState == b2Joint::e_atUpperLimit)
		{
			float32 C = angle - this->m_upperAngle;
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C - b2_angularSlop, 0.0, b2_maxAngularCorrection);
			limitImpulse = -this->m_motorMass * C;
		}

		aA -= this->m_invIA * limitImpulse;
		aB += this->m_invIB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA);
		qB.Set(aB);
		b2Vec2 rA = b2Mul_r_v2(qA, b2Vec2::Subtract(this->m_localAnchorA, this->m_localCenterA));
		b2Vec2 rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));

		b2Vec2 C = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(cB, rB), cA), rA);
		positionError = C.Length();

		float32 mA = this->m_invMassA, mB = this->m_invMassB;
		float32 iA = this->m_invIA, iB = this->m_invIB;

		b2Mat22 K;
		K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		b2Vec2 impulse = K.Solve(C).Negate();

		cA.Subtract(b2Vec2::Multiply(mA, impulse));
		aA -= iA * b2Cross_v2_v2(rA, impulse);

		cB.Add(b2Vec2::Multiply(mB, impulse));
		aB += iB * b2Cross_v2_v2(rB, impulse);
	}

	data.positions[this->m_indexA].c.Assign(cA);
	data.positions[this->m_indexA].a = aA;
	data.positions[this->m_indexB].c.Assign(cB);
	data.positions[this->m_indexB].a = aB;
	
	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2RevoluteJoint::GetAnchorA() const
{
	return this->m_bodyA->GetWorldPoint(this->m_localAnchorA);
}

b2Vec2 b2RevoluteJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2RevoluteJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P(this->m_impulse.x, this->m_impulse.y);
	return b2Vec2::Multiply(inv_dt, P);
}

float32 b2RevoluteJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * this->m_impulse.z;
}

float32 b2RevoluteJoint::GetJointAngle() const
{
	b2Body* bA = this->m_bodyA;
	b2Body* bB = this->m_bodyB;
	return bB->m_sweep.a - bA->m_sweep.a - this->m_referenceAngle;
}

float32 b2RevoluteJoint::GetJointSpeed() const
{
	b2Body* bA = this->m_bodyA;
	b2Body* bB = this->m_bodyB;
	return bB->m_angularVelocity - bA->m_angularVelocity;
}

bool b2RevoluteJoint::IsMotorEnabled() const
{
	return this->m_enableMotor;
}

void b2RevoluteJoint::EnableMotor(bool flag)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_enableMotor = flag;
}

float32 b2RevoluteJoint::GetMotorTorque(float32 inv_dt) const
{
	return inv_dt * this->m_motorImpulse;
}

void b2RevoluteJoint::SetMotorSpeed(float32 speed)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_motorSpeed = speed;
}

void b2RevoluteJoint::SetMaxMotorTorque(float32 torque)
{
	this->m_bodyA->SetAwake(true);
	this->m_bodyB->SetAwake(true);
	this->m_maxMotorTorque = torque;
}

bool b2RevoluteJoint::IsLimitEnabled() const
{
	return this->m_enableLimit;
}

void b2RevoluteJoint::EnableLimit(bool flag)
{
	if (flag != this->m_enableLimit)
	{
		this->m_bodyA->SetAwake(true);
		this->m_bodyB->SetAwake(true);
		this->m_enableLimit = flag;
		this->m_impulse.z = 0.0;
	}
}

float32 b2RevoluteJoint::GetLowerLimit() const
{
	return this->m_lowerAngle;
}

float32 b2RevoluteJoint::GetUpperLimit() const
{
	return this->m_upperAngle;
}

void b2RevoluteJoint::SetLimits(float32 lower, float32 upper)
{
	b2Assert(lower <= upper);
	
	if (lower != this->m_lowerAngle || upper != this->m_upperAngle)
	{
		this->m_bodyA->SetAwake(true);
		this->m_bodyB->SetAwake(true);
		this->m_impulse.z = 0.0;
		this->m_lowerAngle = lower;
		this->m_upperAngle = upper;
	}
}

void b2RevoluteJoint::Dump()
{
	int32 indexA = this->m_bodyA->m_islandIndex;
	int32 indexB = this->m_bodyB->m_islandIndex;

	b2Log("  b2RevoluteJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", this->m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this->m_localAnchorA.x, this->m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this->m_localAnchorB.x, this->m_localAnchorB.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", this->m_referenceAngle);
	b2Log("  jd.enableLimit = bool(%d);\n", this->m_enableLimit);
	b2Log("  jd.lowerAngle = %.15lef;\n", this->m_lowerAngle);
	b2Log("  jd.upperAngle = %.15lef;\n", this->m_upperAngle);
	b2Log("  jd.enableMotor = bool(%d);\n", this->m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", this->m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", this->m_maxMotorTorque);
	b2Log("  joints[%d] = this->m_world->CreateJoint(&jd);\n", this->m_index);
}

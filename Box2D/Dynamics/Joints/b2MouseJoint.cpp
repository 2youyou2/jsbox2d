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

#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

b2MouseJoint::b2MouseJoint(const b2MouseJointDef* def)
: b2Joint(def)
{
	b2Assert(def->target.IsValid());
	b2Assert(b2IsValid(def->maxForce) && def->maxForce >= 0.0);
	b2Assert(b2IsValid(def->frequencyHz) && def->frequencyHz >= 0.0);
	b2Assert(b2IsValid(def->dampingRatio) && def->dampingRatio >= 0.0);

	this->m_targetA = def->target;
	this->m_localAnchorB = b2MulT_t_v2(this->m_bodyB->GetTransform(), this->m_targetA);

	this->m_maxForce = def->maxForce;
	this->m_impulse.SetZero();

	this->m_frequencyHz = def->frequencyHz;
	this->m_dampingRatio = def->dampingRatio;

	this->m_beta = 0.0;
	this->m_gamma = 0.0;
}

void b2MouseJoint::SetTarget(const b2Vec2& target)
{
	if (this->m_bodyB->IsAwake() == false)
	{
		this->m_bodyB->SetAwake(true);
	}
	this->m_targetA = target;
}

const b2Vec2& b2MouseJoint::GetTarget() const
{
	return this->m_targetA;
}

void b2MouseJoint::SetMaxForce(float32 force)
{
	this->m_maxForce = force;
}

float32 b2MouseJoint::GetMaxForce() const
{
	return this->m_maxForce;
}

void b2MouseJoint::SetFrequency(float32 hz)
{
	this->m_frequencyHz = hz;
}

float32 b2MouseJoint::GetFrequency() const
{
	return this->m_frequencyHz;
}

void b2MouseJoint::SetDampingRatio(float32 ratio)
{
	this->m_dampingRatio = ratio;
}

float32 b2MouseJoint::GetDampingRatio() const
{
	return this->m_dampingRatio;
}

void b2MouseJoint::InitVelocityConstraints(const b2SolverData& data)
{
	this->m_indexB = this->m_bodyB->m_islandIndex;
	this->m_localCenterB = this->m_bodyB->m_sweep.localCenter;
	this->m_invMassB = this->m_bodyB->m_invMass;
	this->m_invIB = this->m_bodyB->m_invI;

	b2Vec2 cB = data.positions[this->m_indexB].c;
	float32 aB = data.positions[this->m_indexB].a;
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	b2Rot qB(aB);

	float32 mass = this->m_bodyB->GetMass();

	// Frequency
	float32 omega = 2.0 * b2_pi * this->m_frequencyHz;

	// Damping coefficient
	float32 d = 2.0 * mass * this->m_dampingRatio * omega;

	// Spring stiffness
	float32 k = mass * (omega * omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	float32 h = data.step.dt;
	b2Assert(d + h * k > b2_epsilon);
	this->m_gamma = h * (d + h * k);
	if (this->m_gamma != 0.0)
	{
		this->m_gamma = 1.0 / this->m_gamma;
	}
	this->m_beta = h * k * this->m_gamma;

	// Compute the effective mass matrix.
	this->m_rB = b2Mul_r_v2(qB, b2Vec2::Subtract(this->m_localAnchorB, this->m_localCenterB));

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2Mat22 K;
	K.ex.x = this->m_invMassB + this->m_invIB * this->m_rB.y * this->m_rB.y + this->m_gamma;
	K.ex.y = -this->m_invIB * this->m_rB.x * this->m_rB.y;
	K.ey.x = K.ex.y;
	K.ey.y = this->m_invMassB + this->m_invIB * this->m_rB.x * this->m_rB.x + this->m_gamma;

	this->m_mass = K.GetInverse();

	this->m_C = b2Vec2::Subtract(b2Vec2::Add(cB, this->m_rB), this->m_targetA);
	this->m_C.Multiply(this->m_beta);

	// Cheat with some damping
	wB *= 0.98f;

	if (data.step.warmStarting)
	{
		this->m_impulse.Multiply(data.step.dtRatio);
		vB.Add(b2Vec2::Multiply(this->m_invMassB, this->m_impulse));
		wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, this->m_impulse);
	}
	else
	{
		this->m_impulse.SetZero();
	}

	data.velocities[this->m_indexB].v = vB;
	data.velocities[this->m_indexB].w = wB;
}

void b2MouseJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vB = data.velocities[this->m_indexB].v;
	float32 wB = data.velocities[this->m_indexB].w;

	// Cdot = v + cross(w, r)
	b2Vec2 Cdot = b2Vec2::Add(vB, b2Cross_f_v2(wB, this->m_rB));
	b2Vec2 impulse = b2Mul_m22_v2(this->m_mass, (b2Vec2::Add(b2Vec2::Add(Cdot, this->m_C), b2Vec2::Multiply(this->m_gamma, this->m_impulse))).Negate());

	b2Vec2 oldImpulse = this->m_impulse;
	this->m_impulse.Add(impulse);
	float32 maxImpulse = data.step.dt * this->m_maxForce;
	if (this->m_impulse.LengthSquared() > maxImpulse * maxImpulse)
	{
		this->m_impulse.Multiply(maxImpulse / this->m_impulse.Length());
	}
	impulse = b2Vec2::Subtract(this->m_impulse, oldImpulse);

	vB.Add(b2Vec2::Multiply(this->m_invMassB, impulse));
	wB += this->m_invIB * b2Cross_v2_v2(this->m_rB, impulse);

	data.velocities[this->m_indexB].v = vB;
	data.velocities[this->m_indexB].w = wB;
}

bool b2MouseJoint::SolvePositionConstraints(const b2SolverData& data)
{
	B2_NOT_USED(data);
	return true;
}

b2Vec2 b2MouseJoint::GetAnchorA() const
{
	return this->m_targetA;
}

b2Vec2 b2MouseJoint::GetAnchorB() const
{
	return this->m_bodyB->GetWorldPoint(this->m_localAnchorB);
}

b2Vec2 b2MouseJoint::GetReactionForce(float32 inv_dt) const
{
	return b2Vec2::Multiply(inv_dt, this->m_impulse);
}

float32 b2MouseJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * 0.0;
}

void b2MouseJoint::ShiftOrigin(const b2Vec2& newOrigin)
{
	this->m_targetA.Subtract(newOrigin);
}

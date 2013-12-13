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

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
function b2WheelJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_wheelJoint;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.localAxisA = new b2Vec2(1.0, 0.0);
	this.enableMotor = false;
	this.maxMotorTorque = 0.0;
	this.motorSpeed = 0.0;
	this.frequencyHz = 2.0;
	this.dampingRatio = 0.7;
}

b2WheelJointDef.prototype =
{
	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	Initialize: function(bA, bB, anchor, axis)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.Assign(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.Assign(this.bodyB.GetLocalPoint(anchor));
		this.localAxisA.Assign(this.bodyA.GetLocalVector(axis));
	}
};

b2WheelJointDef._extend(b2JointDef);

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. You can use a
/// joint limit to restrict the range of motion and a joint motor to drive
/// the rotation or to model rotational friction.
/// This joint is designed for vehicle suspensions.
function b2WheelJoint(def)
{
	this.parent.call(this, def);

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();
	this.m_localXAxisA = def.localAxisA.Clone();
	this.m_localYAxisA = b2Cross_f_v2(1.0, this.m_localXAxisA);

	this.m_mass = 0.0;
	this.m_impulse = 0.0;
	this.m_motorMass = 0.0;
	this.m_motorImpulse = 0.0;
	this.m_springMass = 0.0;
	this.m_springImpulse = 0.0;

	this.m_maxMotorTorque = def.maxMotorTorque;
	this.m_motorSpeed = def.motorSpeed;
	this.m_enableMotor = def.enableMotor;

	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;

	this.m_bias = 0.0;
	this.m_gamma = 0.0;

	this.m_ax = new b2Vec2();
	this.m_ay = new b2Vec2();
	this.m_sAx = this.m_sBx = 0;
	this.m_sAy = this.m_sBy = 0;
}

b2WheelJoint.prototype =
{
	GetAnchorA: function() { return this.m_bodyA.GetWorldPoint(this.m_localAnchorA); },
	GetAnchorB: function() { return this.m_bodyB.GetWorldPoint(this.m_localAnchorB); },

	GetReactionForce: function(inv_dt) { return b2Vec2.Multiply(inv_dt, b2Vec2.Add(b2Vec2.Multiply(this.m_impulse, this.m_ay), b2Vec2.Multiply(this.m_springImpulse, this.m_ax))); },
	GetReactionTorque: function(inv_dt) { return inv_dt * this.m_motorImpulse; },

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// The local joint axis relative to bodyA.
	GetLocalAxisA: function() { return this.m_localXAxisA; },

	/// Get the current joint translation, usually in meters.
	GetJointTranslation: function()
	{
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;

		var pA = bA.GetWorldPoint(this.m_localAnchorA);
		var pB = bB.GetWorldPoint(this.m_localAnchorB);
		var d = b2Vec2.Subtract(pB, pA);
		var axis = bA.GetWorldVector(this.m_localXAxisA);

		var translation = b2Dot_v2_v2(d, axis);
		return translation;
	},

	/// Get the current joint translation speed, usually in meters per second.
	GetJointSpeed: function()
	{
		var wA = this.m_bodyA.m_angularVelocity;
		var wB = this.m_bodyB.m_angularVelocity;
		return wB - wA;
	},

	/// Is the joint motor enabled?
	IsMotorEnabled: function() { return this.m_enableMotor; },

	/// Enable/disable the joint motor.
	EnableMotor: function(flag)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	},

	/// Set the motor speed, usually in radians per second.
	SetMotorSpeed: function(speed)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	},

	/// Get the motor speed, usually in radians per second.
	GetMotorSpeed: function()
	{
		return this.m_motorSpeed;
	},

	/// Set/Get the maximum motor force, usually in N-m.
	SetMaxMotorTorque: function(torque)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorTorque = torque;
	},
	GetMaxMotorTorque: function()
	{
		return this.m_maxMotorTorque;
	},

	/// Get the current motor torque given the inverse time step, usually in N-m.
	GetMotorTorque: function(inv_dt) { return inv_dt * this.m_motorImpulse; },

	/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	SetSpringFrequencyHz: function(hz)
	{
		this.m_frequencyHz = hz;
	},
	GetSpringFrequencyHz: function()
	{
		return this.m_frequencyHz;
	},

	/// Set/Get the spring damping ratio
	SetSpringDampingRatio: function(ratio)
	{
		this.m_dampingRatio = ratio;
	},
	GetSpringDampingRatio: function()
	{
		return this.m_dampingRatio;
	},

	InitVelocityConstraints: function(data)
	{
		this.m_indexA = this.m_bodyA.m_islandIndex;
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_localCenterA.Assign(this.m_bodyA.m_sweep.localCenter);
		this.m_localCenterB.Assign(this.m_bodyB.m_sweep.localCenter);
		this.m_invMassA = this.m_bodyA.m_invMass;
		this.m_invMassB = this.m_bodyB.m_invMass;
		this.m_invIA = this.m_bodyA.m_invI;
		this.m_invIB = this.m_bodyB.m_invI;

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		// Compute the effective masses.
		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var d = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);

		// Point to line constraint
		{
			this.m_ay.Assign(b2Mul_r_v2(qA, this.m_localYAxisA));
			this.m_sAy = b2Cross_v2_v2(b2Vec2.Add(d, rA), this.m_ay);
			this.m_sBy = b2Cross_v2_v2(rB, this.m_ay);

			this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

			if (this.m_mass > 0.0)
			{
				this.m_mass = 1.0 / this.m_mass;
			}
		}

		// Spring constraint
		this.m_springMass = 0.0;
		this.m_bias = 0.0;
		this.m_gamma = 0.0;
		if (this.m_frequencyHz > 0.0)
		{
			this.m_ax.Assign(b2Mul_r_v2(qA, this.m_localXAxisA));
			this.m_sAx = b2Cross_v2_v2(b2Vec2.Add(d, rA), this.m_ax);
			this.m_sBx = b2Cross_v2_v2(rB, this.m_ax);

			var invMass = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;

			if (invMass > 0.0)
			{
				this.m_springMass = 1.0 / invMass;

				var C = b2Dot_v2_v2(d, this.m_ax);

				// Frequency
				var omega = 2.0 * b2_pi * this.m_frequencyHz;

				// Damping coefficient
				var d = 2.0 * this.m_springMass * this.m_dampingRatio * omega;

				// Spring stiffness
				var k = this.m_springMass * omega * omega;

				// magic formulas
				var h = data.step.dt;
				this.m_gamma = h * (d + h * k);
				if (this.m_gamma > 0.0)
				{
					this.m_gamma = 1.0 / this.m_gamma;
				}

				this.m_bias = C * h * k * this.m_gamma;

				this.m_springMass = invMass + this.m_gamma;
				if (this.m_springMass > 0.0)
				{
					this.m_springMass = 1.0 / this.m_springMass;
				}
			}
		}
		else
		{
			this.m_springImpulse = 0.0;
		}

		// Rotational motor
		if (this.m_enableMotor)
		{
			this.m_motorMass = iA + iB;
			if (this.m_motorMass > 0.0)
			{
				this.m_motorMass = 1.0 / this.m_motorMass;
			}
		}
		else
		{
			this.m_motorMass = 0.0;
			this.m_motorImpulse = 0.0;
		}

		if (data.step.warmStarting)
		{
			// Account for variable time step.
			this.m_impulse *= data.step.dtRatio;
			this.m_springImpulse *= data.step.dtRatio;
			this.m_motorImpulse *= data.step.dtRatio;

			var P = b2Vec2.Add(b2Vec2.Multiply(this.m_impulse, this.m_ay), b2Vec2.Multiply(this.m_springImpulse, this.m_ax));
			var LA = this.m_impulse * this.m_sAy + this.m_springImpulse * this.m_sAx + this.m_motorImpulse;
			var LB = this.m_impulse * this.m_sBy + this.m_springImpulse * this.m_sBx + this.m_motorImpulse;

			vA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
			wA -= this.m_invIA * LA;

			vB.Add(b2Vec2.Multiply(this.m_invMassB, P));
			wB += this.m_invIB * LB;
		}
		else
		{
			this.m_impulse = 0.0;
			this.m_springImpulse = 0.0;
			this.m_motorImpulse = 0.0;
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},

	SolveVelocityConstraints: function(data)
	{
		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		// Solve spring constraint
		{
			var Cdot = b2Dot_v2_v2(this.m_ax, b2Vec2.Subtract(vB, vA)) + this.m_sBx * wB - this.m_sAx * wA;
			var impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
			this.m_springImpulse += impulse;

			var P = b2Vec2.Multiply(impulse, this.m_ax);
			var LA = impulse * this.m_sAx;
			var LB = impulse * this.m_sBx;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}

		// Solve rotational motor constraint
		{
			var Cdot = wB - wA - this.m_motorSpeed;
			var impulse = -this.m_motorMass * Cdot;

			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = data.step.dt * this.m_maxMotorTorque;
			this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve point to line constraint
		{
			var Cdot = b2Dot_v2_v2(this.m_ay, b2Vec2.Subtract(vB, vA)) + this.m_sBy * wB - this.m_sAy * wA;
			var impulse = -this.m_mass * Cdot;
			this.m_impulse += impulse;

			var P = b2Vec2.Multiply(impulse, this.m_ay);
			var LA = impulse * this.m_sAy;
			var LB = impulse * this.m_sBy;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},

	SolvePositionConstraints: function(data)
	{
		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var d = b2Vec2.Add(b2Vec2.Subtract(cB, cA), b2Vec2.Subtract(rB, rA));

		var ay = b2Mul_r_v2(qA, this.m_localYAxisA);

		var sAy = b2Cross_v2_v2(b2Vec2.Add(d, rA), ay);
		var sBy = b2Cross_v2_v2(rB, ay);

		var C = b2Dot_v2_v2(d, ay);

		var k = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;

		var impulse;
		if (k != 0.0)
		{
			impulse = - C / k;
		}
		else
		{
			impulse = 0.0;
		}

		var P = b2Vec2.Multiply(impulse, ay);
		var LA = impulse * sAy;
		var LB = impulse * sBy;

		cA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
		aA -= this.m_invIA * LA;
		cB.Add(b2Vec2.Multiply(this.m_invMassB, P));
		aB += this.m_invIB * LB;

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return b2Abs(C) <= b2_linearSlop;
	}
};

b2WheelJoint._extend(b2Joint);

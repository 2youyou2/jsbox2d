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

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
function b2DistanceJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_distanceJoint;
	this.localAnchorA = new b2Vec2(0.0, 0.0);
	this.localAnchorB = new b2Vec2(0.0, 0.0);
	this.length = 1.0;
	this.frequencyHz = 0.0;
	this.dampingRatio = 0.0;

	Object.seal(this);
}

b2DistanceJointDef.prototype =
{
	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	Initialize: function(b1, b2,
					anchor1, anchor2)
	{
		this.bodyA = b1;
		this.bodyB = b2;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor1);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor2);
		var d = b2Vec2.Subtract(anchor2, anchor1);
		this.length = d.Length();
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.localAnchorA._deserialize(data['localAnchorA']);
		this.localAnchorB._deserialize(data['localAnchorB']);
		this.length = data['length'];
		this.frequencyHz = data['frequencyHz'];
		this.dampingRatio = data['dampingRatio'];
	}
};

b2DistanceJointDef._extend(b2JointDef);

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
function b2DistanceJoint(def)
{
	this.parent.call(this, def);

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();
	this.m_length = def.length;
	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;
	this.m_impulse = 0.0;
	this.m_gamma = 0.0;
	this.m_bias = 0.0;

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_u = new b2Vec2();
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_mass = 0;
}

b2DistanceJoint.prototype =
{
	GetAnchorA: function()
	{
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},
	GetAnchorB: function()
	{
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	GetReactionForce: function(inv_dt)
	{
		var F = b2Vec2.Multiply((inv_dt * this.m_impulse), this.m_u);
		return F;
	},

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	GetReactionTorque: function(inv_dt) { return 0.0; },

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	SetLength: function(length)
	{
		this.m_length = length;
	},
	GetLength: function()
	{
		return this.m_length;
	},

	/// Set/get frequency in Hz.
	SetFrequency: function(hz)
	{
		this.m_frequencyHz = hz;
	},
	GetFrequency: function()
	{
		return this.m_frequencyHz;
	},

	/// Set/get damping ratio.
	SetDampingRatio: function(ratio)
	{
		this.m_dampingRatio = ratio;
	},
	GetDampingRatio: function()
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

		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		this.m_rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		this.m_rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		this.m_u = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, this.m_rB), cA), this.m_rA);

		// Handle singularity.
		var length = this.m_u.Length();
		if (length > b2_linearSlop)
		{
			this.m_u.Multiply(1.0 / length);
		}
		else
		{
			this.m_u.Set(0.0, 0.0);
		}

		var crAu = b2Cross_v2_v2(this.m_rA, this.m_u);
		var crBu = b2Cross_v2_v2(this.m_rB, this.m_u);
		var invMass = this.m_invMassA + this.m_invIA * crAu * crAu + this.m_invMassB + this.m_invIB * crBu * crBu;

		// Compute the effective mass matrix.
		this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

		if (this.m_frequencyHz > 0.0)
		{
			var C = length - this.m_length;

			// Frequency
			var omega = 2.0 * b2_pi * this.m_frequencyHz;

			// Damping coefficient
			var d = 2.0 * this.m_mass * this.m_dampingRatio * omega;

			// Spring stiffness
			var k = this.m_mass * omega * omega;

			// magic formulas
			var h = data.step.dt;
			this.m_gamma = h * (d + h * k);
			this.m_gamma = this.m_gamma != 0.0 ? 1.0 / this.m_gamma : 0.0;
			this.m_bias = C * h * k * this.m_gamma;

			invMass += this.m_gamma;
			this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
		}
		else
		{
			this.m_gamma = 0.0;
			this.m_bias = 0.0;
		}

		if (data.step.warmStarting)
		{
			// Scale the impulse to support a variable time step.
			this.m_impulse *= data.step.dtRatio;

			var P = b2Vec2.Multiply(this.m_impulse, this.m_u);
			vA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
			wA -= this.m_invIA * b2Cross_v2_v2(this.m_rA, P);
			vB.Add(b2Vec2.Multiply(this.m_invMassB, P));
			wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, P);
		}
		else
		{
			this.m_impulse = 0.0;
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},

	SolveVelocityConstraints: function(data)
	{
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		// Cdot = dot(u, v + cross(w, r))
		var vpA = b2Vec2.Add(vA, b2Cross_f_v2(wA, this.m_rA));
		var vpB = b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB));
		var Cdot = b2Dot_v2_v2(this.m_u, b2Vec2.Subtract(vpB, vpA));

		var impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
		this.m_impulse += impulse;

		var P = b2Vec2.Multiply(impulse, this.m_u);
		vA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
		wA -= this.m_invIA * b2Cross_v2_v2(this.m_rA, P);
		vB.Add(b2Vec2.Multiply(this.m_invMassB, P));
		wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, P);

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},

	SolvePositionConstraints: function(data)
	{
		if (this.m_frequencyHz > 0.0)
		{
			// There is no position correction for soft distance constraints.
			return true;
		}

		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var u = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);

		var length = u.Normalize();
		var C = length - this.m_length;
		C = b2Clamp(C, -b2_maxLinearCorrection, b2_maxLinearCorrection);

		var impulse = -this.m_mass * C;
		var P = b2Vec2.Multiply(impulse, u);

		cA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
		aA -= this.m_invIA * b2Cross_v2_v2(rA, P);
		cB.Add(b2Vec2.Multiply(this.m_invMassB, P));
		aB += this.m_invIB * b2Cross_v2_v2(rB, P);

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return b2Abs(C) < b2_linearSlop;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['localAnchorA'] = this.m_localAnchorA._serialize();
		obj['localAnchorB'] = this.m_localAnchorB._serialize();
		obj['length'] = this.m_length;
		obj['frequencyHz'] = this.m_frequencyHz;
		obj['dampingRatio'] = this.m_dampingRatio;

		return obj;
	}
};

b2DistanceJoint._extend(b2Joint);
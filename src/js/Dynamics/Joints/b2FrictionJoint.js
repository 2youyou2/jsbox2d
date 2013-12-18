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

/// Friction joint definition.
function b2FrictionJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_frictionJoint;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.maxForce = 0.0;
	this.maxTorque = 0.0;

	Object.seal(this);
}

b2FrictionJointDef.prototype =
{
	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	Initialize: function(bA, bB, anchor)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.Assign(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.Assign(this.bodyB.GetLocalPoint(anchor));
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.localAnchorA._deserialize(data['localAnchorA']);
		this.localAnchorB._deserialize(data['localAnchorB']);
		this.maxForce = data['maxForce'];
		this.maxTorque = data['maxTorque'];
	}
};

b2FrictionJointDef._extend(b2JointDef);

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
function b2FrictionJoint(def)
{
	this.parent.call(this, def);

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();

	this.m_linearImpulse = new b2Vec2();
	this.m_angularImpulse = 0.0;

	this.m_maxForce = def.maxForce;
	this.m_maxTorque = def.maxTorque;

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_linearMass = new b2Mat22();
	this.m_angularMass = 0;
}

b2FrictionJoint.prototype =
{
	GetAnchorA: function()
	{
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},
	GetAnchorB: function()
	{
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	GetReactionForce: function(inv_dt)
	{
		return b2Vec2.Multiply(inv_dt, this.m_linearImpulse);
	},
	GetReactionTorque: function(inv_dt)
	{
		return inv_dt * this.m_angularImpulse;
	},

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// Set the maximum friction force in N.
	SetMaxForce: function(force)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(force) && force >= 0.0);
'#endif';
		this.m_maxForce = force;
	},

	/// Get the maximum friction force in N.
	GetMaxForce: function()
	{
		return this.m_maxForce;
	},

	/// Set the maximum friction torque in N*m.
	SetMaxTorque: function(torque)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(torque) && torque >= 0.0);
'#endif';
		this.m_maxTorque = torque;
	},

	/// Get the maximum friction torque in N*m.
	GetMaxTorque: function()
	{
		return this.m_maxTorque;
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

		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		// Compute the effective mass matrix.
		this.m_rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		this.m_rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var K = new b2Mat22();
		K.ex.x = mA + mB + iA * this.m_rA.y * this.m_rA.y + iB * this.m_rB.y * this.m_rB.y;
		K.ex.y = -iA * this.m_rA.x * this.m_rA.y - iB * this.m_rB.x * this.m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * this.m_rA.x * this.m_rA.x + iB * this.m_rB.x * this.m_rB.x;

		this.m_linearMass = K.GetInverse();

		this.m_angularMass = iA + iB;
		if (this.m_angularMass > 0.0)
		{
			this.m_angularMass = 1.0 / this.m_angularMass;
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			this.m_linearImpulse.Multiply(data.step.dtRatio);
			this.m_angularImpulse *= data.step.dtRatio;

			var P = new b2Vec2(this.m_linearImpulse.x, this.m_linearImpulse.y);
			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + this.m_angularImpulse);
			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + this.m_angularImpulse);
		}
		else
		{
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0.0;
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

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var h = data.step.dt;

		// Solve angular friction
		{
			var Cdot = wB - wA;
			var impulse = -this.m_angularMass * Cdot;

			var oldImpulse = this.m_angularImpulse;
			var maxImpulse = h * this.m_maxTorque;
			this.m_angularImpulse = b2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			var Cdot = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA));

			var impulse = b2Mul_m22_v2(this.m_linearMass, Cdot).Negate();
			var oldImpulse = this.m_linearImpulse;
			this.m_linearImpulse.Add(impulse);

			var maxImpulse = h * this.m_maxForce;

			if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				this.m_linearImpulse.Normalize();
				this.m_linearImpulse.Multiply(maxImpulse);
			}

			impulse = b2Vec2.Subtract(this.m_linearImpulse, oldImpulse);

			vA.Subtract(b2Vec2.Multiply(mA, impulse));
			wA -= iA * b2Cross_v2_v2(this.m_rA, impulse);

			vB.Add(b2Vec2.Multiply(mB, impulse));
			wB += iB * b2Cross_v2_v2(this.m_rB, impulse);
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},
	SolvePositionConstraints: function(data) { return true; },

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['localAnchorA'] = this.m_localAnchorA._serialize();
		obj['localAnchorB'] = this.m_localAnchorB._serialize();
		obj['maxForce'] = this.m_maxForce;
		obj['maxTorque'] = this.m_maxTorque;

		return obj;
	}
};

b2FrictionJoint._extend(b2Joint);
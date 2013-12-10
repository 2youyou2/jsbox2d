/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
function b2MouseJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_mouseJoint;
	this.target = new b2Vec2(0.0, 0.0);
	this.maxForce = 0.0;
	this.frequencyHz = 5.0;
	this.dampingRatio = 0.7;
}

b2MouseJointDef._extend(b2JointDef);

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
function b2MouseJoint(def)
{
	this.parent.call(this, def);

	b2Assert(def.target.IsValid());
	b2Assert(b2IsValid(def.maxForce) && def.maxForce >= 0.0);
	b2Assert(b2IsValid(def.frequencyHz) && def.frequencyHz >= 0.0);
	b2Assert(b2IsValid(def.dampingRatio) && def.dampingRatio >= 0.0);

	this.m_targetA = def.target.Clone();
	this.m_localAnchorB = b2MulT_t_v2(this.m_bodyB.GetTransform(), this.m_targetA);

	this.m_maxForce = def.maxForce;
	this.m_impulse = new b2Vec2();

	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;

	this.m_beta = 0.0;
	this.m_gamma = 0.0;

	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_rB = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassB = 0;
	this.m_invIB = 0;
	this.m_mass = new b2Mat22();
	this.m_C = new b2Vec2();
}

b2MouseJoint.prototype =
{
	/// Implements b2Joint.
	GetAnchorA: function()
	{
		return this.m_targetA;
	},

	/// Implements b2Joint.
	GetAnchorB: function()
	{
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	/// Implements b2Joint.
	GetReactionForce: function(inv_dt)
	{
		return b2Vec2.Multiply(inv_dt, this.m_impulse);
	},

	/// Implements b2Joint.
	GetReactionTorque: function(inv_dt)
	{
		return inv_dt * 0.0;
	},

	/// Use this to update the target point.
	SetTarget: function(target)
	{
		if (this.m_bodyB.IsAwake() == false)
		{
			this.m_bodyB.SetAwake(true);
		}
		this.m_targetA.Assign(target);
	},
	GetTarget: function()
	{
		return this.m_targetA;
	},

	/// Set/get the maximum force in Newtons.
	SetMaxForce: function(force)
	{
		this.m_maxForce = force;
	},
	GetMaxForce: function()
	{
		return this.m_maxForce;
	},

	/// Set/get the frequency in Hertz.
	SetFrequency: function(hz)
	{
		this.m_frequencyHz = hz;
	},
	GetFrequency: function()
	{
		return this.m_frequencyHz;
	},

	/// Set/get the damping ratio (dimensionless).
	SetDampingRatio: function(ratio)
	{
		this.m_dampingRatio = ratio;
	},
	GetDampingRatio: function()
	{
		return this.m_dampingRatio;
	},

	/// Implement b2Joint::ShiftOrigin
	ShiftOrigin: function(newOrigin)
	{
		this.m_targetA.Subtract(newOrigin);
	},

	InitVelocityConstraints: function(data)
	{
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_localCenterB = this.m_bodyB.m_sweep.localCenter;
		this.m_invMassB = this.m_bodyB.m_invMass;
		this.m_invIB = this.m_bodyB.m_invI;

		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qB = new b2Rot(aB);

		var mass = this.m_bodyB.GetMass();

		// Frequency
		var omega = 2.0 * b2_pi * this.m_frequencyHz;

		// Damping coefficient
		var d = 2.0 * mass * this.m_dampingRatio * omega;

		// Spring stiffness
		var k = mass * (omega * omega);

		// magic formulas
		// gamma has units of inverse mass.
		// beta has units of inverse time.
		var h = data.step.dt;
		b2Assert(d + h * k > b2_epsilon);
		this.m_gamma = h * (d + h * k);
		if (this.m_gamma != 0.0)
		{
			this.m_gamma = 1.0 / this.m_gamma;
		}
		this.m_beta = h * k * this.m_gamma;

		// Compute the effective mass matrix.
		this.m_rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		var K = new b2Mat22();
		K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
		K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;

		this.m_mass = K.GetInverse();

		this.m_C = b2Vec2.Subtract(b2Vec2.Add(cB, this.m_rB), this.m_targetA);
		this.m_C.Multiply(this.m_beta);

		// Cheat with some damping
		wB *= 0.98;

		if (data.step.warmStarting)
		{
			this.m_impulse.Multiply(data.step.dtRatio);
			vB.Add(b2Vec2.Multiply(this.m_invMassB, this.m_impulse));
			wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, this.m_impulse);
		}
		else
		{
			this.m_impulse.SetZero();
		}

		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},
	SolveVelocityConstraints: function(data)
	{
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		// Cdot = v + cross(w, r)
		var Cdot = b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB));
		var impulse = b2Mul_m22_v2(this.m_mass, (b2Vec2.Add(b2Vec2.Add(Cdot, this.m_C), b2Vec2.Multiply(this.m_gamma, this.m_impulse))).Negate());

		var oldImpulse = this.m_impulse;
		this.m_impulse.Add(impulse);
		var maxImpulse = data.step.dt * this.m_maxForce;
		if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse)
		{
			this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
		}
		impulse = b2Vec2.Subtract(this.m_impulse, oldImpulse);

		vB.Add(b2Vec2.Multiply(this.m_invMassB, impulse));
		wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, impulse);

		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},
	SolvePositionConstraints: function(data)
	{
		return true;
	}
};

b2MouseJoint._extend(b2Joint);
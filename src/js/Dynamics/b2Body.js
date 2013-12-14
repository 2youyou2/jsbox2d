"use strict";

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
function b2BodyDef()
{
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	this.type = b2Body.b2_staticBody;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	this.position = new b2Vec2(0.0, 0.0);

	/// The world angle of the body in radians.
	this.angle = 0.0;

	/// The linear velocity of the body's origin in world co-ordinates.
	this.linearVelocity = new b2Vec2(0.0, 0.0);

	/// The angular velocity of the body.
	this.angularVelocity = 0.0;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	this.linearDamping = 0.0;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	this.angularDamping = 0.0;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	this.allowSleep = true;

	/// Is this body initially awake or sleeping?
	this.awake = true;

	/// Should this body be prevented from rotating? Useful for characters.
	this.fixedRotation = false;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	this.bullet = false;

	/// Does this body start out active?
	this.active = true;

	/// Use this to store application specific body data.
	this.userData = null;

	/// Scale the gravity applied to this body.
	this.gravityScale = 1.0;
}

b2BodyDef.prototype =
{
	_deserialize: function(data)
	{
		this.type = data['type'];
		this.position._deserialize(data['position']);
		this.angle = data['angle'];
		this.linearVelocity._deserialize(data['linearVelocity']);
		this.angularVelocity = data['angularVelocity'];
		this.linearDamping = data['linearDamping'];
		this.angularDamping = data['angularDamping'];
		this.allowSleep = data['allowSleep'];
		this.awake = data['awake'];
		this.fixedRotation = data['fixedRotation'];
		this.bullet = data['bullet'];
		this.active = data['active'];
		this.gravityScale = data['gravityScale'];
	}
};

function b2Body(bd, world)
{
	b2Assert(bd.position.IsValid());
	b2Assert(bd.linearVelocity.IsValid());
	b2Assert(b2IsValid(bd.angle));
	b2Assert(b2IsValid(bd.angularVelocity));
	b2Assert(b2IsValid(bd.angularDamping) && bd.angularDamping >= 0.0);
	b2Assert(b2IsValid(bd.linearDamping) && bd.linearDamping >= 0.0);

	this.m_islandIndex = 0;
	this.m_flags = 0;

	if (bd.bullet)
	{
		this.m_flags |= b2Body.e_bulletFlag;
	}
	if (bd.fixedRotation)
	{
		this.m_flags |= b2Body.e_fixedRotationFlag;
	}
	if (bd.allowSleep)
	{
		this.m_flags |= b2Body.e_autoSleepFlag;
	}
	if (bd.awake)
	{
		this.m_flags |= b2Body.e_awakeFlag;
	}
	if (bd.active)
	{
		this.m_flags |= b2Body.e_activeFlag;
	}

	this.m_world = world;

	this.m_xf = new b2Transform();
	this.m_xf.p.Assign(bd.position);
	this.m_xf.q.Set(bd.angle);

	this.m_sweep = new b2Sweep();
	this.m_sweep.localCenter.SetZero();
	this.m_sweep.c0.Assign(this.m_xf.p);
	this.m_sweep.c.Assign(this.m_xf.p);
	this.m_sweep.a0 = bd.angle;
	this.m_sweep.a = bd.angle;
	this.m_sweep.alpha0 = 0.0;

	this.m_jointList = null;
	this.m_contactList = null;
	this.m_prev = null;
	this.m_next = null;

	this.m_linearVelocity = bd.linearVelocity.Clone();
	this.m_angularVelocity = bd.angularVelocity;

	this.m_linearDamping = bd.linearDamping;
	this.m_angularDamping = bd.angularDamping;
	this.m_gravityScale = bd.gravityScale;

	this.m_force = new b2Vec2();
	this.m_torque = 0.0;

	this.m_sleepTime = 0.0;

	this.m_type = bd.type;

	if (this.m_type == b2Body.b2_dynamicBody)
	{
		this.m_mass = 1.0;
		this.m_invMass = 1.0;
	}
	else
	{
		this.m_mass = 0.0;
		this.m_invMass = 0.0;
	}

	this.m_I = 0.0;
	this.m_invI = 0.0;

	this.m_userData = bd.userData;

	this.m_fixtureList = null;
	this.m_fixtureCount = 0;
}

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
b2Body.b2_staticBody = 0;
b2Body.b2_kinematicBody = 1;
b2Body.b2_dynamicBody = 2;
	// TODO_ERIN
	//b2_bulletBody,

b2Body.e_islandFlag = 0x0001;
b2Body.e_awakeFlag = 0x0002;
b2Body.e_autoSleepFlag = 0x0004;
b2Body.e_bulletFlag = 0x0008;
b2Body.e_fixedRotationFlag = 0x0010;
b2Body.e_activeFlag = 0x0020;
b2Body.e_toiFlag = 0x0040;

/// A rigid body. These are created via b2World.CreateBody.
b2Body.prototype =
{
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	CreateFixture: function(def, density)
	{
		if (typeof(density) !== 'undefined')
		{
			var ndef = new b2FixtureDef();
			ndef.shape = def;
			ndef.density = density;

			return this.CreateFixture(ndef);
		}

		b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true)
		{
			return null;
		}

		var fixture = new b2Fixture();
		fixture.Create(this, def);

		if (this.m_flags & b2Body.e_activeFlag)
		{
			var broadPhase = this.m_world.m_contactManager.m_broadPhase;
			fixture.CreateProxies(broadPhase, this.m_xf);
		}

		fixture.m_next = this.m_fixtureList;
		this.m_fixtureList = fixture;
		++this.m_fixtureCount;

		fixture.m_body = this;

		// Adjust mass properties if needed.
		if (fixture.m_density > 0.0)
		{
			this.ResetMassData();
		}

		// Let the world know we have a new fixture. This will cause new contacts
		// to be created at the beginning of the next time step.
		this.m_world.m_flags |= b2World.e_newFixture;

		return fixture;
	},

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	DestroyFixture: function(fixture)
	{
		b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true)
		{
			return;
		}

		b2Assert(fixture.m_body == this);

		// Remove the fixture from this body's singly linked list.
		b2Assert(this.m_fixtureCount > 0);
		var node = this.m_fixtureList;
		var found = false;

		while (node != null)
		{
			if (node == fixture)
			{
				this.m_fixtureList = node = fixture.m_next;
				found = true;
				break;
			}

			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		b2Assert(found);

		// Destroy any contacts associated with the fixture.
		var edge = this.m_contactList;
		while (edge)
		{
			var c = edge.contact;
			edge = edge.next;

			var fixtureA = c.GetFixtureA();
			var fixtureB = c.GetFixtureB();

			if (fixture == fixtureA || fixture == fixtureB)
			{
				// This destroys the contact and removes it from
				// this body's contact list.
				this.m_world.m_contactManager.Destroy(c);
			}
		}

		if (this.m_flags & b2Body.e_activeFlag)
		{
			var broadPhase = this.m_world.m_contactManager.m_broadPhase;
			fixture.DestroyProxies(broadPhase);
		}

		fixture.Destroy();
		fixture.m_body = null;
		fixture.m_next = null;

		--this.m_fixtureCount;

		// Reset the mass data.
		this.ResetMassData();
	},

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b2World.Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	SetTransform: function(position, angle)
	{
		b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true)
		{
			return;
		}

		this.m_xf.q.Set(angle);
		this.m_xf.p.Assign(position);

		this.m_sweep.c.Assign(b2Mul_t_v2(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.a = angle;

		this.m_sweep.c0.Assign(this.m_sweep.c);
		this.m_sweep.a0 = angle;

		var broadPhase = this.m_world.m_contactManager.m_broadPhase;
		for (var f = this.m_fixtureList; f; f = f.m_next)
		{
			f.Synchronize(broadPhase, this.m_xf, this.m_xf);
		}
	},

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	GetTransform: function()
	{
		return this.m_xf;
	},

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	GetPosition: function()
	{
		return this.m_xf.p;
	},

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	GetAngle: function()
	{
		return this.m_sweep.a;
	},

	/// Get the world position of the center of mass.
	GetWorldCenter: function()
	{
		return this.m_sweep.c;
	},

	/// Get the local position of the center of mass.
	GetLocalCenter: function()
	{
		return this.m_sweep.localCenter;
	},

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	SetLinearVelocity: function(v)
	{
		if (this.m_type == b2Body.b2_staticBody)
		{
			return;
		}

		if (b2Dot_v2_v2(v, v) > 0.0)
		{
			this.SetAwake(true);
		}

		this.m_linearVelocity = v;
	},

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	GetLinearVelocity: function()
	{
		return this.m_linearVelocity;
	},

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	SetAngularVelocity: function(w)
	{
		if (this.m_type == b2Body.b2_staticBody)
		{
			return;
		}

		if (w * w > 0.0)
		{
			this.SetAwake(true);
		}

		this.m_angularVelocity = w;
	},

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	GetAngularVelocity: function()
	{
		return this.m_angularVelocity;
	},

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	ApplyForce: function(force, point, wake)
	{
		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		if (wake && (this.m_flags & b2Body.e_awakeFlag) == 0)
		{
			this.SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping.
		if (this.m_flags & b2Body.e_awakeFlag)
		{
			this.m_force.Add(force);
			this.m_torque += b2Cross_v2_v2(b2Vec2.Subtract(point, this.m_sweep.c), force);
		}
	},

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	ApplyForceToCenter: function(force, wake)
	{
		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		if (wake && (this.m_flags & b2Body.e_awakeFlag) == 0)
		{
			this.SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if (this.m_flags & b2Body.e_awakeFlag)
		{
			this.m_force.Add(force);
		}
	},

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	ApplyTorque: function(torque, wake)
	{
		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		if (wake && (this.m_flags & b2Body.e_awakeFlag) == 0)
		{
			this.SetAwake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if (this.m_flags & b2Body.e_awakeFlag)
		{
			this.m_torque += torque;
		}
	},

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	ApplyLinearImpulse: function(impulse, point, wake)
	{
		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		if (wake && (this.m_flags & b2Body.e_awakeFlag) == 0)
		{
			this.SetAwake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if (this.m_flags & b2Body.e_awakeFlag)
		{
			this.m_linearVelocity.Add(b2Vec2.Multiply(this.m_invMass, impulse));
			this.m_angularVelocity += this.m_invI * b2Cross_v2_v2(b2Vec2.Subtract(point, this.m_sweep.c), impulse);
		}
	},

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	ApplyAngularImpulse: function(impulse, wake)
	{
		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		if (wake && (this.m_flags & b2Body.e_awakeFlag) == 0)
		{
			this.SetAwake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if (this.m_flags & b2Body.e_awakeFlag)
		{
			this.m_angularVelocity += this.m_invI * impulse;
		}
	},

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	GetMass: function()
	{
		return this.m_mass;
	},

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	GetInertia: function()
	{
		return this.m_I + this.m_mass * b2Dot_v2_v2(this.m_sweep.localCenter, this.m_sweep.localCenter);
	},

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	GetMassData: function(data)
	{
		data.mass = this.m_mass;
		data.I = this.m_I + this.m_mass * b2Dot_v2_v2(this.m_sweep.localCenter, this.m_sweep.localCenter);
		data.center = this.m_sweep.localCenter;
	},

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	SetMassData: function(massData)
	{
		b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true)
		{
			return;
		}

		if (this.m_type != b2Body.b2_dynamicBody)
		{
			return;
		}

		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;

		this.m_mass = massData.mass;
		if (this.m_mass <= 0.0)
		{
			this.m_mass = 1.0;
		}

		this.m_invMass = 1.0 / this.m_mass;

		if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			this.m_I = massData.I - this.m_mass * b2Dot_v2_v2(massData.center, massData.center);
			b2Assert(this.m_I > 0.0);
			this.m_invI = 1.0 / this.m_I;
		}

		// Move center of mass.
		var oldCenter = this.m_sweep.c.Clone();
		this.m_sweep.localCenter.Assign(massData.center);
		this.m_sweep.c0.Assign(b2Mul_t_v2(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.Assign(this.m_sweep.c0);

		// Update center of mass velocity.
		this.m_linearVelocity.Add(b2Cross_f_v2(this.m_angularVelocity, b2Vec2.Subtract(this.m_sweep.c, oldCenter)));
	},

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	ResetMassData: function()
	{
		// Compute mass data from shapes. Each shape has its own density.
		this.m_mass = 0.0;
		this.m_invMass = 0.0;
		this.m_I = 0.0;
		this.m_invI = 0.0;
		this.m_sweep.localCenter.SetZero();

		// Static and kinematic bodies have zero mass.
		if (this.m_type == b2Body.b2_staticBody || this.m_type == b2Body.b2_kinematicBody)
		{
			this.m_sweep.c0.Assign(this.m_xf.p);
			this.m_sweep.c.Assign(this.m_xf.p);
			this.m_sweep.a0 = this.m_sweep.a;
			return;
		}

		b2Assert(this.m_type == b2Body.b2_dynamicBody);

		// Accumulate mass over all fixtures.
		var localCenter = new b2Vec2(0, 0);
		for (var f = this.m_fixtureList; f; f = f.m_next)
		{
			if (f.m_density == 0.0)
			{
				continue;
			}

			var massData = new b2MassData();
			f.GetMassData(massData);
			this.m_mass += massData.mass;
			localCenter.Add(b2Vec2.Multiply(massData.mass, massData.center));
			this.m_I += massData.I;
		}

		// Compute center of mass.
		if (this.m_mass > 0.0)
		{
			this.m_invMass = 1.0 / this.m_mass;
			localCenter.Multiply(this.m_invMass);
		}
		else
		{
			// Force all dynamic bodies to have a positive mass.
			this.m_mass = 1.0;
			this.m_invMass = 1.0;
		}

		if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0)
		{
			// Center the inertia about the center of mass.
			this.m_I -= this.m_mass * b2Dot_v2_v2(localCenter, localCenter);
			b2Assert(this.m_I > 0.0);
			this.m_invI = 1.0 / this.m_I;

		}
		else
		{
			this.m_I = 0.0;
			this.m_invI = 0.0;
		}

		// Move center of mass.
		var oldCenter = this.m_sweep.c.Clone();
		this.m_sweep.localCenter.Assign(localCenter);
		this.m_sweep.c0.Assign(b2Mul_t_v2(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.Assign(this.m_sweep.c0);

		// Update center of mass velocity.
		this.m_linearVelocity.Add(b2Cross_f_v2(this.m_angularVelocity, b2Vec2.Subtract(this.m_sweep.c, oldCenter)));
	},

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	GetWorldPoint: function(localPoint)
	{
		return b2Mul_t_v2(this.m_xf, localPoint);
	},

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	GetWorldVector: function(localVector)
	{
		return b2Mul_r_v2(this.m_xf.q, localVector);
	},

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	GetLocalPoint: function(worldPoint)
	{
		return b2MulT_t_v2(this.m_xf, worldPoint);
	},

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	GetLocalVector: function(worldVector)
	{
		return b2MulT_r_v2(this.m_xf.q, worldVector);
	},

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	GetLinearVelocityFromWorldPoint: function(worldPoint)
	{
		return b2Vec2.Add(this.m_linearVelocity, b2Cross_f_v2(this.m_angularVelocity, b2Vec2.Subtract(worldPoint, this.m_sweep.c)));
	},

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	GetLinearVelocityFromLocalPoint: function(localPoint)
	{
		return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint));
	},

	/// Get the linear damping of the body.
	GetLinearDamping: function()
	{
		return this.m_linearDamping;
	},

	/// Set the linear damping of the body.
	SetLinearDamping: function(linearDamping)
	{
		this.m_linearDamping = linearDamping;
	},

	/// Get the angular damping of the body.
	GetAngularDamping: function()
	{
		return this.m_angularDamping;
	},

	/// Set the angular damping of the body.
	SetAngularDamping: function(angularDamping)
	{
		this.m_angularDamping = angularDamping;
	},

	/// Get the gravity scale of the body.
	GetGravityScale: function()
	{
		return this.m_gravityScale;
	},

	/// Set the gravity scale of the body.
	SetGravityScale: function(scale)
	{
		this.m_gravityScale = scale;
	},

	/// Set the type of this body. This may alter the mass and velocity.
	SetType: function(type)
	{
		b2Assert(this.m_world.IsLocked() == false);
		if (this.m_world.IsLocked() == true)
		{
			return;
		}

		if (this.m_type == type)
		{
			return;
		}

		this.m_type = type;

		this.ResetMassData();

		if (this.m_type == b2Body.b2_staticBody)
		{
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			this.m_sweep.a0 = this.m_sweep.a;
			this.m_sweep.c0.Assign(this.m_sweep.c);
			this.SynchronizeFixtures();
		}

		this.SetAwake(true);

		this.m_force.SetZero();
		this.m_torque = 0.0;

		// Delete the attached contacts.
		var ce = this.m_contactList;
		while (ce)
		{
			var ce0 = ce;
			ce = ce.next;
			this.m_world.m_contactManager.Destroy(ce0.contact);
		}
		this.m_contactList = null;

		// Touch the proxies so that new contacts will be created (when appropriate)
		var broadPhase = this.m_world.m_contactManager.m_broadPhase;
		for (var f = this.m_fixtureList; f; f = f.m_next)
		{
			var proxyCount = f.m_proxyCount;
			for (var i = 0; i < proxyCount; ++i)
			{
				broadPhase.TouchProxy(f.m_proxies[i].proxyId);
			}
		}
	},

	/// Get the type of this body.
	GetType: function()
	{
		return this.m_type;
	},

	/// Should this body be treated like a bullet for continuous collision detection?
	SetBullet: function(flag)
	{
		if (flag)
		{
			this.m_flags |= b2Body.e_bulletFlag;
		}
		else
		{
			this.m_flags &= ~b2Body.e_bulletFlag;
		}
	},

	/// Is this body treated like a bullet for continuous collision detection?
	IsBullet: function()
	{
		return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
	},

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	SetSleepingAllowed: function(flag)
	{
		if (flag)
		{
			this.m_flags |= b2Body.e_autoSleepFlag;
		}
		else
		{
			this.m_flags &= ~b2Body.e_autoSleepFlag;
			this.SetAwake(true);
		}
	},

	/// Is this body allowed to sleep
	IsSleepingAllowed: function()
	{
		return (this.m_flags & b2Body.e_autoSleepFlag) == b2Body.e_autoSleepFlag;
	},

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	SetAwake: function(flag)
	{
		if (flag)
		{
			if ((this.m_flags & b2Body.e_awakeFlag) == 0)
			{
				this.m_flags |= b2Body.e_awakeFlag;
				this.m_sleepTime = 0.0;
			}
		}
		else
		{
			this.m_flags &= ~b2Body.e_awakeFlag;
			this.m_sleepTime = 0.0;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0.0;
			this.m_force.SetZero();
			this.m_torque = 0.0;
		}
	},

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	IsAwake: function()
	{
		return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
	},

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b2World object and remains
	/// in the body list.
	SetActive: function(flag)
	{
		b2Assert(this.m_world.IsLocked() == false);

		if (flag == this.IsActive())
		{
			return;
		}

		if (flag)
		{
			this.m_flags |= b2Body.e_activeFlag;

			// Create all proxies.
			var broadPhase = this.m_world.m_contactManager.m_broadPhase;
			for (var f = this.m_fixtureList; f; f = f.m_next)
			{
				f.CreateProxies(broadPhase, this.m_xf);
			}

			// Contacts are created the next time step.
		}
		else
		{
			this.m_flags &= ~b2Body.e_activeFlag;

			// Destroy all proxies.
			var broadPhase = this.m_world.m_contactManager.m_broadPhase;
			for (var f = this.m_fixtureList; f; f = f.m_next)
			{
				f.DestroyProxies(broadPhase);
			}

			// Destroy the attached contacts.
			var ce = this.m_contactList;
			while (ce)
			{
				var ce0 = ce;
				ce = ce.next;
				this.m_world.m_contactManager.Destroy(ce0.contact);
			}
			this.m_contactList = null;
		}
	},

	/// Get the active state of the body.
	IsActive: function()
	{
		return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
	},

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	SetFixedRotation: function(flag)
	{
		var status = (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
		if (status == flag)
		{
			return;
		}

		if (flag)
		{
			this.m_flags |= b2Body.e_fixedRotationFlag;
		}
		else
		{
			this.m_flags &= ~b2Body.e_fixedRotationFlag;
		}

		this.m_angularVelocity = 0.0;

		this.ResetMassData();
	},

	/// Does this body have fixed rotation?
	IsFixedRotation: function()
	{
		return (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
	},

	/// Get the list of all fixtures attached to this body.
	GetFixtureList: function()
	{
		return this.m_fixtureList;
	},

	/// Get the list of all joints attached to this body.
	GetJointList: function()
	{
		return this.m_jointList;
	},

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use b2ContactListener.
	GetContactList: function()
	{
		return this.m_contactList;
	},

	/// Get the next body in the world's body list.
	GetNext: function()
	{
		return this.m_next;
	},

	/// Get the user data pointer that was provided in the body definition.
	GetUserData: function()
	{
		return this.m_userData;
	},

	/// Set the user data. Use this to store your application specific data.
	SetUserData: function(data)
	{
		this.m_userData = data;
	},

	/// Get the parent world of this body.
	GetWorld: function()
	{
		return this.m_world;
	},

	SynchronizeFixtures: function()
	{
		var xf1 = new b2Transform();
		xf1.q.Set(this.m_sweep.a0);
		xf1.p.Assign(b2Vec2.Subtract(this.m_sweep.c0, b2Mul_r_v2(xf1.q, this.m_sweep.localCenter)));

		var broadPhase = this.m_world.m_contactManager.m_broadPhase;
		for (var f = this.m_fixtureList; f; f = f.m_next)
		{
			f.Synchronize(broadPhase, xf1, this.m_xf);
		}
	},
	SynchronizeTransform: function()
	{
		this.m_xf.q.Set(this.m_sweep.a);
		this.m_xf.p.Assign(b2Vec2.Subtract(this.m_sweep.c, b2Mul_r_v2(this.m_xf.q, this.m_sweep.localCenter)));
	},

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	ShouldCollide: function(other)
	{
		// At least one body should be dynamic.
		if (this.m_type != b2Body.b2_dynamicBody && other.m_type != b2Body.b2_dynamicBody)
		{
			return false;
		}

		// Does a joint prevent collision?
		for (var jn = this.m_jointList; jn; jn = jn.next)
		{
			if (jn.other == other)
			{
				if (jn.joint.m_collideConnected == false)
				{
					return false;
				}
			}
		}

		return true;
	},

	Advance: function(alpha)
	{
		// Advance to the new safe time. This doesn't sync the broad-phase.
		this.m_sweep.Advance(alpha);
		this.m_sweep.c.Assign(this.m_sweep.c0);
		this.m_sweep.a = this.m_sweep.a0;
		this.m_xf.q.Set(this.m_sweep.a);
		this.m_xf.p.Assign(b2Vec2.Subtract(this.m_sweep.c, b2Mul_r_v2(this.m_xf.q, this.m_sweep.localCenter)));
	},

	_serialize: function(out)
	{
		var obj = out || {};

		// this will be filled in later by the serializer
		obj['fixtures'] = null;
		obj['type'] = this.m_type;
		obj['position'] = this.GetPosition()._serialize();
		obj['angle'] = this.GetAngle();
		obj['linearVelocity'] = this.GetLinearVelocity()._serialize();
		obj['angularVelocity'] = this.GetAngularVelocity();
		obj['linearDamping'] = this.GetLinearDamping();
		obj['angularDamping'] = this.GetAngularDamping();
		obj['allowSleep'] = this.IsSleepingAllowed();
		obj['awake'] = this.IsAwake();
		obj['fixedRotation'] = this.IsFixedRotation();
		obj['bullet'] = this.IsBullet();
		obj['active'] = this.IsActive();
		obj['gravityScale'] = this.GetGravityScale();

		return obj;
	}
};

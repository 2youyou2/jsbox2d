"use strict";

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
function b2World(gravity)
{
	this.m_contactManager = new b2ContactManager();
	this.m_profile = new b2Profile();

	this.m_destructionListener = null;
	this.g_debugDraw = null;

	this.m_bodyList = null;
	this.m_jointList = null;

	this.m_bodyCount = 0;
	this.m_jointCount = 0;

	this.m_warmStarting = true;
	this.m_continuousPhysics = true;
	this.m_subStepping = false;

	this.m_stepComplete = true;

	this.m_allowSleep = true;
	this.m_gravity = gravity;

	this.m_flags = b2World.e_clearForces;

	this.m_inv_dt0 = 0.0;
}

function b2WorldQueryWrapper()
{
	this.broadPhase = null;
	this.callback = null;
}

b2WorldQueryWrapper.prototype =
{
	QueryCallback: function(proxyId)
	{
		var proxy = this.broadPhase.GetUserData(proxyId);
		return this.callback.ReportFixture(proxy.fixture);
	}
};

function b2WorldRayCastWrapper()
{
	this.broadPhase = null;
	this.callback = null;
}

b2WorldRayCastWrapper.prototype =
{
	RayCastCallback: function(input, proxyId)
	{
		var userData = this.broadPhase.GetUserData(proxyId);
		var proxy = userData;
		var fixture = proxy.fixture;
		var index = proxy.childIndex;
		var output = new b2RayCastOutput();
		var hit = fixture.RayCast(output, input, index);

		if (hit)
		{
			var fraction = output.fraction;
			var point = b2Vec2.Add(b2Vec2.Multiply((1.0 - fraction), input.p1), b2Vec2.Multiply(fraction, input.p2));
			return this.callback.ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}
};

b2World.m_local_sweep_backupA = new b2Sweep();
b2World.m_local_sweep_backupB = new b2Sweep();
b2World.m_local_sweep_backupC = new b2Sweep();

b2World.prototype =
{
	Destroy: function()
	{
		// Some shapes allocate using b2Alloc.
		var b = this.m_bodyList;
		while (b)
		{
			var bNext = b.m_next;

			var f = b.m_fixtureList;
			while (f)
			{
				var fNext = f.m_next;
				f.m_proxyCount = 0;
				f.Destroy();
				f = fNext;
			}

			b = bNext;
		}
	},

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	SetDestructionListener: function(listener)
	{
		this.m_destructionListener = listener;
	},

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter). The listener is
	/// owned by you and must remain in scope.
	SetContactFilter: function(filter)
	{
		this.m_contactManager.m_contactFilter = filter;
	},

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	SetContactListener: function(listener)
	{
		this.m_contactManager.m_contactListener = listener;
	},

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with b2World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	SetDebugDraw: function(debugDraw)
	{
		this.g_debugDraw = debugDraw;
	},

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	CreateBody: function(def)
	{
		b2Assert(this.IsLocked() == false);
		if (this.IsLocked())
		{
			return null;
		}

		var b = new b2Body(def, this);

		// Add to world doubly linked list.
		b.m_prev = null;
		b.m_next = this.m_bodyList;
		if (this.m_bodyList)
		{
			this.m_bodyList.m_prev = b;
		}
		this.m_bodyList = b;
		++this.m_bodyCount;

		return b;
	},

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	DestroyBody: function(b)
	{
		b2Assert(this.m_bodyCount > 0);
		b2Assert(this.IsLocked() == false);
		if (this.IsLocked())
		{
			return;
		}

		// Delete the attached joints.
		var je = b.m_jointList;
		while (je)
		{
			var je0 = je;
			je = je.next;

			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeJoint(je0.joint);
			}

			this.DestroyJoint(je0.joint);

			b.m_jointList = je;
		}
		b.m_jointList = null;

		// Delete the attached contacts.
		var ce = b.m_contactList;
		while (ce)
		{
			var ce0 = ce;
			ce = ce.next;
			this.m_contactManager.Destroy(ce0.contact);
		}
		b.m_contactList = null;

		// Delete the attached fixtures. This destroys broad-phase proxies.
		var f = b.m_fixtureList;
		while (f)
		{
			var f0 = f;
			f = f.m_next;

			if (this.m_destructionListener)
			{
				this.m_destructionListener.SayGoodbyeFixture(f0);
			}

			f0.DestroyProxies(this.m_contactManager.m_broadPhase);
			f0.Destroy();

			b.m_fixtureList = f;
			b.m_fixtureCount -= 1;
		}
		b.m_fixtureList = null;
		b.m_fixtureCount = 0;

		// Remove world body list.
		if (b.m_prev)
		{
			b.m_prev.m_next = b.m_next;
		}

		if (b.m_next)
		{
			b.m_next.m_prev = b.m_prev;
		}

		if (b == this.m_bodyList)
		{
			this.m_bodyList = b.m_next;
		}

		b.m_destroyed = true;

		--this.m_bodyCount;
	},

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	CreateJoint: function(def)
	{
		b2Assert(this.IsLocked() == false);
		if (this.IsLocked())
		{
			return null;
		}

		var j = b2Joint.Create(def);

		// Connect to the world list.
		j.m_prev = null;
		j.m_next = this.m_jointList;
		if (this.m_jointList)
		{
			this.m_jointList.m_prev = j;
		}
		this.m_jointList = j;
		++this.m_jointCount;

		// Connect to the bodies' doubly linked lists.
		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;
		if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
		j.m_bodyA.m_jointList = j.m_edgeA;

		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;
		if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
		j.m_bodyB.m_jointList = j.m_edgeB;

		var bodyA = def.bodyA;
		var bodyB = def.bodyB;

		// If the joint prevents collisions, then flag any contacts for filtering.
		if (def.collideConnected == false)
		{
			var edge = bodyB.GetContactList();
			while (edge)
			{
				if (edge.other == bodyA)
				{
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}

		// Note: creating a joint doesn't wake the bodies.

		return j;
	},

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	DestroyJoint: function(j)
	{
		b2Assert(this.IsLocked() == false);
		if (this.IsLocked())
		{
			return;
		}

		var collideConnected = j.m_collideConnected;

		// Remove from the doubly linked list.
		if (j.m_prev)
		{
			j.m_prev.m_next = j.m_next;
		}

		if (j.m_next)
		{
			j.m_next.m_prev = j.m_prev;
		}

		if (j == this.m_jointList)
		{
			this.m_jointList = j.m_next;
		}

		// Disconnect from island graph.
		var bodyA = j.m_bodyA;
		var bodyB = j.m_bodyB;

		// Wake up connected bodies.
		bodyA.SetAwake(true);
		bodyB.SetAwake(true);

		// Remove from body 1.
		if (j.m_edgeA.prev)
		{
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}

		if (j.m_edgeA.next)
		{
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}

		if (j.m_edgeA == bodyA.m_jointList)
		{
			bodyA.m_jointList = j.m_edgeA.next;
		}

		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;

		// Remove from body 2
		if (j.m_edgeB.prev)
		{
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}

		if (j.m_edgeB.next)
		{
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}

		if (j.m_edgeB == bodyB.m_jointList)
		{
			bodyB.m_jointList = j.m_edgeB.next;
		}

		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;

		b2Joint.Destroy(j);

		b2Assert(this.m_jointCount > 0);
		--this.m_jointCount;

		// If the joint prevents collisions, then flag any contacts for filtering.
		if (collideConnected == false)
		{
			var edge = bodyB.GetContactList();
			while (edge)
			{
				if (edge.other == bodyA)
				{
					// Flag the contact for filtering at the next time step (where either
					// body is awake).
					edge.contact.FlagForFiltering();
				}

				edge = edge.next;
			}
		}
	},

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	Step: function(dt,
				velocityIterations,
				positionIterations)
	{
		var stepTimer = new b2Timer();

		// If new fixtures were added, we need to find the new contacts.
		if (this.m_flags & b2World.e_newFixture)
		{
			this.m_contactManager.FindNewContacts();
			this.m_flags &= ~b2World.e_newFixture;
		}

		this.m_flags |= b2World.e_locked;

		var step = new b2TimeStep();
		step.dt = dt;
		step.velocityIterations	= velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0)
		{
			step.inv_dt = 1.0 / dt;
		}
		else
		{
			step.inv_dt = 0.0;
		}

		step.dtRatio = this.m_inv_dt0 * dt;

		step.warmStarting = this.m_warmStarting;

		// Update contacts. This is where some contacts are destroyed.
		{
			var timer = new b2Timer();
			this.m_contactManager.Collide();
			this.m_profile.collide = timer.GetMilliseconds();
		}

		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (this.m_stepComplete && step.dt > 0.0)
		{
			timer = new b2Timer();
			this.Solve(step);
			this.m_profile.solve = timer.GetMilliseconds();
		}

		// Handle TOI events.
		if (this.m_continuousPhysics && step.dt > 0.0)
		{
			timer = new b2Timer();
			this.SolveTOI(step);
			this.m_profile.solveTOI = timer.GetMilliseconds();
		}

		if (step.dt > 0.0)
		{
			this.m_inv_dt0 = step.inv_dt;
		}

		if (this.m_flags & b2World.e_clearForces)
		{
			this.ClearForces();
		}

		this.m_flags &= ~b2World.e_locked;

		this.m_profile.step = stepTimer.GetMilliseconds();
	},

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// ClearForces after all sub-steps are complete in one pass of your game loop.
	/// @see SetAutoClearForces
	ClearForces: function()
	{
		for (var body = this.m_bodyList; body; body = body.GetNext())
		{
			body.m_force.SetZero();
			body.m_torque = 0.0;
		}
	},

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	DrawDebugData: function()
	{
		if (this.g_debugDraw == null)
		{
			return;
		}

		var flags = this.g_debugDraw.GetFlags();

		if (flags & b2Draw.e_shapeBit)
		{
			for (var b = this.m_bodyList; b; b = b.GetNext())
			{
				var xf = b.GetTransform();
				for (var f = b.GetFixtureList(); f; f = f.GetNext())
				{
					if (b.IsActive() == false)
					{
						this.DrawShape(f, xf, new b2Color(0.5, 0.5, 0.3));
					}
					else if (b.GetType() == b2Body.b2_staticBody)
					{
						this.DrawShape(f, xf, new b2Color(0.5, 0.9, 0.5));
					}
					else if (b.GetType() == b2Body.b2_kinematicBody)
					{
						this.DrawShape(f, xf, new b2Color(0.5, 0.5, 0.9));
					}
					else if (b.IsAwake() == false)
					{
						this.DrawShape(f, xf, new b2Color(0.6, 0.6, 0.6));
					}
					else
					{
						this.DrawShape(f, xf, new b2Color(0.9, 0.7, 0.7));
					}
				}
			}
		}

		if (flags & b2Draw.e_jointBit)
		{
			for (var j = this.m_jointList; j; j = j.GetNext())
			{
				this.DrawJoint(j);
			}
		}

		if (flags & b2Draw.e_pairBit)
		{
			var color = new b2Color(0.3, 0.9, 0.9);

			for (var c = this.m_contactManager.m_contactList; c; c = c.GetNext())
			{
				var fixtureA = c.GetFixtureA();
				var fixtureB = c.GetFixtureB();

				var cA = fixtureA.GetAABB(c.GetChildIndexA()).GetCenter();
				var cB = fixtureB.GetAABB(c.GetChildIndexB()).GetCenter();

				this.g_debugDraw.DrawSegment(cA, cB, color);
			}
		}

		if (flags & b2Draw.e_aabbBit)
		{
			var color = new b2Color(0.9, 0.3, 0.9);
			var bp = this.m_contactManager.m_broadPhase;

			for (var b = this.m_bodyList; b; b = b.GetNext())
			{
				if (b.IsActive() == false)
				{
					continue;
				}

				for (var f = b.GetFixtureList(); f; f = f.GetNext())
				{
					for (var i = 0; i < f.m_proxyCount; ++i)
					{
						var proxy = f.m_proxies[i];
						var aabb = bp.GetFatAABB(proxy.proxyId);
						var vs = [];
						vs[0] = new b2Vec2(aabb.lowerBound.x, aabb.lowerBound.y);
						vs[1] = new b2Vec2(aabb.upperBound.x, aabb.lowerBound.y);
						vs[2] = new b2Vec2(aabb.upperBound.x, aabb.upperBound.y);
						vs[3] = new b2Vec2(aabb.lowerBound.x, aabb.upperBound.y);

						this.g_debugDraw.DrawPolygon(vs, 4, color);
					}
				}
			}
		}

		if (flags & b2Draw.e_centerOfMassBit)
		{
			for (var b = this.m_bodyList; b; b = b.GetNext())
			{
				var xf = b.GetTransform().Clone();
				xf.p = b.GetWorldCenter();
				this.g_debugDraw.DrawTransform(xf);
			}
		}
	},

	/// Query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	QueryAABB: function(callback, aabb)
	{
		var wrapper = new b2WorldQueryWrapper();
		wrapper.broadPhase = this.m_contactManager.m_broadPhase;
		wrapper.callback = callback;
		this.m_contactManager.m_broadPhase.Query(wrapper, aabb);
	},

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	RayCast: function(callback, point1, point2)
	{
		var wrapper = new b2WorldRayCastWrapper();
		wrapper.broadPhase = this.m_contactManager.m_broadPhase;
		wrapper.callback = callback;
		var input = new b2RayCastInput();
		input.maxFraction = 1.0;
		input.p1 = point1;
		input.p2 = point2;
		this.m_contactManager.m_broadPhase.RayCast(wrapper, input);
	},

	/// Get the world body list. With the returned body, use b2Body::GetNext to get
	/// the next body in the world list. A null body indicates the end of the list.
	/** @returns {b2Body} the head of the world body list. */
	GetBodyList: function()
	{
		return this.m_bodyList;
	},

	/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	/// the next joint in the world list. A null joint indicates the end of the list.
	/// @return the head of the world joint list.
	GetJointList: function()
	{
		return this.m_jointList;
	},

	/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	/// the next contact in the world list. A null contact indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use b2ContactListener to avoid missing contacts.
	GetContactList: function()
	{
		return this.m_contactManager.m_contactList;
	},

	/// Enable/disable sleep.
	SetAllowSleeping: function(flag)
	{
		if (flag == this.m_allowSleep)
		{
			return;
		}

		this.m_allowSleep = flag;
		if (this.m_allowSleep == false)
		{
			for (var b = this.m_bodyList; b; b = b.m_next)
			{
				b.SetAwake(true);
			}
		}
	},
	GetAllowSleeping: function() { return this.m_allowSleep; },

	/// Enable/disable warm starting. For testing.
	SetWarmStarting: function(flag) { this.m_warmStarting = flag; },
	GetWarmStarting: function() { return this.m_warmStarting; },

	/// Enable/disable continuous physics. For testing.
	SetContinuousPhysics: function(flag) { this.m_continuousPhysics = flag; },
	GetContinuousPhysics: function() { return this.m_continuousPhysics; },

	/// Enable/disable single stepped continuous physics. For testing.
	SetSubStepping: function(flag) { this.m_subStepping = flag; },
	GetSubStepping: function() { return this.m_subStepping; },

	/// Get the number of broad-phase proxies.
	GetProxyCount: function()
	{
		return this.m_contactManager.m_broadPhase.GetProxyCount();
	},

	/// Get the number of bodies.
	GetBodyCount: function()
	{
		return this.m_bodyCount;
	},

	/// Get the number of joints.
	GetJointCount: function()
	{
		return this.m_jointCount;
	},

	/// Get the number of contacts (each may have 0 or more contact points).
	GetContactCount: function()
	{
		return this.m_contactManager.m_contactCount;
	},

	/// Get the height of the dynamic tree.
	GetTreeHeight: function()
	{
		return this.m_contactManager.m_broadPhase.GetTreeHeight();
	},

	/// Get the balance of the dynamic tree.
	GetTreeBalance: function()
	{
		return this.m_contactManager.m_broadPhase.GetTreeBalance();
	},

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	GetTreeQuality: function()
	{
		return this.m_contactManager.m_broadPhase.GetTreeQuality();
	},

	/// Change the global gravity vector.
	SetGravity: function(gravity)
	{
		this.m_gravity = gravity;
	},

	/// Get the global gravity vector.
	GetGravity: function()
	{
		return this.m_gravity;
	},

	/// Is the world locked (in the middle of a time step).
	IsLocked: function()
	{
		return (this.m_flags & b2World.e_locked) == b2World.e_locked;
	},

	/// Set flag to control automatic clearing of forces after each time step.
	SetAutoClearForces: function(flag)
	{
		if (flag)
		{
			this.m_flags |= b2World.e_clearForces;
		}
		else
		{
			this.m_flags &= ~b2World.e_clearForces;
		}
	},

	/// Get the flag that controls automatic clearing of forces after each time step.
	GetAutoClearForces: function()
	{
		return (this.m_flags & b2World.e_clearForces) == b2World.e_clearForces;
	},

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	ShiftOrigin: function(newOrigin)
	{
		b2Assert((this.m_flags & b2World.e_locked) == 0);
		if ((this.m_flags & b2World.e_locked) == b2World.e_locked)
		{
			return;
		}

		for (var b = this.m_bodyList; b; b = b.m_next)
		{
			b.m_xf.p.Subtract(newOrigin);
			b.m_sweep.c0.Subtract(newOrigin);
			b.m_sweep.c.Subtract(newOrigin);
		}

		for (var j = this.m_jointList; j; j = j.m_next)
		{
			j.ShiftOrigin(newOrigin);
		}

		this.m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
	},

	/// Get the contact manager for testing.
	GetContactManager: function()
	{
		return this.m_contactManager;
	},

	/// Get the current profile.
	GetProfile: function()
	{
		return this.m_profile;
	},

	Solve: function(step)
	{
		this.m_profile.solveInit = 0.0;
		this.m_profile.solveVelocity = 0.0;
		this.m_profile.solvePosition = 0.0;

		// Size the island for the worst case.
		var island = new b2Island(this.m_bodyCount,
						this.m_contactManager.m_contactCount,
						this.m_jointCount,
						this.m_contactManager.m_contactListener);

		// Clear all the island flags.
		for (var b = this.m_bodyList; b; b = b.m_next)
		{
			b.m_flags &= ~b2Body.e_islandFlag;
		}
		for (var c = this.m_contactManager.m_contactList; c; c = c.m_next)
		{
			c.m_flags &= ~b2Contact.e_islandFlag;
		}
		for (var j = this.m_jointList; j; j = j.m_next)
		{
			j.m_islandFlag = false;
		}

		// Build and simulate all awake islands.
		var stackSize = this.m_bodyCount;
		var stack = new Array(stackSize);
		for (var seed = this.m_bodyList; seed; seed = seed.m_next)
		{
			if (seed.m_flags & b2Body.e_islandFlag)
			{
				continue;
			}

			if (seed.IsAwake() == false || seed.IsActive() == false)
			{
				continue;
			}

			// The seed can be dynamic or kinematic.
			if (seed.GetType() == b2Body.b2_staticBody)
			{
				continue;
			}

			// Reset island and stack.
			island.Clear();
			var stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;

			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0)
			{
				// Grab the next body off the stack and add it to the island.
				var b = stack[--stackCount];
				b2Assert(b.IsActive() == true);
				island.AddBody(b);

				// Make sure the body is awake.
				b.SetAwake(true);

				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.GetType() == b2Body.b2_staticBody)
				{
					continue;
				}

				// Search all contacts connected to this body.
				for (var ce = b.m_contactList; ce; ce = ce.next)
				{
					var contact = ce.contact;

					// Has this contact already been added to an island?
					if (contact.m_flags & b2Contact.e_islandFlag)
					{
						continue;
					}

					// Is this contact solid and touching?
					if (contact.IsEnabled() == false ||
						contact.IsTouching() == false)
					{
						continue;
					}

					// Skip sensors.
					var sensorA = contact.m_fixtureA.m_isSensor;
					var sensorB = contact.m_fixtureB.m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					island.AddContact(contact);
					contact.m_flags |= b2Contact.e_islandFlag;

					var other = ce.other;

					// Was the other body already added to this island?
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}

				// Search all joints connect to this body.
				for (var je = b.m_jointList; je; je = je.next)
				{
					if (je.joint.m_islandFlag == true)
					{
						continue;
					}

					var other = je.other;

					// Don't simulate joints connected to inactive bodies.
					if (other.IsActive() == false)
					{
						continue;
					}

					island.AddJoint(je.joint);
					je.joint.m_islandFlag = true;

					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}

					b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}

			var profile = new b2Profile();
			island.Solve(profile, step, this.m_gravity, this.m_allowSleep);
			this.m_profile.solveInit += profile.solveInit;
			this.m_profile.solveVelocity += profile.solveVelocity;
			this.m_profile.solvePosition += profile.solvePosition;

			// Post solve cleanup.
			for (var i = 0; i < island.m_bodyCount; ++i)
			{
				// Allow static bodies to participate in other islands.
				var b = island.m_bodies[i];
				if (b.GetType() == b2Body.b2_staticBody)
				{
					b.m_flags &= ~b2Body.e_islandFlag;
				}
			}
		}

		{
			var timer = new b2Timer();
			// Synchronize fixtures, check for out of range bodies.
			for (var b = this.m_bodyList; b; b = b.GetNext())
			{
				// If a body was not in an island then it did not move.
				if ((b.m_flags & b2Body.e_islandFlag) == 0)
				{
					continue;
				}

				if (b.GetType() == b2Body.b2_staticBody)
				{
					continue;
				}

				// Update fixtures (for broad-phase).
				b.SynchronizeFixtures();
			}

			// Look for new contacts.
			this.m_contactManager.FindNewContacts();
			this.m_profile.broadphase = timer.GetMilliseconds();
		}
	},
	SolveTOI: function(step)
	{
		var island = new b2Island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, this.m_contactManager.m_contactListener);

		if (this.m_stepComplete)
		{
			for (var b = this.m_bodyList; b; b = b.m_next)
			{
				b.m_flags &= ~b2Body.e_islandFlag;
				b.m_sweep.alpha0 = 0.0;
			}

			for (var c = this.m_contactManager.m_contactList; c; c = c.m_next)
			{
				// Invalidate TOI
				c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
				c.m_toiCount = 0;
				c.m_toi = 1.0;
			}
		}

		// Find TOI events and solve them.
		for (;;)
		{
			// Find the first TOI.
			var minContact = null;
			var minAlpha = 1.0;

			for (var c = this.m_contactManager.m_contactList; c; c = c.m_next)
			{
				// Is this contact disabled?
				if (c.IsEnabled() == false)
				{
					continue;
				}

				// Prevent excessive sub-stepping.
				if (c.m_toiCount > b2_maxSubSteps)
				{
					continue;
				}

				var alpha = 1.0;
				if (c.m_flags & b2Contact.e_toiFlag)
				{
					// This contact has a valid cached TOI.
					alpha = c.m_toi;
				}
				else
				{
					var fA = c.GetFixtureA();
					var fB = c.GetFixtureB();

					// Is there a sensor?
					if (fA.IsSensor() || fB.IsSensor())
					{
						continue;
					}

					var bA = fA.GetBody();
					var bB = fB.GetBody();

					var typeA = bA.m_type;
					var typeB = bB.m_type;
					b2Assert(typeA == b2Body.b2_dynamicBody || typeB == b2Body.b2_dynamicBody);

					var activeA = bA.IsAwake() && typeA != b2Body.b2_staticBody;
					var activeB = bB.IsAwake() && typeB != b2Body.b2_staticBody;

					// Is at least one body active (awake and dynamic or kinematic)?
					if (activeA == false && activeB == false)
					{
						continue;
					}

					var collideA = bA.IsBullet() || typeA != b2Body.b2_dynamicBody;
					var collideB = bB.IsBullet() || typeB != b2Body.b2_dynamicBody;

					// Are these two non-bullet dynamic bodies?
					if (collideA == false && collideB == false)
					{
						continue;
					}

					// Compute the TOI for this contact.
					// Put the sweeps onto the same time interval.
					var alpha0 = bA.m_sweep.alpha0;

					if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
					{
						alpha0 = bB.m_sweep.alpha0;
						bA.m_sweep.Advance(alpha0);
					}
					else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
					{
						alpha0 = bA.m_sweep.alpha0;
						bB.m_sweep.Advance(alpha0);
					}

					b2Assert(alpha0 < 1.0);

					var indexA = c.GetChildIndexA();
					var indexB = c.GetChildIndexB();

					// Compute the time of impact in interval [0, minTOI]
					var input = new b2TOIInput();
					input.proxyA.Set(fA.GetShape(), indexA);
					input.proxyB.Set(fB.GetShape(), indexB);
					input.sweepA.Assign(bA.m_sweep);
					input.sweepB.Assign(bB.m_sweep);
					input.tMax = 1.0;

					var output = new b2TOIOutput();
					b2TimeOfImpact(output, input);

					// Beta is the fraction of the remaining portion of the .
					var beta = output.t;
					if (output.state == b2TOIOutput.e_touching)
					{
						alpha = b2Min(alpha0 + (1.0 - alpha0) * beta, 1.0);
					}
					else
					{
						alpha = 1.0;
					}

					c.m_toi = alpha;
					c.m_flags |= b2Contact.e_toiFlag;
				}

				if (alpha < minAlpha)
				{
					// This is the minimum TOI found so far.
					minContact = c;
					minAlpha = alpha;
				}
			}

			if (minContact == null || 1.0 - 10.0 * b2_epsilon < minAlpha)
			{
				// No more TOI events. Done!
				this.m_stepComplete = true;
				break;
			}

			// Advance the bodies to the TOI.
			var fA = minContact.GetFixtureA();
			var fB = minContact.GetFixtureB();
			var bA = fA.GetBody();
			var bB = fB.GetBody();

			b2World.m_local_sweep_backupA.Assign(bA.m_sweep);
			b2World.m_local_sweep_backupB.Assign(bB.m_sweep);

			bA.Advance(minAlpha);
			bB.Advance(minAlpha);

			// The TOI contact likely has some new contact points.
			minContact.Update(this.m_contactManager.m_contactListener);
			minContact.m_flags &= ~b2Contact.e_toiFlag;
			++minContact.m_toiCount;

			// Is the contact solid?
			if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
			{
				// Restore the sweeps.
				minContact.SetEnabled(false);
				bA.m_sweep.Assign(b2World.m_local_sweep_backupA);
				bB.m_sweep.Assign(b2World.m_local_sweep_backupB);
				bA.SynchronizeTransform();
				bB.SynchronizeTransform();
				continue;
			}

			bA.SetAwake(true);
			bB.SetAwake(true);

			// Build the island
			island.Clear();
			island.AddBody(bA);
			island.AddBody(bB);
			island.AddContact(minContact);

			bA.m_flags |= b2Body.e_islandFlag;
			bB.m_flags |= b2Body.e_islandFlag;
			minContact.m_flags |= b2Contact.e_islandFlag;

			// Get contacts on bodyA and bodyB.
			var bodies = [bA, bB];
			for (var i = 0; i < 2; ++i)
			{
				var body = bodies[i];
				if (body.m_type == b2Body.b2_dynamicBody)
				{
					for (var ce = body.m_contactList; ce; ce = ce.next)
					{
						if (island.m_bodyCount == island.m_bodyCapacity)
						{
							break;
						}

						if (island.m_contactCount == island.m_contactCapacity)
						{
							break;
						}

						var contact = ce.contact;

						// Has this contact already been added to the island?
						if (contact.m_flags & b2Contact.e_islandFlag)
						{
							continue;
						}

						// Only add static, kinematic, or bullet bodies.
						var other = ce.other;
						if (other.m_type == b2Body.b2_dynamicBody &&
							body.IsBullet() == false && other.IsBullet() == false)
						{
							continue;
						}

						// Skip sensors.
						var sensorA = contact.m_fixtureA.m_isSensor;
						var sensorB = contact.m_fixtureB.m_isSensor;
						if (sensorA || sensorB)
						{
							continue;
						}

						// Tentatively advance the body to the TOI.
						b2World.m_local_sweep_backupC.Assign(other.m_sweep);
						if ((other.m_flags & b2Body.e_islandFlag) == 0)
						{
							other.Advance(minAlpha);
						}

						// Update the contact points
						contact.Update(this.m_contactManager.m_contactListener);

						// Was the contact disabled by the user?
						if (contact.IsEnabled() == false)
						{
							other.m_sweep.Assign(b2World.m_local_sweep_backupC);
							other.SynchronizeTransform();
							continue;
						}

						// Are there contact points?
						if (contact.IsTouching() == false)
						{
							other.m_sweep.Assign(b2World.m_local_sweep_backupC);
							other.SynchronizeTransform();
							continue;
						}

						// Add the contact to the island
						contact.m_flags |= b2Contact.e_islandFlag;
						island.AddContact(contact);

						// Has the other body already been added to the island?
						if (other.m_flags & b2Body.e_islandFlag)
						{
							continue;
						}

						// Add the other body to the island.
						other.m_flags |= b2Body.e_islandFlag;

						if (other.m_type != b2Body.b2_staticBody)
						{
							other.SetAwake(true);
						}

						island.AddBody(other);
					}
				}
			}

			var subStep = new b2TimeStep();
			subStep.dt = (1.0 - minAlpha) * step.dt;
			subStep.inv_dt = 1.0 / subStep.dt;
			subStep.dtRatio = 1.0;
			subStep.positionIterations = 20;
			subStep.velocityIterations = step.velocityIterations;
			subStep.warmStarting = false;
			island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

			// Reset island flags and synchronize broad-phase proxies.
			for (var i = 0; i < island.m_bodyCount; ++i)
			{
				var body = island.m_bodies[i];
				body.m_flags &= ~b2Body.e_islandFlag;

				if (body.m_type != b2Body.b2_dynamicBody)
				{
					continue;
				}

				body.SynchronizeFixtures();

				// Invalidate all contact TOIs on this displaced body.
				for (var ce = body.m_contactList; ce; ce = ce.next)
				{
					ce.contact.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
				}
			}

			// Commit fixture proxy movements to the broad-phase so that new contacts are created.
			// Also, some contacts can be destroyed.
			this.m_contactManager.FindNewContacts();

			if (this.m_subStepping)
			{
				this.m_stepComplete = false;
				break;
			}
		}
	},

	DrawJoint: function(joint)
	{
		var bodyA = joint.GetBodyA();
		var bodyB = joint.GetBodyB();
		var xf1 = bodyA.GetTransform();
		var xf2 = bodyB.GetTransform();
		var x1 = xf1.p;
		var x2 = xf2.p;
		var p1 = joint.GetAnchorA();
		var p2 = joint.GetAnchorB();

		var color = new b2Color(0.5, 0.8, 0.8);

		switch (joint.GetType())
		{
		case b2Joint.e_distanceJoint:
			this.g_debugDraw.DrawSegment(p1, p2, color);
			break;

		case b2Joint.e_pulleyJoint:
			{
				var pulley = joint;
				var s1 = pulley.GetGroundAnchorA();
				var s2 = pulley.GetGroundAnchorB();
				this.g_debugDraw.DrawSegment(s1, p1, color);
				this.g_debugDraw.DrawSegment(s2, p2, color);
				this.g_debugDraw.DrawSegment(s1, s2, color);
			}
			break;

		case b2Joint.e_mouseJoint:
			// don't draw this
			break;

		case b2Joint.e_motorJoint:
			this.g_debugDraw.DrawPoint(joint.GetLinearOffset(), 5.0, color);

		default:
			this.g_debugDraw.DrawSegment(x1, p1, color);
			this.g_debugDraw.DrawSegment(p1, p2, color);
			this.g_debugDraw.DrawSegment(x2, p2, color);
		}
	},
	DrawShape: function(fixture, xf, color)
	{
		switch (fixture.GetType())
		{
		case b2Shape.e_circle:
			{
				var circle = fixture.GetShape();

				var center = b2Mul_t_v2(xf, circle.m_p);
				var radius = circle.m_radius;
				var axis = b2Mul_r_v2(xf.q, new b2Vec2(1.0, 0.0));

				this.g_debugDraw.DrawSolidCircle(center, radius, axis, color);
			}
			break;

		case b2Shape.e_edge:
			{
				var edge = fixture.GetShape();
				var v1 = b2Mul_t_v2(xf, edge.m_vertex1);
				var v2 = b2Mul_t_v2(xf, edge.m_vertex2);
				this.g_debugDraw.DrawSegment(v1, v2, color);
			}
			break;

		case b2Shape.e_chain:
			{
				var chain = fixture.GetShape();
				var count = chain.m_count;
				var vertices = chain.m_vertices;

				var v1 = b2Mul_t_v2(xf, vertices[0]);
				for (var i = 1; i < count; ++i)
				{
					var v2 = b2Mul_t_v2(xf, vertices[i]);
					this.g_debugDraw.DrawSegment(v1, v2, color);
					this.g_debugDraw.DrawCircle(v1, 0.05, color);
					v1 = v2;
				}
			}
			break;

		case b2Shape.e_polygon:
			{
				var poly = fixture.GetShape();
				var vertexCount = poly.m_count;
				b2Assert(vertexCount <= b2_maxPolygonVertices);
				var vertices = new Array(b2_maxPolygonVertices);

				for (var i = 0; i < vertexCount; ++i)
				{
					vertices[i] = b2Mul_t_v2(xf, poly.m_vertices[i]);
				}

				this.g_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
			}
			break;

		default:
			break;
		}
	}
};

b2World.e_newFixture = 0x0001;
b2World.e_locked = 0x0002;
b2World.e_clearForces = 0x0004;
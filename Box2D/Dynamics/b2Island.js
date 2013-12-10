"use strict";

/// This is an internal class.
function b2Island(
	bodyCapacity,
	contactCapacity,
	jointCapacity,
	listener)
{
	this.m_bodyCapacity = bodyCapacity;
	this.m_contactCapacity = contactCapacity;
	this.m_jointCapacity	 = jointCapacity;
	this.m_bodyCount = 0;
	this.m_contactCount = 0;
	this.m_jointCount = 0;

	this.m_listener = listener;

	this.m_bodies = new Array(bodyCapacity);
	this.m_contacts = new Array(contactCapacity);
	this.m_joints = new Array(jointCapacity);

	this.m_velocities = new Array(this.m_bodyCapacity);
	this.m_positions = new Array(this.m_bodyCapacity);
}

b2Island.prototype =
{
	Clear: function()
	{
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
	},

	Solve: function(profile, step, gravity, allowSleep)
	{
		var timer = new b2Timer();

		var h = step.dt;

		// Integrate velocities and apply damping. Initialize the body state.
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var b = this.m_bodies[i];

			var c = b.m_sweep.c.Clone();
			var a = b.m_sweep.a;
			var v = b.m_linearVelocity.Clone();
			var w = b.m_angularVelocity;

			// Store positions for continuous collision.
			b.m_sweep.c0.Assign(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			if (b.m_type == b2Body.b2_dynamicBody)
			{
				// Integrate velocities.
				v.Add(b2Vec2.Multiply(h, b2Vec2.Add(b2Vec2.Multiply(b.m_gravityScale, gravity), b2Vec2.Multiply(b.m_invMass, b.m_force))));
				w += h * b.m_invI * b.m_torque;

				// Apply damping.
				// ODE: dv/dt + c * v = 0
				// Solution: v(t) = v0 * exp(-c * t)
				// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
				// v2 = exp(-c * dt) * v1
				// Pade approximation:
				// v2 = v1 * 1 / (1 + c * dt)
				v.Multiply(1.0 / (1.0 + h * b.m_linearDamping));
				w *= 1.0 / (1.0 + h * b.m_angularDamping);
			}

			this.m_positions[i].c.Assign(c);
			this.m_positions[i].a = a;
			this.m_velocities[i].v.Assign(v);
			this.m_velocities[i].w = w;
		}

		timer.Reset();

		// Solver data
		var solverData = new b2SolverData();
		solverData.step = step;
		solverData.positions = this.m_positions;
		solverData.velocities = this.m_velocities;

		// Initialize velocity constraints.
		var contactSolverDef = new b2ContactSolverDef();
		contactSolverDef.step = step;
		contactSolverDef.contacts = this.m_contacts;
		contactSolverDef.count = this.m_contactCount;
		contactSolverDef.positions = this.m_positions;
		contactSolverDef.velocities = this.m_velocities;

		var contactSolver = new b2ContactSolver(contactSolverDef);
		contactSolver.InitializeVelocityConstraints();

		if (step.warmStarting)
		{
			contactSolver.WarmStart();
		}

		for (var i = 0; i < this.m_jointCount; ++i)
		{
			this.m_joints[i].InitVelocityConstraints(solverData);
		}

		profile.solveInit = timer.GetMilliseconds();

		// Solve velocity constraints
		timer.Reset();
		for (var i = 0; i < step.velocityIterations; ++i)
		{
			for (var j = 0; j < this.m_jointCount; ++j)
			{
				this.m_joints[j].SolveVelocityConstraints(solverData);
			}

			contactSolver.SolveVelocityConstraints();
		}

		// Store impulses for warm starting
		contactSolver.StoreImpulses();
		profile.solveVelocity = timer.GetMilliseconds();

		// Integrate positions
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var c = this.m_positions[i].c.Clone();
			var a = this.m_positions[i].a;
			var v = this.m_velocities[i].v.Clone();
			var w = this.m_velocities[i].w;

			// Check for large velocities
			var translation = b2Vec2.Multiply(h, v);
			if (b2Dot_v2_v2(translation, translation) > b2_maxTranslationSquared)
			{
				var ratio = b2_maxTranslation / translation.Length();
				v.Multiply(ratio);
			}

			var rotation = h * w;
			if (rotation * rotation > b2_maxRotationSquared)
			{
				var ratio = b2_maxRotation / b2Abs(rotation);
				w *= ratio;
			}

			// Integrate
			c.Add(b2Vec2.Multiply(h, v));
			a += h * w;

			this.m_positions[i].c.Assign(c);
			this.m_positions[i].a = a;
			this.m_velocities[i].v.Assign(v);
			this.m_velocities[i].w = w;
		}

		// Solve position constraints
		timer.Reset();
		var positionSolved = false;
		for (var i = 0; i < step.positionIterations; ++i)
		{
			var contactsOkay = contactSolver.SolvePositionConstraints();

			var jointsOkay = true;
			for (var x = 0; x < this.m_jointCount; ++x)
			{
				var jointOkay = this.m_joints[x].SolvePositionConstraints(solverData);
				jointsOkay = jointsOkay && jointOkay;
			}

			if (contactsOkay && jointsOkay)
			{
				// Exit early if the position errors are small.
				positionSolved = true;
				break;
			}
		}

		// Copy state buffers back to the bodies
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var body = this.m_bodies[i];
			body.m_sweep.c.Assign(this.m_positions[i].c);
			body.m_sweep.a = this.m_positions[i].a;
			body.m_linearVelocity.Assign(this.m_velocities[i].v);
			body.m_angularVelocity = this.m_velocities[i].w;
			body.SynchronizeTransform();
		}

		profile.solvePosition = timer.GetMilliseconds();

		this.Report(contactSolver.m_velocityConstraints);

		if (allowSleep)
		{
			var minSleepTime = b2_maxFloat;

			var linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
			var angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

			for (var i = 0; i < this.m_bodyCount; ++i)
			{
				var b = this.m_bodies[i];
				if (b.GetType() == b2Body.b2_staticBody)
				{
					continue;
				}

				if ((b.m_flags & b2Body.e_autoSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					b2Dot_v2_v2(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
				{
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}
				else
				{
					b.m_sleepTime += h;
					minSleepTime = b2Min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= b2_timeToSleep && positionSolved)
			{
				for (var i = 0; i < this.m_bodyCount; ++i)
				{
					var b = this.m_bodies[i];
					b.SetAwake(false);
				}
			}
		}
	},

	SolveTOI: function(subStep, toiIndexA, toiIndexB)
	{
		b2Assert(toiIndexA < this.m_bodyCount);
		b2Assert(toiIndexB < this.m_bodyCount);

		// Initialize the body state.
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var b = this.m_bodies[i];
			this.m_positions[i].c = b.m_sweep.c.Clone();
			this.m_positions[i].a = b.m_sweep.a;
			this.m_velocities[i].v = b.m_linearVelocity.Clone();
			this.m_velocities[i].w = b.m_angularVelocity;
		}

		var contactSolverDef = new b2ContactSolverDef();
		contactSolverDef.contacts = this.m_contacts;
		contactSolverDef.count = this.m_contactCount;
		contactSolverDef.step = subStep;
		contactSolverDef.positions = this.m_positions;
		contactSolverDef.velocities = this.m_velocities;
		var contactSolver = new b2ContactSolver(contactSolverDef);

		// Solve position constraints.
		for (var i = 0; i < subStep.positionIterations; ++i)
		{
			var contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
			if (contactsOkay)
			{
				break;
			}
		}

		// Leap of faith to new safe state.
		this.m_bodies[toiIndexA].m_sweep.c0 = this.m_positions[toiIndexA].c.Clone();
		this.m_bodies[toiIndexA].m_sweep.a0 = this.m_positions[toiIndexA].a;
		this.m_bodies[toiIndexB].m_sweep.c0 = this.m_positions[toiIndexB].c.Clone();
		this.m_bodies[toiIndexB].m_sweep.a0 = this.m_positions[toiIndexB].a;

		// No warm starting is needed for TOI events because warm
		// starting impulses were applied in the discrete solver.
		contactSolver.InitializeVelocityConstraints();

		// Solve velocity constraints.
		for (var i = 0; i < subStep.velocityIterations; ++i)
		{
			contactSolver.SolveVelocityConstraints();
		}

		// Don't store the TOI contact forces for warm starting
		// because they can be quite large.

		var h = subStep.dt;

		// Integrate positions
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var c = this.m_positions[i].c.Clone();
			var a = this.m_positions[i].a;
			var v = this.m_velocities[i].v.Clone();
			var w = this.m_velocities[i].w;

			// Check for large velocities
			var translation = b2Vec2.Multiply(h, v);
			if (b2Dot_v2_v2(translation, translation) > b2_maxTranslationSquared)
			{
				var ratio = b2_maxTranslation / translation.Length();
				v.Multiply(ratio);
			}

			var rotation = h * w;
			if (rotation * rotation > b2_maxRotationSquared)
			{
				var ratio = b2_maxRotation / b2Abs(rotation);
				w *= ratio;
			}

			// Integrate
			c.Add(b2Vec2.Multiply(h, v));
			a += h * w;

			this.m_positions[i].c.Assign(c);
			this.m_positions[i].a = a;
			this.m_velocities[i].v.Assign(v);
			this.m_velocities[i].w = w;

			// Sync bodies
			var body = this.m_bodies[i];
			body.m_sweep.c.Assign(c);
			body.m_sweep.a = a;
			body.m_linearVelocity.Assign(v);
			body.m_angularVelocity = w;
			body.SynchronizeTransform();
		}

		this.Report(contactSolver.m_velocityConstraints);
	},

	AddBody: function(body)
	{
		b2Assert(this.m_bodyCount < this.m_bodyCapacity);
		body.m_islandIndex = this.m_bodyCount;
		this.m_bodies[this.m_bodyCount] = body;

		if (!this.m_positions[this.m_bodyCount])
		{
			this.m_positions[this.m_bodyCount] = new b2Position();
			this.m_velocities[this.m_bodyCount] = new b2Velocity();
		}

		++this.m_bodyCount;
	},

	AddContact: function(contact)
	{
		b2Assert(this.m_contactCount < this.m_contactCapacity);
		this.m_contacts[this.m_contactCount++] = contact;
	},

	AddJoint: function(joint)
	{
		b2Assert(this.m_jointCount < this.m_jointCapacity);
		this.m_joints[this.m_jointCount++] = joint;
	},

	Report: function(constraints)
	{
		if (this.m_listener == null)
		{
			return;
		}

		for (var i = 0; i < this.m_contactCount; ++i)
		{
			var c = this.m_contacts[i];
			var vc = constraints[i];

			var impulse = new b2ContactImpulse();
			impulse.count = vc.pointCount;

			for (var j = 0; j < vc.pointCount; ++j)
			{
				impulse.normalImpulses[j] = vc.points[j].normalImpulse;
				impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
			}

			this.m_listener.PostSolve(c, impulse);
		}
	}
};
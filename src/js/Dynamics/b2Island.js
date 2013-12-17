/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
"use strict";

/// This is an internal class.
function b2Island()
{
	this.m_bodies = [];
	this.m_contacts = [];
	this.m_joints = [];

	this.m_velocities = [];
	this.m_positions = [];
}

var profile_solve_init = b2Profiler.create("solve initialization", "solve");
var profile_solve_init_warmStarting = b2Profiler.create("warm starting", "solve initialization");
var profile_solve_velocity = b2Profiler.create("solve velocities", "solve");
var profile_solve_position = b2Profiler.create("solve positions", "solve");

b2Island._solverData = new b2SolverData();
b2Island._solverDef = new b2ContactSolverDef();
b2Island._solver = new b2ContactSolver();

b2Island.prototype =
{
	Clear: function()
	{
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
	},

	Initialize: function(bodyCapacity, contactCapacity, jointCapacity, listener)
	{
		this.m_listener = listener;

		this.m_bodyCapacity = bodyCapacity;
		this.m_contactCapacity = contactCapacity;
		this.m_jointCapacity = jointCapacity;
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;

		this.m_bodies.length = bodyCapacity;
		this.m_contacts.length = contactCapacity;
		this.m_joints.length = jointCapacity;

		this.m_velocities.length = bodyCapacity;
		this.m_positions.length = bodyCapacity;
	},

	Solve: function(step, gravity, allowSleep)
	{
		profile_solve_init.start();

		var h = step.dt;

		// Integrate velocities and apply damping. Initialize the body state.
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var b = this.m_bodies[i];

			this.m_positions[i].c.Assign(b.m_sweep.c);
			var a = b.m_sweep.a;
			this.m_velocities[i].v.Assign(b.m_linearVelocity);
			var w = b.m_angularVelocity;

			// Store positions for continuous collision.
			b.m_sweep.c0.Assign(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;

			if (b.m_type == b2Body.b2_dynamicBody)
			{
				// Integrate velocities.
				//this.m_velocities[i].v.Add(b2Vec2.Multiply(h, b2Vec2.Add(b2Vec2.Multiply(b.m_gravityScale, gravity), b2Vec2.Multiply(b.m_invMass, b.m_force))));
				this.m_velocities[i].v.x += h * (b.m_gravityScale * gravity.x) + (b.m_invMass * b.m_force.x);
				this.m_velocities[i].v.y += h * (b.m_gravityScale * gravity.y) + (b.m_invMass * b.m_force.y);
				w += h * b.m_invI * b.m_torque;

				// Apply damping.
				// ODE: dv/dt + c * v = 0
				// Solution: v(t) = v0 * exp(-c * t)
				// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
				// v2 = exp(-c * dt) * v1
				// Pade approximation:
				// v2 = v1 * 1 / (1 + c * dt)
				//this.m_velocities[i].v.Multiply(1.0 / (1.0 + h * b.m_linearDamping));
				this.m_velocities[i].v.x *= 1.0 / (1.0 + h * b.m_linearDamping);
				this.m_velocities[i].v.y *= 1.0 / (1.0 + h * b.m_linearDamping);
				w *= 1.0 / (1.0 + h * b.m_angularDamping);
			}

			this.m_positions[i].a = a;
			this.m_velocities[i].w = w;
		}

		// Solver data
		b2Island._solverData.step = step;
		b2Island._solverData.positions = this.m_positions;
		b2Island._solverData.velocities = this.m_velocities;

		// Initialize velocity constraints.
		b2Island._solverDef.step = step;
		b2Island._solverDef.contacts = this.m_contacts;
		b2Island._solverDef.count = this.m_contactCount;
		b2Island._solverDef.positions = this.m_positions;
		b2Island._solverDef.velocities = this.m_velocities;
		b2Island._solverDef.allocator = this.m_allocator;

		b2Island._solver.Init(b2Island._solverDef);
		b2Island._solver.InitializeVelocityConstraints();

		if (step.warmStarting)
		{
			profile_solve_init_warmStarting.start();
			b2Island._solver.WarmStart();
			profile_solve_init_warmStarting.stop();
		}

		for (var i = 0; i < this.m_jointCount; ++i)
		{
			this.m_joints[i].InitVelocityConstraints(b2Island._solverData);
		}

		profile_solve_init.stop();

		// Solve velocity constraints
		profile_solve_velocity.start();
		for (var i = 0; i < step.velocityIterations; ++i)
		{
			for (var j = 0; j < this.m_jointCount; ++j)
			{
				this.m_joints[j].SolveVelocityConstraints(b2Island._solverData);
			}

			b2Island._solver.SolveVelocityConstraints();
		}

		// Store impulses for warm starting
		b2Island._solver.StoreImpulses();

		profile_solve_velocity.stop();
		profile_solve_position.start();

		// Integrate positions
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var c = this.m_positions[i].c;
			var a = this.m_positions[i].a;
			var v = this.m_velocities[i].v;
			var w = this.m_velocities[i].w;

			// Check for large velocities
			var translationx = h * v.x;
			var translationy = h * v.y;
			var translationl = translationx * translationx + translationy * translationy;

			if (translationl/*b2Dot_v2_v2(translation, translation)*/ > b2_maxTranslationSquared)
			{
				var ratio = b2_maxTranslation / b2Sqrt(translationl);
				v.x *= ratio;
				v.y *= ratio;
			}

			var rotation = h * w;
			if (rotation * rotation > b2_maxRotationSquared)
			{
				var ratio = b2_maxRotation / b2Abs(rotation);
				w *= ratio;
			}

			// Integrate
			//c.Add(b2Vec2.Multiply(h, v));
			c.x += h * v.x;
			c.y += h * v.y;
			a += h * w;

			this.m_positions[i].a = a;
			this.m_velocities[i].w = w;
		}

		// Solve position constraints
		var positionSolved = false;
		for (var i = 0; i < step.positionIterations; ++i)
		{
			var contactsOkay = b2Island._solver.SolvePositionConstraints();

			var jointsOkay = true;
			for (var j = 0; j < this.m_jointCount; ++j)
			{
				var jointOkay = this.m_joints[j].SolvePositionConstraints(b2Island._solverData);
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

		profile_solve_position.stop();

		this.Report(b2Island._solver.m_velocityConstraints);

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
			this.m_positions[i].c.Assign(b.m_sweep.c);
			this.m_positions[i].a = b.m_sweep.a;
			this.m_velocities[i].v.Assign(b.m_linearVelocity);
			this.m_velocities[i].w = b.m_angularVelocity;
		}

		b2Island._solverDef.contacts = this.m_contacts;
		b2Island._solverDef.count = this.m_contactCount;
		b2Island._solverDef.step = subStep;
		b2Island._solverDef.positions = this.m_positions;
		b2Island._solverDef.velocities = this.m_velocities;
		b2Island._solver.Init(b2Island._solverDef);

		// Solve position constraints.
		for (var i = 0; i < subStep.positionIterations; ++i)
		{
			var contactsOkay = b2Island._solver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
			if (contactsOkay)
			{
				break;
			}
		}

		// Leap of faith to new safe state.
		this.m_bodies[toiIndexA].m_sweep.c0.Assign(this.m_positions[toiIndexA].c);
		this.m_bodies[toiIndexA].m_sweep.a0 = this.m_positions[toiIndexA].a;
		this.m_bodies[toiIndexB].m_sweep.c0.Assign(this.m_positions[toiIndexB].c);
		this.m_bodies[toiIndexB].m_sweep.a0 = this.m_positions[toiIndexB].a;

		// No warm starting is needed for TOI events because warm
		// starting impulses were applied in the discrete solver.
		b2Island._solver.InitializeVelocityConstraints();

		// Solve velocity constraints.
		for (var i = 0; i < subStep.velocityIterations; ++i)
		{
			b2Island._solver.SolveVelocityConstraints();
		}

		// Don't store the TOI contact forces for warm starting
		// because they can be quite large.

		var h = subStep.dt;

		// Integrate positions
		for (var i = 0; i < this.m_bodyCount; ++i)
		{
			var c = this.m_positions[i].c;
			var a = this.m_positions[i].a;
			var v = this.m_velocities[i].v;
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

			this.m_positions[i].a = a;
			this.m_velocities[i].w = w;

			// Sync bodies
			var body = this.m_bodies[i];
			body.m_sweep.c.Assign(c);
			body.m_sweep.a = a;
			body.m_linearVelocity.Assign(v);
			body.m_angularVelocity = w;
			body.SynchronizeTransform();
		}

		this.Report(b2Island._solver.m_velocityConstraints);
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
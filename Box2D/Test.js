function ContactPoint()
{
	this.fixtureA = null;
	this.fixtureB = null;
	this.normal = new b2Vec2();
	this.position = new b2Vec2();
	this.state = 0;
	this.normalImpulse = 0;
	this.tangentImpulse = 0;
	this.separation = 0;
}

var k_maxContactPoints = 2048;

function Test()
{
	// Define the gravity vector.
	var gravity = new b2Vec2(0.0, -10.0);

	this.m_debugDraw = new CanvasDebugDraw();

	// Construct a world object, which will hold and simulate the rigid bodies.
	this.m_world = new b2World(gravity);
	this.m_world.SetDebugDraw(this.m_debugDraw);

	this.m_pointCount = 0;
	this.m_points = [];

	this.m_world.SetContactListener(this);

	var bodyDef = new b2BodyDef();
	this.m_groundBody = this.m_world.CreateBody(bodyDef);

	this.m_maxProfile = new b2Profile();
	this.m_totalProfile = new b2Profile();

	this.m_stepCount = 0;
}

function QueryCallback(point)
{
	this.m_point = point;
	this.m_fixture = null;
}

QueryCallback.prototype =
{
	ReportFixture: function(fixture)
	{
		var body = fixture.GetBody();
		if (body.GetType() == b2Body.b2_dynamicBody)
		{
			var inside = fixture.TestPoint(this.m_point);
			if (inside)
			{
				this.m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}
};

Test.prototype =
{
	Initialize: function()
	{
	},

	Step: function()
	{
		var timeStep = 1 / 60;//settings->hz > 0.0 ? 1.0 / settings->hz : float32(0.0);

		/*if (settings->pause)
		{
			if (settings->singleStep)
			{
				settings->singleStep = 0;
			}
			else
			{
				timeStep = 0.0;
			}

			g_debugDraw.DrawString(5, this->m_textLine, "****PAUSED****");
			this->m_textLine += DRAW_STRING_NEW_LINE;
		}

		uint32 flags = 0;
		flags += settings->drawShapes			* b2Draw::e_shapeBit;
		flags += settings->drawJoints			* b2Draw::e_jointBit;
		flags += settings->drawAABBs			* b2Draw::e_aabbBit;
		flags += settings->drawCOMs				* b2Draw::e_centerOfMassBit;
		g_debugDraw.SetFlags(flags);

		this->m_world->SetAllowSleeping(settings->enableSleep);
		this->m_world->SetWarmStarting(settings->enableWarmStarting);
		this->m_world->SetContinuousPhysics(settings->enableContinuous);
		this->m_world->SetSubStepping(settings->enableSubStepping);*/

		this.m_pointCount = 0;

		this.m_world.Step(timeStep, 8, 3);//settings->velocityIterations, settings->positionIterations);

		this.m_debugDraw.context.save();
		this.m_debugDraw.context.translate(this.m_debugDraw.context.canvas.width / 2, this.m_debugDraw.context.canvas.height / 2);
		this.m_debugDraw.context.scale(14, -14);
		this.m_debugDraw.context.lineWidth = 1 / 14;

		this.m_world.DrawDebugData();

		if (timeStep > 0.0)
		{
			++this.m_stepCount;
		}

		if (false)
		{
			var k_impulseScale = 0.1;
			var k_axisScale = 0.3;

			for (var i = 0; i < this.m_pointCount; ++i)
			{
				var point = this.m_points[i];

				if (point.state == b2Manifold.b2_addState)
				{
					// Add
					//this.m_debugDraw.DrawPoint(point.position, 1.0, new b2Color(0.3, 0.95, 0.3));
				}
				else if (point.state == b2Manifold.b2_persistState)
				{
					// Persist
					//this.m_debugDraw.DrawPoint(point.position, .50, new b2Color(0.3, 0.3, 0.95));
				}

				if (false)//settings.drawContactNormals == 1)
				{
					var p1 = point.position;
					var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_axisScale, point.normal));
					this.m_debugDraw.DrawSegment(p1, p2, new b2Color(0.9, 0.9, 0.9));
				}
				else if (false)//settings.drawContactImpulse == 1)
				{
					var p1 = point.position;
					var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_impulseScale, b2Vec2.Multiply(point.normalImpulse, point.normal)));
					this.m_debugDraw.DrawSegment(p1, p2, new b2Color(0.9, 0.9, 0.3));
				}

				if (false)//settings->drawFrictionImpulse == 1)
				{
					var tangent = b2Cross_v2_f(point.normal, 1.0);
					var p1 = point.position;
					var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_impulseScale, b2Vec2.Multiply(point.tangentImpulse, tangent)));
					this.m_debugDraw.DrawSegment(p1, p2, new b2Color(0.9, 0.9, 0.3));
				}
			}
		}

		if (this.m_mouseJoint)
		{
			var p1 = this.m_mouseJoint.GetAnchorB();
			var p2 = this.m_mouseJoint.GetTarget();

			var c = new b2Color();
			c.Set(0.0, 1.0, 0.0);
			this.m_debugDraw.DrawPoint(p1, 4.0 / 14, c);
			this.m_debugDraw.DrawPoint(p2, 4.0 / 14, c);

			c.Set(0.8, 0.8, 0.8);
			this.m_debugDraw.DrawSegment(p1, p2, c);
		}

		this.m_debugDraw.context.restore();

		this.m_debugDraw.context.fillStyle = "rgba(230, 153, 153, 1.0)";
		this.m_debugDraw.context.font = "12px Arial";

		//context.fillText("Test", 0, 12);
		if (true)
		{
			var bodyCount = this.m_world.GetBodyCount();
			var contactCount = this.m_world.GetContactCount();
			var jointCount = this.m_world.GetJointCount();
			this.m_debugDraw.context.fillText("bodies/contacts/joints = " + bodyCount + "/" + contactCount + "/" + jointCount, 0, 12);

			var proxyCount = this.m_world.GetProxyCount();
			var height = this.m_world.GetTreeHeight();
			var balance = this.m_world.GetTreeBalance();
			var quality = this.m_world.GetTreeQuality();
			this.m_debugDraw.context.fillText("proxies/height/balance/quality = " + proxyCount + "/" + height + "/" + balance + "/" + quality, 0, 24);
		}

		// Track maximum profile times
		{
			var p = this.m_world.GetProfile();
			this.m_maxProfile.step = b2Max(this.m_maxProfile.step, p.step);
			this.m_maxProfile.collide = b2Max(this.m_maxProfile.collide, p.collide);
			this.m_maxProfile.solve = b2Max(this.m_maxProfile.solve, p.solve);
			this.m_maxProfile.solveInit = b2Max(this.m_maxProfile.solveInit, p.solveInit);
			this.m_maxProfile.solveVelocity = b2Max(this.m_maxProfile.solveVelocity, p.solveVelocity);
			this.m_maxProfile.solvePosition = b2Max(this.m_maxProfile.solvePosition, p.solvePosition);
			this.m_maxProfile.solveTOI = b2Max(this.m_maxProfile.solveTOI, p.solveTOI);
			this.m_maxProfile.broadphase = b2Max(this.m_maxProfile.broadphase, p.broadphase);

			this.m_totalProfile.step += p.step;
			this.m_totalProfile.collide += p.collide;
			this.m_totalProfile.solve += p.solve;
			this.m_totalProfile.solveInit += p.solveInit;
			this.m_totalProfile.solveVelocity += p.solveVelocity;
			this.m_totalProfile.solvePosition += p.solvePosition;
			this.m_totalProfile.solveTOI += p.solveTOI;
			this.m_totalProfile.broadphase += p.broadphase;
		}

		/*if (this->m_bombSpawning)
		{
			b2Color c;
			c.Set(0.0, 0.0, 1.0);
			g_debugDraw.DrawPoint(this->m_bombSpawnPoint, 4.0, c);

			c.Set(0.8, 0.8, 0.8);
			g_debugDraw.DrawSegment(this->m_mouseWorld, this->m_bombSpawnPoint, c);
		}*/

		if (true)
		{
			var aveProfile = new b2Profile();

			if (this.m_stepCount > 0)
			{
				var scale = 1.0 / this.m_stepCount;
				aveProfile.step = scale * this.m_totalProfile.step;
				aveProfile.collide = scale * this.m_totalProfile.collide;
				aveProfile.solve = scale * this.m_totalProfile.solve;
				aveProfile.solveInit = scale * this.m_totalProfile.solveInit;
				aveProfile.solveVelocity = scale * this.m_totalProfile.solveVelocity;
				aveProfile.solvePosition = scale * this.m_totalProfile.solvePosition;
				aveProfile.solveTOI = scale * this.m_totalProfile.solveTOI;
				aveProfile.broadphase = scale * this.m_totalProfile.broadphase;
			}

			var y = 48;

			for (var x in p)
			{
				if (p.hasOwnProperty(x))
				{
					this.m_debugDraw.context.fillText(x + " [ave] (max) = " + p[x].toFixed(2) + " [" + aveProfile[x].toFixed(2) + "] (" + this.m_maxProfile[x].toFixed(2) + ")", 0, y);
					y += 12;
				}
			}
		}
	},

	Keyboard: function(key)
	{
	},

	KeyboardUp: function(key)
	{
	},

	MouseDown: function(p)
	{
		this.m_mouseWorld = p;

		if (this.m_mouseJoint != null)
		{
			return;
		}

		// Make a small box.
		var aabb = new b2AABB();
		var d = new b2Vec2();
		d.Set(0.001, 0.001);
		aabb.lowerBound = b2Vec2.Subtract(p, d);
		aabb.upperBound = b2Vec2.Add(p, d);

		// Query the world for overlapping shapes.
		var callback = new QueryCallback(p);
		this.m_world.QueryAABB(callback, aabb);

		if (callback.m_fixture)
		{
			var body = callback.m_fixture.GetBody();
			var md = new b2MouseJointDef();
			md.bodyA = this.m_groundBody;
			md.bodyB = body;
			md.target = p;
			md.maxForce = 1000.0 * body.GetMass();
			this.m_mouseJoint = this.m_world.CreateJoint(md);
			body.SetAwake(true);
		}
	},

	MouseUp: function(p)
	{
		if (this.m_mouseJoint)
		{
			this.m_world.DestroyJoint(this.m_mouseJoint);
			this.m_mouseJoint = null;
		}
	},

	MouseMove: function(p)
	{
		this.m_mouseWorld = p;

		if (this.m_mouseJoint)
		{
			this.m_mouseJoint.SetTarget(p);
		}
	},

	BeginContact: function() { },
	EndContact: function() { },
	PostSolve: function() { },

	PreSolve: function(contact, oldManifold)
	{
		var manifold = contact.GetManifold();

		if (manifold.pointCount == 0)
		{
			return;
		}

		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();

		var state1 = new Array(b2_maxManifoldPoints), state2 = new Array(b2_maxManifoldPoints);
		b2GetPointStates(state1, state2, oldManifold, manifold);

		var worldManifold = new b2WorldManifold();
		contact.GetWorldManifold(worldManifold);

		for (var i = 0; i < manifold.pointCount && this.m_pointCount < k_maxContactPoints; ++i)
		{
			var cp = this.m_points[this.m_pointCount] = this.m_points[this.m_pointCount] || new ContactPoint();
			cp.fixtureA = fixtureA;
			cp.fixtureB = fixtureB;
			cp.position.Assign(worldManifold.points[i]);
			cp.normal.Assign(worldManifold.normal);
			cp.state = state2[i];
			cp.normalImpulse = manifold.points[i].normalImpulse;
			cp.tangentImpulse = manifold.points[i].tangentImpulse;
			cp.separation = worldManifold.separations[i];
			++this.m_pointCount;
		}
	}
};
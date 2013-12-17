var profile_draw = b2Profiler.create("draw");

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

	// Construct a world object, which will hold and simulate the rigid bodies.
	this.m_world = new b2World(gravity);

	this.m_pointCount = 0;
	this.m_points = [];

	this.m_world.SetContactListener(this);
	this.m_world.SetDestructionListener(this);

	var bodyDef = new b2BodyDef();
	this.m_groundBody = this.m_world.CreateBody(bodyDef);

	this.m_stepCount = 0;
	this.m_pause = false;
	this.m_singleStep = 0;
	this.m_hz = 1 / 60;
	this.m_velIters = 8;
	this.m_posIters = 3;

	this.m_center = new b2Vec2();
	this.m_scale = 14;
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
	SayGoodbyeJoint: function(joint)
	{
		if (this.m_mouseJoint == joint)
			this.m_mouseJoint = null;
	},

	SayGoodbyeFixture: function(fixture) { },

	Initialize: function()
	{
	},

	recursiveDrawTimers: function(node, tab)
	{
		var totalChildrenValue = 0;

		for (var c in node.children)
		{
			var s = '';

			for (var i = 0; i < tab; ++i)
				s += ' ';

			var p = node.children[c];
			s += p.name + " : " + p.elapsedTime.toFixed(2) + "ms";

			if (node.parent)
			{
				var pcnt = Math.round((p.elapsedTime / node.elapsedTime) * 100);
				s += " (" + pcnt + ")%";
			}

			this.m_drawStringFunc(s);

			this.recursiveDrawTimers(p, tab + 1);

			totalChildrenValue += p.elapsedTime;
		}

		if (node.childrenCount && node.parent)
		{
			totalChildrenValue = node.elapsedTime - totalChildrenValue;
			var s = '';

			for (var i = 0; i < tab; ++i)
				s += ' ';

			s += "[untracked] : ~" + totalChildrenValue.toFixed(2) + "ms";

			pcnt = Math.round((totalChildrenValue / node.elapsedTime) * 100);
			s += " (~" + pcnt + "%)";

			this.m_drawStringFunc(s);
		}
	},

	Step: function()
	{
		var timeStep = this.m_hz > 0.0 ? this.m_hz : 0.0;

		if (this.m_pause)
		{
			if (this.m_singleStep)
			{
				this.m_singleStep = 0;
			}
			else
			{
				timeStep = 0.0;
			}
		}

		this.m_pointCount = 0;

		this.m_world.Step(timeStep, this.m_velIters, this.m_posIters);

		this.m_debugDraw.context.translate((this.m_debugDraw.context.canvas.width / 2), this.m_debugDraw.context.canvas.height - 128);
		this.m_debugDraw.context.scale(this.m_scale, -this.m_scale);
		this.m_debugDraw.context.translate(-this.m_center.x, -this.m_center.y);
		this.m_debugDraw.context.lineWidth = 1 / this.m_scale;

		profile_draw.start();
		this.m_world.DrawDebugData();

		if (timeStep > 0.0)
		{
			++this.m_stepCount;
		}

		if (this.m_debugDraw.m_drawFlags & b2Draw.e_contactPoints)
		{
			var k_impulseScale = 0.1;
			var k_axisScale = 0.3;

			for (var i = 0; i < this.m_pointCount; ++i)
			{
				var point = this.m_points[i];

				if (point.state == b2Manifold.b2_addState)
				{
					// Add
					this.m_debugDraw.DrawPoint(point.position, 10, new b2Color(0.3, 0.95, 0.3));
				}
				else if (point.state == b2Manifold.b2_persistState)
				{
					// Persist
					this.m_debugDraw.DrawPoint(point.position, 5.0, new b2Color(0.3, 0.3, 0.95));
				}

				if (this.m_debugDraw.m_drawFlags & b2Draw.e_contactNormals)
				{
					var p1 = point.position;
					var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_axisScale, point.normal));
					this.m_debugDraw.DrawSegment(p1, p2, new b2Color(0.9, 0.9, 0.9));
				}
				else if (this.m_debugDraw.m_drawFlags & b2Draw.e_contactImpulses)
				{
					var p1 = point.position;
					var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_impulseScale, b2Vec2.Multiply(point.normalImpulse, point.normal)));
					this.m_debugDraw.DrawSegment(p1, p2, new b2Color(0.9, 0.9, 0.3));
				}

				if (this.m_debugDraw.m_drawFlags & b2Draw.e_frictionImpulses)
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
			this.m_debugDraw.DrawPoint(p1, 4.0, c);
			this.m_debugDraw.DrawPoint(p2, 4.0, c);

			c.Set(0.8, 0.8, 0.8);
			this.m_debugDraw.DrawSegment(p1, p2, c);
		}
		profile_draw.stop();

		if (this.m_debugDraw.m_drawFlags & b2Draw.e_statistics)
		{
			var bodyCount = this.m_world.GetBodyCount();
			var contactCount = this.m_world.GetContactCount();
			var jointCount = this.m_world.GetJointCount();
			this.m_drawStringFunc("bodies/contacts/joints = " + bodyCount + "/" + contactCount + "/" + jointCount);

			var proxyCount = this.m_world.GetProxyCount();
			var height = this.m_world.GetTreeHeight();
			var balance = this.m_world.GetTreeBalance();
			var quality = this.m_world.GetTreeQuality();
			this.m_drawStringFunc("proxies/height/balance/quality = " + proxyCount + "/" + height + "/" + balance + "/" + quality);
		}

		if (this.m_debugDraw.m_drawFlags & b2Draw.e_profile)
		{
			/*var aveProfile = new b2Profile();

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
					this.m_drawStringFunc(x + " [ave] (max) = " + p[x].toFixed(2) + " [" + aveProfile[x].toFixed(2) + "] (" + this.m_maxProfile[x].toFixed(2) + ")");
					y += 12;
				}
			}*/

			this.recursiveDrawTimers(b2Profiler.profileRoot, 0);
		}

		b2Profiler.reset();
	},

	m_lastBackup: null,

	Keyboard: function(key)
	{
		if (key === 'P'.charCodeAt())
			this.m_pause = !this.m_pause;
		else if (key === 'Z'.charCodeAt())
			console.log(this.m_lastBackup = JSON.stringify(b2JsonSerializer.serialize(this.m_world)));
		else if (key === 'X'.charCodeAt())
			b2JsonSerializer.deserialize(this.m_lastBackup, this.m_world, true);
		else if (key === 'L'.charCodeAt())
			this.m_singleStep = 1;
	},

	KeyboardUp: function(key)
	{
	},

	MouseDown: function(p)
	{
		p.x += this.m_center.x;
		p.y += this.m_center.y;

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
		p.x += this.m_center.x;
		p.y += this.m_center.y;

		if (this.m_mouseJoint)
		{
			this.m_world.DestroyJoint(this.m_mouseJoint);
			this.m_mouseJoint = null;
		}
	},

	MouseMove: function(p)
	{
		p.x += this.m_center.x;
		p.y += this.m_center.y;

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
// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

// This callback finds the closest hit. Polygon 0 is filtered.
function RayCastClosestCallback()
{
	this.m_hit = false;
	this.m_point = new b2Vec2();
	this.m_normal = new b2Vec2();
}

RayCastClosestCallback.prototype =
{
	ReportFixture: function(fixture, point, normal, fraction)
	{
		var body = fixture.GetBody();
		var userData = body.GetUserData();
		if (userData)
		{
			var index = userData;
			if (index === 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture and
				// continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		this.m_hit = true;
		this.m_point.Assign(point);
		this.m_normal.Assign(normal);

		// By returning the current fraction, we instruct the calling code to clip the ray and
		// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
		// are reported in order. However, by clipping, we can always get the closest fixture.
		return fraction;
	}
};

// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
// just checking for obstruction, so the actual fixture and hit point are irrelevant.
function RayCastAnyCallback()
{
	this.m_hit = false;
	this.m_point = new b2Vec2();
	this.m_normal = new b2Vec2();
}

RayCastAnyCallback.prototype =
{
	ReportFixture: function(fixture, point, normal, fraction)
	{
		var body = fixture.GetBody();
		var userData = body.GetUserData();
		if (userData)
		{
			var index = userData;
			if (index === 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture
				// and continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		this.m_hit = true;
		this.m_point.Assign(point);
		this.m_normal.Assign(normal);

		// At this point we have a hit, so we know the ray is obstructed.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0;
	}
};

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
// The fixtures are not necessary reported in order, so we might not capture
// the closest fixture.
function RayCastMultipleCallback()
{
	this.m_count = 0;
	this.m_points = [];
	this.m_normals = [];
}

RayCastMultipleCallback.e_maxCount = 3;

RayCastMultipleCallback.prototype =
{
	ReportFixture: function(fixture, point, normal, fraction)
	{
		var body = fixture.GetBody();
		var userData = body.GetUserData();
		if (userData)
		{
			var index = userData;
			if (index === 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture
				// and continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		b2Assert(this.m_count < RayCastMultipleCallback.e_maxCount);

		this.m_points[this.m_count] = point.Clone();
		this.m_normals[this.m_count] = normal.Clone();
		++this.m_count;

		if (this.m_count == RayCastMultipleCallback.e_maxCount)
		{
			// At this point the buffer is full.
			// By returning 0, we instruct the calling code to terminate the ray-cast.
			return 0.0;
		}

		// By returning 1, we instruct the caller to continue without clipping the ray.
		return 1.0;
	}
};

function TestRayCast()
{
	this.parent.call(this);
}

TestRayCast.e_maxBodies = 256;

TestRayCast.e_closest = 0;
TestRayCast.e_any = 1;
TestRayCast.e_multiple = 2;

TestRayCast.prototype =
{
	Initialize: function()
	{
		this.m_polygons = [];
		this.m_userData = [];

		// Ground body
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var vertices = [];
			vertices[0] = new b2Vec2(-0.5, 0.0);
			vertices[1] = new b2Vec2(0.5, 0.0);
			vertices[2] = new b2Vec2(0.0, 1.5);
			this.m_polygons[0] = new b2PolygonShape();
			this.m_polygons[0].Set(vertices, 3);
		}

		{
			var vertices = [];
			vertices[0] = new b2Vec2(-0.1, 0.0);
			vertices[1] = new b2Vec2(0.1, 0.0);
			vertices[2] = new b2Vec2(0.0, 1.5);
			this.m_polygons[1] = new b2PolygonShape();
			this.m_polygons[1].Set(vertices, 3);
		}

		{
			var w = 1.0;
			var b = w / (2.0 + Math.sqrt(2.0));
			var s = Math.sqrt(2.0) * b;

			var vertices = [];
			vertices[0] = new b2Vec2(0.5 * s, 0.0);
			vertices[1] = new b2Vec2(0.5 * w, b);
			vertices[2] = new b2Vec2(0.5 * w, b + s);
			vertices[3] = new b2Vec2(0.5 * s, w);
			vertices[4] = new b2Vec2(-0.5 * s, w);
			vertices[5] = new b2Vec2(-0.5 * w, b + s);
			vertices[6] = new b2Vec2(-0.5 * w, b);
			vertices[7] = new b2Vec2(-0.5 * s, 0.0);

			this.m_polygons[2] = new b2PolygonShape();
			this.m_polygons[2].Set(vertices, 8);
		}

		{
			this.m_polygons[3] = new b2PolygonShape();
			this.m_polygons[3].SetAsBox(0.5, 0.5);
		}

		{
			this.m_circle = new b2CircleShape();
			this.m_circle.m_radius = 0.5;
		}

		{
			this.m_edge = new b2EdgeShape();
			this.m_edge.Set(new b2Vec2(-1.0, 0.0), new b2Vec2(1.0, 0.0));
		}

		this.m_bodyIndex = 0;
		this.m_bodies = [];

		this.m_angle = 0.0;

		this.m_mode = TestRayCast.e_closest;
	},

	Create: function(index)
	{
		if (this.m_bodies[this.m_bodyIndex] != null)
		{
			this.m_world.DestroyBody(this.m_bodies[this.m_bodyIndex]);
			this.m_bodies[this.m_bodyIndex] = null;
		}

		var bd = new b2BodyDef();

		var x = b2RandomFloat(-10.0, 10.0);
		var y = b2RandomFloat(0.0, 20.0);
		bd.position.Set(x, y);
		bd.angle = b2RandomFloat(-Math.PI, Math.PI);

		this.m_userData[this.m_bodyIndex] = index;
		bd.userData = this.m_userData + this.m_bodyIndex;

		if (index == 4)
		{
			bd.angularDamping = 0.02;
		}

		this.m_bodies[this.m_bodyIndex] = this.m_world.CreateBody(bd);

		if (index < 4)
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_polygons[index];
			fd.friction = 0.3;
			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}
		else if (index < 5)
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_circle;
			fd.friction = 0.3;

			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}
		else
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_edge;
			fd.friction = 0.3;

			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}

		this.m_bodyIndex = (this.m_bodyIndex + 1) % TestRayCast.e_maxBodies;
	},

	DestroyBody: function()
	{
		for (var i = 0; i < e_maxBodies; ++i)
		{
			if (this.m_bodies[i] != null)
			{
				this.m_world.DestroyBody(this.m_bodies[i]);
				this.m_bodies[i] = null;
				return;
			}
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case '1'.charCodeAt():
		case '2'.charCodeAt():
		case '3'.charCodeAt():
		case '4'.charCodeAt():
		case '5'.charCodeAt():
		case '6'.charCodeAt():
			this.Create(key - '1'.charCodeAt());
			break;

		case 'D'.charCodeAt():
			this.DestroyBody();
			break;

		case 'M'.charCodeAt():
			if (this.m_mode == TestRayCast.e_closest)
			{
				this.m_mode = TestRayCast.e_any;
			}
			else if (this.m_mode == TestRayCast.e_any)
			{
				this.m_mode = TestRayCast.e_multiple;
			}
			else if (this.m_mode == TestRayCast.e_multiple)
			{
				this.m_mode = TestRayCast.e_closest;
			}
		}
	},

	Step: function()
	{
		var advanceRay = this.m_pause == 0 || this.m_singleStep;

		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Press 1-6 to drop stuff, m to change the mode");
		switch (this.m_mode)
		{
		case TestRayCast.e_closest:
			this.m_drawStringFunc("Ray-cast mode: closest - find closest fixture along the ray");
			break;

		case TestRayCast.e_any:
			this.m_drawStringFunc("Ray-cast mode: any - check for obstruction");
			break;

		case TestRayCast.e_multiple:
			this.m_drawStringFunc("Ray-cast mode: multiple - gather multiple fixtures");
			break;
		}

		var L = 11.0;
		var point1 = new b2Vec2(0.0, 10.0);
		var d = new b2Vec2(L * Math.cos(this.m_angle), L * Math.sin(this.m_angle));
		var point2 = b2Vec2.Add(point1, d);

		if (this.m_mode == TestRayCast.e_closest)
		{
			var callback = new RayCastClosestCallback();
			this.m_world.RayCast(callback, point1, point2);

			if (callback.m_hit)
			{
				this.m_debugDraw.DrawPoint(callback.m_point, 5.0, new b2Color(0.4, 0.9, 0.4));
				this.m_debugDraw.DrawSegment(point1, callback.m_point, new b2Color(0.8, 0.8, 0.8));
				var head = b2Vec2.Add(callback.m_point, b2Vec2.Multiply(0.5, callback.m_normal));
				this.m_debugDraw.DrawSegment(callback.m_point, head, new b2Color(0.9, 0.9, 0.4));
			}
			else
			{
				this.m_debugDraw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));
			}
		}
		else if (this.m_mode == TestRayCast.e_any)
		{
			var callback = new RayCastAnyCallback();
			this.m_world.RayCast(callback, point1, point2);

			if (callback.m_hit)
			{
				this.m_debugDraw.DrawPoint(callback.m_point, 5.0, new b2Color(0.4, 0.9, 0.4));
				this.m_debugDraw.DrawSegment(point1, callback.m_point, new b2Color(0.8, 0.8, 0.8));
				var head = b2Vec2.Add(callback.m_point, b2Vec2.Multiply(0.5, callback.m_normal));
				this.m_debugDraw.DrawSegment(callback.m_point, head, new b2Color(0.9, 0.9, 0.4));
			}
			else
			{
				this.m_debugDraw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));
			}
		}
		else if (this.m_mode == TestRayCast.e_multiple)
		{
			var callback = new RayCastMultipleCallback();
			this.m_world.RayCast(callback, point1, point2);
			this.m_debugDraw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));

			for (var i = 0; i < callback.m_count; ++i)
			{
				var p = callback.m_points[i];
				var n = callback.m_normals[i];
				this.m_debugDraw.DrawPoint(p, 5.0, new b2Color(0.4, 0.9, 0.4));
				this.m_debugDraw.DrawSegment(point1, p, new b2Color(0.8, 0.8, 0.8));
				var head = b2Vec2.Add(p, b2Vec2.Multiply(0.5, n));
				this.m_debugDraw.DrawSegment(p, head, new b2Color(0.9, 0.9, 0.4));
			}
		}

		if (advanceRay)
		{
			this.m_angle += 0.25 * Math.PI / 180.0;
		}
	}
};

TestRayCast._extend(Test);
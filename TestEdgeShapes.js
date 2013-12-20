function EdgeShapesCallback()
{
	this.m_fixture = null;
	this.m_point = new b2Vec2();
	this.m_normal = new b2Vec2();
}

EdgeShapesCallback.prototype =
{
	ReportFixture: function(fixture, point,
						  normal, fraction)
	{
		this.m_fixture = fixture;
		this.m_point.Assign(point);
		this.m_normal.Assign(normal);

		return fraction;
	}
};

function TestEdgeShapes()
{
	this.parent.call(this);
}

TestEdgeShapes.prototype =
{
	e_maxBodies: 256,

	Initialize: function()
	{
		// Ground body
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			/*var x1 = -20.0;
			var y1 = 2.0 * Math.cos(x1 / 10.0 * Math.PI);
			for (var i = 0; i < 80; ++i)
			{
				var x2 = x1 + 0.5;
				var y2 = 2.0 * Math.cos(x2 / 10.0 * Math.PI);

				var shape = new b2EdgeShape();
				shape.Set(new b2Vec2(x1, y1), new b2Vec2(x2, y2));
				ground.CreateFixture(shape, 0.0);

				x1 = x2;
				y1 = y2;
			}*/

			var shape = new b2ChainShape();
			var verts = [];

			var w = 20;
			var yS = 4;

			verts.push(new b2Vec2(-w, Math.cos(0) * yS));

			var splits = 300;
			var mps = Math.PI / splits;
			var v = w / splits;

			for (var x = -w + v, pv = mps; x < w + v; x += v, pv += mps)
			{
				verts.push(new b2Vec2(x, Math.cos(pv) * yS));
			}

			shape.CreateChain(verts, verts.length);
			ground.CreateFixture(shape, 0);
		}

		this.m_polygons = [];
		this.m_circle = new b2CircleShape();

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
			this.m_circle.m_radius = 0.5;
		}

		this.m_bodyIndex = 0;
		this.m_bodies = [];

		this.m_angle = 0.0;
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
		var y = b2RandomFloat(10.0, 20.0);
		bd.position.Set(x, y);
		bd.angle = b2RandomFloat(-Math.PI, Math.PI);
		bd.type = b2Body.b2_dynamicBody;

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
			fd.density = 20.0;
			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}
		else
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_circle;
			fd.friction = 0.3;
			fd.density = 20.0;
			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}

		this.m_bodyIndex = (this.m_bodyIndex + 1) % this.e_maxBodies;
	},

	DestroyBody: function()
	{
		for (var i = 0; i < this.e_maxBodies; ++i)
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
			this.Create(key - '1'.charCodeAt());
			break;

		case 'D'.charCodeAt():
			this.DestroyBody();
			break;
		}
	},

	Step: function()
	{
		var advanceRay = this.m_pause == 0 || this.m_singleStep;

		this.parent.prototype.Step.call(this);
		this.m_drawStringFunc("Press 1-5 to drop stuff");

		var L = 25.0;
		var point1 = new b2Vec2(0.0, 10.0);
		var d = new b2Vec2(L * Math.cos(this.m_angle), -L * Math.abs(Math.sin(this.m_angle)));
		var point2 = b2Vec2.Add(point1, d);

		var callback = new EdgeShapesCallback();

		this.m_world.RayCast(callback, point1, point2);

		if (callback.m_fixture)
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

		if (advanceRay)
		{
			this.m_angle += 0.25 * Math.PI / 180.0;
		}
	}
};

TestEdgeShapes._extend(Test);
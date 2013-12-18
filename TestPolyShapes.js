
/// This tests stacking. It also shows how to use b2World::Query
/// and b2TestOverlap.

/// This callback is called by b2World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
function PolyShapesCallback(test)
{
	this.test = test;
	this.m_count = 0;

	this.m_circle = new b2CircleShape();
	this.m_transform = new b2Transform();
}

PolyShapesCallback.e_maxCount = 4;

PolyShapesCallback.prototype =
{
	DrawFixture: function(fixture)
	{
		var color = new b2Color(0.95, 0.95, 0.6);
		var xf = fixture.GetBody().GetTransform();

		switch (fixture.GetType())
		{
		case b2Shape.e_circle:
			{
				var circle = fixture.GetShape();

				var center = b2Mul_t_v2(xf, circle.m_p);
				var radius = circle.m_radius;

				this.test.m_debugDraw.DrawCircle(center, radius, color);
			}
			break;

		case b2Shape.e_polygon:
			{
				var poly = fixture.GetShape();
				var vertexCount = poly.m_count;
				var vertices = [];

				for (var i = 0; i < vertexCount; ++i)
				{
					vertices[i] = b2Mul_t_v2(xf, poly.m_vertices[i]);
				}

				this.test.m_debugDraw.DrawPolygon(vertices, vertexCount, color);
			}
			break;

		default:
			break;
		}
	},

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	ReportFixture: function(fixture)
	{
		if (this.m_count == PolyShapesCallback.e_maxCount)
		{
			return false;
		}

		var body = fixture.GetBody();
		var shape = fixture.GetShape();

		var overlap = b2TestShapeOverlap(shape, 0, this.m_circle, 0, body.GetTransform(), this.m_transform);

		if (overlap)
		{
			this.DrawFixture(fixture);
			++this.m_count;
		}

		return true;
	}
};

function TestPolyShapes()
{
	this.parent.call(this);
}

TestPolyShapes.e_maxBodies = 256;

TestPolyShapes.prototype =
{
	Initialize: function()
	{
		this.m_polygons = [];

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

		this.m_bodyIndex = 0;
		this.m_bodies = [];
	},

	Create: function(index)
	{
		if (this.m_bodies[this.m_bodyIndex] != null)
		{
			this.m_world.DestroyBody(this.m_bodies[this.m_bodyIndex]);
			this.m_bodies[this.m_bodyIndex] = null;
		}

		var bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;

		var x = b2RandomFloat(-2.0, 2.0);
		bd.position.Set(x, 10.0);
		bd.angle = b2RandomFloat(-Math.PI, Math.PI);

		if (index == 4)
		{
			bd.angularDamping = 0.02;
		}

		this.m_bodies[this.m_bodyIndex] = this.m_world.CreateBody(bd);

		if (index < 4)
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_polygons[index];
			fd.density = 1.0;
			fd.friction = 0.3;
			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}
		else
		{
			var fd = new b2FixtureDef();
			fd.shape = this.m_circle;
			fd.density = 1.0;
			fd.friction = 0.3;

			this.m_bodies[this.m_bodyIndex].CreateFixture(fd);
		}

		this.m_bodyIndex = (this.m_bodyIndex + 1) % TestPolyShapes.e_maxBodies;
	},

	DestroyBody: function()
	{
		for (var i = 0; i < TestPolyShapes.e_maxBodies; ++i)
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

		case 'A'.charCodeAt():
			for (var i = 0; i < TestPolyShapes.e_maxBodies; i += 2)
			{
				if (this.m_bodies[i])
				{
					var active = this.m_bodies[i].IsActive();
					this.m_bodies[i].SetActive(!active);
				}
			}
			break;

			case 'D'.charCodeAt():
			this.DestroyBody();
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var callback = new PolyShapesCallback(this);
		callback.m_circle.m_radius = 2.0;
		callback.m_circle.m_p.Set(0.0, 1.1);
		callback.m_transform.SetIdentity();

		var aabb = new b2AABB();
		callback.m_circle.ComputeAABB(aabb, callback.m_transform, 0);

		this.m_world.QueryAABB(callback, aabb);

		var color = new b2Color(0.4, 0.7, 0.8);
		this.m_debugDraw.DrawCircle(callback.m_circle.m_p, callback.m_circle.m_radius, color);

		this.m_drawStringFunc("Press 1-5 to drop stuff");
		this.m_drawStringFunc("Press 'a' to (de)activate some bodies");
		this.m_drawStringFunc("Press 'd' to destroy a body");
	}
};

TestPolyShapes._extend(Test);
function TestDistance()
{
	this.parent.call(this);
}

TestDistance.prototype =
{
	Initialize: function()
	{
		/*this.m_positionB = new b2Vec2();
		this.m_angleB;

		this.m_transformA = new b2Transform();
		this.m_transformB = new b2Transform();
		this.m_polygonA = new b2PolygonShape();
		this.m_polygonB = new b2PolygonShape();

		{
			this.m_transformA.SetIdentity();
			this.m_transformA.p.Set(0.0, -0.2);
			this.m_polygonA.SetAsBox(10.0, 0.2);
		}

		{
			this.m_positionB.Set(12.017401, 0.13678508);
			this.m_angleB = -0.0109265;
			this.m_transformB.Set(this.m_positionB, this.m_angleB);

			this.m_polygonB.SetAsBox(2.0, 0.1);
		}*/

		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var fd = new b2FixtureDef();
			fd.density = 0;
			fd.restitution = 0.2;

			fd.shape = new b2ChainShape();

			var verts = [ new b2Vec2(-20, -20), new b2Vec2(-20, 20), new b2Vec2(20, 20), new b2Vec2(20, -20) ];

			fd.shape.CreateLoop(verts, verts.length);
			ground.CreateFixture(fd);

			fd.shape = new b2PolygonShape();

			fd.shape.SetAsBox(1, 2, new b2Vec2(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16)), b2RandomFloat(0, Math.PI));
			ground.CreateFixture(fd);

			var cx = b2RandomFloat(-16, 16);
			var cy = b2RandomFloat(-16, 16);
			verts = [ new b2Vec2(cx - 1, cy), new b2Vec2(cx, cy - 2), new b2Vec2(cx + 1, cy) ];
			fd.shape.Set(verts, verts.length);
			ground.CreateFixture(fd);

			fd.shape = new b2CircleShape();
			fd.shape.m_radius = b2RandomFloat(1, 4);
			fd.shape.m_p.Set(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16));
			ground.CreateFixture(fd);

			verts = [ new b2Vec2(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16)), new b2Vec2(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16)), new b2Vec2(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16)), new b2Vec2(b2RandomFloat(-16, 16), b2RandomFloat(-16, 16)) ];
			fd.shape = new b2ChainShape();
			fd.shape.CreateChain(verts, verts.length);
			ground.CreateFixture(fd);

			this.ground = ground;
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.gravityScale = 0;

			var body = this.m_world.CreateBody(bd);

			var fd = new b2FixtureDef();
			fd.density = 0.2;
			fd.restitution = 0.2;
			fd.shape = new b2PolygonShape();
			fd.shape.SetAsBox(1, 0.15);

			body.CreateFixture(fd);

			this.m_distanceBody = body;
		}

		this.m_center.y -= 18;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var closestOutput;
		var closestDistance = Number.MAX_VALUE;

		var input = new b2DistanceInput();
		var cache = new b2SimplexCache();

		for (var f = this.ground.GetFixtureList(); f; f = f.GetNext())
		{
			var childIndexLimit = f.GetShape() instanceof b2ChainShape ? f.GetShape().m_vertices.length - 1 : 1;

			for (var i = 0; i < childIndexLimit; ++i)
			{
				var output = new b2DistanceOutput();
				input.proxyA.Set(this.m_distanceBody.GetFixtureList().GetShape(), 0);
				input.proxyB.Set(f.GetShape(), i);
				input.transformA.Assign(this.m_distanceBody.GetTransform());
				input.transformB.Assign(f.GetBody().GetTransform());
				input.useRadii = true;
				cache.count = 0;
				b2DistanceFunc(output, cache, input);

				if (output.distance < closestDistance)
				{
					closestOutput = output;
					closestDistance = output.distance;
				}
			}
		}

		this.m_drawStringFunc("distance = " + output.distance);
		this.m_drawStringFunc("iterations = " + output.iterations);

		var c1 = new b2Color(1.0, 0.0, 0.0);
		this.m_debugDraw.DrawPoint(closestOutput.pointA, 4.0, c1);

		var c2 = new b2Color(1.0, 1.0, 0.0);
		this.m_debugDraw.DrawPoint(closestOutput.pointB, 4.0, c2);

		this.m_debugDraw.DrawSegment(closestOutput.pointA, closestOutput.pointB, new b2Color(0, 1, 0));
	}
};

TestDistance._extend(Test);
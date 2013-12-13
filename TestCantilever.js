function TestCantilever()
{
	this.parent.call(this);
}

TestCantilever.prototype =
{
	e_count: 8,

	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;

			var jd = new b2WeldJointDef();

			var prevBody = ground;
			for (var i = 0; i < this.e_count; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-14.5 + 1.0 * i, 5.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				var anchor = new b2Vec2(-15.0 + 1.0 * i, 5.0);
				jd.Initialize(prevBody, body, anchor);
				this.m_world.CreateJoint(jd);

				prevBody = body;
			}
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(1.0, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;

			var jd = new b2WeldJointDef();
			jd.frequencyHz = 5.0;
			jd.dampingRatio = 0.7;

			var prevBody = ground;
			for (var i = 0; i < 3; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-14.0 + 2.0 * i, 15.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				var anchor = new b2Vec2(-15.0 + 2.0 * i, 15.0);
				jd.Initialize(prevBody, body, anchor);
				this.m_world.CreateJoint(jd);

				prevBody = body;
			}
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;

			var jd = new b2WeldJointDef();

			var prevBody = ground;
			for (var i = 0; i < this.e_count; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-4.5 + 1.0 * i, 5.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				if (i > 0)
				{
					var anchor = new b2Vec2(-5.0 + 1.0 * i, 5.0);
					jd.Initialize(prevBody, body, anchor);
					this.m_world.CreateJoint(jd);
				}

				prevBody = body;
			}
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;

			var jd = new b2WeldJointDef();
			jd.frequencyHz = 8.0;
			jd.dampingRatio = 0.7;

			var prevBody = ground;
			for (var i = 0; i < this.e_count; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(5.5 + 1.0 * i, 10.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				if (i > 0)
				{
					var anchor = new b2Vec2(5.0 + 1.0 * i, 10.0);
					jd.Initialize(prevBody, body, anchor);
					this.m_world.CreateJoint(jd);
				}

				prevBody = body;
			}
		}

		for (var i = 0; i < 2; ++i)
		{
			var vertices = [];
			vertices[0] = new b2Vec2(-0.5, 0.0);
			vertices[1] = new b2Vec2(0.5, 0.0);
			vertices[2] = new b2Vec2(0.0, 1.5);

			var shape = new b2PolygonShape();
			shape.Set(vertices, 3);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-8.0 + 8.0 * i, 12.0);
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(fd);
		}

		for (var i = 0; i < 2; ++i)
		{
			var shape = new b2CircleShape();
			shape.m_radius = 0.5;

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-6.0 + 6.0 * i, 10.0);
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(fd);
		}
	}
};

TestCantilever._extend(Test);
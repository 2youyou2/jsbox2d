function TestVaryingFriction()
{
	this.parent.call(this);
}

TestVaryingFriction.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(13.0, 0.25);

			var bd = new b2BodyDef();
			bd.position.Set(-4.0, 22.0);
			bd.angle = -0.25;

			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.25, 1.0);

			var bd = new b2BodyDef();
			bd.position.Set(10.5, 19.0);

			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(13.0, 0.25);

			var bd = new b2BodyDef();
			bd.position.Set(4.0, 14.0);
			bd.angle = 0.25;

			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.25, 1.0);

			var bd = new b2BodyDef();
			bd.position.Set(-10.5, 11.0);

			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(13.0, 0.25);

			var bd = new b2BodyDef();
			bd.position.Set(-4.0, 6.0);
			bd.angle = -0.25;

			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 25.0;

			var friction = [0.75, 0.5, 0.35, 0.1, 0.0];

			for (var i = 0; i < 5; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-15.0 + 4.0 * i, 28.0);
				var body = this.m_world.CreateBody(bd);

				fd.friction = friction[i];
				body.CreateFixture(fd);
			}
		}
	}
};

TestVaryingFriction._extend(Test);
function TestVaryingRestitution()
{
	this.parent.call(this);
}

TestVaryingRestitution.prototype =
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
			var shape = new b2CircleShape();
			shape.m_radius = 1.0;

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;

			var restitution = [0.0, 0.1, 0.3, 0.5, 0.75, 0.9, 1.0];

			for (var i = 0; i < 7; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-10.0 + 3.0 * i, 20.0);

				var body = this.m_world.CreateBody(bd);

				fd.restitution = restitution[i];
				body.CreateFixture(fd);
			}
		}
	}
};

TestVaryingRestitution._extend(Test);
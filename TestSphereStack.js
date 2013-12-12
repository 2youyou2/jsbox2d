function TestSphereStack()
{
	this.parent.call(this);
}

TestSphereStack.prototype =
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

			var e_count = 10;

			this.m_bodies = [];

			for (var i = 0; i < e_count; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(0.0, 4.0 + 3.0 * i);

				this.m_bodies[i] = this.m_world.CreateBody(bd);

				this.m_bodies[i].CreateFixture(shape, 1.0);

				this.m_bodies[i].SetLinearVelocity(new b2Vec2(0.0, -50.0));
			}
		}
	}
};

TestSphereStack._extend(Test);
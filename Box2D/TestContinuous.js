function TestContinuous()
{
	this.parent.call(this);
}

TestContinuous.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			bd.position.Set(0.0, 0.0);
			var body = this.m_world.CreateBody(bd);

			var edge = new b2EdgeShape();

			edge.Set(new b2Vec2(-10.0, 0.0), new b2Vec2(10.0, 0.0));
			body.CreateFixture(edge, 0.0);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.2, 1.0, new b2Vec2(0.5, 1.0), 0.0);
			body.CreateFixture(shape, 0.0);
		}

		if (true)
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 20.0);
			//bd.angle = 0.1;

			var shape = new b2PolygonShape();
			shape.SetAsBox(2.0, 0.1);

			this.m_body = this.m_world.CreateBody(bd);
			this.m_body.CreateFixture(shape, 1.0);

			this.m_angularVelocity = 0;
			//this.m_angularVelocity = 46.661274f;
			this.m_body.SetLinearVelocity(new b2Vec2(0.0, -100.0));
			this.m_body.SetAngularVelocity(this.m_angularVelocity);
		}
		else
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 2.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = 0.5;
			body.CreateFixture(shape, 1.0);

			bd.bullet = true;
			bd.position.Set(0.0, 10.0);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(shape, 1.0);
			body.SetLinearVelocity(new b2Vec2(0.0, -100.0));
		}

		b2_gjkCalls = 0; b2_gjkIters = 0; b2_gjkMaxIters = 0;
		b2_toiCalls = 0; b2_toiIters = 0;
		b2_toiRootIters = 0; b2_toiMaxRootIters = 0;
		b2_toiTime = 0.0; b2_toiMaxTime = 0.0;
	}
};

TestContinuous._extend(Test);
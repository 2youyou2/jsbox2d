function TestBullet()
{
	this.parent.call(this);
}

TestBullet.prototype =
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

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 4.0);

			var box = new b2PolygonShape();
			box.SetAsBox(2.0, 0.1);

			this.m_body = this.m_world.CreateBody(bd);
			this.m_body.CreateFixture(box, 1.0);

			box.SetAsBox(0.25, 0.25);

			//this->m_x = RandomFloat(-1.0, 1.0);
			this.m_x = 0.20352793;
			bd.position.Set(this.m_x, 10.0);
			bd.bullet = true;

			this.m_bullet = this.m_world.CreateBody(bd);
			this.m_bullet.CreateFixture(box, 100.0);

			this.m_bullet.SetLinearVelocity(new b2Vec2(0.0, -50.0));
		}
	},

	Launch: function()
	{
		this.m_body.SetTransform(new b2Vec2(0.0, 4.0), 0.0);
		this.m_body.SetLinearVelocity(new b2Vec2(0, 0));
		this.m_body.SetAngularVelocity(0.0);

		this.m_x = b2RandomFloat(-1.0, 1.0);
		this.m_bullet.SetTransform(new b2Vec2(this.m_x, 10.0), 0.0);
		this.m_bullet.SetLinearVelocity(new b2Vec2(0.0, -50.0));
		this.m_bullet.SetAngularVelocity(0.0);
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		if (this.m_stepCount % 60 == 0)
		{
			this.Launch();
		}
	}
};

TestBullet._extend(Test);
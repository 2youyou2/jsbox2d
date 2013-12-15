function TestTumbler()
{
	this.parent.call(this);
}

TestTumbler.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_kinematicBody;
			bd.allowSleep = false;
			bd.position.Set(0.0, 10.0);
			bd.angularVelocity = 0.25;
			var body = this.m_world.CreateBody(bd);

			var shape = new b2ChainShape();
			shape.CreateLoop([new b2Vec2(-10, -10), new b2Vec2(10, -10), new b2Vec2(10, 10), new b2Vec2(-10, 10)], 4);
			body.CreateFixture(shape, 5.0);
		}

		this.m_count = 0;
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var e_count = 400;

		if (this.m_count < e_count)
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.125, 0.125);
			body.CreateFixture(shape, 1.0);

			++this.m_count;
		}
	}
};

TestTumbler._extend(Test);
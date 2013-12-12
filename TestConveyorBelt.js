function TestConveyorBelt()
{
	this.parent.call(this);
}

TestConveyorBelt.prototype =
{
	Initialize: function()
	{
		// Ground
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-20.0, 0.0), new b2Vec2(20.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		// Platform
		{
			var bd = new b2BodyDef();
			bd.position.Set(-5.0, 5.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(10.0, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.friction = 0.8;
			this.m_platform = body.CreateFixture(fd);
		}

		var shape = new b2PolygonShape();
		shape.SetAsBox(0.5, 0.5);

		// Boxes
		for (var i = 0; i < 5; ++i)
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-10.0 + 2.0 * i, 7.0);
			var body = this.m_world.CreateBody(bd);

			body.CreateFixture(shape, 20.0);
		}
	},

	PreSolve: function(contact, oldManifold)
	{
		this.parent.prototype.PreSolve.call(this, contact, oldManifold);

		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();

		if (fixtureA == this.m_platform)
		{
			contact.SetTangentSpeed(5.0);
		}

		if (fixtureB == this.m_platform)
		{
			contact.SetTangentSpeed(-5.0);
		}
	}
};

TestConveyorBelt._extend(Test);
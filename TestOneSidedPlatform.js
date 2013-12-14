function TestOneSidedPlatform()
{
	this.parent.call(this);
}

TestOneSidedPlatform.prototype =
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
			bd.position.Set(0.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(3.0, 0.5);
			this.m_platform = body.CreateFixture(shape, 0.0);

			this.m_bottom = 10.0 - 0.5;
			this.m_top = 10.0 + 0.5;
		}

		// Actor
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 12.0);
			var body = this.m_world.CreateBody(bd);

			this.m_radius = 0.5;
			var shape = new b2CircleShape();
			shape.m_radius = this.m_radius;
			this.m_character = body.CreateFixture(shape, 20.0);

			body.SetLinearVelocity(new b2Vec2(0.0, -50.0));
		}
	},

	PreSolve: function(contact, oldManifold)
	{
		this.parent.prototype.PreSolve.call(this, contact, oldManifold);

		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();

		if (fixtureA != this.m_platform && fixtureA != this.m_character)
		{
			return;
		}

		if (fixtureB != this.m_platform && fixtureB != this.m_character)
		{
			return;
		}

		var position = this.m_character.GetBody().GetPosition();

		if (position.y < this.m_top + this.m_radius - 3.0 * b2_linearSlop)
		{
			contact.SetEnabled(false);
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Press: (c) create a shape, (d) destroy a shape.");

        var v = this.m_character.GetBody().GetLinearVelocity();
        this.m_drawStringFunc("Character Linear Velocity: " + v.y);
	}
};

TestOneSidedPlatform._extend(Test);
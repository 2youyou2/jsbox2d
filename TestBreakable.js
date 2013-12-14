function TestBreakable()
{
	this.parent.call(this);
}

TestBreakable.prototype =
{
	Initialize: function()
	{
		// Ground body
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		// Breakable dynamic body
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 40.0);
			bd.angle = 0.25 * Math.PI;
			this.m_body1 = this.m_world.CreateBody(bd);

			this.m_shape1 = new b2PolygonShape();
			this.m_shape1.SetAsBox(0.5, 0.5, new b2Vec2(-0.5, 0.0), 0.0);
			this.m_piece1 = this.m_body1.CreateFixture(this.m_shape1, 1.0);

			this.m_shape2 = new b2PolygonShape();
			this.m_shape2.SetAsBox(0.5, 0.5, new b2Vec2(0.5, 0.0), 0.0);
			this.m_piece2 = this.m_body1.CreateFixture(this.m_shape2, 1.0);
		}

		this.m_break = false;
		this.m_broke = false;
		this.m_velocity = new b2Vec2();
	},

	PostSolve: function(contact, impulse)
	{
		if (this.m_broke)
		{
			// The body already broke.
			return;
		}

		// Should the body break?
		var count = contact.GetManifold().pointCount;

		var maxImpulse = 0.0;
		for (var i = 0; i < count; ++i)
		{
			maxImpulse = Math.max(maxImpulse, impulse.normalImpulses[i]);
		}

		if (maxImpulse > 40.0)
		{
			// Flag the body for breaking.
			this.m_break = true;
		}
	},

	Break: function()
	{
		// Create two bodies from one.
		var body1 = this.m_piece1.GetBody();
		var center = body1.GetWorldCenter();

		body1.DestroyFixture(this.m_piece2);
		this.m_piece2 = null;

		var bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;
		bd.position.Assign(body1.GetPosition());
		bd.angle = body1.GetAngle();

		var body2 = this.m_world.CreateBody(bd);
		this.m_piece2 = body2.CreateFixture(this.m_shape2, 1.0);

		// Compute consistent velocities for new bodies based on
		// cached velocity.
		var center1 = body1.GetWorldCenter();
		var center2 = body2.GetWorldCenter();

		var velocity1 = b2Vec2.Add(this.m_velocity, b2Cross_f_v2(this.m_angularVelocity, b2Vec2.Subtract(center1, center)));
		var velocity2 = b2Vec2.Add(this.m_velocity, b2Cross_f_v2(this.m_angularVelocity, b2Vec2.Subtract(center2, center)));

		body1.SetAngularVelocity(this.m_angularVelocity);
		body1.SetLinearVelocity(velocity1);

		body2.SetAngularVelocity(this.m_angularVelocity);
		body2.SetLinearVelocity(velocity2);
	},

	Step: function()
	{
		if (this.m_break)
		{
			this.Break();
			this.m_broke = true;
			this.m_break = false;
		}

		// Cache velocities to improve movement on breakage.
		if (this.m_broke == false)
		{
			this.m_velocity.Assign(this.m_body1.GetLinearVelocity());
			this.m_angularVelocity = this.m_body1.GetAngularVelocity();
		}

		this.parent.prototype.Step.call(this);
	}
};

TestBreakable._extend(Test);
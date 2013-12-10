function TestSensor()
{
	this.parent.call(this);
}

TestSensor.prototype =
{
	e_count: 7,

	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			{
				var shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
				ground.CreateFixture(shape, 0.0);
			}

			if (false)
			{
				var sd = new b2FixtureDef();
				sd.SetAsBox(10.0, 2.0, new b2Vec2(0.0, 20.0), 0.0);
				sd.isSensor = true;
				this.m_sensor = ground.CreateFixture(sd);
			}
			else
			{
				var shape = new b2CircleShape();
				shape.m_radius = 5.0;
				shape.m_p.Set(0.0, 10.0);

				var fd = new b2FixtureDef();
				fd.shape = shape;
				fd.isSensor = true;
				this.m_sensor = ground.CreateFixture(fd);
			}
		}

		this.m_touching = [];
		this.m_bodies = [];

		{
			var shape = new b2CircleShape();
			shape.m_radius = 1.0;

			for (var i = 0; i < this.e_count; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-10.0 + 3.0 * i, 20.0);
				bd.userData = i;

				this.m_touching[i] = false;
				this.m_bodies[i] = this.m_world.CreateBody(bd);

				this.m_bodies[i].CreateFixture(shape, 1.0);
			}
		}
	},

	// Implement contact listener.
	BeginContact: function(contact)
	{
		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();

		if (fixtureA == this.m_sensor)
		{
			var userData = fixtureB.GetBody().GetUserData();
			if (userData !== null)
			{
				this.m_touching[userData] = true;
			}
		}

		if (fixtureB == this.m_sensor)
		{
			var userData = fixtureA.GetBody().GetUserData();
			if (userData !== null)
			{
				this.m_touching[userData] = true;
			}
		}
	},

	// Implement contact listener.
	EndContact: function(contact)
	{
		var fixtureA = contact.GetFixtureA();
		var fixtureB = contact.GetFixtureB();

		if (fixtureA == this.m_sensor)
		{
			var userData = fixtureB.GetBody().GetUserData();
			if (userData !== null)
			{
				this.m_touching[userData] = false;
			}
		}

		if (fixtureB == this.m_sensor)
		{
			var userData = fixtureA.GetBody().GetUserData();
			if (userData !== null)
			{
				this.m_touching[userData] = false;
			}
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (var i = 0; i < this.e_count; ++i)
		{
			if (this.m_touching[i] == false)
			{
				continue;
			}

			var body = this.m_bodies[i];
			var ground = this.m_sensor.GetBody();

			var circle = this.m_sensor.GetShape();
			var center = ground.GetWorldPoint(circle.m_p);

			var position = body.GetPosition();

			var d = b2Vec2.Subtract(center, position);
			if (d.LengthSquared() < b2_epsilon * b2_epsilon)
			{
				continue;
			}

			d.Normalize();
			var F = b2Vec2.Multiply(100.0, d);
			body.ApplyForce(F, position, false);
		}
	}
};

TestSensor._extend(Test);